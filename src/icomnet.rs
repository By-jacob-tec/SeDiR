//! Icom's network remote protocol, the one RS-BA1 uses: the IC-705 over WLAN, the
//! IC-7300MK2 over its LAN port (and other Icom radios with a network port).
//!
//! Three UDP streams to the radio: control (50001: login, keep-alive), CI-V (50002) and
//! audio (50003). Every stream starts with the same handshake, answers the radio's pings
//! and sends its own. The radio sends the receiver's audio, or its 12 kHz IF when
//! "LAN/WLAN AF/IF Output Select" is IF, as 16-bit PCM.
//!
//! Packet layouts follow kappanhang (Norbert Varga HA2NON, Akos Marton ES1AKOS, MIT
//! license), which documents the protocol from captures.

use std::collections::VecDeque;
use std::hash::{BuildHasher, Hasher};
use std::io;
use std::net::{IpAddr, Ipv4Addr, Ipv6Addr, SocketAddr, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// The radio's control port; CI-V and audio use the next two.
pub const CONTROL_PORT: u16 = 50001;
/// Audio sample rate asked for: 48 kHz carries the whole 12 kHz IF.
pub const SAMPLE_RATE: u32 = 48_000;

/// Answers expected within this time during the login.
const ANSWER_TIMEOUT: Duration = Duration::from_secs(1);
/// The whole login, up to the streams being granted.
const LOGIN_TIMEOUT: Duration = Duration::from_secs(6);
const PING_INTERVAL: Duration = Duration::from_millis(500);
/// Keep-alive (idle) packets: often while there is traffic, once a second otherwise.
const IDLE_BUSY: Duration = Duration::from_millis(100);
const IDLE_QUIET: Duration = Duration::from_secs(1);
const REAUTH_INTERVAL: Duration = Duration::from_secs(60);
/// No audio for this long ends the connection with an error.
const AUDIO_TIMEOUT: Duration = Duration::from_secs(5);
/// Sent packets kept for the radio's retransmit requests.
const KEEP_SENT: usize = 64;
/// Audio packets queued before new ones are dropped (about 2 s at 48 kHz).
const AUDIO_QUEUE: usize = 400;

/// Codec 0x04: linear PCM, 16 bit, one channel.
const CODEC_LPCM16_MONO: u8 = 0x04;

/// Where and how to log in.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Credentials {
    pub host: String,
    pub port: u16,
    pub username: String,
    pub password: String,
}

impl Credentials {
    /// Reads `user:password@host[:port]`. The password may contain ':'; the last '@'
    /// separates the host.
    pub fn parse(s: &str) -> Option<Self> {
        let (auth, host) = s.trim().rsplit_once('@')?;
        let (username, password) = auth.split_once(':')?;
        let (host, port) = match host.rsplit_once(':') {
            Some((h, p)) if !h.contains(':') || h.ends_with(']') => (h, p.parse().ok()?),
            _ => (host, CONTROL_PORT),
        };
        let host = host.trim_start_matches('[').trim_end_matches(']');
        (!host.is_empty() && !username.is_empty()).then(|| Self {
            host: host.to_string(),
            port,
            username: username.to_string(),
            password: password.to_string(),
        })
    }
}

/// What the radio told about itself at login.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RadioInfo {
    /// E.g. "IC-705".
    pub name: String,
    pub civ_address: u8,
}

/// A logged-in connection. Dropping it logs out.
pub struct IcomNet {
    pub info: RadioInfo,
    /// CI-V bytes from the radio.
    pub civ_rx: Receiver<Vec<u8>>,
    /// Audio (or IF) samples from the radio, in packets.
    pub audio_rx: Receiver<Vec<i16>>,
    send: NetSender,
    shared: Arc<Shared>,
    threads: Vec<JoinHandle<()>>,
}

/// State the stream threads share with the owner.
struct Shared {
    stop: AtomicBool,
    error: Mutex<Option<String>>,
    /// Audio packets lost or dropped.
    lost: AtomicU64,
    control: Arc<Stream>,
    auth: Mutex<Auth>,
}

impl Shared {
    fn fail(&self, e: String) {
        self.error.lock().unwrap().get_or_insert(e);
    }
}

#[derive(Default)]
struct Auth {
    inner_seq: u16,
    id: [u8; 6],
}

impl IcomNet {
    /// Logs in and opens the CI-V and audio streams. Takes up to a few seconds.
    pub fn connect(c: &Credentials) -> Result<Self, String> {
        let ip = resolve(&c.host, c.port)?;
        let control = Arc::new(Stream::open(ip, c.port).map_err(|e| format!("{}: {e}", c.host))?);
        control.handshake().map_err(|e| format!("{}:{} {e}", c.host, c.port))?;

        let mut auth = Auth { inner_seq: 0, id: [0; 6] };
        let start = random_u64().to_le_bytes();
        control.send_tracked(login_packet(&control, &mut auth, [start[0], start[1]], &c.username, &c.password));
        let reply = control
            .expect(ANSWER_TIMEOUT, |r| r.len() == 96 && r[0] == 0x60 && r[4..6] == [0, 0])
            .ok_or("the radio did not answer the login")?;
        if reply[48..52] == [0xFF, 0xFF, 0xFF, 0xFE] {
            return Err("wrong user name or password (the radio's Network User)".into());
        }
        auth.id.copy_from_slice(&reply[26..32]);
        let opened = open_streams(c, ip, &control, &mut auth);
        let (info, civ, civ_seq, audio) = match opened {
            Ok(o) => o,
            Err(e) => {
                // Log out, or the radio keeps the session and refuses the next login for a while.
                control.send_tracked(auth_packet(&control, &mut auth, 0x01));
                control.disconnect();
                return Err(e);
            }
        };

        let shared = Arc::new(Shared {
            stop: AtomicBool::new(false),
            error: Mutex::new(None),
            lost: AtomicU64::new(0),
            control: control.clone(),
            auth: Mutex::new(auth),
        });
        let (civ_tx, civ_rx) = mpsc::channel();
        let (audio_tx, audio_rx) = mpsc::sync_channel(AUDIO_QUEUE);
        let threads = vec![
            spawn("icom-control", {
                let s = shared.clone();
                move || control_loop(s)
            }),
            spawn("icom-civ", {
                let (s, st) = (shared.clone(), civ.clone());
                move || civ_loop(s, st, civ_tx)
            }),
            spawn("icom-audio", {
                let (s, a) = (shared.clone(), audio.clone());
                move || audio_loop(s, a, audio_tx)
            }),
        ];
        let send = NetSender { civ, civ_seq, audio, audio_seq: Arc::new(AtomicU16::new(0)) };
        Ok(Self { info, civ_rx, audio_rx, send, shared, threads })
    }

    /// Sends one CI-V frame to the radio.
    pub fn send_civ(&self, frame: &[u8]) {
        self.send.send_civ(frame);
    }

    /// A handle for sending CI-V and transmit audio from another thread.
    pub fn sender(&self) -> NetSender {
        self.send.clone()
    }

    /// Why the connection stopped working, if it did.
    pub fn error(&self) -> Option<String> {
        self.shared.error.lock().unwrap().clone()
    }

    /// Audio packets lost on the way or dropped because nobody read them.
    pub fn lost_packets(&self) -> u64 {
        self.shared.lost.load(Ordering::Relaxed)
    }
}

impl Drop for IcomNet {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
        let civ = &self.send.civ;
        civ.send_tracked(civ_open_close(civ, &self.send.civ_seq, false));
        civ.disconnect();
        let control = &self.shared.control;
        let p = auth_packet(control, &mut self.shared.auth.lock().unwrap(), 0x01);
        control.send_tracked(p);
        // The radio may ask for the log-out again; answer for a moment.
        let until = Instant::now() + Duration::from_millis(300);
        let mut buf = [0u8; 1500];
        while Instant::now() < until {
            if let Some(n) = control.recv(&mut buf, Duration::from_millis(50)) {
                control.handle_common(&buf[..n]);
            }
        }
        control.disconnect();
    }
}

type Opened = (RadioInfo, Arc<Stream>, Arc<AtomicU16>, Arc<Stream>);

/// After the login: authenticates, asks for the CI-V and audio streams and opens them.
fn open_streams(c: &Credentials, ip: IpAddr, control: &Arc<Stream>, auth: &mut Auth) -> Result<Opened, String> {
    control.send_tracked(auth_packet(control, auth, 0x02));
    control.send_tracked(auth_packet(control, auth, 0x05));

    // Wait for the radio's capabilities and the second auth, then ask for the streams.
    let started = Instant::now();
    let (mut info, mut guid, mut auth_ok, mut requested) = (None::<RadioInfo>, [0u8; 16], false, false);
    let mut last_ping = Instant::now();
    let mut buf = [0u8; 1500];
    loop {
        if started.elapsed() > LOGIN_TIMEOUT {
            return Err(if info.is_none() {
                "the radio did not send its details after the login".into()
            } else {
                "the radio did not open the CI-V and audio streams (is another program connected?)".into()
            });
        }
        if last_ping.elapsed() >= PING_INTERVAL {
            control.ping();
            control.send_idle();
            last_ping = Instant::now();
        }
        let Some(n) = control.recv(&mut buf, Duration::from_millis(50)) else { continue };
        let r = &buf[..n];
        if control.handle_common(r) {
            continue;
        }
        match (r.len(), r[0]) {
            (168, 0xA8) if r[1..6] == [0, 0, 0, 0, 0] => {
                guid.copy_from_slice(&r[66..82]);
                info = Some(RadioInfo { name: c_string(&r[82..114]), civ_address: r[148] });
            }
            (64, 0x40) if r[1..6] == [0, 0, 0, 0, 0] && r[21] == 0x05 => auth_ok = true,
            (80, 0x50) if r[1..6] == [0, 0, 0, 0, 0] && r[48..51] == [0xFF, 0xFF, 0xFF] => {
                return Err("the radio refused the connection; try switching it off and on".into());
            }
            (144, 0x90) if r[1..6] == [0, 0, 0, 0, 0] && requested => {
                if r[96] != 1 {
                    return Err("the radio refused the CI-V and audio streams (is another program connected?)".into());
                }
                control.remote_sid.store(u32::from_be_bytes([r[8], r[9], r[10], r[11]]), Ordering::Relaxed);
                control.local_sid.store(u32::from_be_bytes([r[12], r[13], r[14], r[15]]), Ordering::Relaxed);
                auth.id.copy_from_slice(&r[26..32]);
                break;
            }
            _ => {}
        }
        if let (Some(i), true, false) = (&info, auth_ok, requested) {
            control.send_tracked(stream_request(control, auth, &guid, &i.name, &c.username, c.port));
            requested = true;
        }
    }
    let info = info.expect("details before the streams");

    let civ = Arc::new(Stream::open(ip, c.port + 1).map_err(|e| format!("CI-V: {e}"))?);
    civ.handshake().map_err(|e| format!("CI-V {e}"))?;
    let civ_seq = Arc::new(AtomicU16::new(0));
    civ.send_tracked(civ_open_close(&civ, &civ_seq, true));
    let audio = Arc::new(Stream::open(ip, c.port + 2).map_err(|e| format!("audio: {e}"))?);
    audio.handshake().map_err(|e| format!("audio {e}"))?;

    Ok((info, civ, civ_seq, audio))
}

/// Sends CI-V frames and transmit audio to the radio; can be used from any thread.
#[derive(Clone)]
pub struct NetSender {
    civ: Arc<Stream>,
    civ_seq: Arc<AtomicU16>,
    audio: Arc<Stream>,
    audio_seq: Arc<AtomicU16>,
}

impl NetSender {
    /// Sends one CI-V frame to the radio.
    pub fn send_civ(&self, frame: &[u8]) {
        let seq = self.civ_seq.fetch_add(1, Ordering::Relaxed);
        let mut p = self.civ.header(21 + frame.len(), 0x00);
        p.extend([0xC1, frame.len() as u8, 0x00]);
        p.extend(seq.to_be_bytes());
        p.extend_from_slice(frame);
        self.civ.send_tracked(p);
    }

    /// Sends transmit audio (16-bit, 48 kHz, one channel) as RS-BA1 does: every 20 ms
    /// (960 samples) in two packets of 1364 and 556 bytes. The caller paces it.
    pub fn send_audio(&self, pcm: &[i16]) {
        for chunk in pcm.chunks(960) {
            let bytes: Vec<u8> = chunk.iter().flat_map(|v| v.to_le_bytes()).collect();
            let split = bytes.len().min(1364);
            for part in [&bytes[..split], &bytes[split..]] {
                if part.is_empty() {
                    continue;
                }
                let seq = self.audio_seq.fetch_add(1, Ordering::Relaxed);
                let mut p = self.audio.header(24 + part.len(), 0x00);
                p.extend([0x80, 0x00]);
                p.extend(seq.to_be_bytes());
                p.extend([0x00, 0x00]);
                p.extend((part.len() as u16).to_be_bytes());
                p.extend_from_slice(part);
                self.audio.send_tracked(p);
            }
        }
    }
}

fn spawn(name: &str, f: impl FnOnce() + Send + 'static) -> JoinHandle<()> {
    thread::Builder::new().name(name.into()).spawn(f).expect("network thread")
}

fn control_loop(shared: Arc<Shared>) {
    let control = shared.control.clone();
    let mut timers = Timers::new();
    let mut last_auth = Instant::now();
    let mut buf = [0u8; 1500];
    while !shared.stop.load(Ordering::Relaxed) {
        timers.tick(&control, true);
        if last_auth.elapsed() >= REAUTH_INTERVAL {
            let p = auth_packet(&control, &mut shared.auth.lock().unwrap(), 0x05);
            control.send_tracked(p);
            last_auth = Instant::now();
        }
        let Some(n) = control.recv(&mut buf, Duration::from_millis(50)) else { continue };
        let r = &buf[..n];
        if control.handle_common(r) {
            continue;
        }
        if r.len() == 80 && r[0] == 0x50 && r[1..6] == [0, 0, 0, 0, 0] {
            if r[48..51] == [0xFF, 0xFF, 0xFF] {
                shared.fail("the radio ended the session (authentication failed)".into());
            } else if r[48..51] == [0, 0, 0] && r[64] == 0x01 {
                shared.fail("the radio disconnected".into());
            }
        } else if r.len() == 16 && r[4] == 0x05 {
            shared.fail("the radio closed the connection".into());
        }
    }
}

fn civ_loop(shared: Arc<Shared>, civ: Arc<Stream>, out: mpsc::Sender<Vec<u8>>) {
    let mut timers = Timers::new();
    let mut buf = [0u8; 1500];
    while !shared.stop.load(Ordering::Relaxed) {
        timers.tick(&civ, true);
        let Some(n) = civ.recv(&mut buf, Duration::from_millis(50)) else { continue };
        let r = &buf[..n];
        if civ.handle_common(r) {
            continue;
        }
        if r.len() > 21 && r[16] == 0xC1 && usize::from(r[17]) == r.len() - 21 && out.send(r[21..].to_vec()).is_err() {
            return;
        }
    }
}

fn audio_loop(shared: Arc<Shared>, audio: Arc<Stream>, out: SyncSender<Vec<i16>>) {
    let mut timers = Timers::new();
    let mut buf = [0u8; 1500];
    let mut last_seq: Option<u16> = None;
    let mut last_len = 0usize;
    let mut last_audio = Instant::now();
    while !shared.stop.load(Ordering::Relaxed) {
        timers.tick(&audio, false);
        if last_audio.elapsed() > AUDIO_TIMEOUT {
            shared.fail("no audio from the radio".into());
            last_audio = Instant::now();
        }
        let Some(n) = audio.recv(&mut buf, Duration::from_millis(50)) else { continue };
        let r = &buf[..n];
        if audio.handle_common(r) {
            continue;
        }
        let Some(pcm) = audio_payload(r) else { continue };
        last_audio = Instant::now();
        let seq = u16::from_le_bytes([r[6], r[7]]);
        // Late (retransmitted or reordered) packets are dropped; gaps are filled with
        // silence so that the time base stays right.
        let gap = match last_seq {
            Some(prev) => {
                let ahead = seq.wrapping_sub(prev);
                if ahead == 0 || ahead >= 0x8000 {
                    continue;
                }
                usize::from(ahead - 1)
            }
            None => 0,
        };
        last_seq = Some(seq);
        let samples: Vec<i16> = pcm.chunks_exact(2).map(|b| i16::from_le_bytes([b[0], b[1]])).collect();
        if gap > 0 {
            shared.lost.fetch_add(gap as u64, Ordering::Relaxed);
            let fill = vec![0i16; last_len.max(samples.len()) * gap.min(50)];
            if !offer(&out, fill, &shared) {
                return;
            }
        }
        last_len = samples.len();
        if !offer(&out, samples, &shared) {
            return;
        }
    }
}

/// Queues audio; false when the reader is gone.
fn offer(out: &SyncSender<Vec<i16>>, samples: Vec<i16>, shared: &Shared) -> bool {
    match out.try_send(samples) {
        Ok(()) => true,
        Err(TrySendError::Full(_)) => {
            shared.lost.fetch_add(1, Ordering::Relaxed);
            true
        }
        Err(TrySendError::Disconnected(_)) => false,
    }
}

/// The PCM bytes of an audio packet: a 24-byte header, then the data.
fn audio_payload(r: &[u8]) -> Option<&[u8]> {
    let whole = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize == r.len();
    (whole && r.len() > 24 && r[4..6] == [0, 0] && r.len().is_multiple_of(2)).then(|| &r[24..])
}

/// Periodic pings and keep-alive packets of one stream.
struct Timers {
    ping: Instant,
    idle: Instant,
}

impl Timers {
    fn new() -> Self {
        Self { ping: Instant::now(), idle: Instant::now() }
    }

    fn tick(&mut self, s: &Stream, idle: bool) {
        if self.ping.elapsed() >= PING_INTERVAL {
            s.ping();
            self.ping = Instant::now();
        }
        if idle {
            let busy = s.last_tracked.lock().unwrap().elapsed() < IDLE_QUIET;
            if self.idle.elapsed() >= if busy { IDLE_BUSY } else { IDLE_QUIET } {
                s.send_idle();
                self.idle = Instant::now();
            }
        }
    }
}

/// One UDP stream to the radio.
struct Stream {
    socket: UdpSocket,
    local_sid: AtomicU32,
    remote_sid: AtomicU32,
    sent: Mutex<Sent>,
    last_tracked: Mutex<Instant>,
    ping_seq: AtomicU16,
    ping_inner: AtomicU16,
}

/// Sequence numbers and recent packets of the tracked (retransmittable) traffic.
struct Sent {
    seq: u16,
    recent: VecDeque<(u16, Vec<u8>)>,
}

impl Stream {
    fn open(ip: IpAddr, port: u16) -> io::Result<Self> {
        let any: SocketAddr = match ip {
            IpAddr::V4(_) => (Ipv4Addr::UNSPECIFIED, 0).into(),
            IpAddr::V6(_) => (Ipv6Addr::UNSPECIFIED, 0).into(),
        };
        let socket = UdpSocket::bind(any)?;
        socket.connect((ip, port))?;
        let local = socket.local_addr()?;
        // The session id the radio expects: low half of the local address, then the port.
        let ip_bits = match local.ip() {
            IpAddr::V4(v4) => u32::from(v4),
            IpAddr::V6(v6) => {
                let o = v6.octets();
                u32::from_be_bytes([o[12], o[13], o[14], o[15]])
            }
        };
        let sid = (ip_bits & 0xFFFF) << 16 | u32::from(local.port());
        Ok(Self {
            socket,
            local_sid: AtomicU32::new(sid),
            remote_sid: AtomicU32::new(0),
            sent: Mutex::new(Sent { seq: 1, recent: VecDeque::new() }),
            last_tracked: Mutex::new(Instant::now() - IDLE_QUIET),
            ping_seq: AtomicU16::new(1),
            ping_inner: AtomicU16::new(0x8304),
        })
    }

    /// Length, type, sequence number 0 and both session ids; `len` is the whole packet's.
    fn header(&self, len: usize, kind: u16) -> Vec<u8> {
        let mut p = Vec::with_capacity(len);
        p.extend((len as u32).to_le_bytes());
        p.extend(kind.to_le_bytes());
        p.extend([0, 0]);
        p.extend(self.local_sid.load(Ordering::Relaxed).to_be_bytes());
        p.extend(self.remote_sid.load(Ordering::Relaxed).to_be_bytes());
        p
    }

    /// A 16-byte control packet with the given type and sequence number.
    fn control(&self, kind: u16, seq: u16) -> Vec<u8> {
        let mut p = self.header(16, kind);
        p[6..8].copy_from_slice(&seq.to_le_bytes());
        p
    }

    fn send(&self, p: &[u8]) {
        // A lost packet is asked for again by the radio; errors show up as missing answers.
        let _ = self.socket.send(p);
    }

    fn send_twice(&self, p: &[u8]) {
        self.send(p);
        self.send(p);
    }

    /// Sends with the next sequence number and keeps the packet for retransmission.
    fn send_tracked(&self, mut p: Vec<u8>) {
        let mut sent = self.sent.lock().unwrap();
        let seq = sent.seq;
        p[6..8].copy_from_slice(&seq.to_le_bytes());
        self.send(&p);
        sent.seq = seq.wrapping_add(1);
        let idle = p.len() == 16 && p[4] == 0;
        sent.recent.push_back((seq, p));
        if sent.recent.len() > KEEP_SENT {
            sent.recent.pop_front();
        }
        if !idle {
            *self.last_tracked.lock().unwrap() = Instant::now();
        }
    }

    fn send_idle(&self) {
        self.send_tracked(self.control(0x00, 0));
    }

    fn resend(&self, seq: u16) {
        let p = self.sent.lock().unwrap().recent.iter().find(|(s, _)| *s == seq).map(|(_, p)| p.clone());
        match p {
            Some(p) => self.send_twice(&p),
            // Not kept any more: an idle packet with that number tells the radio to move on.
            None => self.send_twice(&self.control(0x00, seq)),
        }
    }

    fn ping(&self) {
        let seq = self.ping_seq.fetch_add(1, Ordering::Relaxed);
        let inner = self.ping_inner.fetch_add(1, Ordering::Relaxed).to_le_bytes();
        let mut p = self.control(0x07, seq);
        p[0] = 0x15;
        p.extend([0x00, random_u64() as u8, inner[0], inner[1], 0x06]);
        self.send(&p);
    }

    fn disconnect(&self) {
        self.send_twice(&self.control(0x05, 0));
    }

    fn recv(&self, buf: &mut [u8], timeout: Duration) -> Option<usize> {
        let _ = self.socket.set_read_timeout(Some(timeout.max(Duration::from_millis(1))));
        match self.socket.recv(buf) {
            Ok(n) if n >= 16 => Some(n),
            _ => None,
        }
    }

    /// Waits for a packet that `accept` takes, handling pings meanwhile.
    fn expect(&self, timeout: Duration, accept: impl Fn(&[u8]) -> bool) -> Option<Vec<u8>> {
        let until = Instant::now() + timeout;
        let mut buf = [0u8; 1500];
        while let Some(left) = until.checked_duration_since(Instant::now()) {
            let Some(n) = self.recv(&mut buf, left) else { continue };
            if accept(&buf[..n]) {
                return Some(buf[..n].to_vec());
            }
            self.handle_common(&buf[..n]);
        }
        None
    }

    /// "Are you there" → "I am here" (gives the radio's session id), then "are you ready".
    fn handshake(&self) -> Result<(), String> {
        let mut here = None;
        for _ in 0..3 {
            self.send_twice(&self.control(0x03, 0));
            here = self.expect(ANSWER_TIMEOUT, |r| r.len() == 16 && r[..6] == [0x10, 0, 0, 0, 0x04, 0]);
            if here.is_some() {
                break;
            }
        }
        let here = here.ok_or("does not answer (is the radio's network remote control on?)")?;
        self.remote_sid.store(u32::from_be_bytes([here[8], here[9], here[10], here[11]]), Ordering::Relaxed);
        self.send_twice(&self.control(0x06, 1));
        self.expect(ANSWER_TIMEOUT, |r| r.len() == 16 && r[..6] == [0x10, 0, 0, 0, 0x06, 0]).ok_or("did not finish the handshake")?;
        Ok(())
    }

    /// Pings and retransmit requests, common to all streams; true when `r` was one.
    fn handle_common(&self, r: &[u8]) -> bool {
        if r.len() == 21 && r[1..6] == [0, 0, 0, 0x07, 0] {
            if r[16] == 0x00 {
                // The radio's ping: answer with its sequence number and id.
                let mut p = self.control(0x07, u16::from_le_bytes([r[6], r[7]]));
                p[0] = 0x15;
                p.push(0x01);
                p.extend_from_slice(&r[17..21]);
                self.send(&p);
            }
            return true;
        }
        if r.len() == 16 && r[..6] == [0x10, 0, 0, 0, 0x01, 0] {
            self.resend(u16::from_le_bytes([r[6], r[7]]));
            return true;
        }
        if r.len() > 16 && r[4..6] == [0x01, 0] {
            for range in r[16..].chunks_exact(4) {
                let (start, end) = (u16::from_le_bytes([range[0], range[1]]), u16::from_le_bytes([range[2], range[3]]));
                let count = end.wrapping_sub(start).min(32);
                for k in 0..=count {
                    self.resend(start.wrapping_add(k));
                }
            }
            return true;
        }
        // Idle packets need nothing.
        r.len() == 16 && r[4..6] == [0, 0]
    }
}

fn resolve(host: &str, port: u16) -> Result<IpAddr, String> {
    (host, port)
        .to_socket_addrs()
        .map_err(|e| format!("{host}: {e}"))?
        .map(|a| a.ip())
        .min_by_key(|ip| ip.is_ipv6())
        .ok_or_else(|| format!("{host}: address not found"))
}

fn login_packet(s: &Stream, auth: &mut Auth, start_id: [u8; 2], user: &str, password: &str) -> Vec<u8> {
    let mut p = s.header(0x80, 0x00);
    p.extend([0x00, 0x00, 0x00, 0x70, 0x01, 0x00, 0x00]);
    p.extend(auth.inner_seq.to_le_bytes());
    p.extend([0x00, start_id[0], start_id[1]]);
    p.resize(64, 0);
    p.extend(passcode(user));
    p.extend(passcode(password));
    p.extend(b"icom-pc\0");
    p.resize(0x80, 0);
    auth.inner_seq = auth.inner_seq.wrapping_add(1);
    p
}

/// 0x02 after the login, 0x05 to stay logged in, 0x01 to log out.
fn auth_packet(s: &Stream, auth: &mut Auth, magic: u8) -> Vec<u8> {
    let mut p = s.header(0x40, 0x00);
    p.extend([0x00, 0x00, 0x00, 0x30, 0x01, magic, 0x00]);
    p.extend(auth.inner_seq.to_le_bytes());
    p.push(0x00);
    p.extend(auth.id);
    p.resize(0x40, 0);
    auth.inner_seq = auth.inner_seq.wrapping_add(1);
    p
}

/// Asks for the CI-V and audio streams: receive audio as 16-bit PCM at 48 kHz.
fn stream_request(s: &Stream, auth: &mut Auth, guid: &[u8; 16], radio: &str, user: &str, control_port: u16) -> Vec<u8> {
    let mut p = s.header(0x90, 0x00);
    p.extend([0x00, 0x00, 0x00, 0x80, 0x01, 0x03, 0x00]);
    p.extend(auth.inner_seq.to_le_bytes());
    p.push(0x00);
    p.extend(auth.id);
    p.extend(guid);
    p.resize(64, 0);
    let name = radio.as_bytes();
    p.extend(&name[..name.len().min(31)]);
    p.resize(96, 0);
    p.extend(passcode(user));
    // Receive on, transmit audio on (nothing is sent), both codecs 16-bit mono PCM.
    p.extend([0x01, 0x01, CODEC_LPCM16_MONO, CODEC_LPCM16_MONO]);
    p.extend(SAMPLE_RATE.to_be_bytes());
    p.extend(SAMPLE_RATE.to_be_bytes());
    p.extend(u32::from(control_port + 1).to_be_bytes());
    p.extend(u32::from(control_port + 2).to_be_bytes());
    // Transmit buffer (ms), then a flag.
    p.extend(300u32.to_be_bytes());
    p.push(0x01);
    p.resize(0x90, 0);
    auth.inner_seq = auth.inner_seq.wrapping_add(1);
    p
}

/// Opens (0x05) or closes (0x00) the CI-V stream.
fn civ_open_close(s: &Stream, seq: &AtomicU16, open: bool) -> Vec<u8> {
    let mut p = s.header(0x16, 0x00);
    p.extend([0xC0, 0x01, 0x00]);
    p.extend(seq.fetch_add(1, Ordering::Relaxed).to_be_bytes());
    p.push(if open { 0x05 } else { 0x00 });
    p
}

/// Substitution table for user names and passwords, for characters 32–126.
const PASSCODE: [u8; 95] = [
    0x47, 0x5d, 0x4c, 0x42, 0x66, 0x20, 0x23, 0x46, 0x4e, 0x57, 0x45, 0x3d, 0x67, 0x76, 0x60, 0x41, //
    0x62, 0x39, 0x59, 0x2d, 0x68, 0x7e, 0x7c, 0x65, 0x7d, 0x49, 0x29, 0x72, 0x73, 0x78, 0x21, 0x6e, //
    0x5a, 0x5e, 0x4a, 0x3e, 0x71, 0x2c, 0x2a, 0x54, 0x3c, 0x3a, 0x63, 0x4f, 0x43, 0x75, 0x27, 0x79, //
    0x5b, 0x35, 0x70, 0x48, 0x6b, 0x56, 0x6f, 0x34, 0x32, 0x6c, 0x30, 0x61, 0x6d, 0x7b, 0x2f, 0x4b, //
    0x64, 0x38, 0x2b, 0x2e, 0x50, 0x40, 0x3f, 0x55, 0x33, 0x37, 0x25, 0x77, 0x24, 0x26, 0x74, 0x6a, //
    0x28, 0x53, 0x4d, 0x69, 0x22, 0x5c, 0x44, 0x31, 0x36, 0x58, 0x3b, 0x7a, 0x51, 0x5f, 0x52,
];

/// How the radio expects user names and passwords (16 bytes, at most 16 characters).
fn passcode(s: &str) -> [u8; 16] {
    let mut out = [0u8; 16];
    for (i, b) in s.bytes().take(16).enumerate() {
        let mut p = usize::from(b) + i;
        if p > 126 {
            p = 32 + p % 127;
        }
        out[i] = p.checked_sub(32).and_then(|k| PASSCODE.get(k)).copied().unwrap_or(0);
    }
    out
}

fn c_string(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).trim().to_string()
}

fn random_u64() -> u64 {
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write_u128(std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos()));
    h.finish()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[test]
    fn credentials_from_text() {
        let c = Credentials::parse("op:se:cret@192.168.1.50").unwrap();
        assert_eq!((c.username.as_str(), c.password.as_str(), c.host.as_str(), c.port), ("op", "se:cret", "192.168.1.50", 50001));
        let c = Credentials::parse("op:pw@ic705.local:50101").unwrap();
        assert_eq!((c.host.as_str(), c.port), ("ic705.local", 50101));
        let c = Credentials::parse("op:pw@[fe80::1]:50001").unwrap();
        assert_eq!(c.host, "fe80::1");
        assert!(Credentials::parse("192.168.1.50").is_none());
        assert!(Credentials::parse("op@192.168.1.50").is_none());
    }

    #[test]
    fn passcode_matches_known_values() {
        // From kappanhang's table: 'a' (97) at position 0 → 0x38, 'b' (98) at 1 → 99 → 0x2e.
        assert_eq!(&passcode("ab")[..3], &[0x38, 0x2e, 0x00]);
        // Wraps above 126: '~' (126) at position 1 → 127 → 32 + 0 → 0x47.
        assert_eq!(passcode("x~")[1], 0x47);
    }

    #[test]
    fn packets_have_the_documented_sizes() {
        let s = Stream::open(IpAddr::V4(Ipv4Addr::LOCALHOST), 9).unwrap();
        let mut auth = Auth::default();
        assert_eq!(login_packet(&s, &mut auth, [1, 2], "u", "p").len(), 0x80);
        assert_eq!(auth_packet(&s, &mut auth, 0x05).len(), 0x40);
        let req = stream_request(&s, &mut auth, &[7; 16], "IC-705", "u", 50001);
        assert_eq!(req.len(), 0x90);
        assert_eq!(&req[112..116], &[1, 1, 4, 4]);
        assert_eq!(&req[116..120], &48_000u32.to_be_bytes());
        assert_eq!(&req[124..128], &50_002u32.to_be_bytes());
        assert_eq!(&req[128..132], &50_003u32.to_be_bytes());
        assert_eq!(auth.inner_seq, 3);
        assert_eq!(civ_open_close(&s, &AtomicU16::new(0), true).len(), 0x16);
    }

    /// A pretend radio on localhost: enough of the protocol to log in, answer CI-V and send
    /// a 13.5 kHz IF tone.
    pub(crate) struct FakeRadio {
        pub(crate) port: u16,
        pub(crate) password: &'static str,
        stop: Arc<AtomicBool>,
        pub(crate) civ_seen: Arc<Mutex<Vec<Vec<u8>>>>,
        pub(crate) audio_seen: Arc<Mutex<Vec<Vec<u8>>>>,
    }

    impl FakeRadio {
        pub(crate) fn start(password: &'static str) -> Self {
            let stop = Arc::new(AtomicBool::new(false));
            let civ_seen = Arc::new(Mutex::new(Vec::new()));
            let audio_seen = Arc::new(Mutex::new(Vec::new()));
            for base in (47_000..48_000u16).step_by(7) {
                let socks: Vec<_> = (0..3).filter_map(|k| UdpSocket::bind(("127.0.0.1", base + k)).ok()).collect();
                if socks.len() < 3 {
                    continue;
                }
                let mut socks = socks.into_iter();
                let (ctl, civ, aud) = (socks.next().unwrap(), socks.next().unwrap(), socks.next().unwrap());
                let st = stop.clone();
                thread::spawn(move || fake_control(ctl, password, st));
                let (st, seen) = (stop.clone(), civ_seen.clone());
                thread::spawn(move || fake_civ(civ, st, seen));
                let (st, seen) = (stop.clone(), audio_seen.clone());
                thread::spawn(move || fake_audio(aud, st, seen));
                return Self { port: base, password, stop, civ_seen, audio_seen };
            }
            panic!("no free ports");
        }
    }

    impl Drop for FakeRadio {
        fn drop(&mut self) {
            self.stop.store(true, Ordering::Relaxed);
        }
    }

    const RADIO_SID: u32 = 0x1234_5678;

    fn reply(sock: &UdpSocket, to: SocketAddr, mut p: Vec<u8>, kind: u16, their_sid: &[u8]) {
        let len = p.len() as u32;
        p[0..4].copy_from_slice(&len.to_le_bytes());
        p[4..6].copy_from_slice(&kind.to_le_bytes());
        p[8..12].copy_from_slice(&RADIO_SID.to_be_bytes());
        p[12..16].copy_from_slice(their_sid);
        sock.send_to(&p, to).unwrap();
    }

    /// Receives one packet, answers the handshake and pings; returns the others.
    fn fake_common(sock: &UdpSocket, buf: &mut [u8], client: &mut Option<(SocketAddr, Vec<u8>)>) -> Option<(SocketAddr, Vec<u8>)> {
        let (n, from) = sock.recv_from(buf).ok()?;
        let r = buf[..n].to_vec();
        if n < 16 {
            return None;
        }
        let sid = r[8..12].to_vec();
        match (n, r[4]) {
            (16, 0x03) => {
                *client = Some((from, sid.clone()));
                reply(sock, from, vec![0; 16], 0x04, &sid);
            }
            (16, 0x06) => reply(sock, from, vec![0; 16], 0x06, &sid),
            (21, 0x07) | (16, _) => {}
            _ => return Some((from, r)),
        }
        None
    }

    fn fake_control(sock: UdpSocket, password: &str, stop: Arc<AtomicBool>) {
        sock.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let mut buf = [0u8; 1500];
        while !stop.load(Ordering::Relaxed) {
            let Some((from, r)) = fake_common(&sock, &mut buf, &mut None) else { continue };
            let sid = r[8..12].to_vec();
            match r[0] {
                0x80 => {
                    let mut p = vec![0u8; 96];
                    p[0] = 0x60;
                    p[26..32].copy_from_slice(&[9, 8, 7, 6, 5, 4]);
                    if r[64..80] != passcode("user") || r[80..96] != passcode(password) {
                        p[48..52].copy_from_slice(&[0xFF, 0xFF, 0xFF, 0xFE]);
                    }
                    reply(&sock, from, p, 0, &sid);
                }
                0x40 => {
                    assert_eq!(&r[26..32], &[9, 8, 7, 6, 5, 4], "auth id echoed");
                    let mut p = r.clone();
                    p[20] = 0x02;
                    reply(&sock, from, p, 0, &sid);
                    if r[21] == 0x02 {
                        let mut cap = vec![0u8; 168];
                        cap[0] = 0xA8;
                        cap[66..82].copy_from_slice(&[0x5A; 16]);
                        cap[82..88].copy_from_slice(b"IC-705");
                        cap[148] = 0xA4;
                        reply(&sock, from, cap, 0, &sid);
                    }
                }
                0x90 => {
                    assert_eq!(&r[32..48], &[0x5A; 16], "capability id echoed");
                    assert_eq!(c_string(&r[64..96]), "IC-705");
                    let mut p = vec![0u8; 144];
                    p[0] = 0x90;
                    p[26..32].copy_from_slice(&[9, 8, 7, 6, 5, 4]);
                    p[96] = 1;
                    reply(&sock, from, p, 0, &sid);
                }
                _ => {}
            }
        }
    }

    fn fake_civ(sock: UdpSocket, stop: Arc<AtomicBool>, seen: Arc<Mutex<Vec<Vec<u8>>>>) {
        sock.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let mut buf = [0u8; 1500];
        while !stop.load(Ordering::Relaxed) {
            let Some((from, r)) = fake_common(&sock, &mut buf, &mut None) else { continue };
            if r.len() > 21 && r[16] == 0xC1 {
                let frame = r[21..].to_vec();
                seen.lock().unwrap().push(frame.clone());
                if frame[4..6] == [0x19, 0x00] {
                    let answer = [0xFE, 0xFE, 0xE0, 0xA4, 0x19, 0x00, 0xA4, 0xFD];
                    let mut p = vec![0u8; 21];
                    p[16] = 0xC1;
                    p[17] = answer.len() as u8;
                    p.extend(answer);
                    reply(&sock, from, p, 0, &r[8..12]);
                }
            }
        }
    }

    fn fake_audio(sock: UdpSocket, stop: Arc<AtomicBool>, seen: Arc<Mutex<Vec<Vec<u8>>>>) {
        sock.set_read_timeout(Some(Duration::from_millis(5))).unwrap();
        let mut buf = [0u8; 1500];
        let (mut client, mut seq, mut t) = (None, 0u16, 0u64);
        let mut next = Instant::now();
        while !stop.load(Ordering::Relaxed) {
            if let Some((_, r)) = fake_common(&sock, &mut buf, &mut client) {
                seen.lock().unwrap().push(r);
            }
            let Some((to, sid)) = client.clone() else {
                next = Instant::now();
                continue;
            };
            // 10 ms of 48 kHz audio per packet; packet 5 is "lost".
            while Instant::now() >= next {
                let mut p = vec![0u8; 24];
                p[6..8].copy_from_slice(&seq.to_le_bytes());
                p[16] = 0x80;
                p[22..24].copy_from_slice(&960u16.to_be_bytes());
                for _ in 0..480 {
                    let v = (0.25 * (std::f64::consts::TAU * 13_500.0 * t as f64 / 48_000.0).cos() * 32767.0) as i16;
                    p.extend(v.to_le_bytes());
                    t += 1;
                }
                if seq != 5 {
                    reply(&sock, to, p, 0, &sid);
                }
                seq = seq.wrapping_add(1);
                next += Duration::from_millis(10);
            }
        }
    }

    #[test]
    fn logs_in_exchanges_civ_and_receives_audio() {
        let radio = FakeRadio::start("secret");
        let creds = Credentials { host: "127.0.0.1".into(), port: radio.port, username: "user".into(), password: radio.password.into() };
        let net = IcomNet::connect(&creds).expect("login");
        assert_eq!(net.info, RadioInfo { name: "IC-705".into(), civ_address: 0xA4 });

        net.send_civ(&[0xFE, 0xFE, 0x00, 0xE0, 0x19, 0x00, 0xFD]);
        let answer = net.civ_rx.recv_timeout(Duration::from_secs(2)).expect("CI-V answer");
        assert_eq!(answer, [0xFE, 0xFE, 0xE0, 0xA4, 0x19, 0x00, 0xA4, 0xFD]);
        assert_eq!(radio.civ_seen.lock().unwrap()[0], [0xFE, 0xFE, 0x00, 0xE0, 0x19, 0x00, 0xFD]);

        let mut samples = Vec::new();
        let started = Instant::now();
        while samples.len() < 48_000 / 4 && started.elapsed() < Duration::from_secs(3) {
            if let Ok(p) = net.audio_rx.recv_timeout(Duration::from_millis(200)) {
                samples.extend(p);
            }
        }
        assert!(samples.len() >= 12_000, "{} samples", samples.len());
        assert_eq!(net.lost_packets(), 1, "packet 5 was missing");
        assert!(samples.iter().any(|&v| v > 8000), "tone arrived");
        assert_eq!(net.error(), None);

        // 20 ms of transmit audio goes out as two packets, like RS-BA1's.
        net.sender().send_audio(&[1000i16; 960]);
        thread::sleep(Duration::from_millis(200));
        let sent = radio.audio_seen.lock().unwrap().clone();
        assert_eq!(sent.iter().map(|p| p.len()).collect::<Vec<_>>(), [24 + 1364, 24 + 556]);
        assert_eq!(&sent[0][16..18], &[0x80, 0x00]);
        assert_eq!(&sent[0][22..24], &1364u16.to_be_bytes());
        assert_eq!(&sent[1][24..26], &1000i16.to_le_bytes());
        drop(net);
    }

    #[test]
    fn wrong_password_is_reported() {
        let radio = FakeRadio::start("secret");
        let creds = Credentials { host: "127.0.0.1".into(), port: radio.port, username: "user".into(), password: "nope".into() };
        let e = IcomNet::connect(&creds).err().expect("refused");
        assert!(e.contains("password"), "{e}");
    }
}
