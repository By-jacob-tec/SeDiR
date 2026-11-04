//! The SeDiR cedar: the tree of `assets/sedir-logo.svg`, drawn in software with distance
//! fields (anti-aliased), so the window icon and the header need no image file.

/// Tree lines in the SVG's coordinates: (x1, x2, y, colour). Every tier is a horizontal
/// stroke with round caps, like a spectrum trace seen side on.
const TIERS: [(f32, f32, f32, [u8; 3]); 10] = [
    (45.0, 51.0, 10.0, [0xf0, 0xd0, 0x3a]),
    (41.0, 56.0, 18.0, [0xf0, 0xa6, 0x3a]),
    (36.0, 59.0, 26.0, [0xe8, 0x89, 0x3a]),
    (31.0, 64.0, 34.0, [0xb9, 0xb2, 0x3f]),
    (26.0, 69.0, 42.0, [0x8f, 0xae, 0x45]),
    (34.0, 62.0, 50.0, [0x75, 0xc3, 0x81]),
    (20.0, 74.0, 58.0, [0x5f, 0xae, 0x6c]),
    (12.0, 82.0, 66.0, [0x4b, 0x9a, 0x5a]),
    (24.0, 72.0, 74.0, [0x3f, 0x8a, 0x4f]),
    (6.0, 90.0, 82.0, [0x2f, 0x7a, 0x6a]),
];
const TIER_WIDTH: f32 = 4.5;
/// Trunk: x, from y, to y, width.
const TRUNK: (f32, f32, f32, f32) = (48.0, 88.0, 112.0, 6.0);
const TRUNK_COLOR: [u8; 3] = [0x8a, 0x60, 0x3c];
/// Square the tree is centred in (SVG units).
const VIEW: f32 = 124.0;
const OFFSET: (f32, f32) = (14.0, 3.0);
const ICON_BACKGROUND: [u8; 3] = [16, 22, 29];

/// Distance from (px, py) to the segment from `a` to `b`.
fn segment(px: f32, py: f32, a: (f32, f32), b: (f32, f32)) -> f32 {
    let (bax, bay) = (b.0 - a.0, b.1 - a.1);
    let len = bax * bax + bay * bay;
    let t = if len > 0.0 { (((px - a.0) * bax + (py - a.1) * bay) / len).clamp(0.0, 1.0) } else { 0.0 };
    (px - a.0 - t * bax).hypot(py - a.1 - t * bay)
}

/// `size × size` RGBA (not premultiplied). With `background` the tree sits on a dark
/// rounded square (window icon); without it the rest is transparent (header).
pub fn rgba(size: usize, background: bool) -> Vec<u8> {
    let scale = VIEW / size as f32;
    let mut out = Vec::with_capacity(size * size * 4);
    for py in 0..size {
        for px in 0..size {
            let (x, y) = ((px as f32 + 0.5) * scale - OFFSET.0, (py as f32 + 0.5) * scale - OFFSET.1);
            let cover = |d: f32| (0.5 - d / scale).clamp(0.0, 1.0);
            let mut color = [0.0f32; 3];
            let mut alpha = 0.0f32;
            if background {
                color = ICON_BACKGROUND.map(f32::from);
                let (cx, cy) = (px as f32 + 0.5 - size as f32 / 2.0, py as f32 + 0.5 - size as f32 / 2.0);
                let (half, r) = (size as f32 / 2.0, size as f32 * 0.22);
                let (qx, qy) = (cx.abs() - (half - r), cy.abs() - (half - r));
                let d = qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - r;
                alpha = (0.5 - d).clamp(0.0, 1.0);
            }
            let mut paint = |c: [u8; 3], a: f32| {
                if a <= 0.0 {
                    return;
                }
                for i in 0..3 {
                    color[i] += (c[i] as f32 - color[i]) * a;
                }
                alpha += (1.0 - alpha) * a;
            };
            let (tx, ty0, ty1, tw) = TRUNK;
            paint(TRUNK_COLOR, cover(segment(x, y, (tx, ty0), (tx, ty1)) - tw / 2.0));
            for (x1, x2, ty, c) in TIERS {
                paint(c, cover(segment(x, y, (x1, ty), (x2, ty)) - TIER_WIDTH / 2.0));
            }
            out.extend(color.iter().map(|&c| c.round() as u8));
            out.push((alpha * 255.0).round() as u8);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tree_is_drawn_and_corners_are_transparent() {
        let size = 64;
        let px = rgba(size, false);
        assert_eq!(px.len(), size * size * 4);
        assert_eq!(px[3], 0, "corner must be transparent");
        assert!(px.chunks_exact(4).filter(|p| p[3] == 255).count() > 200, "tree too small");
        let icon = rgba(size, true);
        let center = (size / 2 * size + size / 2) * 4;
        assert_eq!(icon[center + 3], 255);
        assert_eq!(icon[3], 0, "icon corner must be rounded off");
    }

    /// Preview: `SEDIR_LOGO_OUT=path cargo test --bin sedir logo -- --ignored` writes
    /// 256×256 RGBA (convert with ffmpeg -f rawvideo -pix_fmt rgba -s 256x256).
    #[test]
    #[ignore]
    fn export_preview() {
        if let Some(path) = std::env::var_os("SEDIR_LOGO_OUT") {
            std::fs::write(path, rgba(256, true)).unwrap();
        }
    }
}
