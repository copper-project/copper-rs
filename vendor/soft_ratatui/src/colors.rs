use ratatui_core::style::Color as RatColor;

///Converts a Ratatui Color into a rgb [u8;3]
pub fn rat_to_rgb(rat_col: &RatColor, is_a_fg: bool) -> [u8; 3] {
    match rat_col {
        RatColor::Reset => {
            if is_a_fg {
                [204, 204, 255] // foreground reset color
            } else {
                [5, 1, 121] // background reset color
            }
        }
        RatColor::Black => [0, 0, 0],
        RatColor::Red => [139, 0, 0],
        RatColor::Green => [0, 100, 0],
        RatColor::Yellow => [255, 215, 0],
        RatColor::Blue => [0, 0, 139],
        RatColor::Magenta => [255, 0, 255],
        RatColor::Cyan => [0, 0, 255],
        RatColor::Gray => [128, 128, 128],
        RatColor::DarkGray => [64, 64, 64],
        RatColor::LightRed => [255, 0, 0],
        RatColor::LightGreen => [0, 255, 0],
        RatColor::LightBlue => [173, 216, 230],
        RatColor::LightYellow => [255, 255, 224],
        RatColor::LightMagenta => [139, 0, 139],
        RatColor::LightCyan => [224, 255, 255],
        RatColor::White => [255, 255, 255],
        RatColor::Indexed(i) => {
            let i = *i as u8;
            [i.wrapping_mul(i), i.wrapping_add(i), i]
        }
        RatColor::Rgb(r, g, b) => [*r, *g, *b],
    }
}

pub fn dim_rgb(color: [u8; 3]) -> [u8; 3] {
    let factor = 77; // 77 ≈ 255 * 0.3
    [
        ((color[0] as u32 * factor + 127) / 255) as u8,
        ((color[1] as u32 * factor + 127) / 255) as u8,
        ((color[2] as u32 * factor + 127) / 255) as u8,
    ]
}

/// Blend two RGBA colors using alpha compositing.
///
/// (fg over bg) -> resulting RGB
///
/// * `fg` - [R, G, B, A] foreground color
/// * `bg` - [R, G, B, A] background color
///
/// Returns: blended color as [u8; 4]
pub fn blend_rgba(fg: [u8; 4], bg: [u8; 4]) -> [u8; 3] {
    let fg_a = fg[3] as f32 / 255.0;
    let bg_a = bg[3] as f32 / 255.0;
    let out_a = fg_a + bg_a * (1.0 - fg_a);

    let blend_channel =
        |f: u8, b: u8| ((f as f32 * fg_a + b as f32 * bg_a * (1.0 - fg_a)) / out_a).round() as u8;

    if out_a == 0.0 {
        [0, 0, 0]
    } else {
        [
            blend_channel(fg[0], bg[0]),
            blend_channel(fg[1], bg[1]),
            blend_channel(fg[2], bg[2]),
        ]
    }
}

pub fn blend_ignore_bg_alpha(fg: [u8; 4], bg: [u8; 3]) -> [u8; 3] {
    let alpha = fg[3] as f32 / 255.0;
    let inv_alpha = 1.0 - alpha;

    [
        (fg[0] as f32 * alpha + bg[0] as f32 * inv_alpha).round() as u8,
        (fg[1] as f32 * alpha + bg[1] as f32 * inv_alpha).round() as u8,
        (fg[2] as f32 * alpha + bg[2] as f32 * inv_alpha).round() as u8,
    ]
}
pub fn blend_gamma_corrected(fg: [u8; 4], bg: [u8; 4]) -> [u8; 3] {
    let fg_a = fg[3] as f32 / 255.0;
    let bg_a = bg[3] as f32 / 255.0;
    let out_a = fg_a + bg_a * (1.0 - fg_a);

    if out_a == 0.0 {
        return [0, 0, 0];
    }

    // Convert to linear space (approx gamma 2.2)
    let to_linear = |c: u8| (c as f32 / 255.0).powf(2.2);
    let from_linear = |c: f32| (c.powf(1.0 / 2.2).clamp(0.0, 1.0) * 255.0).round() as u8;

    let blend_channel = |f: u8, b: u8| {
        let f_lin = to_linear(f);
        let b_lin = to_linear(b);
        let result_lin = (f_lin * fg_a + b_lin * bg_a * (1.0 - fg_a)) / out_a;
        from_linear(result_lin)
    };

    [
        blend_channel(fg[0], bg[0]),
        blend_channel(fg[1], bg[1]),
        blend_channel(fg[2], bg[2]),
    ]
}
