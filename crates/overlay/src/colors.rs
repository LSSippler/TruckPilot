//! AR-overlay colour palette.

#[cfg(windows)]
use procmod_overlay::Color;

#[cfg(windows)]
pub const COLOR_ROAD: Color = Color::rgba(150, 150, 150, 180);
#[cfg(windows)]
pub const COLOR_PREFAB: Color = Color::rgba(60, 130, 220, 200);
#[cfg(windows)]
pub const COLOR_NEAREST: Color = Color::rgba(220, 50, 50, 255);
#[cfg(windows)]
pub const COLOR_BIAS_ACCEPTED: Color = Color::rgba(80, 230, 80, 255);
#[cfg(windows)]
pub const COLOR_DS14_TARGET: Color = Color::rgba(0, 230, 220, 230); // cyan/türkis — DS14 road_offset gap-fallback
