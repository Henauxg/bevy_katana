use bevy::color::{palettes::css::*, Color};

pub mod ball;
pub mod fps;
pub mod plugin;

pub const DEFAULT_EXAMPLES_FONT_SIZE: f32 = 17.;

pub const COLORS: &'static [Color] = &[
    Color::Srgba(GREEN),
    Color::Srgba(BLUE),
    Color::BLACK,
    Color::Srgba(RED),
    Color::Srgba(YELLOW),
    Color::Srgba(MAROON),
    Color::Srgba(PURPLE),
    Color::Srgba(SALMON),
    Color::Srgba(ORANGE),
    Color::Srgba(BLUE_VIOLET),
    Color::Srgba(NAVY),
    Color::Srgba(OLIVE),
    Color::Srgba(PINK),
    Color::Srgba(ALICE_BLUE),
    Color::Srgba(CRIMSON),
    Color::Srgba(TURQUOISE),
    Color::Srgba(YELLOW_GREEN),
    Color::Srgba(TEAL),
];
