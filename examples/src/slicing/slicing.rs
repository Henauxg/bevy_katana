use bevy::prelude::*;

use bevy_ghx_destruction::slicing::slicing::slice;
use bevy_rapier3d::{
    plugin::{NoUserData, RapierPhysicsPlugin},
    render::RapierDebugRenderPlugin,
};
use examples::plugin::ExamplesPlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, ExamplesPlugin))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
) {
    let mesh = Cuboid::new(1.0, 1.0, 1.0).mesh();

    commands.spawn(PbrBundle {
        mesh: meshes_assets.add(Cuboid::new(1.0, 1.0, 1.0)),
        material: materials.add(Color::rgb_u8(124, 144, 255)),
        transform: Transform::from_xyz(0.0, 20., 0.0),
        ..default()
    });

    slice(&mut commands, &mesh, &mut meshes_assets, &mut materials);
}
