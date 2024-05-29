use bevy::{
    app::{App, Startup, Update}, asset::Assets, ecs::{component::Component, system::{Commands, Res, ResMut}}, gizmos::gizmos::Gizmos, input::{keyboard::KeyCode, mouse::MouseButton, ButtonInput}, log::info, math::primitives::Cuboid, pbr::{PbrBundle, StandardMaterial}, prelude::default, render::{
        color::Color,
        mesh::{Mesh, Meshable},
    }, transform::components::Transform, DefaultPlugins
};

use bevy_ghx_destruction::
    slicing::slicing::slice
;
use bevy_mod_billboard::plugin::BillboardPlugin;
use bevy_mod_raycast::prelude::*;
use bevy_rapier3d::{plugin::{NoUserData, RapierPhysicsPlugin}, render::RapierDebugRenderPlugin};
use examples::plugin::ExamplesPlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, ExamplesPlugin, BillboardPlugin))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(DefaultRaycastingPlugin)
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, raycast)
        .run();
}

#[derive(Component)]
struct SliceMesh;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
) {
    let mesh = Cuboid::new(1.0, 1.0, 1.0).mesh();

    commands.spawn((
        SliceMesh,
        PbrBundle {
            mesh: meshes_assets.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb_u8(124, 144, 255)),
            transform: Transform::from_xyz(0.0, 20., 0.0),
            ..default()
        },
    ));

    slice(&mut commands, &mesh, &mut meshes_assets, &mut materials);
}

fn raycast(
    mouse: Res<ButtonInput<MouseButton>>,
    key: Res<ButtonInput<KeyCode>>,
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mut gizmos: Gizmos,
) {
    if let Some(cursor_ray) = **cursor_ray {
        raycast.debug_cast_ray(cursor_ray, &default(), &mut gizmos);
        let hits = raycast.cast_ray(cursor_ray, &default());

        for (_, intersection) in hits.iter() {
            let pos = intersection.position();

            if mouse.just_pressed(MouseButton::Left) {
                info!("pos{}", pos);
            }
            if mouse.just_released(MouseButton::Left) {
                info!("pos{}", pos);
            }

            if key.just_pressed(KeyCode::Space) {
                info!("pos{}", pos);
            }
        }
    }
}
