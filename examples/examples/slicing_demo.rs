use bevy::{
    app::{App, PluginGroup, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::{
        palettes::css::{GREEN, RED},
        Color, LinearRgba,
    },
    core_pipeline::{bloom::BloomSettings, tonemapping::Tonemapping},
    hierarchy::DespawnRecursiveExt,
    input::ButtonInput,
    log::info,
    math::{Vec3, Vec3A},
    pbr::{
        wireframe::{Wireframe, WireframeColor, WireframeConfig, WireframePlugin},
        MaterialMeshBundle, PbrBundle, StandardMaterial,
    },
    prelude::{
        AlphaMode, Camera3dBundle, Commands, Component, Cuboid, Entity, Event, EventReader,
        EventWriter, Gizmos, IntoSystemConfigs, KeyCode, MeshBuilder, MouseButton, Query, Res,
        ResMut, Resource, With, Without,
    },
    render::{
        camera::Camera,
        mesh::{Mesh, Meshable},
        settings::{RenderCreation, WgpuFeatures, WgpuSettings},
        RenderPlugin,
    },
    scene::SceneBundle,
    transform::components::{GlobalTransform, Transform},
    utils::default,
    DefaultPlugins,
};

use bevy_ghx_utils::camera::{
    update_pan_orbit_camera, PanOrbitCameraBundle, PanOrbitSettings, PanOrbitState,
};
use bevy_gltf::GltfAssetLabel;
use bevy_katana::{
    slicing::slicing::{slice_bevy_mesh, slice_bevy_mesh_iterative},
    types::Plane,
};
use bevy_mod_raycast::prelude::*;
use bevy_rapier3d::{
    dynamics::RigidBody,
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
    plugin::{NoUserData, RapierPhysicsPlugin},
};
use examples::{
    lines::LineList,
    plugin::{setup_environment, ExamplesPlugin},
};

pub const DRAGON_ASSET_PATH: &str = "dragon_low_poly.glb";
pub const DRAGON_SPAWN_TRANSLATION: Vec3 = Vec3::new(0.0, 2., 0.0);

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            RapierPhysicsPlugin::<NoUserData>::default(),
            // ExamplesPlugin,
        ))
        // .add_event::<SpawnSliceableEvent>()
        // .add_event::<SliceEvent>()
        // .add_event::<FragmenterEvent>()
        // .add_event::<DeterministicSliceEvent>()
        .add_systems(Startup, (setup_scene, setup_camera, setup_environment))
        .add_systems(Update, update_pan_orbit_camera)
        .run();
}

pub fn setup_camera(mut commands: Commands) {
    // Camera
    let camera_position = Vec3::new(0., 1., 10.5);
    let look_target = DRAGON_SPAWN_TRANSLATION;
    commands.spawn((
        PanOrbitCameraBundle {
            camera: Camera3dBundle {
                transform: Transform::from_translation(camera_position)
                    .looking_at(look_target, Vec3::Y),
                camera: Camera {
                    hdr: true,
                    ..default()
                },
                tonemapping: Tonemapping::TonyMcMapface,
                ..default()
            },
            state: PanOrbitState {
                radius: (look_target - camera_position).length(),
                ..Default::default()
            },
            settings: PanOrbitSettings {
                auto_orbit: false,
                ..Default::default()
            },
        },
        BloomSettings::NATURAL,
    ));
}

fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
) {
    // TODO May spawn only the mesh + mat
    commands.spawn((SceneBundle {
        scene: asset_server.load(GltfAssetLabel::Scene(0).from_asset(DRAGON_ASSET_PATH)),
        transform: Transform::from_translation(DRAGON_SPAWN_TRANSLATION),
        ..default()
    },));

    commands.spawn((MaterialMeshBundle {
        mesh: meshes_assets.add(LineList {
            lines: vec![(Vec3::new(-2., 4., -2.), Vec3::new(-2., 4., 2.))],
        }),
        material: materials.add(StandardMaterial {
            base_color: Color::Srgba(RED),
            alpha_mode: AlphaMode::Opaque,
            emissive: LinearRgba::rgb(75., 10., 10.),
            ..default()
        }),
        ..default()
    },));

    let grid_pos = Vec3::new(0., 0., -3.);
    let row_dir = Vec3::Y;
    let col_dir = Vec3::X;
    let grid_rows_count = 15;
    let grid_rows_delta = 0.1;

    let mut grid = vec![];
    let mut line_start_pos = grid_pos - ((grid_rows_count as f32 * grid_rows_delta) / 2.) * col_dir;
    for x in 0..grid_rows_count {
        for y in 0..grid_rows_count {
            grid.push((line_start_pos, Vec3::new(-2., 4., 2.)))
        }
    }
}

// fn key_mapping(
//     mut commands: Commands,
//     key: Res<ButtonInput<KeyCode>>,
//     mut spawn_sliceable_events: EventWriter<SpawnSliceableEvent>,
//     mut deterministic_slice_events: EventWriter<DeterministicSliceEvent>,
//     mut spawn_fragmenter_events: EventWriter<FragmenterEvent>,
//     query_despawn1: Query<Entity, With<Sliced>>,
//     query_despawn2: Query<Entity, With<SliceableObject>>,
// ) {
//     if key.just_pressed(KeyCode::KeyN) {
//         spawn_sliceable_events.send(SpawnSliceableEvent);
//     }
//     if key.just_pressed(KeyCode::KeyT) {
//         deterministic_slice_events.send(DeterministicSliceEvent);
//     } else if key.just_pressed(KeyCode::Enter) {
//         for entity in query_despawn1.iter() {
//             commands.entity(entity).despawn();
//         }
//         for entity in query_despawn2.iter() {
//             commands.entity(entity).despawn();
//         }
//     } else if key.just_pressed(KeyCode::KeyH) {
//         spawn_fragmenter_events.send(FragmenterEvent);
//     }
// }

// fn spawn_fragmented_object(
//     mut commands: Commands,
//     mut materials: ResMut<Assets<StandardMaterial>>,
//     mut meshes_assets: ResMut<Assets<Mesh>>,
//     mut spawn_fragmenter_events: EventReader<FragmenterEvent>,
// ) {
//     for _ in spawn_fragmenter_events.read() {
//         let mesh = Cuboid::new(1., 1., 1.).mesh().build();

//         let fragments = slice_bevy_mesh_iterative(&mesh, 2, None);
//         spawn_fragments(
//             fragments.as_slice(),
//             &mut materials,
//             &mut meshes_assets,
//             &mut commands,
//             mesh.compute_aabb().unwrap().center.into(),
//         );
//     }
// }

// fn spawn_sliceable_object(
//     mut commands: Commands,
//     mut materials: ResMut<Assets<StandardMaterial>>,
//     mut meshes_assets: ResMut<Assets<Mesh>>,
//     mut spawn_sliceable_events: EventReader<SpawnSliceableEvent>,
//     query_despawn: Query<Entity, With<Sliced>>,
// ) {
//     for _ in spawn_sliceable_events.read() {
//         for entity in query_despawn.iter() {
//             commands.entity(entity).despawn();
//         }

//         commands.spawn((
//             SliceMesh,
//             PbrBundle {
//                 //mesh: meshes_assets.add(Cylinder::new(1.0, 5.0)),
//                 mesh: meshes_assets.add(Cuboid::new(1.0, 1.0, 1.0)),
//                 material: materials.add(Color::srgb_u8(124, 144, 255)),
//                 transform: Transform::from_xyz(5.0, 0.5, 0.0),
//                 ..default()
//             },
//             SliceableObject,
//         ));
//     }
// }

// fn spawn_fragments(
//     mesh_fragments: &[Mesh],
//     materials: &mut ResMut<Assets<StandardMaterial>>,
//     meshes_assets: &mut ResMut<Assets<Mesh>>,
//     commands: &mut Commands,
//     sliced_mesh_pos: Vec3,
// ) {
//     for mesh in mesh_fragments {
//         let mesh_handle = meshes_assets.add(mesh.clone());
//         commands.spawn((
//             PbrBundle {
//                 mesh: mesh_handle.clone(),
//                 transform: Transform::from_translation(sliced_mesh_pos),
//                 material: materials.add(Color::srgb_u8(124, 144, 255)),
//                 ..default()
//             },
//             SliceableObject,
//             Wireframe,
//             WireframeColor {
//                 color: Color::Srgba(GREEN),
//             },
//             RigidBody::Dynamic,
//             Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull).unwrap(),
//             ActiveCollisionTypes::default(),
//             Friction::coefficient(0.7),
//             Restitution::coefficient(0.05),
//             ColliderMassProperties::Density(2.0),
//         ));
//     }
// }
