use std::time::Duration;

use bevy::{
    app::{App, Startup, Update},
    asset::{AssetServer, Assets, Handle},
    color::{palettes::css::RED, Color, LinearRgba},
    core_pipeline::{bloom::BloomSettings, tonemapping::Tonemapping},
    hierarchy::DespawnRecursiveExt,
    math::{Dir3, Vec3, Vec3A},
    pbr::{MaterialMeshBundle, PbrBundle, StandardMaterial},
    prelude::{
        in_state, AlphaMode, AppExtStates, Camera3dBundle, Commands, Component, Condition, Entity,
        Event, EventReader, EventWriter, GlobalTransform, IntoSystemConfigs, NextState, OnEnter,
        Plane3d, Query, Res, ResMut, Resource, States, TransformPoint, With,
    },
    render::{camera::Camera, mesh::Mesh},
    time::{Time, Timer, TimerMode},
    transform::components::Transform,
    utils::default,
    DefaultPlugins,
};

use bevy_ghx_utils::camera::{
    update_pan_orbit_camera, PanOrbitCameraBundle, PanOrbitSettings, PanOrbitState,
};
use bevy_gltf::{Gltf, GltfMesh};
use bevy_katana::{slicing::slicing::slice_bevy_mesh, types::Plane};
use bevy_rapier3d::{
    dynamics::RigidBody,
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
    plugin::{NoUserData, RapierPhysicsPlugin},
};
use bevy_tweening::{lens::TransformPositionLens, Animator, EaseFunction, Tween, TweeningPlugin};
use examples::{lines::LineList, plugin::setup_environment};

pub const DRAGON_ASSET_PATH: &str = "dragon_low_poly.glb";
pub const DRAGON_SPAWN_TRANSLATION: Vec3 = Vec3::new(0.0, 2., 0.0);

fn main() {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins,
        RapierPhysicsPlugin::<NoUserData>::default(),
        TweeningPlugin,
    ));
    app.add_event::<SliceEvent>();

    app.init_state::<State>();
    app.add_systems(Startup, (load_assets, setup_camera, setup_environment));
    app.add_systems(Update, update_pan_orbit_camera);
    app.add_systems(
        Update,
        (continue_to_running.run_if(in_state(State::Loading).and_then(all_assets_loaded)),),
    );
    app.add_systems(OnEnter(State::Running), setup_scene);
    app.add_systems(
        Update,
        ((update_lasers, slice_meshes)
            .chain()
            .run_if(in_state(State::Running)),),
    );

    app.run();
}

#[derive(States, Debug, Hash, PartialEq, Eq, Clone, Default)]
pub enum State {
    #[default]
    Loading,
    Running,
}

#[derive(Resource)]
struct AssetHandles {
    dragon_gltf: Handle<Gltf>,
    // dragon_material: Handle<StandardMaterial>,
}

fn load_assets(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    // mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let dragon_handle = asset_server.load(DRAGON_ASSET_PATH);
    // let dragon_material = materials.add(StandardMaterial { ..default() });

    commands.insert_resource(AssetHandles {
        dragon_gltf: dragon_handle,
        // dragon_material,
    });
}

fn all_assets_loaded(asset_server: Res<AssetServer>, assets: Res<AssetHandles>) -> bool {
    asset_server.is_loaded_with_dependencies(&assets.dragon_gltf)
}

fn continue_to_running(mut next_state: ResMut<NextState<State>>) {
    next_state.set(State::Running);
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

#[derive(Debug, Clone, Copy)]
struct SlicePlane {
    origin: Vec3,
    normal: Dir3,
}

#[derive(Component)]
struct Laser {
    timer: Timer,
    plane: SlicePlane,
}

#[derive(Component)]
struct Sliceable;

fn setup_scene(
    mut commands: Commands,
    assets: Res<AssetHandles>,
    assets_gltf: Res<Assets<Gltf>>,
    assets_gltfmesh: Res<Assets<GltfMesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
) {
    let Some(gltf) = assets_gltf.get(&assets.dragon_gltf) else {
        return;
    };
    let Some(gltf_mesh) = assets_gltfmesh.get(&gltf.meshes[0]) else {
        return;
    };
    let mesh_handle = &gltf_mesh.primitives[0].mesh;
    let mat_handle = &gltf.materials[0];

    commands.spawn((
        PbrBundle {
            mesh: mesh_handle.clone_weak(),
            // material: assets.dragon_material.clone_weak(),
            material: mat_handle.clone_weak(),
            transform: Transform::from_translation(DRAGON_SPAWN_TRANSLATION),
            ..default()
        },
        Sliceable,
    ));

    // TODO Put in a Resource
    let laser_material = materials.add(StandardMaterial {
        base_color: Color::Srgba(RED),
        alpha_mode: AlphaMode::Opaque,
        emissive: LinearRgba::rgb(150., 10., 10.),
        ..default()
    });

    let laser_cut_duration_ms = 2000;
    let laser_center_start = Vec3::new(-2., DRAGON_SPAWN_TRANSLATION.y - 0.25, 0.);
    let laser_center_end = laser_center_start + Vec3::new(1.75, 1.75, 0.);
    let cut_animation = Tween::new(
        EaseFunction::QuarticInOut,
        Duration::from_millis(laser_cut_duration_ms),
        TransformPositionLens {
            start: laser_center_start,
            end: laser_center_end,
        },
    );
    let half_length = 3.;
    let dir = Vec3::Z;
    let laser_tip_a = laser_center_start - half_length * dir;
    let laser_tip_b = laser_center_start + half_length * dir;
    let slice_normal = (laser_tip_a - laser_tip_b)
        .cross((laser_center_end) - laser_tip_b)
        .normalize();

    let local_laser_tip_a = -half_length * dir;
    let local_laser_tip_b = half_length * dir;
    commands.spawn((
        MaterialMeshBundle {
            mesh: meshes_assets.add(LineList {
                lines: vec![(local_laser_tip_a, local_laser_tip_b)],
            }),
            material: laser_material.clone(),
            ..default()
        },
        Animator::new(cut_animation),
        Laser {
            timer: Timer::new(
                Duration::from_millis(laser_cut_duration_ms),
                TimerMode::Once,
            ),
            plane: SlicePlane {
                origin: laser_center_start,
                normal: Dir3::new_unchecked(slice_normal),
            },
        },
    ));

    // let grid_rows_count = 20;
    // let grid_tiles_delta = 0.25;
    // let grid = make_grid(
    //     Vec3::new(0., 2., -3.),
    //     Vec3::Y,
    //     Vec3::X,
    //     grid_rows_count,
    //     grid_tiles_delta,
    // );
    // let grid_2 = make_grid(
    //     Vec3::new(3., 2., 0.),
    //     Vec3::Y,
    //     Vec3::Z,
    //     grid_rows_count,
    //     grid_tiles_delta,
    // );
    // commands.spawn((MaterialMeshBundle {
    //     mesh: meshes_assets.add(LineList { lines: grid }),
    //     material: mat.clone_weak(),
    //     ..default()
    // },));
    // commands.spawn((MaterialMeshBundle {
    //     mesh: meshes_assets.add(LineList { lines: grid_2 }),
    //     material: mat,
    //     ..default()
    // },));
}

fn make_grid(
    grid_pos: Vec3,
    row_dir: Vec3,
    col_dir: Vec3,
    grid_rows_count: u32,
    grid_tiles_delta: f32,
) -> Vec<(Vec3, Vec3)> {
    let grid_length = grid_rows_count as f32 * grid_tiles_delta;

    let mut grid = vec![];
    let bottom_left = grid_pos - (grid_length / 2.) * col_dir - (grid_length / 2.) * row_dir;
    for x in 0..=grid_rows_count {
        let col_bottom = bottom_left + x as f32 * grid_tiles_delta * col_dir;
        grid.push((col_bottom, col_bottom + grid_length * row_dir));
    }
    for y in 0..=grid_rows_count {
        let line_start = bottom_left + y as f32 * grid_tiles_delta * row_dir;
        grid.push((line_start, line_start + grid_length * col_dir));
    }
    grid
}

#[derive(Event)]
struct SliceEvent(SlicePlane);

fn update_lasers(
    mut commands: Commands,
    time: Res<Time>,
    mut lasers: Query<(Entity, &mut Laser)>,
    mut slice_events: EventWriter<SliceEvent>,
) {
    for (entity, mut laser) in lasers.iter_mut() {
        laser.timer.tick(time.delta());
        if laser.timer.finished() {
            slice_events.send(SliceEvent(laser.plane));
            commands.entity(entity).despawn_recursive();
        }
    }
}

fn slice_meshes(
    mut commands: Commands,
    mut slice_events: EventReader<SliceEvent>,
    sliceables_meshes: Query<
        (
            Entity,
            &Transform,
            &GlobalTransform,
            &Handle<Mesh>,
            &Handle<StandardMaterial>,
        ),
        With<Sliceable>,
    >,
    mut meshes_assets: ResMut<Assets<Mesh>>,
) {
    for slice_event in slice_events.read() {
        for (sliced_entity, transform, gtrsform, mesh_handle, mat_handle) in
            sliceables_meshes.iter()
        {
            let Some(mesh) = meshes_assets.get(mesh_handle) else {
                continue;
            };

            let slice_plane = slice_event.0;
            let inverse_trsfrm = gtrsform.affine().inverse();
            // let local_plane_origin = inverse_trsfrm.transform_point(slice_plane.origin);
            let local_plane_origin = inverse_trsfrm.matrix3 * Vec3A::from(slice_plane.origin)
                + inverse_trsfrm.translation;
            // TODO
            // let local_plane_normal = inverse_trsfrm.transform_point(slice_plane.normal);
            // let local_plane_normal =
            //     inverse_trsfrm.matrix3 * Vec3A::from(slice_plane.normal.as_vec3());
            let local_plane_normal = Vec3A::from(slice_plane.normal.as_vec3());

            let Some(fragments) = slice_bevy_mesh(
                Plane::new(local_plane_origin.into(), local_plane_normal.into()),
                mesh,
            ) else {
                continue;
            };

            commands.entity(sliced_entity).despawn_recursive();

            for frag_mesh in fragments.iter() {
                let Some(collider) =
                    Collider::from_bevy_mesh(frag_mesh, &ComputedColliderShape::ConvexHull)
                else {
                    continue;
                };
                // let Some(aabb) = fragment_mesh.compute_aabb() else {
                //     continue;
                // };
                let frag_mesh_handle = meshes_assets.add(frag_mesh.clone());
                let frag_entity = commands
                    .spawn((
                        // Name::new("Fragment"),
                        // StateScoped(Screen::Playing),
                        PbrBundle {
                            mesh: frag_mesh_handle.clone(),
                            transform: transform.clone(),
                            material: mat_handle.clone(),
                            ..default()
                        },
                        // Physics
                        RigidBody::Dynamic,
                        collider,
                        ActiveCollisionTypes::default(),
                        Friction::coefficient(0.5),
                        Restitution::coefficient(0.05),
                        ColliderMassProperties::Density(2.0),
                        // Logic
                        Sliceable,
                        // SlicedFragment::new(),
                    ))
                    .id();

                // let slice_center =
                //     (fragments_info.slice_positions.0 + fragments_info.slice_positions.1) / 2.;
                // let local_slice_center = fragments_info
                //     .sliced_object_transform
                //     .compute_matrix()
                //     .inverse()
                //     .transform_point(slice_center);
                // let local_frag_center: Vec3 = aabb.center.into();
                // let separating_impulse = FRAGMENTS_SEPARATION_IMPULSE_FACTOR
                //     * (local_frag_center - local_slice_center).normalize();

                // let slice_direction = (fragments_info.slice_positions.1
                //     - fragments_info.slice_positions.0)
                //     .normalize();
                // let slice_direction_impulse =
                //     slice_direction * FRAGMENTS_SLICE_DIRECTION_IMPULSE_FACTOR;

                // let torque_impulse = fragments_info.sliced_object_transform.right()
                //     * FRAGMENTS_TORQUE_IMPULSE_FACTOR;

                // commands.entity(frag_entity).insert(ExternalImpulse {
                //     impulse: separating_impulse + slice_direction_impulse,
                //     torque_impulse,
                // });
            }
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
