use bevy::{
    math::Vec3A,
    pbr::wireframe::{Wireframe, WireframeColor, WireframeConfig, WireframePlugin},
    prelude::*,
    render::{
        settings::{RenderCreation, WgpuFeatures, WgpuSettings},
        RenderPlugin,
    },
};

use bevy_ghx_destruction::{
    slicing::slicing::{fragment_mesh, slice_mesh},
    types::Plane,
};
use bevy_mod_billboard::plugin::BillboardPlugin;
use bevy_mod_raycast::prelude::*;
use bevy_rapier3d::{
    dynamics::RigidBody,
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
    plugin::{NoUserData, RapierPhysicsPlugin},
};
use examples::plugin::ExamplesPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(RenderPlugin {
                render_creation: RenderCreation::Automatic(WgpuSettings {
                    features: WgpuFeatures::POLYGON_MODE_LINE,
                    ..default()
                }),
                ..default()
            }),
            WireframePlugin,
        ))
        .insert_resource(WireframeConfig {
            global: true,
            default_color: Color::WHITE,
        })
        .add_event::<SpawnSliceableEvent>()
        .add_event::<SliceEvent>()
        .add_event::<FragmenterEvent>()
        .add_plugins((ExamplesPlugin, BillboardPlugin))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(DefaultRaycastingPlugin)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                raycast,
                slice_from_mouse,
                key_mapping,
                spawn_sliceable_object,
                spawn_fragmented_object,
                // spawn_slice,
            )
                .chain(),
        )
        .run();
}

#[derive(Component)]
struct SliceMesh;

#[derive(Event)]
struct SpawnSliceableEvent;

#[derive(Event)]
struct FragmenterEvent;

#[derive(Event)]
struct SliceEvent {
    begin: Vec3,
    end: Vec3,
    entity: Entity,
}

#[derive(Component)]
struct Sliced;

#[derive(Component)]
struct SliceableObject;

#[derive(Resource, Default)]
struct Slicer {
    begin: Vec3,
    end: Vec3,
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
) {
    // commands.spawn((
    //     SliceMesh,
    //     PbrBundle {
    //         mesh: meshes_assets.add(Cylinder::new(1.0, 5.0)),
    //         material: materials.add(Color::rgb_u8(124, 144, 255)),
    //         transform: Transform::from_xyz(5.0, 20., 0.0),
    //         ..default()
    //     },
    //     SliceableObject,
    // ));

    commands.spawn((
        SliceMesh,
        PbrBundle {
            mesh: meshes_assets.add(Cuboid::new(1.0, 1.0, 1.0)),
            material: materials.add(Color::rgb_u8(124, 144, 255)),
            transform: Transform::from_xyz(5.0, 0.5, 0.0),
            ..default()
        },
        SliceableObject,
    ));

    commands.init_resource::<Slicer>();
}

fn raycast(
    mouse: Res<ButtonInput<MouseButton>>,
    cursor_ray: Res<CursorRay>,
    mut raycast: Raycast,
    mut gizmos: Gizmos,
    mut spawn_sliceable_events: EventWriter<SliceEvent>,
    mut slicer: ResMut<Slicer>,
) {
    if let Some(cursor_ray) = **cursor_ray {
        raycast.debug_cast_ray(cursor_ray, &default(), &mut gizmos);
        let hits = raycast.cast_ray(cursor_ray, &default());

        for (entity, intersection) in hits.iter() {
            let pos = intersection.position();

            if mouse.just_pressed(MouseButton::Left) {
                slicer.begin = pos;
            }
            if mouse.just_released(MouseButton::Left) {
                slicer.end = pos;
                if slicer.begin != slicer.end {
                    spawn_sliceable_events.send(SliceEvent {
                        begin: slicer.begin,
                        end: slicer.end,
                        entity: *entity,
                    });
                }
            }
        }
    }
}

fn slice_from_mouse(
    mut materials: ResMut<Assets<StandardMaterial>>,
    cameras: Query<&mut Transform, With<Camera>>,
    sliceables: Query<
        (&Transform, &GlobalTransform, &Handle<Mesh>),
        (With<SliceableObject>, Without<Camera>),
    >,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    mut commands: Commands,
    mut spawn_sliceable_events: EventReader<SliceEvent>,
) {
    let camera_tranform = cameras.single();
    for event in spawn_sliceable_events.read() {
        if let Ok((transform, global_transform, mesh_handle)) = sliceables.get(event.entity) {
            let mesh = meshes_assets.get(mesh_handle).unwrap();

            let slice_center = Vec3A::from(transform.translation);

            let qr: Vec3 = event.begin - camera_tranform.translation;
            let qs = event.end - camera_tranform.translation;

            //            let plane_normal = qr.cross(qs).normalize();

            let local_begin = global_transform.transform_point(event.begin);
            let local_end = global_transform.transform_point(event.end);

            let local_cam = global_transform.transform_point(camera_tranform.translation);
            let local_qr = local_begin - local_cam;
            let local_qs = local_end - local_cam;

            let local_normal = local_qr.cross(local_qs).normalize();

            // let plane_normal = ;

            let plane = Plane::new(local_begin.into(), local_normal.into());

            info!("plane {:?}", plane);

            let meshes = slice_mesh(plane, &mesh);

            commands.entity(event.entity).despawn();

            commands.spawn((
                PbrBundle {
                    mesh: meshes_assets.add(Plane3d::new(qr.cross(qs))),
                    transform: Transform::from_translation(event.begin),
                    material: materials.add(Color::rgb_u8(124, 144, 255)),
                    ..default()
                },
                SliceableObject,
            ));

            spawn_fragment(
                meshes,
                &mut materials,
                &mut meshes_assets,
                &mut commands,
                slice_center,
            );
        }
    }
}

fn key_mapping(
    mut commands: Commands,
    key: Res<ButtonInput<KeyCode>>,
    mut spawn_sliceable_events: EventWriter<SpawnSliceableEvent>,
    mut spawn_fragmenter_events: EventWriter<FragmenterEvent>,
    query_despawn1: Query<Entity, With<Sliced>>,
    query_despawn2: Query<Entity, With<SliceableObject>>,
) {
    if key.just_pressed(KeyCode::Space) {
        spawn_sliceable_events.send(SpawnSliceableEvent);
    } else if key.just_pressed(KeyCode::Enter) {
        for entity in query_despawn1.iter() {
            commands.entity(entity).despawn();
        }
        for entity in query_despawn2.iter() {
            commands.entity(entity).despawn();
        }
    } else if key.just_pressed(KeyCode::KeyH) {
        spawn_fragmenter_events.send(FragmenterEvent);
    }
}

fn spawn_fragmented_object(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    mut spawn_fragmenter_events: EventReader<FragmenterEvent>,
) {
    for _ in spawn_fragmenter_events.read() {
        //let mesh = Cylinder::new(1.0, 5.0).mesh().build();

        let mesh = Cuboid::new(1., 1., 1.).mesh();

        let fragments = fragment_mesh(&mesh, 10);
        spawn_fragment(
            fragments.into(),
            &mut materials,
            &mut meshes_assets,
            &mut commands,
            mesh.compute_aabb().unwrap().center,
        );
    }
}

fn spawn_sliceable_object(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    mut spawn_sliceable_events: EventReader<SpawnSliceableEvent>,
    query_despawn: Query<Entity, With<Sliced>>,
) {
    for _ in spawn_sliceable_events.read() {
        for entity in query_despawn.iter() {
            commands.entity(entity).despawn();
        }

        commands.spawn((
            SliceMesh,
            PbrBundle {
                //mesh: meshes_assets.add(Cylinder::new(1.0, 5.0)),
                mesh: meshes_assets.add(Cuboid::new(1.0, 1.0, 1.0)),
                material: materials.add(Color::rgb_u8(124, 144, 255)),
                transform: Transform::from_xyz(5.0, 0.5, 0.0),
                ..default()
            },
            SliceableObject,
        ));
    }
}

fn spawn_fragment(
    meshes: Vec<Mesh>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    pos: Vec3A,
) {
    //Spawn the fragment for each mesh
    for mesh in meshes {
        let mesh_handle = meshes_assets.add(mesh.clone());
        commands.spawn((
            PbrBundle {
                mesh: mesh_handle.clone(),
                transform: Transform::from_xyz(pos.x, pos.y, pos.z),
                material: materials.add(Color::rgb_u8(124, 144, 255)),
                ..default()
            },
            SliceableObject,
            Wireframe,
            WireframeColor {
                color: Color::GREEN,
            },
            RigidBody::Dynamic,
            Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull).unwrap(),
            ActiveCollisionTypes::default(),
            Friction::coefficient(0.7),
            Restitution::coefficient(0.05),
            ColliderMassProperties::Density(2.0),
        ));
    }
}
