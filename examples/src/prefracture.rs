use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_rapier3d::prelude::*;
use std::f32::consts::*;

use bevy::{
    input::common_conditions::input_just_pressed, pbr::CascadeShadowConfigBuilder, prelude::*,
};
use bevy_ghx_utils::camera::{toggle_auto_orbit, update_pan_orbit_camera, PanOrbitCamera};

pub const CUBE_FRAC_ASSET_PATH: &str = "cube_frac.glb#Scene0";
pub const CUBE_ASSET_PATH: &str = "cube.glb#Scene0";

fn main() {
    App::new()
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0 / 5.0f32,
        })
        .insert_resource(Time::<Fixed>::from_seconds(10.5))
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        // .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(WorldInspectorPlugin::new())
        .add_systems(Startup, (setup_camera, setup_scene))
        .add_systems(
            Update,
            (
                toggle_auto_orbit.run_if(input_just_pressed(KeyCode::F5)),
                update_pan_orbit_camera,
            ),
        )
        .add_systems(
            Update,
            (respawn_cube, attach_physics_components_to_cells).chain(),
        )
        .run();
}

pub fn setup_camera(mut commands: Commands) {
    // Camera
    let camera_position = Vec3::new(0., 4.5, 7.5);
    let look_target = Vec3::ZERO;
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_translation(camera_position)
                .looking_at(look_target, Vec3::Y),
            ..default()
        },
        PanOrbitCamera {
            radius: (look_target - camera_position).length(),
            ..Default::default()
        },
    ));
}

#[derive(Component, Debug)]
struct FragmentedCubeRoot {
    physics_applied: bool,
}

impl Default for FragmentedCubeRoot {
    fn default() -> Self {
        Self {
            physics_applied: false,
        }
    }
}

#[derive(Component)]
struct ExampleCube;

fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // // Plane
    let radius = 2000.;
    let height = 200.;
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder::new(radius, height)),
            material: materials.add(Color::WHITE),
            transform: Transform::from_xyz(0.0, -height / 2., 0.0),
            ..default()
        },
        Collider::cylinder(height / 2., radius),
        (ActiveCollisionTypes::default()),
        Friction::coefficient(0.7),
        Restitution::coefficient(0.3),
    ));

    // Light
    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::ZYX, 0.0, 1.0, -PI / 4.)),
        directional_light: DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        cascade_shadow_config: CascadeShadowConfigBuilder {
            first_cascade_far_bound: 200.0,
            maximum_distance: 400.0,
            ..default()
        }
        .into(),
        ..default()
    });

    spawn_frac_cube(&asset_server, &mut commands);
}

fn spawn_cube(asset_server: &Res<AssetServer>, commands: &mut Commands) {
    commands.spawn((
        SceneBundle {
            scene: asset_server.load(CUBE_ASSET_PATH),
            transform: Transform::from_xyz(0., 2., 0.),
            ..default()
        },
        ExampleCube,
    ));
}

fn spawn_frac_cube(asset_server: &Res<AssetServer>, commands: &mut Commands) {
    commands.spawn((
        SceneBundle {
            scene: asset_server.load(CUBE_FRAC_ASSET_PATH),
            transform: Transform::from_xyz(0., 2., 0.),
            ..default()
        },
        FragmentedCubeRoot::default(),
        ExampleCube,
    ));
}

fn respawn_cube(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    query_cubes: Query<Entity, With<ExampleCube>>,
    asset_server: Res<AssetServer>,
    mut commands: Commands,
) {
    if keyboard_input.pressed(KeyCode::KeyF) {
        for entity in query_cubes.iter() {
            commands.entity(entity).despawn_recursive();
        }
        spawn_cube(&asset_server, &mut commands);
    }
    if keyboard_input.pressed(KeyCode::KeyG) {
        for entity in query_cubes.iter() {
            commands.entity(entity).despawn_recursive();
        }
        spawn_frac_cube(&asset_server, &mut commands);
    }
}

fn attach_physics_components_to_cells(
    mut commands: Commands,
    mut fractured_scene: Query<(Entity, &mut FragmentedCubeRoot)>,
    children: Query<&Children>,
    meshes_handles: Query<&Handle<Mesh>>,
    meshes: ResMut<Assets<Mesh>>,
) {
    for (fractured_scene_entity, mut fragments_root) in fractured_scene.iter_mut() {
        // We only want to attach the physics components once
        if !fragments_root.physics_applied {
            for entity in children.iter_descendants(fractured_scene_entity) {
                // Attach the physics components only to the meshes entities
                if let Ok(mesh_handle) = meshes_handles.get(entity) {
                    let mesh = meshes.get(mesh_handle).unwrap();
                    info!("Attaching physics components to entity {:?}", entity);
                    let collider =
                        Collider::from_bevy_mesh(mesh, &ComputedColliderShape::ConvexHull).unwrap();
                    commands.entity(entity).insert((
                        RigidBody::Dynamic,
                        collider,
                        ActiveCollisionTypes::default(),
                        Friction::coefficient(0.7),
                        Restitution::coefficient(0.05),
                        ColliderMassProperties::Density(2.0),
                    ));
                    // Children do not seem to be created at the same time as the cube root entity, so we only update the flag once children are present
                    fragments_root.physics_applied = true;
                }
            }
        }
    }
}
