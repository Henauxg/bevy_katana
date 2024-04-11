//! Loads and renders a glTF file as a scene.

use bevy_rapier3d::prelude::*;
use std::f32::consts::*;

use bevy::{
    input::common_conditions::input_just_pressed, pbr::CascadeShadowConfigBuilder, prelude::*,
};
use bevy_ghx_utils::camera::{toggle_auto_orbit, update_pan_orbit_camera, PanOrbitCamera};

fn main() {
    App::new()
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1.0 / 5.0f32,
        })
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, (setup_camera, setup_scene))
        .add_systems(
            Update,
            (
                toggle_auto_orbit.run_if(input_just_pressed(KeyCode::F5)),
                update_pan_orbit_camera,
            ),
        )
        .add_systems(Update, attach_physics_components_to_cells)
        // .add_systems(FixedUpdate, change_object)
        .run();
}

pub fn setup_camera(mut commands: Commands) {
    // Camera
    let camera_position = Vec3::new(0., 1.5, 2.5);
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
struct FragmentsRoot {
    physics_applied: bool,
}

impl Default for FragmentsRoot {
    fn default() -> Self {
        Self {
            physics_applied: false,
        }
    }
}

#[derive(Component)]
struct Full;

fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // // Plane
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cylinder::new(20.0, 0.1)),
            material: materials.add(Color::WHITE),
            // transform: Transform::from_rotation(Quat::from_rotation_x(
            //     -std::f32::consts::FRAC_PI_2,
            // )),
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            ..default()
        },
        Collider::cylinder(0.1, 20.0),
        (ActiveCollisionTypes::default()),
        // Sensor,
        // TransformBundle::from(Transform::from_xyz(0.0, 0.0, 0.0)),
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

    commands.spawn((
        SceneBundle {
            scene: asset_server.load("cube_frac.glb#Scene0"),
            transform: Transform::from_xyz(0., 2., 0.),
            ..default()
        },
        FragmentsRoot {
            physics_applied: false,
        },
    ));
}

fn change_object(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    query_fractured: Query<Entity, With<FragmentsRoot>>,
    query_full: Query<Entity, With<Full>>,
    asset_server: Res<AssetServer>,
    mut commands: Commands,
) {
    if keyboard_input.pressed(KeyCode::KeyF) {
        for entity in query_fractured.iter() {
            commands.entity(entity).despawn_recursive();
        }
        commands.spawn((
            SceneBundle {
                scene: asset_server.load("cube.glb#Scene0"),
                transform: Transform::from_xyz(0., 2., 0.),
                ..default()
            },
            Full,
        ));
    }
    if keyboard_input.pressed(KeyCode::KeyG) {
        for entity in query_full.iter() {
            commands.entity(entity).despawn_recursive();
        }
        commands.spawn((
            SceneBundle {
                scene: asset_server.load("cube_frac.glb#Scene0"),
                transform: Transform::from_xyz(0., 2., 0.),
                ..default()
            },
            FragmentsRoot::default(),
        ));
    }
}

fn attach_physics_components_to_cells(
    mut commands: Commands,
    mut fractured_scene: Query<(Entity, &mut FragmentsRoot)>,
    children: Query<&Children>,
) {
    for (fractured_scene_entity, mut fragments_root) in fractured_scene.iter_mut() {
        if !fragments_root.physics_applied {
            for entity in children.iter_descendants(fractured_scene_entity) {
                info!("Attaching physics components to entoty {:?}", entity);
                commands.entity(entity).insert((
                    RigidBody::Dynamic,
                    Collider::ball(0.2),
                    ActiveCollisionTypes::default(),
                    Friction::coefficient(0.7),
                    Restitution::coefficient(0.3),
                    ColliderMassProperties::Density(2.0),
                    ExternalForce::default(),
                    ExternalImpulse::default(),
                ));
                fragments_root.physics_applied = true;
            }
        }
    }
}
