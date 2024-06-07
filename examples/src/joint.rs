use bevy::{
    app::{App, Startup},
    asset::Assets,
    ecs::{
        entity::Entity,
        system::{Commands, ResMut},
    },
    hierarchy::BuildChildren,
    math::{primitives::Cuboid, Vec3},
    pbr::{PbrBundle, StandardMaterial},
    render::mesh::{Mesh, Meshable},
    transform::components::Transform,
    utils::default,
    DefaultPlugins,
};
use bevy_rapier3d::{
    dynamics::{FixedJointBuilder, ImpulseJoint, RigidBody},
    geometry::{Collider, ComputedColliderShape},
    plugin::{NoUserData, RapierPhysicsPlugin},
    render::RapierDebugRenderPlugin,
};
use examples::plugin::ExamplesPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExamplesPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_systems(Startup, setup_physics)
        .run();
}

fn create_fixed_joints(
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    pos: Vec<Vec3>,
) {
    let mut body_entities = Vec::<Entity>::new();

    let rigid_body = RigidBody::Fixed;

    for (i, position) in pos.iter().enumerate() {
        let child_entity = commands
            .spawn((
                PbrBundle {
                    //mesh: meshes_assets.add(Cuboid::new(1., 1., 1.)),
                    transform: Transform::from_xyz(position.x, position.y, position.z),
                    //material: materials.add(Color::rgb_u8(124, 144, 255)),
                    ..default()
                },
                rigid_body,
                Collider::from_bevy_mesh(
                    &Cuboid::new(1.0, 1.0, 1.0).mesh(),
                    &ComputedColliderShape::ConvexHull,
                )
                .unwrap(),
            ))
            .id();

        if i > 0 {
            let parent_entity = *body_entities.last().unwrap();
            let joint = FixedJointBuilder::new().local_anchor2(Vec3::new(0., 0., 0.));
            commands.entity(child_entity).with_children(|children| {
                children.spawn(ImpulseJoint::new(parent_entity, joint));
            });
        }

        body_entities.push(child_entity);
    }
}

pub fn setup_physics(
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    mut commands: Commands,
) {
    let mut pos = Vec::new();
    pos.push(Vec3::new(0., 0.5, 0.));
    pos.push(Vec3::new(0., 0.5, 1.));
    pos.push(Vec3::new(1., 0.5, 0.));
    pos.push(Vec3::new(1., 0.5, 1.));

    pos.push(Vec3::new(0., 1.5, 0.));
    pos.push(Vec3::new(0., 1.5, 1.));
    pos.push(Vec3::new(1., 1.5, 0.));
    pos.push(Vec3::new(1., 1.5, 1.));

    create_fixed_joints(&mut materials, &mut meshes_assets, &mut commands, pos);
}
