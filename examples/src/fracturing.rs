use std::collections::HashMap;

use bevy::{
    app::{App, Startup},
    asset::Assets,
    hierarchy::BuildChildren,
    log::info,
    math::{Vec3, Vec3A},
    pbr::{wireframe::WireframePlugin, PbrBundle, StandardMaterial},
    prelude::{default, Commands, Cuboid, Entity, ResMut},
    render::{
        color::Color,
        mesh::{Mesh, Meshable},
    },
    transform::components::Transform,
    DefaultPlugins,
};
use bevy_ghx_destruction::slicing::slicing::slice_bevy_mesh_iterative;
use bevy_mod_raycast::prelude::*;
use bevy_rapier3d::{
    dynamics::{FixedJointBuilder, ImpulseJoint, RigidBody},
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
    na::Isometry3,
    parry::{math::Isometry, shape::Shape},
    plugin::{NoUserData, RapierPhysicsPlugin},
    rapier::geometry::BoundingVolume,
    render::RapierDebugRenderPlugin,
};
use examples::plugin::ExamplesPlugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, WireframePlugin))
        .add_plugins((
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_plugins(ExamplesPlugin)
        .add_plugins(DefaultRaycastingPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    commands: Commands,
    materials: ResMut<Assets<StandardMaterial>>,
    meshes_assets: ResMut<Assets<Mesh>>,
) {
    let mesh = Cuboid::new(1., 1., 1.).mesh();
    let meshes = slice_bevy_mesh_iterative(&mesh, 2, Some(Vec3A::X));

    let mut colliders: Vec<Collider> = Vec::with_capacity(meshes.len());
    for mesh in meshes.iter() {
        // It's a bevy Mesh we generated, we know that it is in a compatible format
        let collider = Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull).unwrap();
        colliders.push(collider);
    }

    let mut all_joints = HashMap::new();
    for (i, collider_1) in colliders.iter().enumerate() {
        let mut joints = Vec::new();
        let attach_point_1 = collider_1.raw.0.compute_local_aabb().center();
        for (j, collider_2) in colliders[i + 1..colliders.len()].iter().enumerate() {
            if check_collision(
                collider_1.into(),
                &&Isometry3::identity(),
                collider_2.into(),
                &Isometry3::identity(),
            ) {
                let attach_point_2 = collider_2.raw.0.compute_local_aabb().center();
                joints.push((j + i + 1, (attach_point_2 - attach_point_1).into()));
            }
        }
        if !joints.is_empty() {
            all_joints.insert(i, joints);
        }
    }

    spawn_fragments_entities(
        commands,
        materials,
        meshes_assets,
        &meshes,
        &colliders,
        &all_joints,
    );
}

fn spawn_fragments_entities(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    frag_meshes: &Vec<Mesh>,
    frag_colliders: &Vec<Collider>,
    all_joints: &HashMap<usize, Vec<(usize, Vec3)>>,
) {
    let mut entity_map = Vec::new();

    for (mesh, collider) in frag_meshes.iter().zip(frag_colliders.iter()) {
        let pos = Vec3::new(0. as f32, 3., 0.);
        let entity = create_fragment_entity(
            &mut materials,
            &mut meshes_assets,
            &mut commands,
            mesh,
            collider,
            pos,
        );
        entity_map.push(entity);
    }

    for (&from, to_fragments) in all_joints.iter() {
        let parent_entity = entity_map[from];
        for (to, anchor) in to_fragments.iter() {
            let child = entity_map[*to];
            let joint = FixedJointBuilder::new()
                .local_anchor1(*anchor)
                .local_anchor2(*anchor);
            let child_entity = commands.spawn((ImpulseJoint::new(child, joint),)).id();
            commands.entity(parent_entity).add_child(child_entity);
        }
    }
}

fn create_fragment_entity(
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    mesh: &Mesh,
    collider: &Collider,
    pos: Vec3,
) -> Entity {
    let mesh_handle = meshes_assets.add(mesh.clone());
    commands
        .spawn((
            PbrBundle {
                mesh: mesh_handle.clone(),
                transform: Transform::from_translation(pos),
                material: materials.add(Color::rgb_u8(124, 144, 255)),
                ..default()
            },
            RigidBody::Dynamic,
            collider.clone(),
            ActiveCollisionTypes::default(),
            Friction::coefficient(0.7),
            Restitution::coefficient(0.05),
            ColliderMassProperties::Density(2.0),
        ))
        .id()
}

fn check_collision(
    shape1: &dyn Shape,
    pos1: &Isometry<f32>,
    shape2: &dyn Shape,
    pos2: &Isometry<f32>,
) -> bool {
    let aabb1 = shape1.compute_aabb(pos1);
    let aabb2 = shape2.compute_aabb(pos2);

    aabb1.intersects(&aabb2)
}
