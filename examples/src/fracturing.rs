use std::collections::HashMap;

use bevy::{
    app::{App, Startup, Update},
    asset::Assets,
    color::Color,
    hierarchy::{BuildChildren, DespawnRecursiveExt},
    input::ButtonInput,
    log::info,
    math::Vec3,
    pbr::{wireframe::WireframePlugin, PbrBundle, StandardMaterial},
    prelude::{
        default, Commands, Component, Cuboid, Entity, EventReader, IntoSystemConfigs, KeyCode,
        MeshBuilder, Query, Res, ResMut, With,
    },
    render::mesh::{Mesh, Meshable},
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
    pipeline::ContactForceEvent,
    plugin::{NoUserData, RapierPhysicsPlugin},
    rapier::geometry::BoundingVolume,
    render::RapierDebugRenderPlugin,
};
use examples::plugin::ExamplesPlugin;

#[derive(Component)]
struct ExampleMesh;

#[derive(Component)]
struct FragmentedMesh;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, WireframePlugin))
        .add_plugins((
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_plugins(ExamplesPlugin)
        .add_plugins(CursorRayPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (respawn_mesh, handle_collisions).chain())
        .run();
}

fn setup(
    commands: Commands,
    materials: ResMut<Assets<StandardMaterial>>,
    meshes_assets: ResMut<Assets<Mesh>>,
) {
    spawn_frac_mesh(commands, materials, meshes_assets);
}

fn spawn_mesh(
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    mesh: &Mesh,
    pos: Vec3,
) {
    let mesh_handle = meshes_assets.add(mesh.clone());
    commands.spawn((
        PbrBundle {
            mesh: mesh_handle.clone(),
            transform: Transform::from_xyz(pos.x, pos.y, pos.z),
            material: materials.add(Color::srgb_u8(124, 144, 255)),
            ..default()
        },
        ExampleMesh,
        RigidBody::Dynamic,
        Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull).unwrap(),
        ActiveCollisionTypes::default(),
        Friction::coefficient(0.7),
        Restitution::coefficient(0.05),
        ColliderMassProperties::Density(2.0),
    ));
}

fn spawn_frac_mesh(
    commands: Commands,
    materials: ResMut<Assets<StandardMaterial>>,
    meshes_assets: ResMut<Assets<Mesh>>,
) {
    let mesh = Cuboid::new(1., 1., 1.).mesh().build();
    let meshes = slice_bevy_mesh_iterative(&mesh, 4, None);

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
                &Isometry3::identity(),
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

fn respawn_mesh(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    query_cubes: Query<Entity, With<ExampleMesh>>,
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
) {
    if keyboard_input.pressed(KeyCode::KeyF) {
        for entity in query_cubes.iter() {
            commands.entity(entity).despawn_recursive();
        }
        let mesh = Cuboid::new(1., 1., 1.).mesh().build();
        let pos = Vec3::new(0., 0., 0.);
        spawn_mesh(
            &mut materials,
            &mut meshes_assets,
            &mut commands,
            &mesh,
            pos,
        );
    }
    if keyboard_input.pressed(KeyCode::KeyG) {
        for entity in query_cubes.iter() {
            commands.entity(entity).despawn_recursive();
        }
        spawn_frac_mesh(commands, materials, meshes_assets);
    }
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
        info!("from {:?}, to_fragments{:?}", from, to_fragments);
        for (to, anchor) in to_fragments.iter() {
            let child = entity_map[*to];
            let joint = FixedJointBuilder::new()
                .local_anchor1(*anchor)
                .local_anchor2(*anchor);
            let child_entity = commands.spawn((ImpulseJoint::new(child, joint),)).id();
            commands.entity(parent_entity).add_child(child_entity);
            // .insert(ImpulseJoint::new(child, joint));
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
                material: materials.add(Color::srgb_u8(124, 144, 255)),
                ..default()
            },
            ExampleMesh,
            FragmentedMesh,
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

fn handle_collisions(
    mut commands: Commands,
    mut contact_force_events: EventReader<ContactForceEvent>,
    fragments: Query<&FragmentedMesh>,
) {
    for contact_force_event in contact_force_events.read() {
        let fragment = if let Ok(_) = fragments.get(contact_force_event.collider1) {
            Some(contact_force_event.collider1)
        } else if let Ok(_) = fragments.get(contact_force_event.collider2) {
            Some(contact_force_event.collider2)
        } else {
            None
        };
        if let Some(fragment_entity) = fragment {
            commands.entity(fragment_entity).despawn_descendants();
        }
    }
}
