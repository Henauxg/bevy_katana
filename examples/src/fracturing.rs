use std::collections::HashMap;

use bevy::{
    app::{App, PluginGroup, Startup},
    asset::Assets,
    hierarchy::BuildChildren,
    log::info,
    math::{Vec3, Vec3A},
    pbr::{
        wireframe::{Wireframe, WireframeColor, WireframeConfig, WireframePlugin},
        PbrBundle, StandardMaterial,
    },
    prelude::{default, Commands, Cuboid, Entity, ResMut, SpatialBundle},
    render::{
        color::Color,
        mesh::{Mesh, Meshable},
        settings::{RenderCreation, WgpuFeatures, WgpuSettings},
        RenderPlugin,
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
    na::{Isometry3, Translation3, UnitQuaternion},
    parry::{
        bounding_volume::Aabb,
        math::{Isometry, Point, Translation},
        shape::{Shape, TriMesh},
    },
    plugin::{NoUserData, RapierPhysicsPlugin},
    rapier::geometry::BoundingVolume,
    render::RapierDebugRenderPlugin,
};
use examples::plugin::ExamplesPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            // DefaultPlugins.set(RenderPlugin {
            //     render_creation: RenderCreation::Automatic(WgpuSettings {
            //         features: WgpuFeatures::POLYGON_MODE_LINE,
            //         ..default()
            //     }),
            //     ..default()
            // }),
            WireframePlugin,
        ))
        .add_plugins((
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_plugins(ExamplesPlugin)
        .add_plugins(DefaultRaycastingPlugin)
        .insert_resource(WireframeConfig {
            global: false,
            default_color: Color::WHITE,
        })
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    commands: Commands,
    materials: ResMut<Assets<StandardMaterial>>,
    meshes_assets: ResMut<Assets<Mesh>>,
) {
    let mesh = Cuboid::new(1., 1., 1.).mesh();
    let meshes = slice_bevy_mesh_iterative(&mesh, 1, Some(Vec3A::X));

    let mut colliders: Vec<Collider> = Vec::with_capacity(meshes.len());
    for mesh in meshes.iter() {
        // It's a bevy Mesh we generated, we know that it is in a compatible format
        let mut collider =
            Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull).unwrap();
        // collider.set_scale(1.05 * Vec3::ONE, 2);
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
                info!("Collision between frag {} and {}", i, j + i + 1);
                let attach_point_2 = collider_2.raw.0.compute_local_aabb().center();
                joints.push((j, (attach_point_2 - attach_point_1).into()));
            } else {
                info!("No collision between frag {} and {}", i, j + i + 1);
            }
        }
        if !joints.is_empty() {
            // TODO Do better than this

            all_joints.insert(i, joints);
        }
    }

    for collider in colliders.iter_mut() {
        // collider.set_scale(0.5 * Vec3::ONE, 2);
        // collider.promote_scaled_shape();
    }

    spawn_fragments_entities(
        commands,
        materials,
        meshes_assets,
        &meshes,
        &colliders,
        &all_joints, // &joints,
    );
}

fn spawn_fragments_entities(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    frag_meshes: &Vec<Mesh>,
    frag_colliders: &Vec<Collider>,
    all_joints: &HashMap<usize, Vec<(usize, Vec3)>>, // translations: &Vec<Translation3<f32>>,
                                                     // joints: &HashMap<usize, Vec<usize>>,
) {
    let mut entity_map = Vec::new();

    for (index, (mesh, collider)) in frag_meshes.iter().zip(frag_colliders.iter()).enumerate() {
        let pos = Vec3::new(0. * index as f32, 3., 0.);
        info!("Spawning frag at pos {:?}, index {}", pos, index);
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
        let from_entity = entity_map[from];
        for (to, anchor) in to_fragments.iter() {
            let to_entity = entity_map[*to];
            // TODO attach point
            // let anchor = Vec3::new(-1., 0., 0.); // TODO Tmp Override
            let joint = FixedJointBuilder::new()
                .local_anchor1(*anchor)
                .local_anchor2(*anchor);
            info!("Spawning joint with anchor: {:?} ", anchor);
            let child_entity = commands
                .spawn((
                    // SpatialBundle::default(),
                    ImpulseJoint::new(to_entity, joint),
                ))
                .id();
            commands.entity(from_entity).add_child(child_entity);
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
            // Wireframe,
            // WireframeColor {
            //     color: Color::GREEN,
            // },
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
