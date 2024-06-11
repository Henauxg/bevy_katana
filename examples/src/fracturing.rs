use std::{collections::VecDeque, ptr::null};

use bevy::{
    app::{App, Startup},
    asset::Assets,
    ecs::{
        entity::Entity,
        system::{Commands, Res, ResMut},
    },
    hierarchy::BuildChildren,
    log::info,
    math::{primitives::Cuboid, Vec3},
    pbr::PbrBundle,
    render::{
        mesh::{Mesh, Meshable},
        primitives::Aabb,
    },
    transform::components::Transform,
    utils::default,
    DefaultPlugins,
};
use bevy_ghx_destruction::slicing::slicing::fragment_mesh;
use bevy_rapier3d::{
    dynamics::{FixedJointBuilder, ImpulseJoint, RigidBody},
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
    parry::query::intersection_test,
    plugin::{NoUserData, RapierContext, RapierPhysicsPlugin},
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
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    rapier_context: Res<RapierContext>,
) {
    let mesh = Cuboid::new(1., 1., 1.).mesh();

    // slice mesh
    let sliced_mesh = fragment_mesh(&mesh, 12);

    // create the chunks
    let chunks = create_chunks(&mut commands, &mut meshes_assets, &sliced_mesh);

    // connect the chunks
    create_joints(&mut commands, rapier_context, &chunks);

    let fractured_entity = commands
        .spawn((PbrBundle { ..default() }, RigidBody::Dynamic))
        .id();

    for (chunk, _) in chunks {
        commands.entity(fractured_entity).push_children(&[chunk]);
    }
}

fn create_chunks(
    commands: &mut Commands,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    sliced_mesh: &VecDeque<Mesh>,
) -> Vec<(Entity, Aabb)> {
    let mut chunks = Vec::new();

    for mesh in sliced_mesh {
        let aabb = mesh.compute_aabb().unwrap();
        let pos = aabb.center;
        let mesh_handle = meshes_assets.add(mesh.clone());
        let chunk = commands
            .spawn((
                PbrBundle {
                    mesh: mesh_handle.clone(),
                    transform: Transform::from_xyz(pos.x, pos.y, pos.z),
                    ..default()
                },
                RigidBody::Dynamic,
                Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull).unwrap(),
                // ActiveCollisionTypes::default(),
                // Friction::coefficient(0.7),
                // Restitution::coefficient(0.05),
                // ColliderMassProperties::Density(2.0),
            ))
            .id();

        chunks.push((chunk, aabb));
    }

    chunks
}

fn create_joints(
    commands: &mut Commands,
    rapier_context: Res<RapierContext>,
    chunks: &Vec<(Entity, Aabb)>,
) {
    for (chunk1, aabb) in chunks {
        for (chunk2, _) in chunks {
            if chunk1 != chunk2 {
                rapier_context.colliders_with_aabb_intersecting_aabb(*aabb, |entity| {
                    println!(
                        "The entity {:?} has an AABB intersecting our test AABB",
                        entity
                    );
                    true // Return `false` instead if we want to stop searching for other colliders that contain this point.
                });

                //     intersection_test(pos1, g1, pos2, g2)

                //     match rapier_context.contact_pair(chunk1.clone(), chunk2.clone()) {
                //         Some(contact_pair_view) => {
                //             if contact_pair_view.has_any_active_contacts() {
                //                 info!("entity 1 {:?}, entioty 2 {:?}", chunk1, chunk2);
                //                 let joint =
                //                     FixedJointBuilder::new().local_anchor2(Vec3::new(0., 0., 0.));
                //                 commands.entity(*chunk2).with_children(|children| {
                //                     children.spawn(ImpulseJoint::new(*chunk1, joint));
                //                 });
                //             }
                //         }
                //         None => (),
                //     }
            }
        }
    }
}
