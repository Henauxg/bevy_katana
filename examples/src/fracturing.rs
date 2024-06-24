use std::collections::HashMap;

use bevy::{
    app::{App, Startup},
    asset::Assets,
    math::Vec3,
    pbr::{PbrBundle, StandardMaterial},
    prelude::{default, Commands, Cuboid, Entity, ResMut},
    render::{
        color::Color,
        mesh::{Mesh, Meshable},
    },
    transform::components::Transform,
    DefaultPlugins,
};
use bevy_ghx_destruction::{slicing::slicing::slice_mesh_iterative, types::SlicedMesh};
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
};
use examples::plugin::ExamplesPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(ExamplesPlugin)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
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
    // let mesh_aabb = Cuboid::new(1., 1., 1.).mesh().compute_aabb().unwrap();

    // TODO Do not compute trimeshes. We just need aabbs
    let trimesh = create_parry_trimesh(&SlicedMesh::from_bevy_mesh(&mesh));

    let meshes = slice_mesh_iterative(&mesh, 1);

    // TODO Less back & forth between != mesh formats
    let chunks = create_chunks(&meshes);

    let (translations, joints) =
        create_joints(&chunks, &trimesh.compute_aabb(&Isometry3::identity()));

    spawn_entities(
        commands,
        materials,
        meshes_assets,
        &chunks,
        &translations,
        &joints,
    );
}

fn spawn_entities(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes_assets: ResMut<Assets<Mesh>>,
    chunks: &Vec<SlicedMesh>,
    translations: &Vec<Translation3<f32>>,
    joints: &HashMap<usize, Vec<usize>>,
) {
    let mut entity_map = HashMap::new();

    for (i, chunk) in chunks.iter().enumerate() {
        let mesh = chunk.to_bevy_mesh();
        let entity = create_entity(
            &mut materials,
            &mut meshes_assets,
            &mut commands,
            &mesh,
            translations[i],
        );

        entity_map.insert(i, entity);
    }

    for (i, joint_indices) in joints.iter() {
        if let Some(&parent_entity) = entity_map.get(i) {
            for &j in joint_indices.iter() {
                if let Some(&child_entity) = entity_map.get(&j) {
                    let joint = FixedJointBuilder::new().local_anchor1(Vec3::new(0.0, 0.0, 0.0));

                    commands
                        .entity(child_entity)
                        .insert(ImpulseJoint::new(parent_entity, joint));
                }
            }
        }
    }
}

fn create_joints(
    chunks: &Vec<SlicedMesh>,
    aabb: &Aabb,
) -> (Vec<Translation3<f32>>, HashMap<usize, Vec<usize>>) {
    let mut joints = HashMap::new();
    let mut translations = vec![Translation::new(0.0, 0.0, 0.0); chunks.len()];

    for (i, chunk1) in chunks.iter().enumerate() {
        let mut joint = Vec::new();

        let trimesh1 = create_parry_trimesh(chunk1);
        let translation1 = calculate_translation(aabb, &trimesh1);
        let isometry1 = create_isometry(translation1.x, translation1.y, translation1.z, 0.);

        translations[i] = translation1;

        for (j, chunk2) in chunks.iter().enumerate() {
            if i == j {
                continue;
            }

            let trimesh2 = create_parry_trimesh(chunk2);
            let translation2 = calculate_translation(aabb, &trimesh2);
            let isometry2 = create_isometry(translation2.x, translation2.y, translation2.z, 0.);

            if check_collision(&trimesh1, &isometry1, &trimesh2, &isometry2) {
                joint.push(j);
            }
        }
        joints.insert(i, joint);
    }

    (translations, joints)
}

fn create_entity(
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    commands: &mut Commands,
    mesh: &Mesh,
    pos: Translation<f32>,
) -> Entity {
    let mesh_handle = meshes_assets.add(mesh.clone());
    commands
        .spawn((
            PbrBundle {
                mesh: mesh_handle.clone(),
                transform: Transform::from_xyz(pos.x, pos.y, pos.z),
                material: materials.add(Color::rgb_u8(124, 144, 255)),
                ..default()
            },
            RigidBody::Fixed,
            Collider::from_bevy_mesh(&mesh, &ComputedColliderShape::ConvexHull).unwrap(),
            ActiveCollisionTypes::default(),
            Friction::coefficient(0.7),
            Restitution::coefficient(0.05),
            ColliderMassProperties::Density(2.0),
        ))
        .id()
}

fn calculate_translation(main_aabb: &Aabb, sub_mesh: &TriMesh) -> Translation3<f32> {
    let main_center = main_aabb.center();

    let sub_aabb = sub_mesh.compute_aabb(&Isometry3::identity());
    let sub_center = sub_aabb.center();

    let translation_vector = sub_center - main_center;

    Translation3::from(translation_vector)
}

fn create_isometry(tx: f32, ty: f32, tz: f32, angle: f32) -> Isometry3<f32> {
    let translation = Translation3::new(tx, ty, tz);
    let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, angle);
    Isometry3::from_parts(translation, rotation)
}

fn create_parry_trimesh(chunk: &SlicedMesh) -> TriMesh {
    let vertices = chunk.vertices();
    let mut parry_vertices = Vec::new();
    for vertex in vertices {
        let pos = vertex.pos();
        parry_vertices.push(Point::new(pos.x, pos.y, pos.z));
    }

    let triangles = chunk.indices();
    let mut parry_indices = Vec::new();
    for triangle_id in (0..triangles.len()).step_by(3) {
        parry_indices.push([
            triangles[triangle_id] as u32,
            triangles[triangle_id + 1] as u32,
            triangles[triangle_id + 2] as u32,
        ]);
    }

    TriMesh::new(parry_vertices, parry_indices)
}

fn create_chunks(meshes: &Vec<Mesh>) -> Vec<SlicedMesh> {
    let mut chunks = Vec::new();
    for mesh in meshes {
        chunks.push(SlicedMesh::from_bevy_mesh(mesh));
    }

    chunks
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
