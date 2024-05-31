use bevy::{
    asset::{Assets, Handle},
    ecs::system::{Commands, ResMut},
    math::{Vec2, Vec3, Vec3A},
    pbr::{
        wireframe::{Wireframe, WireframeColor},
        PbrBundle, StandardMaterial,
    },
    render::{color::Color, mesh::Mesh},
    transform::components::Transform,
    utils::default,
};
use bevy_rapier3d::{
    dynamics::RigidBody,
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
};

use crate::{
    types::{CutDirection, MeshBuilder, Plane},
    utils::{find_intersection_line_plane_bis_bis, get_random_normalized_vec, is_above_plane},
};
use ghx_constrained_delaunay::types::{Edge, VertexId};

/// Operate the slice on a mesh
pub fn slice(
    commands: &mut Commands,
    mesh: &Mesh,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    // Mesh center
    let mesh_center = mesh.compute_aabb().unwrap().center;

    // Random normalized vector for the cut plane
    let normal_vec = get_random_normalized_vec();

    // Plane from point and normal
    let plane = Plane::new(mesh_center, normal_vec);

    // Execute the slice
    internal_slice(commands, plane, mesh, materials, meshes_assets)
}

pub fn internal_slice(
    commands: &mut Commands,
    plane: Plane,
    mesh: &Mesh,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
) {
    // Top and bottom meshes from the slice operation
    let (top_mesh, bottom_mesh) = compute_slice(plane, mesh);

    // Spawn the fragments from the meshes at the pos
    spawn_fragment(
        vec![top_mesh, bottom_mesh],
        plane.origin(),
        materials,
        meshes_assets,
        commands,
    );
}

pub fn compute_slice(plane: Plane, mesh: &Mesh) -> (Mesh, Mesh) {
    // Convert mesh into mesh builder to get easier access to mesh attributes
    let mut mesh_builder = MeshBuilder::mesh_mapping_from_mesh(mesh);

    // Split in half the mesh builder into top and bottom mesh builders
    let (top_mesh_builder, bottom_mesh_builder) = cut_mesh_mapping(plane, &mut mesh_builder);

    (
        top_mesh_builder.create_mesh(),
        bottom_mesh_builder.create_mesh(),
    )
}

fn cut_mesh_mapping(plane: Plane, mesh_builder: &mut MeshBuilder) -> (MeshBuilder, MeshBuilder) {
    // Track the position of the vertices along the slice plane
    let mut sides = vec![CutDirection::Unknow; mesh_builder.vertices().len()];

    // Initializing the top/ bottom mesh builders
    let mut top_mesh_builder = MeshBuilder::new_from(&mesh_builder);
    let mut bottom_mesh_builder = MeshBuilder::new_from(&mesh_builder);

    // Add the vertices from the sliced mesh to the top and bottom mesh builders
    for (index, _) in mesh_builder.vertices().iter().enumerate() {
        let vertex = mesh_builder.vertices()[index];
        sides[index] = is_above_plane(vertex.pos(), plane);
        if sides[index] == CutDirection::Top {
            top_mesh_builder.add_mapped_vertex(vertex, index);
        } else {
            bottom_mesh_builder.add_mapped_vertex(vertex, index);
        }
    }

    // Split in half all triangles interescting the slicing plane
    split_triangles(
        mesh_builder,
        plane,
        &sides,
        &mut top_mesh_builder,
        &mut bottom_mesh_builder,
    );

    // Fill the cut face newly created
    fill_cut_face(&plane, &mut top_mesh_builder, &mut bottom_mesh_builder);

    (top_mesh_builder, bottom_mesh_builder)
}

//todo: clean with our triangulation
fn fill_cut_face(
    plane: &Plane,
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
) {
    // Erease duplicated sliced vertices
    top_mesh_builder.shrink_sliced_vertices();
    bottom_mesh_builder.shrink_sliced_vertices();

    // Apply the triangulation to the top cut face. No need to do it for the bottom cute face since they both share the same vertices, we will just reverse the normal.
    let mut vertices = Vec::new();
    for vertex in top_mesh_builder.sliced_vertices().iter() {
        vertices.push(vertex.pos());
    }

    let planar_vertices = transform_to_2d_planar_coordinate_system(&mut vertices, plane.normal());

    let triangulation_vertices: Vec<(f64, f64)> = planar_vertices
        .iter()
        .map(|v| (v.x.into(), v.y.into()))
        .collect();

    let mut constraints: Vec<(VertexId, VertexId)> = Vec::new();
    for edge in top_mesh_builder.constraints().iter() {
        constraints.push((edge.from, edge.to));
    }

    let triangulation = cdt::triangulate_with_edges(&triangulation_vertices, &constraints).unwrap();

    //TODO: get the factor from triangulation
    let (mut x_min, mut y_min, mut x_max, mut y_max) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for vertex in planar_vertices.iter() {
        if vertex.x < x_min {
            x_min = vertex.x;
        }
        if vertex.x > x_max {
            x_max = vertex.x;
        }
        if vertex.y < y_min {
            y_min = vertex.y;
        }
        if vertex.y > y_max {
            y_max = vertex.y;
        }
    }

    let scale_factor: f32 = (x_max - x_min).max(y_max - y_min);

    // Apply normals and uv to sliced vertices
    for (id, vertex) in top_mesh_builder
        .sliced_vertices_mut()
        .iter_mut()
        .enumerate()
    {
        // Vertices inside the triangulation have been normalized, it is requiered to multiply by the scale factor:
        let pos = triangulation_vertices[id];
        let uv = Vec2::new((pos.0 as f32) * scale_factor, (pos.1 as f32) * scale_factor);

        *vertex.normal_mut() = -plane.normal();
        *vertex.uv_mut() = uv;

        // Reverse the normal for the bottom cut face
        *bottom_mesh_builder.sliced_vertices_mut()[id].normal_mut() = plane.normal();
        *bottom_mesh_builder.sliced_vertices_mut()[id].uv_mut() = uv;
    }

    // We need to add the sliced triangles after the non sliced triangles
    let top_slice = top_mesh_builder.vertices().len();
    let bottom_slice = bottom_mesh_builder.vertices().len();
    for triangle in triangulation.iter() {
        let t = *triangle;

        // We need to change the orientation of the triangles for one of the cut face:
        top_mesh_builder.push_triangle(top_slice + t.0, top_slice + t.1, top_slice + t.2);
        bottom_mesh_builder.push_triangle(
            bottom_slice + t.0,
            bottom_slice + t.2,
            bottom_slice + t.1,
        );
    }
}

fn transform_to_2d_planar_coordinate_system(
    vertices: &mut Vec<Vec3A>,
    plane_normal: Vec3A,
) -> Vec<Vec2> {
    // Create a base, using the first two vertices as the first base vector and plane_normal as the second
    let basis_1 = (vertices[0] - vertices[1]).normalize();
    // basis_3 is already normalized since basis_1 and plane_normal are normalized and orthogonal
    let basis_3 = basis_1.cross(plane_normal);

    // Project every vertices into the base B
    let mut vertices_2d = Vec::with_capacity(vertices.len());
    for vertex in vertices {
        vertices_2d.push(Vec2::new(vertex.dot(basis_1), vertex.dot(basis_3)));
    }
    vertices_2d
}

pub fn split_triangles(
    mesh_builder: &mut MeshBuilder,
    plane: Plane,
    side: &Vec<CutDirection>,
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
) {
    // Access to the mesh's triangles
    let triangles = mesh_builder.triangles();

    for (index, _) in triangles.iter().enumerate().step_by(3) {
        // Vertices of the current triangle
        let vertex_indexe_a = triangles[index];
        let vertex_indexe_b = triangles[index + 1];
        let vertex_indexe_c = triangles[index + 2];

        // If the triangle is completly above the slicing plane
        if side[vertex_indexe_a] == CutDirection::Top
            && side[vertex_indexe_b] == CutDirection::Top
            && side[vertex_indexe_c] == CutDirection::Top
        {
            top_mesh_builder.push_mapped_triangle(
                vertex_indexe_a,
                vertex_indexe_b,
                vertex_indexe_c,
            );
        }
        // If the triangle is completly bellow the slicing plane
        else if side[vertex_indexe_a] == CutDirection::Bottom
            && side[vertex_indexe_b] == CutDirection::Bottom
            && side[vertex_indexe_c] == CutDirection::Bottom
        {
            bottom_mesh_builder.push_mapped_triangle(
                vertex_indexe_a,
                vertex_indexe_b,
                vertex_indexe_c,
            );
        }
        // The triangle is intersecting the slicing plane:
        else {
            let mut vertex_indexes: [VertexId; 3] = [0, 0, 0];
            let mut two_edges_on_top = false;

            //Two vertices above and one below:
            if side[vertex_indexe_a] == CutDirection::Top
                && side[vertex_indexe_b] == CutDirection::Top
                && side[vertex_indexe_c] == CutDirection::Bottom
            {
                vertex_indexes = [vertex_indexe_a, vertex_indexe_b, vertex_indexe_c];
                two_edges_on_top = true;
            }

            if side[vertex_indexe_a] == CutDirection::Top
                && side[vertex_indexe_b] == CutDirection::Bottom
                && side[vertex_indexe_c] == CutDirection::Top
            {
                vertex_indexes = [vertex_indexe_c, vertex_indexe_a, vertex_indexe_b];
                two_edges_on_top = true;
            }

            if side[vertex_indexe_a] == CutDirection::Bottom
                && side[vertex_indexe_b] == CutDirection::Top
                && side[vertex_indexe_c] == CutDirection::Top
            {
                vertex_indexes = [vertex_indexe_b, vertex_indexe_c, vertex_indexe_a];
                two_edges_on_top = true;
            }

            // Two vertices below and one above:
            if side[vertex_indexe_a] == CutDirection::Top
                && side[vertex_indexe_b] == CutDirection::Bottom
                && side[vertex_indexe_c] == CutDirection::Bottom
            {
                vertex_indexes = [vertex_indexe_b, vertex_indexe_c, vertex_indexe_a];
            }

            if side[vertex_indexe_a] == CutDirection::Bottom
                && side[vertex_indexe_b] == CutDirection::Top
                && side[vertex_indexe_c] == CutDirection::Bottom
            {
                vertex_indexes = [vertex_indexe_c, vertex_indexe_a, vertex_indexe_b];
            }

            if side[vertex_indexe_a] == CutDirection::Bottom
                && side[vertex_indexe_b] == CutDirection::Bottom
                && side[vertex_indexe_c] == CutDirection::Top
            {
                vertex_indexes = [vertex_indexe_a, vertex_indexe_b, vertex_indexe_c];
            }

            // Subdivide the triangle into three new triangles
            split_small_triangles(
                plane,
                top_mesh_builder,
                bottom_mesh_builder,
                vertex_indexes,
                two_edges_on_top,
                &mut mesh_builder.clone(),
            );
        }
    }
}

fn spawn_fragment(
    meshes: Vec<Mesh>,
    pos: Vec3A,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    commands: &mut Commands,
) {
    //Spawn the fragment for each mesh
    for mesh in meshes {
        let mesh_handle = meshes_assets.add(mesh.clone());
        spawn_fragment_internal(&mesh, &mesh_handle, materials, commands, pos.into());
    }
}

fn spawn_fragment_internal(
    fragment_mesh: &Mesh,
    fragment_mesh_handle: &Handle<Mesh>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    commands: &mut Commands,
    _pos: Vec3,
) {
    // Spawn fragment, and applying physic to it
    commands.spawn((
        PbrBundle {
            mesh: fragment_mesh_handle.clone(),
            transform: Transform::from_xyz(5., 20., 0.0),
            material: materials.add(Color::rgb_u8(124, 144, 255)),
            ..default()
        },
        Wireframe,
        WireframeColor {
            color: Color::GREEN,
        },
        RigidBody::Dynamic,
        Collider::from_bevy_mesh(fragment_mesh, &ComputedColliderShape::ConvexHull).unwrap(),
        ActiveCollisionTypes::default(),
        Friction::coefficient(0.7),
        Restitution::coefficient(0.05),
        ColliderMassProperties::Density(2.0),
    ));
}

/// if two edges on top:
///```text
///  1-----------------2
///   |              |
///     |          |
///   --13|-------| 23---- plane
///         |   |
///           |
///           3
///```
///
/// if two edges bellow:
///```text
///           3
///           |
///         |   |
///   --23|-------| 13---- plane
///     |           |
///   |               |
///  2------------------1
///```
fn split_small_triangles(
    plane: Plane,
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
    vertex_indexes: [VertexId; 3],
    two_edges_on_top: bool,
    mesh_builder: &mut MeshBuilder,
) {
    // Vertices of the current triangle crossed by the slicing plane
    let v1 = mesh_builder.vertices()[vertex_indexes[0]];
    let v2 = mesh_builder.vertices()[vertex_indexes[1]];
    let v3 = mesh_builder.vertices()[vertex_indexes[2]];

    let interection_result_13;
    let intersection_result_23;

    // a*b is not equal to b*a for floats, so we need to take in count the edges orientation when calculating the plane-edges intersections
    if two_edges_on_top {
        interection_result_13 = find_intersection_line_plane_bis_bis(
            v1.pos(),
            v3.pos(),
            plane.origin(),
            plane.normal(),
        );
        intersection_result_23 = find_intersection_line_plane_bis_bis(
            v2.pos(),
            v3.pos(),
            plane.origin(),
            plane.normal(),
        );
    } else {
        interection_result_13 = find_intersection_line_plane_bis_bis(
            v3.pos(),
            v1.pos(),
            plane.origin(),
            plane.normal(),
        );
        intersection_result_23 = find_intersection_line_plane_bis_bis(
            v3.pos(),
            v2.pos(),
            plane.origin(),
            plane.normal(),
        );
    }

    // Check if the two edges of the triangle are crossed
    match (interection_result_13, intersection_result_23) {
        // Check if the cut plane do intersect the triangle
        (None, None) => (),
        (None, Some(_)) => (),
        (Some(_), None) => (),
        (Some((v13, s13)), Some((v23, s23))) => {
            // /!\ Interpolate normals and UV coordinates
            let norm13 = (v1.normal() + s13 * (v3.normal() - v1.normal())).normalize();
            let norm23 = (v2.normal() + s23 * (v3.normal() - v2.normal())).normalize();
            let uv13 = v1.uv() + s13 * (v3.uv() - v1.uv());
            let uv23: Vec2 = v2.uv() + s23 * (v3.uv() - v2.uv());

            // Adding the new vertices to each mesh builders
            top_mesh_builder.add_sliced_vertex(v13, uv13, norm13);
            top_mesh_builder.add_sliced_vertex(v23, uv23, norm23);

            bottom_mesh_builder.add_sliced_vertex(v13, uv13, norm13);
            bottom_mesh_builder.add_sliced_vertex(v23, uv23, norm23);

            // Ids of the vertices for each mesh builders
            let id13_top = top_mesh_builder.vertices().len() - 2;
            let id23_top = top_mesh_builder.vertices().len() - 1;
            let id13_bottom = top_mesh_builder.vertices().len() - 2;
            let id23_bottom = top_mesh_builder.vertices().len() - 1;

            if two_edges_on_top {
                // Add two triangles on top
                top_mesh_builder.push_triangle(
                    id23_top,
                    id13_top,
                    top_mesh_builder.index_map()[vertex_indexes[1]],
                );
                top_mesh_builder.push_triangle(
                    id13_top,
                    top_mesh_builder.index_map()[vertex_indexes[0]],
                    top_mesh_builder.index_map()[vertex_indexes[1]],
                );

                // And one to the bottom
                bottom_mesh_builder.push_triangle(
                    bottom_mesh_builder.index_map()[vertex_indexes[2]],
                    id13_bottom,
                    id23_bottom,
                );

                let top_sliced_vertices_nbr = top_mesh_builder.sliced_vertices().len();
                let bottom_sliced_vertices_nbr = bottom_mesh_builder.sliced_vertices().len();

                // Need to be carfull with constrained edges orientation:
                top_mesh_builder.constraints().push(Edge::new(
                    top_sliced_vertices_nbr - 2,
                    top_sliced_vertices_nbr - 1,
                ));
                bottom_mesh_builder.constraints().push(Edge::new(
                    bottom_sliced_vertices_nbr - 1,
                    bottom_sliced_vertices_nbr - 2,
                ));
            } else {
                // Add two triangles bellow
                bottom_mesh_builder.push_triangle(
                    bottom_mesh_builder.index_map()[vertex_indexes[0]],
                    bottom_mesh_builder.index_map()[vertex_indexes[1]],
                    id13_bottom,
                );

                bottom_mesh_builder.push_triangle(
                    bottom_mesh_builder.index_map()[vertex_indexes[1]],
                    id23_bottom,
                    id13_bottom,
                );

                //And one on top
                top_mesh_builder.push_triangle(
                    id13_top,
                    id23_top,
                    top_mesh_builder.index_map()[vertex_indexes[2]],
                );

                let top_sliced_vertices_nbr = top_mesh_builder.sliced_vertices().len();
                let bottom_sliced_vertices_nbr = bottom_mesh_builder.sliced_vertices().len();

                // Need to be carfull with constrained edges orientation:
                top_mesh_builder.constraints().push(Edge::new(
                    top_sliced_vertices_nbr - 1,
                    top_sliced_vertices_nbr - 2,
                ));
                bottom_mesh_builder.constraints().push(Edge::new(
                    bottom_sliced_vertices_nbr - 2,
                    bottom_sliced_vertices_nbr - 1,
                ));
            }
        }
    }
}
