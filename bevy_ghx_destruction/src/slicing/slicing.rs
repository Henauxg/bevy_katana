use bevy::{
    asset::{Assets, Handle}, ecs::system::{Commands, ResMut}, log::info, math::{Vec2, Vec3, Vec3A}, pbr::{PbrBundle, StandardMaterial}, render::{color::Color, mesh::Mesh}, transform::components::Transform, utils::default
};
use bevy_rapier3d::{
    dynamics::RigidBody,
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
};

use crate::{
    types::{CutDirection, MeshBuilder, MeshBuilderVertex, Plane},
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

    // execute the slice
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
    spawn_fragments(
        vec![top_mesh, bottom_mesh],
        plane.origin(),
        materials,
        meshes_assets,
        commands,
    );
}

pub fn compute_slice(plane: Plane, mesh: &Mesh) -> (Mesh, Mesh) {
    // Convert mesh into mesh mapping
    let mut mesh_mapping = MeshBuilder::mesh_mapping_from_mesh(mesh);

    info!("mesh mapping {:?}", mesh_mapping);

    // Split in half the mesh mapping into top and bottom mesh mappings
    let (top_mesh_mapping, bottom_mesh_mapping) = cut_mesh_mapping(plane, &mut mesh_mapping);

    info!("top_mesh_mapping triangle{:?}",top_mesh_mapping.triangles());

    (
        top_mesh_mapping.create_mesh(),
        bottom_mesh_mapping.create_mesh(),
    )
}

fn cut_mesh_mapping(plane: Plane, mesh_mapping: &mut MeshBuilder) -> (MeshBuilder, MeshBuilder) {
    let mapping_clone = mesh_mapping.clone();

    // Track the position of the vertices along the slice plane
    let mut sides = vec![CutDirection::Top; mapping_clone.vertices().len()];

    // Creating the index buffers for the top/bottom mesh mappings
    let mut top_mesh_builder = MeshBuilder::new_from(&mapping_clone);
    let mut bottom_mesh_builder = MeshBuilder::new_from(&mapping_clone);

   

    for (index, _) in mapping_clone.vertices().iter().enumerate() {
        
        let vertex = mapping_clone.vertices()[index];
        sides[index] = is_above_plane(vertex.pos(), plane);
        if sides[index] == CutDirection::Top {
            info!("index top {}", index);
            top_mesh_builder.add_vertex(vertex, index);
        } else {
            info!("index bottom{}", index);
            bottom_mesh_builder.add_vertex(vertex, index);
        }
    }

    info!("top_mesh_builder{:?}",top_mesh_builder.index_map());
    info!("bottom_mesh_builder{:?}",bottom_mesh_builder.index_map());

    // Split in half the mesh mapping, by spliting into new triangles all triangles intersecting the slice plane
    split_triangles(
        mesh_mapping,
        plane,
        &sides,
        &mut top_mesh_builder,
        &mut bottom_mesh_builder,
    );

    // Fill the cut face
    fill_cut_face(
        &plane,
        &mut top_mesh_builder,
        &mut bottom_mesh_builder,
        // &cut_face_vertices,
        // &constrained_edges,
        // &mut top_indexes,
        // &mut bottom_indexes,
        // &mesh_mapping,
    );

    (top_mesh_builder, bottom_mesh_builder)
}

//todo: clean with our triangulation
fn fill_cut_face(
    plane: &Plane,
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
    // cut_face_vertices: &Vec<[f32; 3]>,
    // constrained_edges: &HashSet<Edge>,
    // index_buffer_top_slice: &mut Vec<VertexId>,
    // index_buffer_bottom_slice: &mut Vec<VertexId>,
    // mesh_mapping: &MeshBuilder,
) // -> (Vec<Vec2>, Vec<Vec3A>, Vec<Vec3A>)
{

    top_mesh_builder.shrink_sliced_vertices();

    let mut vertices = Vec::new();
    for vertex in top_mesh_builder.vertices().iter() {
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

    let triangulation = cdt::triangulate_with_edges(&triangulation_vertices, &constraints);

    // let mut vertices: Vec<Vec3A> = cut_face_vertices
    //     .iter()
    //     .map(|v| Vec3A::from_array(*v))
    //     .collect();

    let (mut x_min, mut y_min, mut x_max, mut y_max) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);

    //TODO: get the factor from triangulation
    for vertex in vertices.iter() {
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

    // let mut uvs = Vec::new();
    // let mut top_normal = Vec::new();
    // let mut bottom_normal = Vec::new();

    for (id, vertex) in top_mesh_builder.sliced_vertices_mut().iter_mut().enumerate() {
        let pos = planar_vertices[id];
        let uv = Vec2::new(pos.x * (x_max - x_min), pos.y * (y_max - y_min));

        *vertex.normal_mut() = plane.normal();
        *vertex.uv_mut() = uv;

        *bottom_mesh_builder.sliced_vertices_mut()[id].normal_mut() =  -plane.normal();
        *bottom_mesh_builder.sliced_vertices_mut()[id].uv_mut() =  uv;

    }

    // let planar_vertices = transform_to_2d_planar_coordinate_system(&mut vertices, plane.normal());

    // let triangulation_vertices: Vec<(f64, f64)> = planar_vertices
    //     .iter()
    //     .map(|v| (v.x.into(), v.y.into()))
    //     .collect();

    // let edges: Vec<(usize, usize)> = constrained_edges.iter().map(|v| (v.from, v.to)).collect();

    // let triangulation = cdt::triangulate_with_edges(&triangulation_vertices, &edges).unwrap();

    // let cut_face: Vec<(VertexId, VertexId, VertexId)> = triangulation
    //     .iter()
    //     .map(|v| {
    //         (
    //             v.0 + mesh_mapping.initial_size(),
    //             v.1 + mesh_mapping.initial_size(),
    //             v.2 + mesh_mapping.initial_size(),
    //         )
    //     })
    //     .collect();

    let top_slice = top_mesh_builder.vertices().len();
    let bottom_slice = bottom_mesh_builder.vertices().len();
    for (id, triangle) in triangulation.iter().enumerate() {
        top_mesh_builder.push_triangle(
            top_slice + triangle[id].0,
            top_slice + triangle[id].1,
            top_slice + triangle[id].2,
        );

        bottom_mesh_builder.push_triangle(
            bottom_slice + triangle[id].0,
            bottom_slice + triangle[id].2,
            bottom_slice + triangle[id].1,
        );
    }

    // (uvs, top_normal, bottom_normal)
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
    mesh_mapping: &mut MeshBuilder,
    plane: Plane,
    side: &Vec<CutDirection>,
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
) // -> (HashSet<Edge>, Vec<[f32; 3]>)
{
    // let mut vertices_added: HashMap<[OrderedFloat<f32>; 3], VertexId> = HashMap::new();

    // let mut constrained_edges: HashSet<Edge> = HashSet::new();

    // let mut cut_face_vertices_id: Vec<VertexId> = Vec::new();

    // for (index, vertex) in mesh_mapping.vertex().iter().enumerate() {
    //     let key = [
    //         OrderedFloat(vertex[0]),
    //         OrderedFloat(vertex[1]),
    //         OrderedFloat(vertex[2]),
    //     ];

    //     let value = mesh_mapping.index()[index];
    //     vertices_added.insert(key, value);
    // }

    let triangles = mesh_mapping.triangles();
    for index in triangles.iter().step_by(3) {
        let vertex_indexe_1 = triangles[*index];
        let vertex_indexe_2 = triangles[*index + 1];
        let vertex_indexe_3 = triangles[*index + 2];

        // if triangle is completely above plane
        if side[vertex_indexe_1] == CutDirection::Top
            && side[vertex_indexe_2] == CutDirection::Top
            && side[vertex_indexe_3] == CutDirection::Top
        {
            top_mesh_builder.push_mapped_triangle(
                vertex_indexe_1,
                vertex_indexe_2,
                vertex_indexe_3,
            );
        }
        // if triangle is bellow plane
        else if side[vertex_indexe_1] == CutDirection::Bottom
            && side[vertex_indexe_2] == CutDirection::Bottom
            && side[vertex_indexe_3] == CutDirection::Bottom
        {
            bottom_mesh_builder.push_mapped_triangle(
                vertex_indexe_1,
                vertex_indexe_2,
                vertex_indexe_3,
            );
        }
        // the triangle is beeing intercept by the slide plane:
        else {
            let mut vertex_indexes: [VertexId; 3] = [0, 0, 0];
            let mut two_edges_on_top = false;

            //two vertices above and one below:
            if side[vertex_indexe_1] == CutDirection::Top
                && side[vertex_indexe_2] == CutDirection::Top
                && side[vertex_indexe_3] == CutDirection::Bottom
            {
                vertex_indexes = [vertex_indexe_3, vertex_indexe_1, vertex_indexe_2];
                two_edges_on_top = true;
            }

            if side[vertex_indexe_1] == CutDirection::Top
                && side[vertex_indexe_2] == CutDirection::Bottom
                && side[vertex_indexe_3] == CutDirection::Top
            {
                vertex_indexes = [vertex_indexe_2, vertex_indexe_3, vertex_indexe_1];
                two_edges_on_top = true;
            }

            if side[vertex_indexe_1] == CutDirection::Bottom
                && side[vertex_indexe_2] == CutDirection::Top
                && side[vertex_indexe_3] == CutDirection::Top
            {
                vertex_indexes = [vertex_indexe_1, vertex_indexe_2, vertex_indexe_3];
                two_edges_on_top = true;
            }

            // two vertices below and one above:
            if side[vertex_indexe_1] == CutDirection::Top
                && side[vertex_indexe_2] == CutDirection::Bottom
                && side[vertex_indexe_3] == CutDirection::Bottom
            {
                vertex_indexes = [vertex_indexe_1, vertex_indexe_2, vertex_indexe_3];
            }

            if side[vertex_indexe_1] == CutDirection::Bottom
                && side[vertex_indexe_2] == CutDirection::Top
                && side[vertex_indexe_3] == CutDirection::Bottom
            {
                vertex_indexes = [vertex_indexe_2, vertex_indexe_3, vertex_indexe_1];
            }

            if side[vertex_indexe_1] == CutDirection::Bottom
                && side[vertex_indexe_2] == CutDirection::Bottom
                && side[vertex_indexe_3] == CutDirection::Top
            {
                vertex_indexes = [vertex_indexe_3, vertex_indexe_1, vertex_indexe_2];
            }

            split_small_triangles(
                plane,
                top_mesh_builder,
                bottom_mesh_builder,
                vertex_indexes,
                two_edges_on_top,
                // &mut vertices_added,
                // &mut constrained_edges,
                // &mut cut_face_vertices_id,
                &mut mesh_mapping.clone(),
            );
        }
    }

    // (
    //     constrained_edges,
    //     to_vertex(&cut_face_vertices_id, mesh_mapping),
    // )
}

fn spawn_fragments(
    meshes: Vec<Mesh>,
    pos: Vec3A,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    commands: &mut Commands,
) {
    for mesh in meshes {
        let mesh_handle = meshes_assets.add(mesh.clone());
        spawn_fragment(&mesh, &mesh_handle, materials, commands, pos.into());
    }
}

fn spawn_fragment(
    fragment_mesh: &Mesh,
    fragment_mesh_handle: &Handle<Mesh>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    commands: &mut Commands,
    _pos: Vec3,
) {
    commands.spawn((
        PbrBundle {
            mesh: fragment_mesh_handle.clone(),
            transform: Transform::from_xyz(5., 20., 0.0),
            material: materials.add(Color::rgb_u8(124, 144, 255)),
            ..default()
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
///  2-----------------3
///   |              |
///     |          |
///   --13|-------| 12---- plane
///         |   |
///           |
///           1
///```
///
/// if two edges bellow:
///```text
///           1
///           |
///         |   |
///   --12|-------| 13---- plane
///     |           |
///   |               |
///  3------------------2
///```
fn split_small_triangles(
    plane: Plane,
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
    vertex_indexes: [VertexId; 3],
    two_edges_on_top: bool,
    // vertices_added: &mut HashMap<[OrderedFloat<f32>; 3], VertexId>,
    // constrained_edges: &mut HashSet<Edge>,
    // cut_face_vertices: &mut Vec<VertexId>,
    mesh_mapping: &mut MeshBuilder,
) {
    // vertices of the current triangle crossed by the slice plane
    let v1 = mesh_mapping.vertices()[vertex_indexes[0]];
    let v2 = mesh_mapping.vertices()[vertex_indexes[1]];
    let v3 = mesh_mapping.vertices()[vertex_indexes[2]];

    let interection_result_12;
    let intersection_result_13;

    // a*b is not equal to b*a for floats, so we need to take in count the edges orientation when calculating the plane-edges intersections
    if two_edges_on_top {
        interection_result_12 = find_intersection_line_plane_bis_bis(
            v1.pos(),
            v3.pos(),
            plane.origin(),
            plane.normal(),
        );
        intersection_result_13 = find_intersection_line_plane_bis_bis(
            v1.pos(),
            v2.pos(),
            plane.origin(),
            plane.normal(),
        );
    } else {
        interection_result_12 = find_intersection_line_plane_bis_bis(
            v3.pos(),
            v1.pos(),
            plane.origin(),
            plane.normal(),
        );
        intersection_result_13 = find_intersection_line_plane_bis_bis(
            v2.pos(),
            v1.pos(),
            plane.origin(),
            plane.normal(),
        );
    }

    // check if the two edges of the triangle are crossed
    match (interection_result_12, intersection_result_13) {
        // Check if the cut plane do intersect the triangle
        (None, None) => (),
        (None, Some(_)) => (),
        (Some(_), None) => (),
        (Some((v13, s13)), Some((v12, s12))) => {
            // /!\ Interpolate normals and UV coordinates
            let norm13 = (v1.normal() + s13 * (v3.normal() - v1.normal())).normalize();
            let norm12 = (v2.normal() + s12 * (v3.normal() - v2.normal())).normalize();
            let uv13 = v1.uv() + s13 * (v3.uv() - v1.uv());
            let uv12: Vec2 = v2.uv() + s12 * (v3.uv() - v2.uv());

            top_mesh_builder.add_sliced_vertex(v12, uv12, norm12);
            top_mesh_builder.add_sliced_vertex(v13, uv13, norm13);

            bottom_mesh_builder.add_sliced_vertex(v12, uv12, norm12);
            bottom_mesh_builder.add_sliced_vertex(v13, uv13, norm13);

            let id13_top = top_mesh_builder.vertices().len() - 2;
            let id12_top = top_mesh_builder.vertices().len() - 1;
            let id13_bottom = top_mesh_builder.vertices().len() - 2;
            let id12_bottom = top_mesh_builder.vertices().len() - 1;

            // // /!\ Add vertices/normals/uv for the intersection points to each mesh

            // let ordered_v13: [OrderedFloat<f32>; 3] = [
            //     OrderedFloat(v13.x),
            //     OrderedFloat(v13.y),
            //     OrderedFloat(v13.z),
            // ];

            // let ordered_v12 = [
            //     OrderedFloat(v12.x),
            //     OrderedFloat(v12.y),
            //     OrderedFloat(v12.z),
            // ];

            // // create the indices
            // let index13 = single_index(
            //     ordered_v13,
            //     mesh_mapping,
            //     vertices_added,
            //     cut_face_vertices,
            //     v13,
            //     uv13,
            //     norm13,
            // );
            // let index12 = single_index(
            //     ordered_v12,
            //     mesh_mapping,
            //     vertices_added,
            //     cut_face_vertices,
            //     v12,
            //     uv12,
            //     norm12,
            // );

            if two_edges_on_top {
                // add two triangles on top
                top_mesh_builder.push_triangle(
                    id12_top,
                    id13_top,
                    top_mesh_builder.index_map()[vertex_indexes[2]],
                );
                top_mesh_builder.push_triangle(
                    id13_top,
                    top_mesh_builder.index_map()[vertex_indexes[1]],
                    top_mesh_builder.index_map()[vertex_indexes[2]],
                );
                bottom_mesh_builder.push_triangle(
                    bottom_mesh_builder.index_map()[vertex_indexes[0]],
                    id13_bottom,
                    id12_bottom,
                );

                let top_sliced_vertices_nbr = top_mesh_builder.sliced_vertices().len();
                let bottom_sliced_vertices_nbr = bottom_mesh_builder.sliced_vertices().len();

                top_mesh_builder.constraints().push(Edge::new(
                    top_sliced_vertices_nbr - 2,
                    top_sliced_vertices_nbr - 1,
                ));
                bottom_mesh_builder.constraints().push(Edge::new(
                    bottom_sliced_vertices_nbr - 1,
                    bottom_sliced_vertices_nbr - 2,
                ));

                // constrained_edges.insert(Edge::new(
                //     index12 - mesh_mapping.initial_size(),
                //     index13 - mesh_mapping.initial_size(),
                // ));
            } else {
                // add two triangles bellow
                bottom_mesh_builder.push_triangle(
                    id12_bottom,
                    id13_bottom,
                    bottom_mesh_builder.index_map()[vertex_indexes[1]],
                );

                bottom_mesh_builder.push_triangle(
                    id12_bottom,
                    bottom_mesh_builder.index_map()[vertex_indexes[1]],
                    bottom_mesh_builder.index_map()[vertex_indexes[2]],
                );

                top_mesh_builder.push_triangle(
                    id13_top,
                    id12_top,
                    top_mesh_builder.index_map()[vertex_indexes[0]],
                );

                // constrained_edges.insert(Edge::new(
                //     index13 - mesh_mapping.initial_size(),
                //     index12 - mesh_mapping.initial_size(),
                // ));

                let top_sliced_vertices_nbr = top_mesh_builder.sliced_vertices().len();
                let bottom_sliced_vertices_nbr = bottom_mesh_builder.sliced_vertices().len();

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
