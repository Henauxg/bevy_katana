use bevy::{
    asset::{Assets, Handle},
    ecs::system::{Commands, ResMut},
    log::warn,
    math::{Vec3, Vec3A},
    pbr::{
        wireframe::{Wireframe, WireframeColor},
        PbrBundle, StandardMaterial,
    },
    render::{color::Color, mesh::Mesh},
    transform::components::Transform,
    utils::{default, HashMap},
};
use bevy_rapier3d::{
    dynamics::RigidBody,
    geometry::{
        ActiveCollisionTypes, Collider, ColliderMassProperties, ComputedColliderShape, Friction,
        Restitution,
    },
};
use glam::Vec2;

use crate::{
    types::{MeshBuilderVertex, Plane, PlaneSide, SlicedMesh},
    utils::{get_random_normalized_vec, is_above_plane, line_plane_intersection},
};
use ghx_constrained_delaunay::{
    constrained_triangulation::ConstrainedTriangulationConfiguration,
    triangulation::transform_to_2d_planar_coordinate_system,
    types::{Edge, IndexType, TriangleVertexIndex, VertexId},
};

pub fn fragment_mesh(mesh: &Mesh, iteration_count: u32) -> Vec<Mesh> {
    if iteration_count == 0 {
        return vec![mesh.clone()];
    }

    let mut buffer_1 = Vec::with_capacity(2_u32.pow(iteration_count) as usize);
    let mut buffer_2 = Vec::with_capacity(2_u32.pow(iteration_count) as usize);

    let mut buffer_to_cut = &mut buffer_1;
    let mut cut_buffer = &mut buffer_2;

    buffer_to_cut.push(mesh.clone());

    for _ in 0..iteration_count {
        cut_buffer.clear();

        cut_meshes(&buffer_to_cut, cut_buffer);

        // Swap buffer
        let tmp = cut_buffer;
        cut_buffer = buffer_to_cut;
        buffer_to_cut = tmp;
    }

    if iteration_count % 2 == 0 {
        buffer_1
    } else {
        buffer_2
    }
}

fn cut_meshes(buffer_to_cut: &Vec<Mesh>, cut_buffer: &mut Vec<Mesh>) {
    for mesh in buffer_to_cut.iter() {
        // Mesh center approximation
        let mesh_center = mesh.compute_aabb().unwrap().center;
        // Random normalized vector for the cut plane
        let normal_vec = get_random_normalized_vec();
        let plane = Plane::new(mesh_center, normal_vec);
        let frags = slice_mesh(plane, mesh);
        cut_buffer.extend(frags)
    }
}

/// Operate the slice on a mesh and spawn fragments
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

    // TODO Crashes triangulation
    // let normal_vec = Vec3A::X;

    // Plane from point and normal
    let plane = Plane::new(mesh_center, normal_vec);

    // Execute the slice
    let meshes = slice_mesh(plane, mesh);

    // Spawn the fragments from the meshes at the pos
    spawn_fragment(meshes, plane.origin(), materials, meshes_assets, commands);
}

/// Operate the slice on a mesh
pub fn slice_mesh(plane: Plane, mesh: &Mesh) -> Vec<Mesh> {
    let mut slice_mesh = SlicedMesh::from_bevy_mesh(mesh);

    // Split in half the mesh builder into top and bottom mesh builders
    match internal_slice_mesh(plane, &mut slice_mesh) {
        (None, None) => return Vec::new(),
        (None, Some(bottom_slice)) => return vec![bottom_slice.to_bevy_mesh()],
        (Some(top_slice), None) => return vec![top_slice.to_bevy_mesh()],
        (Some(top_slice), Some(bottom_slice)) => {
            return vec![top_slice.to_bevy_mesh(), bottom_slice.to_bevy_mesh()]
        }
    }
}

fn internal_slice_mesh(
    plane: Plane,
    mesh_to_slice: &mut SlicedMesh,
) -> (Option<SlicedMesh>, Option<SlicedMesh>) {
    // Track the position of the vertices along the slice plane
    let mut sides = Vec::with_capacity(
        mesh_to_slice.vertices().len() + mesh_to_slice.sliced_face_vertices().len(),
    );

    // Initializing the top/bottom mesh builders
    let mut top_slice = SlicedMesh::initialize_with(&mesh_to_slice);
    let mut bottom_slice = SlicedMesh::initialize_with(&mesh_to_slice);

    // Add the vertices from the sliced mesh to the top and bottom mesh builders
    for vertex in mesh_to_slice
        .vertices()
        .iter()
        // TODO Why do we have a different sliced_vertices buffer here ?
        .chain(mesh_to_slice.sliced_face_vertices().iter())
    {
        let side = is_above_plane(vertex.pos(), plane);
        sides.push(side);
        // Vertices are split from top to bottom. If a vertex on the slicing plane, it will be fixed later
        match side {
            PlaneSide::Top => top_slice.push_mapped_vertex(*vertex),
            PlaneSide::Bottom => bottom_slice.push_mapped_vertex(*vertex),
            PlaneSide::OnPlane => (),
        }
    }

    divide_triangles(
        mesh_to_slice,
        plane,
        &sides,
        &mut top_slice,
        &mut bottom_slice,
    );

    // TODO Could we easily know beforehand if the mesh intersects the slicing plane ?
    if top_slice.vertices().is_empty() {
        return (None, Some(bottom_slice));
    } else if bottom_slice.vertices().is_empty() {
        return (Some(top_slice), None);
    } else {
        // Fill the newly created sliced face
        fill_sliced_face(&plane, &mut top_slice, &mut bottom_slice);
        (Some(top_slice), Some(bottom_slice))
    }
}

//todo: clean with our triangulation
fn fill_sliced_face(
    plane: &Plane,
    top_mesh_builder: &mut SlicedMesh,
    bottom_mesh_builder: &mut SlicedMesh,
) {
    // // Erease duplicated sliced vertices
    // top_mesh_builder.shrink_sliced_vertices();
    // bottom_mesh_builder.shrink_sliced_vertices();

    // Apply the triangulation to the top cut face. No need to do it for the bottom cute face since they both
    // share the same vertices, we will just reverse the normal.
    // TODO Would not need a copy if we had a SoA
    let vertices = top_mesh_builder
        .sliced_face_vertices()
        .iter()
        .map(|v| v.pos())
        .collect();
    let planar_vertices = transform_to_2d_planar_coordinate_system(&vertices, -plane.normal());
    let triangulation = ghx_constrained_delaunay::constrained_triangulation_from_2d_vertices(
        &planar_vertices,
        top_mesh_builder.constraints(),
        ConstrainedTriangulationConfiguration::default(),
    );

    // Apply normals and uv to sliced vertices
    for (index, (v_top, v_bottom)) in top_mesh_builder
        .sliced_face_vertices_mut()
        .iter_mut()
        .zip(bottom_mesh_builder.sliced_face_vertices_mut())
        .enumerate()
    {
        // Vertices inside the triangulation have been normalized, it is requiered to multiply by the scale factor:
        let pos = planar_vertices[index];
        let uv = Vec2::new((pos.x as f32), (pos.y as f32)); // TODO Redo uv mapping. May use scale factor from triangulation to be in [0,1] ?
        *v_top.normal_mut() = -plane.normal();
        *v_top.uv_mut() = uv;
        // Reverse the normal for the bottom cut face
        *v_bottom.normal_mut() = plane.normal();
        *v_bottom.uv_mut() = uv;
    }

    // TODO Why can't we merge the two verts buffers already ?
    // We need to add the sliced triangles after the non sliced triangles
    let top_slice_verts_count = top_mesh_builder.vertices().len() as IndexType;
    let bottom_slice_verts_count = bottom_mesh_builder.vertices().len() as IndexType;
    for t in triangulation.triangles.iter() {
        top_mesh_builder.push_triangle(
            top_slice_verts_count + t[0],
            top_slice_verts_count + t[1],
            top_slice_verts_count + t[2],
        );
        // We need to change the orientation of the triangles for one of the cut face:
        bottom_mesh_builder.push_triangle(
            bottom_slice_verts_count + t[0],
            bottom_slice_verts_count + t[2],
            bottom_slice_verts_count + t[1],
        );
    }
}

pub enum TrianglePlaneIntersection {
    OneVertexTop(TriangleVertexIndex),
    OneVertexBottom(TriangleVertexIndex),
    OneEdgeTop((TriangleVertexIndex, TriangleVertexIndex)),
    OneEdgeBottom((TriangleVertexIndex, TriangleVertexIndex)),
    OneVertexOneCrossedEdge {
        vertex: TriangleVertexIndex,
        edge: (TriangleVertexIndex, TriangleVertexIndex),
    },
    TwoCrossedEdgesTopSide {
        verts: [TriangleVertexIndex; 3],
    },
    TwoCrossedEdgesBottomSide {
        verts: [TriangleVertexIndex; 3],
    },
    FlatTriangle,
}

// u32 with  3 side info packed on 6 bits ?
// const TRIANGLES_SIDES_ARRAY: [TrianglePlaneIntersectionC; 42] = [];
use lazy_static::lazy_static;
lazy_static! {
    static ref TRIANGLE_PLANE_INTERSECTIONS: HashMap<(PlaneSide,PlaneSide,PlaneSide), TrianglePlaneIntersection> = {
         HashMap::from([
            // ((PlaneSide::Top, PlaneSide::Top, PlaneSide::Top),0.0),
            // ((PlaneSide::Bottom, PlaneSide::Bottom, PlaneSide::Bottom),TrianglePlaneIntersectionC::Edges),
           ((PlaneSide::Top, PlaneSide::OnPlane, PlaneSide::Top),TrianglePlaneIntersection::OneVertexTop(1)),
           ((PlaneSide::OnPlane, PlaneSide::Top, PlaneSide::Top),TrianglePlaneIntersection::OneVertexTop(0)),
           ((PlaneSide::Top, PlaneSide::Top, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexTop(2)),

           ((PlaneSide::Bottom, PlaneSide::Bottom, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexBottom(2)),
           ((PlaneSide::Bottom, PlaneSide::OnPlane, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexBottom(1)),
           ((PlaneSide::OnPlane, PlaneSide::Bottom, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexBottom(0)),

           ((PlaneSide::Top, PlaneSide::OnPlane, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeTop((1,2))),
           ((PlaneSide::OnPlane, PlaneSide::Top, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeTop((0,2))),
           ((PlaneSide::OnPlane, PlaneSide::OnPlane, PlaneSide::Top),TrianglePlaneIntersection::OneEdgeTop((1,1))),

           ((PlaneSide::OnPlane, PlaneSide::Bottom, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeBottom((0,2))),
           ((PlaneSide::Bottom, PlaneSide::OnPlane, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeBottom((1,2))),
           ((PlaneSide::OnPlane, PlaneSide::OnPlane, PlaneSide::Bottom),TrianglePlaneIntersection::OneEdgeBottom((0,1))),

           // Edge indexes order is important for those cases. Top -> Bottom
           ((PlaneSide::Top, PlaneSide::OnPlane, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:1, edge: (0,2)}),
           ((PlaneSide::Top, PlaneSide::Bottom, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:2, edge: (0,1)}),
           ((PlaneSide::Bottom, PlaneSide::Top, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:2, edge: (1,0)}),
           ((PlaneSide::Bottom, PlaneSide::OnPlane, PlaneSide::Top),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:1, edge: (2,0)}),
           ((PlaneSide::OnPlane, PlaneSide::Top, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:0, edge: (1,2)}),
           ((PlaneSide::OnPlane, PlaneSide::Bottom, PlaneSide::Top),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:0, edge: (2,1)}),

           ((PlaneSide::Top, PlaneSide::Top, PlaneSide::Bottom),TrianglePlaneIntersection::TwoCrossedEdgesTopSide{verts:[0,1,2]}),
           ((PlaneSide::Top, PlaneSide::Bottom, PlaneSide::Top),TrianglePlaneIntersection::TwoCrossedEdgesTopSide{verts:[2,0,1]}),
           ((PlaneSide::Bottom, PlaneSide::Top, PlaneSide::Top),TrianglePlaneIntersection::TwoCrossedEdgesTopSide{verts:[1,2,0]}),

           ((PlaneSide::Top, PlaneSide::Bottom, PlaneSide::Bottom),TrianglePlaneIntersection::TwoCrossedEdgesBottomSide{verts:[1,2,0]}),
           ((PlaneSide::Bottom, PlaneSide::Top, PlaneSide::Bottom),TrianglePlaneIntersection::TwoCrossedEdgesBottomSide{verts:[2,0,1]}),
           ((PlaneSide::Bottom, PlaneSide::Bottom, PlaneSide::Top),TrianglePlaneIntersection::TwoCrossedEdgesBottomSide{verts:[0,1,2]}),

           ((PlaneSide::OnPlane, PlaneSide::OnPlane, PlaneSide::OnPlane),TrianglePlaneIntersection::FlatTriangle),
        ])
    };
}

/// Divide triangles among the bottom and top slices
pub fn divide_triangles(
    original_mesh: &mut SlicedMesh,
    plane: Plane,
    sides: &Vec<PlaneSide>,
    top_slice: &mut SlicedMesh,
    bottom_slice: &mut SlicedMesh,
) {
    for triangle in original_mesh.indices().chunks_exact(3) {
        let (v1, v2, v3) = (triangle[0], triangle[1], triangle[2]);
        // TODO Optimization: Those two cases should eb the more common ones. See if they need to be kept separated
        // If the triangle is completly above the slicing plane
        if sides[v1 as usize] == PlaneSide::Top
            && sides[v2 as usize] == PlaneSide::Top
            && sides[v3 as usize] == PlaneSide::Top
        {
            top_slice.push_mapped_triangle(v1, v2, v3);
            continue;
        }
        // If the triangle is completly bellow the slicing plane
        if sides[v1 as usize] == PlaneSide::Bottom
            && sides[v2 as usize] == PlaneSide::Bottom
            && sides[v3 as usize] == PlaneSide::Bottom
        {
            bottom_slice.push_mapped_triangle(v1, v2, v3);
            continue;
        }

        // TODO Optimization: Could be sped up if needed (could pack the enum and/or the 3 sides)
        match TRIANGLE_PLANE_INTERSECTIONS
            .get(&(sides[v1 as usize], sides[v2 as usize], sides[v3 as usize]))
            .unwrap()
        {
            TrianglePlaneIntersection::OneVertexOneCrossedEdge { vertex, edge } => {
                split_on_plane_triangle(
                    top_slice,
                    bottom_slice,
                    [
                        triangle[*vertex as usize],
                        triangle[edge.0 as usize],
                        triangle[edge.1 as usize],
                    ],
                    original_mesh,
                    plane,
                );
            }
            // TODO The two TwoCrossedEdgesTopSide cases could/should be abstracted away into one. Same for `split_intersected_triangle`
            TrianglePlaneIntersection::TwoCrossedEdgesTopSide { verts } => {
                let verts_indexes = [
                    triangle[verts[0] as usize],
                    triangle[verts[1] as usize],
                    triangle[verts[2] as usize],
                ];
                split_intersected_triangle(
                    plane,
                    top_slice,
                    bottom_slice,
                    verts_indexes,
                    true,
                    &original_mesh,
                );
            }
            TrianglePlaneIntersection::TwoCrossedEdgesBottomSide { verts } => {
                let verts_indexes = [
                    triangle[verts[0] as usize],
                    triangle[verts[1] as usize],
                    triangle[verts[2] as usize],
                ];
                split_intersected_triangle(
                    plane,
                    top_slice,
                    bottom_slice,
                    verts_indexes,
                    false,
                    &original_mesh,
                );
            }
            TrianglePlaneIntersection::OneVertexTop(vertex) => {
                // TODO Add vertex to sliced face, we don't care yet about uvs and normals. No need to add vertex to original mesh since it is not a new vertex
                top_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2])
            }
            TrianglePlaneIntersection::OneVertexBottom(vertex) => {
                // TODO Add vertex to sliced face
                bottom_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2])
            }
            TrianglePlaneIntersection::OneEdgeBottom(edge) => {
                // TODO Add edges vertices to sliced face
                bottom_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2])
            }
            TrianglePlaneIntersection::OneEdgeTop(edge) => {
                // TODO Add edges vertices to sliced face
                top_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2])
            }
            TrianglePlaneIntersection::FlatTriangle => {
                // TODO Handle flat triangles ont the slicing plane. We may just discard them (indices) but keep their vertices
                warn!("Handle flat triangles ont the slicing plane. We may just discard them (indices) but keep their vertices");
            }
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

pub fn on_plane_triangle(
    top_mesh_builder: &mut SlicedMesh,
    bottom_mesh_builder: &mut SlicedMesh,
    side: &Vec<PlaneSide>,
    vertices: [u64; 3],
    on_plane: Vec<usize>,
    mesh_builder: &SlicedMesh,
    plane: Plane,
) {
    if on_plane.len() == 1 {
        let out_of_plane_vertices: Vec<u64> = vertices
            .iter()
            .enumerate()
            .filter_map(|(j, &v)| if j != on_plane[0] { Some(v) } else { None })
            .collect();

        if side[out_of_plane_vertices[0] as usize] == PlaneSide::Top
            && side[out_of_plane_vertices[1] as usize] == PlaneSide::Top
        {
            top_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        } else if side[out_of_plane_vertices[0] as usize] == PlaneSide::Bottom
            && side[out_of_plane_vertices[1] as usize] == PlaneSide::Bottom
        {
            bottom_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        } else if side[out_of_plane_vertices[0] as usize] == PlaneSide::Bottom
            && side[out_of_plane_vertices[1] as usize] == PlaneSide::Top
        {
            split_on_plane_triangle(
                top_mesh_builder,
                bottom_mesh_builder,
                [
                    vertices[on_plane[0]],
                    out_of_plane_vertices[1],
                    out_of_plane_vertices[0],
                ],
                mesh_builder,
                plane,
            );
        } else {
            split_on_plane_triangle(
                top_mesh_builder,
                bottom_mesh_builder,
                [
                    vertices[on_plane[0]],
                    out_of_plane_vertices[0],
                    out_of_plane_vertices[1],
                ],
                mesh_builder,
                plane,
            );
        }
    } else if on_plane.len() == 2 {
        let out_of_plane_vertex: Vec<u64> = vertices
            .iter()
            .enumerate()
            .filter_map(|(j, &v)| {
                if j != on_plane[0] || j != on_plane[1] {
                    Some(v)
                } else {
                    None
                }
            })
            .collect();

        if side[out_of_plane_vertex[0] as usize] == PlaneSide::Top {
            top_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        } else {
            bottom_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        }
    } else {
        // TODO Handle flat triangles ont the slicing plane. We may just discard them (indices) but keep their vertices
        warn!("Handle flat triangles ont the slicing plane. We may just discard them (indices) but keep their vertices");
    }
}

///            * top
///         /  |
///       /    |
///    --*-----m-- plane
///       \    |
///         \  |
///            * bottom
///
/// The vertices are given as [on plane, top, bottom]:
fn split_on_plane_triangle(
    top_mesh_builder: &mut SlicedMesh,
    bottom_mesh_builder: &mut SlicedMesh,
    vertices: [u64; 3],
    mesh_builder: &SlicedMesh,
    plane: Plane,
) {
    let mut v = vec![MeshBuilderVertex::new(Vec3A::ZERO, Vec2::ZERO, Vec3A::ZERO); 3];

    for (i, vertex_id) in vertices.iter().enumerate() {
        let v_id = *vertex_id as usize;
        if v_id < mesh_builder.vertices().len() {
            v[i] = mesh_builder.vertices()[v_id as usize];
        } else {
            v[i] =
                mesh_builder.sliced_face_vertices()[v_id as usize - mesh_builder.vertices().len()];
        }
    }

    let (m, s) =
        line_plane_intersection(v[1].pos(), v[2].pos(), plane.origin(), plane.normal()).unwrap();
    let norm = (v[1].normal() + s * (v[2].normal() - v[1].normal())).normalize();
    let uv: Vec2 = v[1].uv() + s * (v[2].uv() - v[1].uv());

    top_mesh_builder.add_sliced_vertex(m, uv, norm);
    bottom_mesh_builder.add_sliced_vertex(m, uv, norm);

    let id_top = top_mesh_builder.vertices().len() as VertexId - 1;
    let id_bottom = bottom_mesh_builder.vertices().len() as VertexId - 1;

    top_mesh_builder.push_triangle(
        id_top,
        top_mesh_builder.index_map()[vertices[0] as usize],
        top_mesh_builder.index_map()[vertices[1] as usize],
    );

    top_mesh_builder.push_triangle(
        id_bottom,
        top_mesh_builder.index_map()[vertices[2] as usize],
        top_mesh_builder.index_map()[vertices[0] as usize],
    );

    let top_sliced_vertices_nbr = top_mesh_builder.sliced_face_vertices().len() as VertexId;
    let bottom_sliced_vertices_nbr = bottom_mesh_builder.sliced_face_vertices().len() as VertexId;

    // Need to be careful with constrained edges orientation:
    top_mesh_builder.constraints_mut().push(Edge::new(
        top_sliced_vertices_nbr - 2,
        top_sliced_vertices_nbr - 1,
    ));
    bottom_mesh_builder.constraints_mut().push(Edge::new(
        bottom_sliced_vertices_nbr - 1,
        bottom_sliced_vertices_nbr - 2,
    ));
}

/// if two vertices on top:
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
/// if two vertices below:
///```text
///           3
///           |
///         |   |
///   --23|-------| 13---- plane
///     |           |
///   |               |
///  2------------------1
///```
fn split_intersected_triangle(
    plane: Plane,
    top_mesh_builder: &mut SlicedMesh,
    bottom_mesh_builder: &mut SlicedMesh,
    verts_indexes: [VertexId; 3],
    two_vertices_on_top: bool,
    mesh_builder: &SlicedMesh,
) {
    let mut v = vec![MeshBuilderVertex::new(Vec3A::ZERO, Vec2::ZERO, Vec3A::ZERO); 3];

    for (i, vertex_id) in verts_indexes.iter().enumerate() {
        let v_id = *vertex_id as usize;
        if v_id < mesh_builder.vertices().len() {
            v[i] = mesh_builder.vertices()[v_id];
        } else {
            v[i] = mesh_builder.sliced_face_vertices()[v_id - mesh_builder.vertices().len()];
        }
    }

    let intersection_result_13;
    let intersection_result_23;

    // a*b is not equal to b*a for floats, so we need to take in count the edges orientation when calculating the plane-edges intersections
    if two_vertices_on_top {
        intersection_result_13 =
            line_plane_intersection(v[0].pos(), v[2].pos(), plane.origin(), plane.normal());
        intersection_result_23 =
            line_plane_intersection(v[1].pos(), v[2].pos(), plane.origin(), plane.normal());
    } else {
        intersection_result_13 =
            line_plane_intersection(v[2].pos(), v[0].pos(), plane.origin(), plane.normal());
        intersection_result_23 =
            line_plane_intersection(v[2].pos(), v[1].pos(), plane.origin(), plane.normal());
    }

    // Check if the two edges of the triangle are crossed
    match (intersection_result_13, intersection_result_23) {
        // Check if the cut plane intersects the triangle
        (None, None) => (),
        (None, Some(_)) => (),
        (Some(_), None) => (),
        (Some((v13, s13)), Some((v23, s23))) => {
            // /!\ Interpolate normals and UV coordinates
            let norm13 = (v[0].normal() + s13 * (v[2].normal() - v[0].normal())).normalize();
            let norm23 = (v[1].normal() + s23 * (v[2].normal() - v[1].normal())).normalize();
            let uv13 = v[0].uv() + s13 * (v[2].uv() - v[0].uv());
            let uv23: Vec2 = v[1].uv() + s23 * (v[2].uv() - v[1].uv());

            // Add the new vertices to each mesh builder
            top_mesh_builder.add_sliced_vertex(v13, uv13, norm13);
            top_mesh_builder.add_sliced_vertex(v23, uv23, norm23);

            bottom_mesh_builder.add_sliced_vertex(v13, uv13, norm13);
            bottom_mesh_builder.add_sliced_vertex(v23, uv23, norm23);

            // Ids of the vertices for each mesh builder
            let id13_top = top_mesh_builder.vertices().len() as VertexId - 2;
            let id23_top = top_mesh_builder.vertices().len() as VertexId - 1;
            let id13_bottom = bottom_mesh_builder.vertices().len() as VertexId - 2;
            let id23_bottom = bottom_mesh_builder.vertices().len() as VertexId - 1;

            if two_vertices_on_top {
                // Add two triangles on top
                top_mesh_builder.push_triangle(
                    id23_top,
                    id13_top,
                    top_mesh_builder.index_map()[verts_indexes[1] as usize],
                );
                top_mesh_builder.push_triangle(
                    id13_top,
                    top_mesh_builder.index_map()[verts_indexes[0] as usize],
                    top_mesh_builder.index_map()[verts_indexes[1] as usize],
                );

                // And one to the bottom
                bottom_mesh_builder.push_triangle(
                    bottom_mesh_builder.index_map()[verts_indexes[2] as usize],
                    id13_bottom,
                    id23_bottom,
                );

                let top_sliced_vertices_nbr =
                    top_mesh_builder.sliced_face_vertices().len() as VertexId;
                let bottom_sliced_vertices_nbr =
                    bottom_mesh_builder.sliced_face_vertices().len() as VertexId;

                // Need to be carfull with constrained edges orientation:
                top_mesh_builder.constraints_mut().push(Edge::new(
                    top_sliced_vertices_nbr - 2,
                    top_sliced_vertices_nbr - 1,
                ));
                bottom_mesh_builder.constraints_mut().push(Edge::new(
                    bottom_sliced_vertices_nbr - 1,
                    bottom_sliced_vertices_nbr - 2,
                ));
            } else {
                // Add two triangles bellow
                bottom_mesh_builder.push_triangle(
                    bottom_mesh_builder.index_map()[verts_indexes[0] as usize],
                    bottom_mesh_builder.index_map()[verts_indexes[1] as usize],
                    id13_bottom,
                );

                bottom_mesh_builder.push_triangle(
                    bottom_mesh_builder.index_map()[verts_indexes[1] as usize],
                    id23_bottom,
                    id13_bottom,
                );

                //And one on top
                top_mesh_builder.push_triangle(
                    id13_top,
                    id23_top,
                    top_mesh_builder.index_map()[verts_indexes[2] as usize],
                );

                let top_sliced_vertices_nbr =
                    top_mesh_builder.sliced_face_vertices().len() as VertexId;
                let bottom_sliced_vertices_nbr =
                    bottom_mesh_builder.sliced_face_vertices().len() as VertexId;

                // Need to be carfull with constrained edges orientation:
                top_mesh_builder.constraints_mut().push(Edge::new(
                    top_sliced_vertices_nbr - 1,
                    top_sliced_vertices_nbr - 2,
                ));
                bottom_mesh_builder.constraints_mut().push(Edge::new(
                    bottom_sliced_vertices_nbr - 2,
                    bottom_sliced_vertices_nbr - 1,
                ));
            }
        }
    }
}
