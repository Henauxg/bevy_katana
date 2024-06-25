use std::borrow::Borrow;

use bevy::render::mesh::Mesh as RenderMesh;
use bevy::{
    log::warn,
    math::{Vec3, Vec3A},
    utils::HashMap,
};
use ghx_constrained_delaunay::types::Vertex;
use glam::Vec2;

use crate::types::{SlicedMeshData, Submesh};
use crate::{
    types::{Plane, PlaneSide},
    utils::{edge_plane_intersection, get_random_normalized_vec},
};
use ghx_constrained_delaunay::{
    constrained_triangulation::ConstrainedTriangulationConfiguration,
    types::{Edge, TriangleVertexIndex, VertexId},
};

pub fn slice_bevy_mesh(plane: Plane, mesh: &RenderMesh) -> Vec<RenderMesh> {
    let mut sliced_mesh_data = SlicedMeshData::from_bevy_render_mesh(mesh);
    let mut initial_submesh = Submesh::from_bevy_render_mesh(mesh);

    let frags = match internal_slice_submesh(&mut initial_submesh, plane, &mut sliced_mesh_data) {
        None => vec![initial_submesh], // TODO may optimize the uncut case
        Some(frags) => frags.to_vec(), // TODO No to vec
    };

    frags
        .iter()
        .map(|f| f.to_bevy_render_mesh(&sliced_mesh_data))
        .collect()
}

pub fn slice_bevy_mesh_iterative(mesh: &RenderMesh, iteration_count: u32) -> Vec<RenderMesh> {
    if iteration_count == 0 {
        return vec![mesh.clone()];
    }

    let mut buffer_1 = Vec::with_capacity(2_u32.pow(iteration_count) as usize);
    let mut buffer_2 = Vec::with_capacity(2_u32.pow(iteration_count) as usize);

    let mut submeshes_to_slice = &mut buffer_1;
    let mut sliced_submeshes = &mut buffer_2;

    let mut sliced_mesh_data = SlicedMeshData::from_bevy_render_mesh(mesh);
    let initial_submesh = Submesh::from_bevy_render_mesh(mesh);

    submeshes_to_slice.push(initial_submesh);

    for _ in 0..iteration_count {
        internal_slice_submeshes(
            &mut submeshes_to_slice,
            sliced_submeshes,
            &mut sliced_mesh_data,
        );

        // Swap buffer
        let tmp = sliced_submeshes;
        sliced_submeshes = submeshes_to_slice;
        submeshes_to_slice = tmp;
    }

    // TODO Clean
    if iteration_count % 2 == 0 {
        buffer_1
            .iter()
            .map(|f| f.to_bevy_render_mesh(&sliced_mesh_data))
            .collect()
    } else {
        buffer_2
            .iter()
            .map(|f| f.to_bevy_render_mesh(&sliced_mesh_data))
            .collect()
    }
}

fn internal_slice_submeshes(
    submeshes_to_slice: &mut Vec<Submesh>,
    sliced_submeshes: &mut Vec<Submesh>,
    sliced_mesh_data: &mut SlicedMeshData,
) {
    while let Some(mut mesh_to_slice) = submeshes_to_slice.pop() {
        // Use aabb for mesh center approximation
        let aabb = match mesh_to_slice.cached_aabb() {
            Some(aabb) => aabb,
            None => mesh_to_slice.compute_aabb(sliced_mesh_data.buffers().positions()),
        };
        // Random normalized vector for the cut plane
        let normal_vec = get_random_normalized_vec();
        let plane = Plane::new(aabb.center, normal_vec);

        if let Some(fragments) = internal_slice_submesh(&mut mesh_to_slice, plane, sliced_mesh_data)
        {
            sliced_submeshes.extend(fragments);
        } else {
            sliced_submeshes.push(mesh_to_slice);
        };
    }
}

fn internal_slice_submesh(
    mesh_to_slice: &mut Submesh,
    plane: Plane,
    sliced_mesh_data: &mut SlicedMeshData,
) -> Option<[Submesh; 2]> {
    let mut top_fragment = Submesh::new();
    let mut bottom_fragment = Submesh::new();

    // TODO Could share constraints buffer at a higher level
    let mut sliced_contour = Vec::new();
    divide_mesh_triangles(
        mesh_to_slice,
        plane,
        &mut top_fragment,
        &mut bottom_fragment,
        &mut sliced_contour,
        sliced_mesh_data,
    );

    sliced_mesh_data.update_generation();

    // TODO Could we easily know beforehand if the mesh intersects the slicing plane ?
    // TODO Could maybe at least do a convex hull/aabb check against the plane to return early if w are sure that there won't be an intersection.
    if top_fragment.indices().is_empty() || bottom_fragment.indices().is_empty() {
        None
    } else {
        triangulate_and_fill_sliced_faces(
            &plane,
            &mut top_fragment,
            &mut bottom_fragment,
            &sliced_contour,
            sliced_mesh_data,
        );
        Some([top_fragment, bottom_fragment])
    }
}

/// Transforms 3d coordinates of all vertices into 2d coordinates on a plane defined by the given normal and vertices.
/// - Input vertices need to all belong to the same 3d plan
/// - There must be at least two vertices
pub fn iterator_transform_to_2d_planar_coordinate_system<T: Borrow<Vec3>>(
    vertices_iter: impl IntoIterator<Item = T>,
    plane_normal: Vec3A,
) -> Vec<Vertex> {
    let mut vertices = vertices_iter.into_iter().map(|p| *p.borrow());

    // TODO Could also make sure there are at least two vertices before ?
    let Some(v0) = vertices.next() else {
        return vec![];
    };
    let Some(v1) = vertices.next() else {
        return vec![];
    };
    // Create a base, using the first two vertices as the first base vector and plane_normal as the second
    let basis_1 = (v0 - v1).normalize();
    // basis_3 is already normalized since basis_1 and plane_normal are normalized and orthogonal
    let basis_3 = basis_1.cross(plane_normal.into());

    // Project every vertices into the base B
    let mut vertices_2d = Vec::with_capacity(vertices.size_hint().0);
    for vertex in vertices {
        vertices_2d.push(Vertex::new(vertex.dot(basis_1), vertex.dot(basis_3)));
    }
    vertices_2d
}

fn triangulate_and_fill_sliced_faces(
    plane: &Plane,
    top_fragment: &mut Submesh,
    bottom_fragment: &mut Submesh,
    sliced_contour: &Vec<Edge>,
    sliced_mesh_data: &mut SlicedMeshData,
) {
    let mut planar_vert_id_to_3d_vert_id = Vec::with_capacity(sliced_contour.len());
    // TODO Could we build a local edges contour while slicing ?
    // let mut local_edges = Vec::with_capacity(sliced_contour.len());

    // let mut local_edge_mapping = HashMap::new();
    let planar_vertices = iterator_transform_to_2d_planar_coordinate_system(
        sliced_contour
            .iter()
            .enumerate()
            .map(|(local_index, edge)| {
                planar_vert_id_to_3d_vert_id.push(edge.to);
                // local_edge_mapping.insert(edge.to, local_index);
                sliced_mesh_data.pos(edge.to)
            }),
        plane.normal(),
    );
    // TODO Check constraints orientation
    let mut constraints: Vec<Edge> = (0..planar_vertices.len() - 1)
        .map(|i| Edge::new(i as VertexId, i as VertexId + 1))
        .collect();
    constraints.push(Edge::new(planar_vertices.len() as VertexId - 1, 0));
    let triangulation = ghx_constrained_delaunay::constrained_triangulation_from_2d_vertices(
        &planar_vertices,
        &constraints,
        ConstrainedTriangulationConfiguration::default(),
    );

    // TODO Optimization: share allocation
    let mut planar_vert_id_to_frag_verts_ids: Vec<Option<(VertexId, VertexId)>> =
        vec![None; planar_vertices.len()];
    let mut top_triangle = [0, 0, 0];
    let mut bottom_triangle = [0, 0, 0];

    for t in triangulation.triangles.iter() {
        for (index, &v_id) in t.iter().enumerate() {
            let (top_vert, bottom_vert) = match planar_vert_id_to_frag_verts_ids[v_id as usize] {
                Some(pair) => pair,
                None => {
                    let pos = sliced_mesh_data.pos(planar_vert_id_to_3d_vert_id[v_id as usize]);
                    let uv = Vec2::new((pos.x as f32), (pos.y as f32)); // TODO Redo uv mapping. May use scale factor from triangulation to be in [0,1] ?
                    let pair = (
                        sliced_mesh_data.push_new_vertex(pos, uv, -plane.normal()),
                        sliced_mesh_data.push_new_vertex(pos, uv, plane.normal()),
                    );
                    planar_vert_id_to_frag_verts_ids[v_id as usize] = Some(pair);
                    pair
                }
            };

            top_triangle[index] = top_vert;
            // We need to change the orientation of the triangles for one of the sliced face:
            bottom_triangle[2 - index] = bottom_vert;
        }

        top_fragment.indices_mut().extend(top_triangle);
        bottom_fragment.indices_mut().extend(bottom_triangle);
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

use lazy_static::lazy_static;
lazy_static! {
    // u32 with  3 side info packed on 6 bits ?
    // const TRIANGLES_SIDES_ARRAY: [TrianglePlaneIntersectionC; 42] = [];
    static ref TRIANGLE_PLANE_INTERSECTIONS: HashMap<(PlaneSide,PlaneSide,PlaneSide), TrianglePlaneIntersection> = {
         HashMap::from([
            ((PlaneSide::OnPlane, PlaneSide::Top, PlaneSide::Top),TrianglePlaneIntersection::OneVertexTop(0)),
            ((PlaneSide::Top, PlaneSide::OnPlane, PlaneSide::Top),TrianglePlaneIntersection::OneVertexTop(1)),
            ((PlaneSide::Top, PlaneSide::Top, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexTop(2)),

            ((PlaneSide::OnPlane, PlaneSide::Bottom, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexBottom(0)),
            ((PlaneSide::Bottom, PlaneSide::OnPlane, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexBottom(1)),
            ((PlaneSide::Bottom, PlaneSide::Bottom, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexBottom(2)),

            ((PlaneSide::OnPlane, PlaneSide::OnPlane, PlaneSide::Top),TrianglePlaneIntersection::OneEdgeTop((0,1))),
            ((PlaneSide::Top, PlaneSide::OnPlane, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeTop((1,2))),
            ((PlaneSide::OnPlane, PlaneSide::Top, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeTop((0,2))),


            ((PlaneSide::OnPlane, PlaneSide::OnPlane, PlaneSide::Bottom),TrianglePlaneIntersection::OneEdgeBottom((0,1))),
            ((PlaneSide::Bottom, PlaneSide::OnPlane, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeBottom((1,2))),
            ((PlaneSide::OnPlane, PlaneSide::Bottom, PlaneSide::OnPlane),TrianglePlaneIntersection::OneEdgeBottom((0,2))),

            // Edge indexes order is important for those cases. Top -> Bottom
            ((PlaneSide::OnPlane, PlaneSide::Top, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:0, edge: (1,2)}),
            ((PlaneSide::OnPlane, PlaneSide::Bottom, PlaneSide::Top),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:0, edge: (2,1)}),
            ((PlaneSide::Bottom, PlaneSide::OnPlane, PlaneSide::Top),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:1, edge: (2,0)}),
            ((PlaneSide::Top, PlaneSide::OnPlane, PlaneSide::Bottom),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:1, edge: (0,2)}),
            ((PlaneSide::Top, PlaneSide::Bottom, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:2, edge: (0,1)}),
            ((PlaneSide::Bottom, PlaneSide::Top, PlaneSide::OnPlane),TrianglePlaneIntersection::OneVertexOneCrossedEdge{vertex:2, edge: (1,0)}),

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

/// Divides triangles among the bottom and top slices
pub fn divide_mesh_triangles(
    sliced_submesh: &mut Submesh,
    plane: Plane,
    // sides: &Vec<PlaneSide>,
    top_fragment: &mut Submesh,
    bottom_fragment: &mut Submesh,
    sliced_contour: &mut Vec<Edge>,
    sliced_mesh_data: &mut SlicedMeshData,
) {
    for triangle in sliced_submesh.indices().chunks_exact(3) {
        let sides = (
            sliced_mesh_data.get_current_side_data(triangle[0], plane),
            sliced_mesh_data.get_current_side_data(triangle[1], plane),
            sliced_mesh_data.get_current_side_data(triangle[2], plane),
        );

        // TODO Optimization: Those two cases should be the more common ones. See if they need to be kept separated
        // If the triangle is completly above the slicing plane
        if sides == (PlaneSide::Top, PlaneSide::Top, PlaneSide::Top) {
            top_fragment.indices_mut().extend(triangle);
            continue;
        }
        // If the triangle is completly bellow the slicing plane
        if sides == (PlaneSide::Bottom, PlaneSide::Bottom, PlaneSide::Bottom) {
            bottom_fragment.indices_mut().extend(triangle);
            continue;
        }

        // TODO Optimization: Could be sped up if needed (could pack the enum and/or the 3 sides)
        match TRIANGLE_PLANE_INTERSECTIONS.get(&sides).unwrap() {
            TrianglePlaneIntersection::OneVertexOneCrossedEdge { vertex, edge } => {
                split_on_plane_triangle(
                    plane,
                    top_fragment,
                    bottom_fragment,
                    sliced_contour,
                    sliced_mesh_data,
                    (
                        triangle[*vertex as usize],
                        triangle[edge.0 as usize],
                        triangle[edge.1 as usize],
                    ),
                );
            }
            // TODO The two TwoCrossedEdgesTopSide cases could/should be abstracted away into one. Same for `split_intersected_triangle`
            TrianglePlaneIntersection::TwoCrossedEdgesTopSide { verts } => {
                split_intersected_triangle(
                    plane,
                    top_fragment,
                    bottom_fragment,
                    sliced_contour,
                    sliced_mesh_data,
                    (
                        triangle[verts[0] as usize],
                        triangle[verts[1] as usize],
                        triangle[verts[2] as usize],
                    ),
                    true,
                );
            }
            TrianglePlaneIntersection::TwoCrossedEdgesBottomSide { verts } => {
                split_intersected_triangle(
                    plane,
                    top_fragment,
                    bottom_fragment,
                    sliced_contour,
                    sliced_mesh_data,
                    (
                        triangle[verts[0] as usize],
                        triangle[verts[1] as usize],
                        triangle[verts[2] as usize],
                    ),
                    false,
                );
            }
            TrianglePlaneIntersection::OneVertexTop(_v_index) => {
                top_fragment.indices_mut().extend(triangle)
            }
            TrianglePlaneIntersection::OneVertexBottom(_v_index) => {
                bottom_fragment.indices_mut().extend(triangle);
            }
            TrianglePlaneIntersection::OneEdgeBottom((v0, v1)) => {
                bottom_fragment.indices_mut().extend(triangle);
                let edge = (triangle[*v0 as usize], triangle[*v1 as usize]);
                sliced_contour.push(Edge::new(edge.0, edge.1));
            }
            TrianglePlaneIntersection::OneEdgeTop((v0, v1)) => {
                top_fragment.indices_mut().extend(triangle);
                let edge = (triangle[*v0 as usize], triangle[*v1 as usize]);
                sliced_contour.push(Edge::new(edge.0, edge.1));
            }
            TrianglePlaneIntersection::FlatTriangle => {
                // TODO Handle flat triangles ont the slicing plane. We may just discard them (indices) but keep their vertices
                warn!("TODO Handle flat triangles on the slicing plane. We may just discard them (indices) but keep their vertices");
            }
        }
    }
}

///           /* 1 top
///         /  |
///     0 /    |
///    --*-----m-- plane
///       \    |
///         \  |
///           \* 2 bottom
///
/// The vertices are given as [on plane, top, bottom]:
fn split_on_plane_triangle(
    plane: Plane,
    top_fragment: &mut Submesh,
    bottom_fragment: &mut Submesh,
    sliced_contour: &mut Vec<Edge>,
    vertices: &mut SlicedMeshData,
    v: (VertexId, VertexId, VertexId),
) {
    let Some((pos, s)) =
        edge_plane_intersection(vertices.pos(v.1).into(), vertices.pos(v.2).into(), &plane)
    else {
        // Not possible in practice, the edge intersects the plane.
        warn!("Internal error, no intersection found between an edge and a plane whereas one was expected.");
        return;
    };
    let norm =
        (vertices.normal(v.1) + s * (vertices.normal(v.2) - vertices.normal(v.1))).normalize();
    let uv: Vec2 = vertices.uv(v.1) + s * (vertices.uv(v.2) - vertices.uv(v.1));

    let new_vertex_id = vertices.push_new_vertex(pos.into(), uv, norm);

    top_fragment.indices_mut().extend([new_vertex_id, v.0, v.1]);
    bottom_fragment
        .indices_mut()
        .extend([new_vertex_id, v.2, v.0]);

    // Left to right
    // TODO sliced verts Create later after triangualtion
    // Uvs for sliced face will be set later, after triangfulation (just for the sake fo reusing the 2D scale factor)
    // let top_frag_sliced_face_vertex_id =
    //     vertices.push_new_vertex(pos.into(), Vec2::ZERO, -plane.normal());
    // let bottom_frag_sliced_face_vertex_id =
    //     vertices.push_new_vertex(pos.into(), Vec2::ZERO, plane.normal());
    sliced_contour.push(Edge::new(v.0, new_vertex_id));
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
    top_fragment: &mut Submesh,
    bottom_fragment: &mut Submesh,
    sliced_contour: &mut Vec<Edge>,
    vertices: &mut SlicedMeshData,
    v: (VertexId, VertexId, VertexId),
    two_vertices_on_top: bool,
) {
    let (intersection_result_13, intersection_result_23) = if two_vertices_on_top {
        (
            edge_plane_intersection(vertices.pos(v.0).into(), vertices.pos(v.2).into(), &plane),
            edge_plane_intersection(vertices.pos(v.1).into(), vertices.pos(v.2).into(), &plane),
        )
    } else {
        (
            edge_plane_intersection(vertices.pos(v.2).into(), vertices.pos(v.0).into(), &plane),
            edge_plane_intersection(vertices.pos(v.2).into(), vertices.pos(v.1).into(), &plane),
        )
    };

    // Check if the two edges of the triangle are crossed
    match (intersection_result_13, intersection_result_23) {
        (Some((pos13, s13)), Some((pos23, s23))) => {
            // /!\ Interpolate normals and UV coordinates
            let norm13 = (vertices.normal(v.0)
                + s13 * (vertices.normal(v.2) - vertices.normal(v.0)))
            .normalize();
            let norm23 = (vertices.normal(v.1)
                + s23 * (vertices.normal(v.2) - vertices.normal(v.1)))
            .normalize();
            let uv13 = vertices.uv(v.0) + s13 * (vertices.uv(v.2) - vertices.uv(v.0));
            let uv23: Vec2 = vertices.uv(v.1) + s23 * (vertices.uv(v.2) - vertices.uv(v.1));

            let v13_id = vertices.push_new_vertex(pos13.into(), uv13, norm13);
            let v23_id = vertices.push_new_vertex(pos23.into(), uv23, norm23);

            if two_vertices_on_top {
                // Add two triangles on top and one for the bottom
                top_fragment.indices_mut().extend([v13_id, v.0, v.1]);
                top_fragment.indices_mut().extend([v23_id, v13_id, v.1]);
                bottom_fragment.indices_mut().extend([v.2, v13_id, v23_id]);

                // Left to right
                sliced_contour.push(Edge::new(v13_id, v23_id));
            } else {
                // Add two triangles below and one on top
                bottom_fragment.indices_mut().extend([v.0, v.1, v13_id]);
                bottom_fragment.indices_mut().extend([v.1, v23_id, v13_id]);
                top_fragment.indices_mut().extend([v13_id, v23_id, v.2]);

                // Left to right
                sliced_contour.push(Edge::new(v23_id, v13_id));
            }
        }
        _ => {
            // Not possible in practice, the edges intersects the plane.
            warn!("Internal error, no intersection found between two edges and a plane whereas two were expected.")
        }
    }
}
