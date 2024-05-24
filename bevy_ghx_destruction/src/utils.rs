use std::collections::HashMap;

use bevy::{
    math::{Vec2, Vec3A},
    utils::petgraph::matrix_graph::Zero,
};
use ghx_constrained_delaunay::types::VertexId;
use ordered_float::OrderedFloat;
use rand::Rng;

use crate::types::{CutDirection, MeshMapping, Plane};

#[inline]
pub fn get_random_normalized_vec() -> Vec3A {
    let mut rng = rand::thread_rng();
    Vec3A::new(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>()).normalize()
}

pub fn find_intersection_line_plane(
    line_point: Vec3A,
    line_direction: Vec3A,
    origin_point: Vec3A,
    normal_vec: Vec3A,
) -> Option<Vec3A> {
    Some(
        line_point
            - line_direction * (line_point - origin_point).dot(normal_vec)
                / line_direction.dot(normal_vec),
    )
}

pub fn find_intersection_line_plane_bis(
    line_point_start: Vec3A,
    line_point_end: Vec3A,
    origin_point: Vec3A,
    normal_vec: Vec3A,
) -> Option<(Vec3A, f32)> {
    // Handle degenerate cases
    if line_point_start == line_point_end {
        return None;
    } else if normal_vec == Vec3A::ZERO {
        return None;
    }

    // `s` is the parameter for the line segment a -> b where 0.0 <= s <= 1.0
    let s = (origin_point - line_point_start).dot(normal_vec)
        / (line_point_end - line_point_start).dot(normal_vec);

    if s >= 0. && s <= 1. {
        return Some((
            line_point_start + (line_point_end - line_point_start) * s,
            s,
        ));
    }

    return None;
}

//todo:voir si colineaire ok
pub fn is_above_plane(point: Vec3A, plane: Plane) -> CutDirection {
    let vector_to_plane = (point - plane.origin()).normalize();
    let distance = -vector_to_plane.dot(plane.normal());
    if distance < 0. {
        return CutDirection::Top;
    }

    CutDirection::Bottom
}

pub fn to_vertex(
    cut_face_vertices_id: &Vec<VertexId>,
    mesh_mapping: &MeshMapping,
) -> Vec<[f32; 3]> {
    let mut cut_face_vertices = Vec::new();

    for vertex in cut_face_vertices_id.iter() {
        let v = [
            mesh_mapping.vertex()[*vertex].x,
            mesh_mapping.vertex()[*vertex].y,
            mesh_mapping.vertex()[*vertex].z,
        ];
        cut_face_vertices.push(v);
    }
    cut_face_vertices
}

pub fn single_index(
    oredered_float: [OrderedFloat<f32>; 3],
    mesh_mapping: &mut MeshMapping,
    vertices_added: &mut HashMap<[OrderedFloat<f32>; 3], VertexId>,
    cut_face_vertices: &mut Vec<VertexId>,
    vertex: Vec3A,
    uv: Vec2,
    normal: Vec3A,
) -> VertexId {
    let index;
    if !vertices_added.contains_key(&oredered_float) {
        index = mesh_mapping.vertex().len();
        // add the new vertices in the hash map
        vertices_added.insert(oredered_float, index);

        // add the new vertices in the list
        mesh_mapping.vertex_mut().push(vertex);

        mesh_mapping.uv_mut().push(uv);

        mesh_mapping.normal_mut().push(normal);

        cut_face_vertices.push(index);
    } else {
        index = *vertices_added.get(&oredered_float).unwrap();
    }

    index
}

pub fn push_triangle(index_buffer: &mut Vec<VertexId>, v1: VertexId, v2: VertexId, v3: VertexId) {
    index_buffer.push(v1);
    index_buffer.push(v2);
    index_buffer.push(v3);
}
