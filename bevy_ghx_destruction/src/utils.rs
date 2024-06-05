use bevy::{log::info, math::Vec3A};
use ghx_constrained_delaunay::types::{Edge, VertexId};
use glam::Vec2;
use rand::Rng;

use crate::types::{CutDirection, MeshBuilder, MeshBuilderVertex, Plane};

#[inline]
pub fn get_random_normalized_vec() -> Vec3A {
    let mut rng = rand::thread_rng();
    Vec3A::new(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>()).normalize()
}

#[derive(PartialEq)]
pub enum TriangleOnPlane {
    OnPlane(Vec<usize>),
    OutOfPlane,
}

pub fn is_triangle_on_plane(side: &Vec<CutDirection>, vertices: [u64; 3]) -> TriangleOnPlane {
    let on_plane: Vec<usize> = vertices
        .iter()
        .enumerate()
        .filter_map(|(j, &v)| {
            if side[v as usize] == CutDirection::OnPlane {
                Some(j)
            } else {
                None
            }
        })
        .collect();

    if on_plane.len() == 0 {
        return TriangleOnPlane::OutOfPlane;
    } else {
        return TriangleOnPlane::OnPlane(on_plane);
    }
}

pub fn on_plane_triangle(
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
    side: &Vec<CutDirection>,
    vertices: [u64; 3],
    on_plane: Vec<usize>,
    mesh_builder: &MeshBuilder,
    plane: Plane,
) {
    if on_plane.len() == 1 {
        let out_of_plane_vertices: Vec<u64> = vertices
            .iter()
            .enumerate()
            .filter_map(|(j, &v)| if j != on_plane[0] { Some(v) } else { None })
            .collect();

        if side[out_of_plane_vertices[0] as usize] == CutDirection::Top
            && side[out_of_plane_vertices[1] as usize] == CutDirection::Top
        {
            top_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        }
        if side[out_of_plane_vertices[0] as usize] == CutDirection::Bottom
            && side[out_of_plane_vertices[1] as usize] == CutDirection::Bottom
        {
            bottom_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        }
        if side[out_of_plane_vertices[0] as usize] == CutDirection::Bottom
            && side[out_of_plane_vertices[1] as usize] == CutDirection::Top
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
        }
        if side[out_of_plane_vertices[0] as usize] == CutDirection::Top
            && side[out_of_plane_vertices[1] as usize] == CutDirection::Bottom
        {
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
    } else {
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

        if side[out_of_plane_vertex[0] as usize] == CutDirection::Top {
            top_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        }

        if side[out_of_plane_vertex[0] as usize] == CutDirection::Bottom {
            bottom_mesh_builder.push_mapped_triangle(vertices[0], vertices[1], vertices[2]);
        }
    }
}

///            * top
///         /  |
///       /    |
///    --*-----m-- plane
///       \    |
///         \  |
///            * bottom
fn split_on_plane_triangle(
    top_mesh_builder: &mut MeshBuilder,
    bottom_mesh_builder: &mut MeshBuilder,
    vertices: [u64; 3],
    mesh_builder: &MeshBuilder,
    plane: Plane,
) {
    info!("split_on_plane_triangle");
    // The vertices list is given as [on plane, top, bottom]:
    let mut v = vec![MeshBuilderVertex::new(Vec3A::ZERO, Vec2::ZERO, Vec3A::ZERO); 3];

    for (i, vertex_id) in vertices.iter().enumerate() {
        let v_id = *vertex_id as usize;
        if v_id < mesh_builder.vertices().len() {
            v[i] = mesh_builder.vertices()[v_id as usize];
        } else {
            v[i] = mesh_builder.sliced_vertices()[v_id as usize - mesh_builder.vertices().len()];
        }
    }

    let (m, s) =
        find_intersection_line_plane(v[1].pos(), v[2].pos(), plane.origin(), plane.normal())
            .unwrap();
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

    let top_sliced_vertices_nbr = top_mesh_builder.sliced_vertices().len() as VertexId;
    let bottom_sliced_vertices_nbr = bottom_mesh_builder.sliced_vertices().len() as VertexId;

    // Need to be carfull with constrained edges orientation:
    top_mesh_builder.constraints_mut().push(Edge::new(
        top_sliced_vertices_nbr - 2,
        top_sliced_vertices_nbr - 1,
    ));
    bottom_mesh_builder.constraints_mut().push(Edge::new(
        bottom_sliced_vertices_nbr - 1,
        bottom_sliced_vertices_nbr - 2,
    ));
}

pub fn find_intersection_line_plane(
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

    let s = (origin_point - line_point_start).dot(normal_vec)
        / (line_point_end - line_point_start).dot(normal_vec);

    if s >= 0. && s <= 1. {
        return Some((
            line_point_start
                - (line_point_end - line_point_start)
                    * (line_point_start - origin_point).dot(normal_vec)
                    / (line_point_end - line_point_start).dot(normal_vec),
            s,
        ));
    }

    return None;
}

pub fn is_above_plane(point: Vec3A, plane: Plane) -> CutDirection {
    let vector_to_plane = (point - plane.origin()).normalize();
    let distance = -vector_to_plane.dot(plane.normal());
    if distance < 0. {
        return CutDirection::Top;
    } else if distance.is_nan() {
        return CutDirection::OnPlane;
    }

    CutDirection::Bottom
}
