use bevy::{math::Vec3A, render::primitives::Aabb};
use rand::{thread_rng, Rng};

use crate::types::{CutDirection, Plane};

fn random_point_from_aabb(aabb: &Aabb) -> Vec3A {
    let mut rng = thread_rng();
    Vec3A::new(
        rng.gen_range(aabb.min().x..=aabb.max().x),
        rng.gen_range(aabb.min().y..=aabb.max().y),
        rng.gen_range(aabb.min().z..=aabb.max().z),
    )
}

pub fn random_plane(aabb: &Aabb) -> Plane {
    let point = random_point_from_aabb(aabb);
    let normal = get_random_normalized_vec();
    Plane::new(point, normal)
}

#[inline]
pub fn get_random_normalized_vec() -> Vec3A {
    let mut rng = rand::thread_rng();
    Vec3A::new(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>()).normalize()
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
