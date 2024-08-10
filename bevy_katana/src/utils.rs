use bevy::{math::Vec3A, render::primitives::Aabb};
use rand::{thread_rng, Rng};

use crate::types::{Plane, PlaneSide};

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

/// If there is an intersection, returns the intersection point and the parameterization `s` of the intersection where:
/// intersection = edge.0 + (edge.1 - edge.0) * s
pub fn edge_plane_intersection(
    edge_0: Vec3A,
    edge_1: Vec3A,
    plane: &Plane,
) -> Option<(Vec3A, f32)> {
    // Handle degenerate cases
    if edge_0 == edge_1 {
        return None;
    } else if plane.normal() == Vec3A::ZERO {
        return None;
    }

    let s = (plane.origin() - edge_0).dot(plane.normal()) / (edge_1 - edge_0).dot(plane.normal());

    if s >= 0. && s <= 1. {
        return Some((
            edge_0
                - (edge_1 - edge_0) * (edge_0 - plane.origin()).dot(plane.normal())
                    / (edge_1 - edge_0).dot(plane.normal()),
            s,
        ));
    }

    return None;
}

pub fn is_above_plane(point: Vec3A, plane: Plane) -> PlaneSide {
    let vector_to_plane = (point - plane.origin()).normalize();
    let distance = -vector_to_plane.dot(plane.normal());
    if distance < 0. {
        return PlaneSide::Top;
    } else if distance.is_nan() {
        return PlaneSide::OnPlane;
    }

    PlaneSide::Bottom
}
