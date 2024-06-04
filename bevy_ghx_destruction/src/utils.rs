use bevy::math::Vec3A;
use rand::Rng;

use crate::types::{CutDirection, Plane};

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

//todo:voir si colineaire ok
// pub fn is_above_plane(point: Vec3A, plane: Plane) -> CutDirection {
//     let vector_to_plane = (point - plane.origin()).normalize();
//     let distance = -vector_to_plane.dot(plane.normal());
//     if distance < 0. {
//         return CutDirection::Top;
//     }

//     CutDirection::Bottom
// }

pub fn is_above_plane(point: Vec3A, plane: Plane) -> CutDirection {
    if (plane.normal().x * (point.x - plane.origin().x)
        + plane.normal().y * (point.y - plane.origin().y)
        + plane.normal().z * (point.z - plane.origin().z))
        >= 0.
    {
        return CutDirection::Top;
    }

    return CutDirection::Bottom;
}
