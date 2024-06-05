pub mod slicing;
pub mod types;
pub mod utils;

pub use ghx_constrained_delaunay as delaunay;

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use crate::{
        types::{CutDirection, Plane},
        utils::is_above_plane,
    };

    #[test]
    fn is_colinear() {
        let plane = Plane::new(Vec3A::ZERO, Vec3A::Y);
        assert_eq!(is_above_plane(Vec3A::ZERO, plane), CutDirection::OnPlane);
    }

    #[test]
    fn point_above_plane() {
        let plane = Plane::new(Vec3A::ZERO, Vec3A::Y);
        assert_eq!(
            is_above_plane(Vec3A::new(0., 5., 0.), plane),
            CutDirection::Top
        );
    }

    #[test]
    fn point_below_plane() {
        let plane = Plane::new(Vec3A::ZERO, Vec3A::Y);
        assert_eq!(
            is_above_plane(Vec3A::new(0., -5., 0.), plane),
            CutDirection::Bottom
        );
    }
}
