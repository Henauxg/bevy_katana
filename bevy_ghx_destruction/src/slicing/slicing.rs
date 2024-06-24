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

pub fn slice_mesh_iterative(mesh: &Mesh, iteration_count: u32) -> Vec<Mesh> {
    if iteration_count == 0 {
        return vec![mesh.clone()];
    }

    let mut buffer_1 = Vec::with_capacity(2_u32.pow(iteration_count) as usize);
    let mut buffer_2 = Vec::with_capacity(2_u32.pow(iteration_count) as usize);

    let mut meshes_to_slice_buffer = &mut buffer_1;
    let mut sliced_meshes_buffer = &mut buffer_2;

    meshes_to_slice_buffer.push(mesh.clone());

    for _ in 0..iteration_count {
        sliced_meshes_buffer.clear();

        internal_slice_meshes(&meshes_to_slice_buffer, sliced_meshes_buffer);

        // Swap buffer
        let tmp = sliced_meshes_buffer;
        sliced_meshes_buffer = meshes_to_slice_buffer;
        meshes_to_slice_buffer = tmp;
    }

    if iteration_count % 2 == 0 {
        buffer_1
    } else {
        buffer_2
    }
}

fn internal_slice_meshes(meshes_to_slice: &Vec<Mesh>, sliced_meshes: &mut Vec<Mesh>) {
    for mesh in meshes_to_slice.iter() {
        // Mesh center approximation
        let mesh_center = mesh.compute_aabb().unwrap().center;
        // Random normalized vector for the cut plane
        let normal_vec = get_random_normalized_vec();
        let plane = Plane::new(mesh_center, normal_vec);
        let frags = slice_mesh(plane, mesh);
        sliced_meshes.extend(frags)
    }
}

/// Operate the slice on a mesh and spawn fragments
pub fn slice(
    commands: &mut Commands,
    mesh: &Mesh,
    meshes_assets: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let mesh_center = mesh.compute_aabb().unwrap().center;
    let normal_vec = get_random_normalized_vec();

    // TODO Crashes triangulation
    // let normal_vec = Vec3A::X;

    let plane = Plane::new(mesh_center, normal_vec);
    let meshes = slice_mesh(plane, mesh);
    spawn_fragments(meshes, plane.origin(), materials, meshes_assets, commands);
}

/// Operate the slice on a mesh
pub fn slice_mesh(plane: Plane, mesh: &Mesh) -> Vec<Mesh> {
    let mut slice_mesh = SlicedMesh::from_bevy_mesh(mesh);
    match internal_slice_mesh(plane, &mut slice_mesh) {
        None => vec![],
        Some((top_slice, bottom_slice)) => {
            vec![top_slice.to_bevy_mesh(), bottom_slice.to_bevy_mesh()]
        }
    }
}

fn internal_slice_mesh(
    plane: Plane,
    mesh_to_slice: &mut SlicedMesh,
) -> Option<(SlicedMesh, SlicedMesh)> {
    // Track the position of the vertices along the slice plane
    let mut sides = Vec::with_capacity(
        mesh_to_slice.vertices().len() + mesh_to_slice.sliced_face_vertices().len(),
    );

    // Initializing the top/bottom mesh builders
    let mut top_slice = SlicedMesh::initialize_with(&mesh_to_slice);
    let mut bottom_slice = SlicedMesh::initialize_with(&mesh_to_slice);

    // Add the vertices from the sliced mesh to the top and bottom mesh builders
    for (vertex_id, vertex) in mesh_to_slice
        .vertices()
        .iter()
        // TODO Why do we have a different sliced_vertices buffer here ? COuld we avoid it ?
        .chain(mesh_to_slice.sliced_face_vertices().iter())
        .enumerate()
    {
        let side = is_above_plane(vertex.pos(), plane);
        sides.push(side);
        // Vertices are split from top to bottom. If a vertex on the slicing plane, it will be fixed later
        match side {
            PlaneSide::Top => top_slice.push_mapped_vertex(*vertex, vertex_id),
            PlaneSide::Bottom => bottom_slice.push_mapped_vertex(*vertex, vertex_id),
            PlaneSide::OnPlane => {
                top_slice.push_mapped_vertex(*vertex, vertex_id);
                bottom_slice.push_mapped_vertex(*vertex, vertex_id);
            }
        }
    }

    // TODO Could share buffer at a higher level
    let mut sliced_contour = Vec::new();
    divide_mesh_triangles(
        mesh_to_slice,
        plane,
        &sides,
        &mut top_slice,
        &mut bottom_slice,
        &mut sliced_contour,
    );

    // TODO Could we easily know beforehand if the mesh intersects the slicing plane ?
    // TODO Could maybe at least do a convex hull/aabb check against the plane to return early if w are sure that there won't be an intersection.
    if top_slice.vertices().is_empty() || bottom_slice.vertices().is_empty() {
        None
    } else {
        // Fill the newly created sliced face
        fill_sliced_face(&plane, &mut top_slice, &mut bottom_slice, &sliced_contour);
        Some((top_slice, bottom_slice))
    }
}

//todo: clean with our triangulation
fn fill_sliced_face(
    plane: &Plane,
    top_slice: &mut SlicedMesh,
    bottom_slice: &mut SlicedMesh,
    sliced_contour: &Vec<Edge>,
) {
    // // Erease duplicated sliced vertices
    // top_mesh_builder.shrink_sliced_vertices();
    // bottom_mesh_builder.shrink_sliced_vertices();

    // Apply the triangulation to the top cut face. No need to do it for the bottom cute face since they both
    // share the same vertices, we will just reverse the normal.
    // TODO Would not need a copy if we had a SoA
    let vertices = top_slice
        .sliced_face_vertices()
        .iter()
        .map(|v| v.pos())
        .collect();
    let planar_vertices = transform_to_2d_planar_coordinate_system(&vertices, -plane.normal());
    let triangulation = ghx_constrained_delaunay::constrained_triangulation_from_2d_vertices(
        &planar_vertices,
        sliced_contour,
        ConstrainedTriangulationConfiguration::default(),
    );

    // Apply normals and uv to sliced vertices
    for (index, (v_top, v_bottom)) in top_slice
        .sliced_face_vertices_mut()
        .iter_mut()
        .zip(bottom_slice.sliced_face_vertices_mut())
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
    let top_slice_verts_count = top_slice.vertices().len() as IndexType;
    let bottom_slice_verts_count = bottom_slice.vertices().len() as IndexType;
    for t in triangulation.triangles.iter() {
        top_slice.push_triangle(
            top_slice_verts_count + t[0],
            top_slice_verts_count + t[1],
            top_slice_verts_count + t[2],
        );
        // We need to change the orientation of the triangles for one of the cut face:
        bottom_slice.push_triangle(
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

/// Divide triangles among the bottom and top slices
pub fn divide_mesh_triangles(
    sliced_mesh: &mut SlicedMesh,
    plane: Plane,
    sides: &Vec<PlaneSide>,
    top_slice: &mut SlicedMesh,
    bottom_slice: &mut SlicedMesh,
    sliced_contour: &mut Vec<Edge>,
) {
    for triangle in sliced_mesh.indices().chunks_exact(3) {
        let (v1, v2, v3) = (triangle[0], triangle[1], triangle[2]);
        // TODO Optimization: Those two cases should be the more common ones. See if they need to be kept separated
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
                    sliced_contour,
                    [
                        triangle[*vertex as usize],
                        triangle[edge.0 as usize],
                        triangle[edge.1 as usize],
                    ],
                    sliced_mesh,
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
                    sliced_contour,
                    verts_indexes,
                    true,
                    &sliced_mesh,
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
                    sliced_contour,
                    verts_indexes,
                    false,
                    &sliced_mesh,
                );
            }
            TrianglePlaneIntersection::OneVertexTop(v_index) => {
                top_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2]);
                let vert = sliced_mesh.vert(triangle[*v_index as usize]);
                // TODO Could we push directly in normal vertices buffer (use a slice for tirangulation)
                top_slice.sliced_face_vertices_mut().push(vert.clone());
                bottom_slice.sliced_face_vertices_mut().push(vert.clone());
            }
            TrianglePlaneIntersection::OneVertexBottom(v_index) => {
                bottom_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2]);
                let vert = sliced_mesh.vert(triangle[*v_index as usize]);
                // TODO Could we push directly in normal vertices buffer
                top_slice.sliced_face_vertices_mut().push(vert.clone());
                bottom_slice.sliced_face_vertices_mut().push(vert.clone());
            }
            TrianglePlaneIntersection::OneEdgeBottom((v0, v1)) => {
                bottom_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2]);
                let edge = (triangle[*v0 as usize], triangle[*v1 as usize]);

                let vert = sliced_mesh.vert(triangle[edge.0 as usize]);
                // TODO Could we push directly in normal vertices buffer (use a slice for tirangulation)
                top_slice.sliced_face_vertices_mut().push(vert.clone());
                bottom_slice.sliced_face_vertices_mut().push(vert.clone());

                let vert = sliced_mesh.vert(triangle[edge.1 as usize]);
                // TODO Could we push directly in normal vertices buffer (use a slice for tirangulation)
                top_slice.sliced_face_vertices_mut().push(vert.clone());
                bottom_slice.sliced_face_vertices_mut().push(vert.clone());

                sliced_contour.push(Edge::new(
                    top_slice.index_map()[edge.0 as usize],
                    top_slice.index_map()[edge.1 as usize],
                ));
            }
            TrianglePlaneIntersection::OneEdgeTop((v0, v1)) => {
                top_slice.push_mapped_triangle(triangle[0], triangle[1], triangle[2]);
                let edge = (triangle[*v0 as usize], triangle[*v1 as usize]);

                let vert = sliced_mesh.vert(triangle[edge.0 as usize]);
                // TODO Could we push directly in normal vertices buffer (use a slice for tirangulation)
                top_slice.sliced_face_vertices_mut().push(vert.clone());
                bottom_slice.sliced_face_vertices_mut().push(vert.clone());

                let vert = sliced_mesh.vert(triangle[edge.1 as usize]);
                // TODO Could we push directly in normal vertices buffer (use a slice for tirangulation)
                top_slice.sliced_face_vertices_mut().push(vert.clone());
                bottom_slice.sliced_face_vertices_mut().push(vert.clone());

                sliced_contour.push(Edge::new(
                    top_slice.index_map()[edge.0 as usize],
                    top_slice.index_map()[edge.1 as usize],
                ));
            }
            TrianglePlaneIntersection::FlatTriangle => {
                // TODO Handle flat triangles ont the slicing plane. We may just discard them (indices) but keep their vertices
                warn!("Handle flat triangles ont the slicing plane. We may just discard them (indices) but keep their vertices");
            }
        }
    }
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
        internal_spawn_fragment(&mesh, &mesh_handle, materials, commands, pos.into());
    }
}

fn internal_spawn_fragment(
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
    top_slice: &mut SlicedMesh,
    bottom_slice: &mut SlicedMesh,
    sliced_contour: &mut Vec<Edge>,
    vertices: [u64; 3],
    sliced_mesh: &SlicedMesh,
    plane: Plane,
) {
    let mut v = vec![MeshBuilderVertex::new(Vec3A::ZERO, Vec2::ZERO, Vec3A::ZERO); 3];

    for (i, vertex_id) in vertices.iter().enumerate() {
        let v_id = *vertex_id as usize;
        if v_id < sliced_mesh.vertices().len() {
            v[i] = sliced_mesh.vertices()[v_id as usize];
        } else {
            v[i] = sliced_mesh.sliced_face_vertices()[v_id as usize - sliced_mesh.vertices().len()];
        }
    }

    let (m, s) =
        line_plane_intersection(v[1].pos(), v[2].pos(), plane.origin(), plane.normal()).unwrap();
    let norm = (v[1].normal() + s * (v[2].normal() - v[1].normal())).normalize();
    let uv: Vec2 = v[1].uv() + s * (v[2].uv() - v[1].uv());

    top_slice.add_sliced_vertex(m, uv, norm);
    bottom_slice.add_sliced_vertex(m, uv, norm);

    let id_top = top_slice.vertices().len() as VertexId - 1;
    let id_bottom = bottom_slice.vertices().len() as VertexId - 1;

    top_slice.push_triangle(
        id_top,
        top_slice.index_map()[vertices[0] as usize],
        top_slice.index_map()[vertices[1] as usize],
    );

    top_slice.push_triangle(
        id_bottom,
        top_slice.index_map()[vertices[2] as usize],
        top_slice.index_map()[vertices[0] as usize],
    );

    // Left to right
    sliced_contour.push(Edge::new(
        top_slice.index_map()[vertices[0] as usize],
        (top_slice.sliced_face_vertices().len() - 1) as VertexId,
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
    top_slice: &mut SlicedMesh,
    bottom_slice: &mut SlicedMesh,
    sliced_contour: &mut Vec<Edge>,
    verts_indexes: [VertexId; 3],
    two_vertices_on_top: bool,
    sliced_mesh: &SlicedMesh,
) {
    let mut v = vec![MeshBuilderVertex::new(Vec3A::ZERO, Vec2::ZERO, Vec3A::ZERO); 3];

    for (i, vertex_id) in verts_indexes.iter().enumerate() {
        let v_id = *vertex_id as usize;
        if v_id < sliced_mesh.vertices().len() {
            v[i] = sliced_mesh.vertices()[v_id];
        } else {
            v[i] = sliced_mesh.sliced_face_vertices()[v_id - sliced_mesh.vertices().len()];
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
            top_slice.add_sliced_vertex(v13, uv13, norm13);
            top_slice.add_sliced_vertex(v23, uv23, norm23);

            bottom_slice.add_sliced_vertex(v13, uv13, norm13);
            bottom_slice.add_sliced_vertex(v23, uv23, norm23);

            // Ids of the vertices for each mesh builder
            let id13_top = top_slice.vertices().len() as VertexId - 2;
            let id23_top = top_slice.vertices().len() as VertexId - 1;
            let id13_bottom = bottom_slice.vertices().len() as VertexId - 2;
            let id23_bottom = bottom_slice.vertices().len() as VertexId - 1;

            if two_vertices_on_top {
                // Add two triangles on top
                top_slice.push_triangle(
                    id23_top,
                    id13_top,
                    top_slice.index_map()[verts_indexes[1] as usize],
                );
                top_slice.push_triangle(
                    id13_top,
                    top_slice.index_map()[verts_indexes[0] as usize],
                    top_slice.index_map()[verts_indexes[1] as usize],
                );

                // And one to the bottom
                bottom_slice.push_triangle(
                    bottom_slice.index_map()[verts_indexes[2] as usize],
                    id13_bottom,
                    id23_bottom,
                );

                // Left to right
                sliced_contour.push(Edge::new(
                    (top_slice.sliced_face_vertices().len() - 2) as VertexId,
                    (top_slice.sliced_face_vertices().len() - 1) as VertexId,
                ));
            } else {
                // Add two triangles below
                bottom_slice.push_triangle(
                    bottom_slice.index_map()[verts_indexes[0] as usize],
                    bottom_slice.index_map()[verts_indexes[1] as usize],
                    id13_bottom,
                );

                bottom_slice.push_triangle(
                    bottom_slice.index_map()[verts_indexes[1] as usize],
                    id23_bottom,
                    id13_bottom,
                );

                //And one on top
                top_slice.push_triangle(
                    id13_top,
                    id23_top,
                    top_slice.index_map()[verts_indexes[2] as usize],
                );

                // Left to right
                sliced_contour.push(Edge::new(
                    (top_slice.sliced_face_vertices().len() - 1) as VertexId,
                    (top_slice.sliced_face_vertices().len() - 2) as VertexId,
                ));
            }
        }
    }
}
