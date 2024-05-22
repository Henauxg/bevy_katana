use bevy::{
    math::Vec3A,
    render::{
        mesh::{Indices, Mesh, PrimitiveTopology, VertexAttributeValues},
        render_asset::RenderAssetUsages,
    },
};
use ghx_constrained_delaunay::types::VertexId;

trait Indexable {
    fn at(&self, idx: usize) -> usize;
}

impl Indexable for Indices {
    fn at(&self, idx: usize) -> usize {
        match self {
            Indices::U16(vec) => vec[idx] as usize,
            Indices::U32(vec) => vec[idx] as usize,
        }
    }
}

#[derive(PartialEq)]
pub enum CutDirection {
    Top,
    Bottom,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Plane {
    pub origin_point: Vec3A,
    pub normal_vec: Vec3A,
}

impl Plane {
    #[inline]
    pub fn new(origin_point: Vec3A, normal_vec: Vec3A) -> Self {
        Self {
            origin_point,
            normal_vec,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MeshMapping {
    vertex_buffer: Vec<Vec3A>,
    index_buffer: Vec<VertexId>,
    initial_size: usize,
}

impl MeshMapping {
    pub fn new(
        vertex_buffer: Vec<Vec3A>,
        index_buffer: Vec<VertexId>,
        initial_size: usize,
    ) -> MeshMapping {
        MeshMapping {
            vertex_buffer,
            index_buffer,
            initial_size,
        }
    }

    pub fn initial_size(&self) -> &usize {
        &self.initial_size
    }

    pub fn vertex(&self) -> &Vec<Vec3A> {
        &self.vertex_buffer
    }

    pub fn vertex_mut(&mut self) -> &mut Vec<Vec3A> {
        &mut self.vertex_buffer
    }

    pub fn mesh_mapping_from_mesh(mesh: &Mesh) -> MeshMapping {
        let Some(VertexAttributeValues::Float32x3(mesh_vertices)) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION)
        else {
            return MeshMapping::new(Vec::new(), Vec::new(), 0);
        };

        let mut vertices = Vec::new();

        for vertex in mesh_vertices.iter() {
            vertices.push(Vec3A::from_array(*vertex));
        }

        // Get the vertices indexes of the current mesh
        let mesh_indices = mesh.indices().unwrap();

        let mut vertices_id = Vec::new();

        for indice in 0..mesh_indices.len() {
            vertices_id.push(mesh_indices.at(indice));
        }

        MeshMapping::new(vertices, vertices_id, mesh_vertices.len())
    }

    pub fn create_mesh(&self) -> Mesh {
        let mut fragment_mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::MAIN_WORLD,
        );

        //set vertices
        fragment_mesh.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            MeshMapping::into_bevy_vertices(&self.vertex()),
        );

        // //set normals
        // fragment_top.insert_attribute(Mesh::ATTRIBUTE_NORMAL);

        // //set uv
        // fragment_top.insert_attribute(Mesh::ATTRIBUTE_UV_0);

        fragment_mesh
    }

    pub fn into_bevy_vertices(vertex_buffer: &Vec<Vec3A>) -> Vec<[f32; 3]> {
        let mut vertices: Vec<[f32; 3]> = vec![[0., 0., 0.]; vertex_buffer.len()];
        for (index, vertex) in vertex_buffer.iter().enumerate() {
            vertices[index] = vertex.to_array();
        }

        vertices
    }

    pub fn index(&self) -> &Vec<VertexId> {
        &self.index_buffer
    }

    pub fn add_triangle(&mut self, i1: VertexId, i2: VertexId, i3: VertexId) {
        let _ = &self.index_buffer.push(i1);
        let _ = &self.index_buffer.push(i2);
        let _ = &self.index_buffer.push(i3);
    }

    pub fn add_mapped_vertex(&mut self, vertex: &Vec3A, vertex_id: VertexId) {
        let _ = &self.vertex_buffer.push(*vertex);
        self.index_buffer.push(vertex_id);
        // TODO Mapping
    }
}
