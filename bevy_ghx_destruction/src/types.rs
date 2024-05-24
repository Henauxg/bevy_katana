use bevy::{
    math::{Vec2, Vec3A},
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
    origin_point: Vec3A,
    normal_vec: Vec3A,
}

impl Plane {
    #[inline]
    pub fn new(origin_point: Vec3A, normal_vec: Vec3A) -> Self {
        Self {
            origin_point,
            normal_vec,
        }
    }

    pub fn origin(&self) -> Vec3A {
        *&self.origin_point
    }

    pub fn normal(&self) -> Vec3A {
        *&self.normal_vec
    }
}

#[derive(Debug, Clone)]
pub struct MeshMapping {
    vertex_buffer: Vec<Vec3A>,
    index_buffer: Vec<VertexId>,
    uv_buffer: Vec<Vec2>,
    normal_buffer: Vec<Vec3A>,
    initial_size: usize,
}

impl MeshMapping {
    pub fn new(
        vertex_buffer: Vec<Vec3A>,
        index_buffer: Vec<VertexId>,
        uv_buffer: Vec<Vec2>,
        normal_buffer: Vec<Vec3A>,
        initial_size: usize,
    ) -> MeshMapping {
        MeshMapping {
            vertex_buffer,
            index_buffer,
            uv_buffer,
            normal_buffer,
            initial_size,
        }
    }

    pub fn initial_size(&self) -> &usize {
        &self.initial_size
    }

    pub fn vertex(&self) -> &Vec<Vec3A> {
        &self.vertex_buffer
    }

    pub fn uv(&self) -> &Vec<Vec2> {
        &self.uv_buffer
    }

    pub fn normal(&self) -> &Vec<Vec3A> {
        &self.normal_buffer
    }

    pub fn vertex_mut(&mut self) -> &mut Vec<Vec3A> {
        &mut self.vertex_buffer
    }

    pub fn uv_mut(&mut self) -> &mut Vec<Vec2> {
        &mut self.uv_buffer
    }

    pub fn normal_mut(&mut self) -> &mut Vec<Vec3A> {
        &mut self.normal_buffer
    }

    pub fn to_mesh_mapping(
        indexes: Vec<VertexId>,
        uvs: &mut Vec<Vec2>,
        normal: &mut Vec<Vec3A>,
        mesh_mapping: &mut MeshMapping,
    ) -> MeshMapping {
        mesh_mapping.uv_mut().append(uvs);
        mesh_mapping.normal_mut().append(normal);

        let mut mesh_mapping_vertices = vec![Vec3A::ZERO; indexes.len()];
        let mut mesh_mapping_index: Vec<VertexId> = vec![0; indexes.len()];
        let mut mesh_mapping_uvs = vec![Vec2::ZERO; indexes.len()];
        let mut mesh_mapping_normals = vec![Vec3A::ZERO; indexes.len()];

        for (i, index) in indexes.iter().enumerate() {
            mesh_mapping_vertices[i] = mesh_mapping.vertex()[*index];
            mesh_mapping_index[i] = i;
            mesh_mapping_uvs[i] = mesh_mapping.uv()[*index];
            mesh_mapping_normals[i] = mesh_mapping.normal()[*index];
        }

        MeshMapping::new(
            mesh_mapping_vertices,
            mesh_mapping_index,
            mesh_mapping_uvs,
            mesh_mapping_normals,
            mesh_mapping.vertex().len(),
        )
    }

    pub fn mesh_mapping_from_mesh(mesh: &Mesh) -> MeshMapping {
        let vertices: Vec<Vec3A> = mesh
            .attribute(Mesh::ATTRIBUTE_POSITION)
            .unwrap()
            .as_float3()
            .unwrap()
            .iter()
            .map(|v| Vec3A::from_array(*v))
            .collect();

        // Get the vertices indexes of the current mesh
        let mesh_indices = mesh.indices().unwrap();

        let mut vertices_id = Vec::new();

        for indice in 0..mesh_indices.len() {
            vertices_id.push(mesh_indices.at(indice));
        }

        let uv = match mesh.attribute(Mesh::ATTRIBUTE_UV_0).unwrap() {
            VertexAttributeValues::Float32x2(values) => Some(values),
            _ => None,
        }
        .unwrap();

        let normal = match mesh.attribute(Mesh::ATTRIBUTE_NORMAL).unwrap() {
            VertexAttributeValues::Float32x3(values) => Some(values),
            _ => None,
        }
        .unwrap();

        MeshMapping::new(
            vertices,
            vertices_id,
            uv.clone().iter().map(|v| Vec2::from_array(*v)).collect(),
            normal
                .clone()
                .iter()
                .map(|v| Vec3A::from_array(*v))
                .collect(),
            mesh.attribute(Mesh::ATTRIBUTE_POSITION)
                .unwrap()
                .as_float3()
                .unwrap()
                .len(),
        )
    }

    pub fn create_mesh(&self) -> Mesh {
        let fragment_mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_POSITION,
            MeshMapping::into_bevy_vertices(&self.vertex()),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, MeshMapping::into_bevy_uvs(&self.uv()))
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_NORMAL,
            MeshMapping::into_bevy_normal(&self.normal()),
        )
        .with_inserted_indices(Indices::U32(MeshMapping::into_bevy_indices(&self.index())));
        fragment_mesh
    }

    pub fn into_bevy_vertices(vertex_buffer: &Vec<Vec3A>) -> Vec<[f32; 3]> {
        vertex_buffer.iter().map(|v| v.to_array()).collect()
    }

    pub fn into_bevy_indices(index_buffer: &Vec<VertexId>) -> Vec<u32> {
        index_buffer.iter().map(|i| *i as u32).collect()
    }

    pub fn into_bevy_uvs(uvs_buffer: &Vec<Vec2>) -> Vec<[f32; 2]> {
        uvs_buffer.iter().map(|i| i.to_array()).collect()
    }

    pub fn into_bevy_normal(normal_buffer: &Vec<Vec3A>) -> Vec<[f32; 3]> {
        normal_buffer.iter().map(|i| i.to_array()).collect()
    }

    pub fn index(&self) -> &Vec<VertexId> {
        &self.index_buffer
    }
}
