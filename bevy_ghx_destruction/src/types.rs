use std::collections::HashSet;

use bevy::{
    math::{Vec2, Vec3A},
    prelude::Entity,
    render::{
        mesh::{Indices, Mesh, PrimitiveTopology, VertexAttributeValues},
        render_asset::RenderAssetUsages,
    },
};
use bevy_rapier3d::dynamics::{ImpulseJoint, RigidBody};
use ghx_constrained_delaunay::types::{Edge, VertexId};

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

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum CutDirection {
    Top,
    Bottom,
    OnPlane,
    Unknow,
}

#[derive(PartialEq, Eq, Clone, Copy, Debug, Hash)]
pub enum LinkType {
    Broken,
    Fixed,
    Nil,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Link {
    from: Node,
    to: Node,
    joint: ImpulseJoint,
    link_type: LinkType,
}

impl Link {
    pub fn new(from: Node, to: Node, joint: ImpulseJoint) -> Link {
        Link {
            from: from,
            to: to,
            joint: joint,
            link_type: LinkType::Nil,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Node {
    chunk: Entity,
    links: Vec<Link>,
    neighbors: Vec<Node>,
    rigid_body: RigidBody,
    has_broken_links: bool,
}

impl Node {
    pub fn new(chunk: Entity) -> Node {
        Node {
            chunk: chunk,
            links: Vec::new(),
            neighbors: Vec::new(),
            rigid_body: RigidBody::Fixed, // freeze
            has_broken_links: false,
        }
    }

    fn broken_links(&mut self) {
        self.has_broken_links = true;
    }

    fn clean_links(&mut self, link: Link) -> Link {
        let link = self.links.remove(
            self.links
                .iter()
                .position(|x| *x == link)
                .expect("not such element in links"), //TODO: better error log
        );

        self.neighbors.remove(
            self.neighbors
                .iter()
                .position(|x| *x == link.to)
                .expect("not such element in neighbors"), //TODO: better error log
        );
        link
    }

    fn unfreeze(&mut self) {
        self.rigid_body = RigidBody::Dynamic;
    }

    pub fn clean_node(&mut self) {
        let broken_links: Vec<_> = self
            .links
            .iter()
            .filter(|link| link.link_type == LinkType::Broken)
            .cloned()
            .collect();

        for link in broken_links {
            let mut broken_link = self.clean_links(link.clone());
            broken_link.to.clean_links(link);
        }

        self.has_broken_links = false;
    }
}

pub enum GraphUpdate {
    Change,
    UnChange,
}

#[derive(Debug, Clone)]
pub struct ChunkGraph {
    graph: Vec<Node>,
}

impl ChunkGraph {
    pub fn new(graph: Vec<Node>) -> ChunkGraph {
        ChunkGraph { graph: graph }
    }

    fn remove_node_from_graph(&mut self, mut node: Node) {
        self.graph.remove(
            self.graph
                .iter()
                .position(|x| *x == node)
                .expect("not such element in neighbors"),
        );
        node.unfreeze();
    }

    pub fn update_graph(&mut self) -> GraphUpdate {
        let mut update = false;
        for node in self.graph.iter_mut() {
            if node.has_broken_links {
                node.clean_node();
                update = true;
            }
        }

        if update {
            self.clean_graph();
            GraphUpdate::Change
        } else {
            GraphUpdate::UnChange
        }
    }

    fn clean_graph(&mut self) {
        let fixed_nodes: Vec<Node> = self
            .graph
            .iter()
            .filter(|node| node.rigid_body == RigidBody::Fixed)
            .cloned()
            .collect();

        let mut search = self.graph.clone();

        for fixed_node in fixed_nodes {
            if search.contains(&fixed_node) {
                let mut visited = Vec::new();
                self.travel(&fixed_node, &search, &mut visited);
                search = search
                    .iter()
                    .filter(|node| visited.contains(node) == false)
                    .cloned()
                    .collect();
            }
        }

        for restricted_node in search {
            self.remove_node_from_graph(restricted_node);
        }
    }

    //TODO: no recursive
    fn travel(&self, fixed_node: &Node, search: &Vec<Node>, visited: &mut Vec<Node>) {
        if search.contains(fixed_node) && !visited.contains(fixed_node) {
            visited.push(fixed_node.clone());

            for neighbor_id in 0..fixed_node.neighbors.len() {
                let neighbor = &fixed_node.neighbors[neighbor_id];
                self.travel(neighbor, search, visited);
            }
        }
    }
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

#[derive(Debug, Clone, Copy)]
pub struct MeshBuilderVertex {
    pos: Vec3A,
    uv: Vec2,
    normal: Vec3A,
}

impl MeshBuilderVertex {
    pub fn new(pos: Vec3A, uv: Vec2, normal: Vec3A) -> MeshBuilderVertex {
        MeshBuilderVertex { pos, uv, normal }
    }

    pub fn pos(&self) -> Vec3A {
        *&self.pos
    }

    pub fn uv(&self) -> Vec2 {
        *&self.uv
    }

    pub fn uv_mut(&mut self) -> &mut Vec2 {
        &mut self.uv
    }

    pub fn normal(&self) -> Vec3A {
        *&self.normal
    }

    // pub fn normal_mut(&mut self, normal: &mut Vec3A) {
    //     *&mut self.normal = *normal;
    // }

    pub fn normal_mut(&mut self) -> &mut Vec3A {
        &mut self.normal
    }
}

#[derive(Debug, Clone)]
pub struct MeshBuilder {
    vertices: Vec<MeshBuilderVertex>,
    sliced_vertices: Vec<MeshBuilderVertex>,
    triangles: Vec<VertexId>,
    index_map: Vec<VertexId>,
    constraints: Vec<Edge>,
}

impl MeshBuilder {
    pub fn new(
        vertices: Vec<MeshBuilderVertex>,
        sliced_vertices: Vec<MeshBuilderVertex>,
        triangles: Vec<VertexId>,
        index_map: Vec<VertexId>,
        constraints: Vec<Edge>,
    ) -> MeshBuilder {
        MeshBuilder {
            vertices,
            sliced_vertices,
            triangles,
            index_map,
            constraints,
        }
    }

    pub fn new_from(mesh_mapping: &MeshBuilder) -> MeshBuilder {
        MeshBuilder::new_from_mesh_builder(
            mesh_mapping.vertices().len() + mesh_mapping.sliced_vertices().len(),
        )
    }

    pub fn new_from_mesh_builder(vertex_count: usize) -> MeshBuilder {
        MeshBuilder {
            vertices: Vec::new(),
            sliced_vertices: Vec::new(),
            triangles: Vec::new(),
            index_map: vec![0; vertex_count],
            constraints: Vec::new(),
        }
    }

    pub fn constraints(&self) -> &Vec<Edge> {
        &self.constraints
    }
    pub fn constraints_mut(&mut self) -> &mut Vec<Edge> {
        &mut self.constraints
    }

    pub fn vertices(&self) -> &Vec<MeshBuilderVertex> {
        &self.vertices
    }

    pub fn sliced_vertices(&self) -> &Vec<MeshBuilderVertex> {
        &self.sliced_vertices
    }

    // pub fn sliced_vertices_mut(&mut self, sliced_vertex: &mut MeshBuilderVertex, id: VertexId) {
    //     *&mut self.sliced_vertices[id] = *sliced_vertex;
    // }

    pub fn sliced_vertices_mut(&mut self) -> &mut Vec<MeshBuilderVertex> {
        &mut self.sliced_vertices
    }

    pub fn triangles(&self) -> &Vec<VertexId> {
        &self.triangles
    }

    pub fn index_map(&self) -> &Vec<VertexId> {
        &self.index_map
    }

    pub fn add_sliced_vertex(&mut self, pos: Vec3A, uv: Vec2, normal: Vec3A) {
        let vertex = MeshBuilderVertex::new(pos, uv, normal);
        self.vertices.push(vertex);
        self.sliced_vertices.push(vertex);
    }

    pub fn add_mapped_vertex(&mut self, vertex: MeshBuilderVertex, index: usize) {
        self.vertices.push(vertex);
        self.index_map[index] = self.vertices.len() as VertexId - 1;
    }

    pub fn push_mapped_triangle(&mut self, v1: VertexId, v2: VertexId, v3: VertexId) {
        self.triangles.push(self.index_map[v1 as usize]);
        self.triangles.push(self.index_map[v2 as usize]);
        self.triangles.push(self.index_map[v3 as usize]);
    }

    pub fn push_triangle(&mut self, v1: VertexId, v2: VertexId, v3: VertexId) {
        self.triangles.push(v1);
        self.triangles.push(v2);
        self.triangles.push(v3);
    }

    pub fn mesh_mapping_from_mesh(mesh: &Mesh) -> MeshBuilder {
        let pos: Vec<Vec3A> = Self::vertex_from_mesh(mesh);
        let uv = Self::uv_from_mesh(mesh);
        let normal = Self::normal_from_mesh(mesh);

        let mut vertices = Vec::new();
        for index in 0..pos.len() {
            vertices.push(MeshBuilderVertex::new(pos[index], uv[index], normal[index]));
        }

        let sliced_vertices = Vec::new();

        let constraints = Vec::new();

        let index_map: Vec<VertexId> = vec![0; vertices.len()];

        let triangles: Vec<VertexId> = Self::triangles_from_mesh(mesh);

        MeshBuilder::new(vertices, sliced_vertices, triangles, index_map, constraints)
    }

    pub fn triangles_from_mesh(mesh: &Mesh) -> Vec<VertexId> {
        let mut triangles: Vec<VertexId> = Vec::new();
        let indices = mesh.indices().unwrap();

        for index in 0..indices.len() {
            triangles.push(indices.at(index) as VertexId);
        }

        triangles
    }

    pub fn vertex_from_mesh(mesh: &Mesh) -> Vec<Vec3A> {
        mesh.attribute(Mesh::ATTRIBUTE_POSITION)
            .unwrap()
            .as_float3()
            .unwrap()
            .iter()
            .map(|v| Vec3A::from_array(*v))
            .collect()
    }

    pub fn uv_from_mesh(mesh: &Mesh) -> Vec<Vec2> {
        let uv = match mesh.attribute(Mesh::ATTRIBUTE_UV_0).unwrap() {
            VertexAttributeValues::Float32x2(values) => Some(values),
            _ => None,
        }
        .unwrap();
        uv.clone().iter().map(|v| Vec2::from_array(*v)).collect()
    }

    pub fn normal_from_mesh(mesh: &Mesh) -> Vec<Vec3A> {
        let normal = match mesh.attribute(Mesh::ATTRIBUTE_NORMAL).unwrap() {
            VertexAttributeValues::Float32x3(values) => Some(values),
            _ => None,
        }
        .unwrap();
        normal
            .clone()
            .iter()
            .map(|v| Vec3A::from_array(*v))
            .collect()
    }

    //todo: use ordered float
    pub fn shrink_sliced_vertices(&mut self) {
        let mut shrink_vertices: Vec<MeshBuilderVertex> =
            Vec::with_capacity(self.sliced_vertices.len());

        let mut index_map = vec![0; self.sliced_vertices.len()];

        let mut k = 0;

        for i in 0..self.sliced_vertices.len() {
            let mut duplicate = false;
            for j in 0..shrink_vertices.len() {
                if self.sliced_vertices[i].pos == shrink_vertices[j].pos {
                    index_map[i] = j;
                    duplicate = true;
                    break;
                }
            }

            if !duplicate {
                shrink_vertices.push(self.sliced_vertices[i].clone());
                index_map[i] = k;
                k += 1;
            }
        }

        for edge in self.constraints.iter_mut() {
            edge.from = index_map[edge.from as usize] as VertexId;
            edge.to = index_map[edge.to as usize] as VertexId;
        }

        shrink_vertices.shrink_to_fit();

        self.sliced_vertices = shrink_vertices;
    }

    pub fn create_mesh(&self) -> Mesh {
        let fragment_mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_POSITION,
            MeshBuilder::into_bevy_vertices(&self.vertices(), &self.sliced_vertices()),
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_UV_0,
            MeshBuilder::into_bevy_uvs(&self.vertices(), &self.sliced_vertices()),
        )
        .with_inserted_attribute(
            Mesh::ATTRIBUTE_NORMAL,
            MeshBuilder::into_bevy_normal(&self.vertices(), &self.sliced_vertices()),
        )
        .with_inserted_indices(Indices::U32(MeshBuilder::into_bevy_indices(
            &self.triangles(),
        )));
        fragment_mesh
    }

    pub fn into_bevy_vertices(
        vertex_buffer: &Vec<MeshBuilderVertex>,
        sliced_vertices: &Vec<MeshBuilderVertex>,
    ) -> Vec<[f32; 3]> {
        let mut vertices_uncut = Vec::new();
        for vertex in 0..vertex_buffer.len() {
            vertices_uncut.push(vertex_buffer[vertex].pos);
        }

        let mut vertices_cut = Vec::new();
        for vertex in 0..sliced_vertices.len() {
            vertices_cut.push(sliced_vertices[vertex].pos);
        }
        vertices_uncut.append(&mut vertices_cut);

        vertices_uncut.iter().map(|v| v.to_array()).collect()
    }

    pub fn into_bevy_indices(index_buffer: &Vec<VertexId>) -> Vec<u32> {
        index_buffer.iter().map(|i| *i as u32).collect()
    }

    pub fn into_bevy_uvs(
        vertex_buffer: &Vec<MeshBuilderVertex>,
        sliced_vertices: &Vec<MeshBuilderVertex>,
    ) -> Vec<[f32; 2]> {
        let mut vertices_uncut = Vec::new();
        for vertex in 0..vertex_buffer.len() {
            vertices_uncut.push(vertex_buffer[vertex].uv);
        }

        let mut vertices_cut = Vec::new();
        for vertex in 0..sliced_vertices.len() {
            vertices_cut.push(sliced_vertices[vertex].uv);
        }
        vertices_uncut.append(&mut vertices_cut);

        vertices_uncut.iter().map(|v| v.to_array()).collect()
    }

    pub fn into_bevy_normal(
        vertex_buffer: &Vec<MeshBuilderVertex>,
        sliced_vertices: &Vec<MeshBuilderVertex>,
    ) -> Vec<[f32; 3]> {
        let mut vertices_uncut = Vec::new();
        for vertex in 0..vertex_buffer.len() {
            vertices_uncut.push(vertex_buffer[vertex].normal);
        }

        let mut vertices_cut = Vec::new();
        for vertex in 0..sliced_vertices.len() {
            vertices_cut.push(sliced_vertices[vertex].normal);
        }

        vertices_uncut.append(&mut vertices_cut);

        vertices_uncut.iter().map(|v| v.to_array()).collect()
    }

    pub fn index(&self) -> &Vec<VertexId> {
        &self.triangles
    }
}
