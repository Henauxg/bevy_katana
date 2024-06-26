use bevy::render::mesh::Mesh as RenderMesh;
use bevy::render::primitives::Aabb;
use bevy::{
    math::{Vec2, Vec3A},
    prelude::Entity,
    render::{
        mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
        render_asset::RenderAssetUsages,
    },
};
use bevy_rapier3d::dynamics::{ImpulseJoint, RigidBody};
use ghx_constrained_delaunay::hashbrown::HashMap;
use ghx_constrained_delaunay::types::VertexId;
use glam::Vec3;

use crate::utils::is_above_plane;

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

#[derive(PartialEq, Eq, Hash, Clone, Copy, Debug)]
pub enum PlaneSide {
    Top,
    Bottom,
    OnPlane,
}

#[derive(PartialEq, Eq, Clone, Copy, Debug, Hash)]
pub enum LinkType {
    Broken,
    Fixed,
    Nil,
}

/// Junction between two chunks. A link is oriented: from -> to. A link is rather fixed (do exist), broken or nil(do not exist)
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

/// Contains the chunk, its links, the chunk's neihbor and its rigid body
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
        // Remove the broken link from the list of all the links of the node
        let link = self.links.remove(
            self.links
                .iter()
                .position(|x| *x == link)
                .expect("can't find the link in the node"),
        );

        // Remove the neighbor which do not have any connection with this node anymore
        self.neighbors.remove(
            self.neighbors
                .iter()
                .position(|x| *x == link.to)
                .expect("can't find the neighbor of the node for the current link"),
        );
        link
    }

    fn unfreeze(&mut self) {
        // Allow the chunk to move
        self.rigid_body = RigidBody::Dynamic;
    }

    pub fn clean_node(&mut self) {
        // List of all the broken links
        let broken_links: Vec<_> = self
            .links
            .iter()
            .filter(|link| link.link_type == LinkType::Broken)
            .cloned()
            .collect();

        // Remove the broken links from the node and the connections between the node and the neighbors
        for link in broken_links {
            let mut broken_link = self.clean_links(link.clone());
            broken_link.to.clean_links(link);
        }

        //No more broken links inside the node
        self.has_broken_links = false;
    }
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
        // Look for the node to be removed from the graph
        self.graph.remove(
            self.graph
                .iter()
                .position(|x| *x == node)
                .expect("can't find the node in the graph"),
        );

        //Since the node is not connected to the graph, we can allow the chunk to move
        node.unfreeze();
    }

    /// Update the graph node only if a link is broken
    pub fn update_graph(&mut self) {
        // Look for all nodes with broken links
        for node in self.graph.iter_mut() {
            if node.has_broken_links {
                // Clean the node from its broken links
                node.clean_node();
            }
        }
        // Update the graph's connections
        self.clean_graph();
    }

    fn clean_graph(&mut self) {
        // Look for all the fixed chunks
        let fixed_nodes: Vec<Node> = self
            .graph
            .iter()
            .filter(|node| node.rigid_body == RigidBody::Fixed)
            .cloned()
            .collect();

        let mut search = self.graph.clone();

        // For each nodes in the current searching process, we look at each of them are not connected to on of those current nodes. For those remaining,we do reapeat theproces until all nodes are not connected together or if there is no more nodes in thesearching process
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

        // For all the remaining nodes, we allow their respecting chunks to move
        for restricted_node in search {
            self.remove_node_from_graph(restricted_node);
        }
    }

    //TODO: no recursive
    fn travel(&self, fixed_node: &Node, search: &Vec<Node>, visited: &mut Vec<Node>) {
        // Look at all the conections from the main node. Thos node would be remove from the searching process
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

    #[inline]
    pub fn origin(&self) -> Vec3A {
        *&self.origin_point
    }

    #[inline]
    pub fn normal(&self) -> Vec3A {
        *&self.normal_vec
    }
}

pub type Generation = u16;
const UNSET_GENERATION: Generation = 0;
const INITIAL_GENERATION: Generation = 1;

#[derive(Debug, Clone)]
pub struct VertexSideData {
    pub side: PlaneSide,
    pub generation: Generation,
}

const UNSET_SIDE_DATA: VertexSideData = VertexSideData {
    side: PlaneSide::Bottom,
    generation: UNSET_GENERATION,
};

/// Global vertices positions array containing all the original mesh vertices + all the vertices created during an iterative slice), during the slicing process a fragment only needs to index into this global array. Only reconstruct data when exporting the fragments as output mesh format
#[derive(Debug, Clone)]
pub struct SlicedMeshData {
    buffers: MeshBuffers,
    generation: Generation,
    sides: Vec<VertexSideData>,
}
impl SlicedMeshData {
    pub(crate) fn get_current_side_data(&mut self, v: VertexId, plane: Plane) -> PlaneSide {
        let side = if self.sides[v as usize].generation == self.generation {
            self.sides[v as usize].side
        } else {
            self.sides[v as usize].side =
                is_above_plane(self.buffers.positions()[v as usize].into(), plane);
            self.sides[v as usize].generation = self.generation;
            self.sides[v as usize].side
        };
        side
    }

    #[inline]
    pub(crate) fn buffers(&self) -> &MeshBuffers {
        &self.buffers
    }

    #[inline]
    pub(crate) fn pos(&self, v: VertexId) -> Vec3 {
        self.buffers.positions()[v as usize]
    }

    #[inline]
    pub(crate) fn normal(&self, v: VertexId) -> Vec3A {
        self.buffers.normals()[v as usize]
    }

    #[inline]
    pub(crate) fn uv(&self, v: VertexId) -> Vec2 {
        self.buffers.uvs()[v as usize]
    }

    pub(crate) fn push_new_vertex(&mut self, pos: Vec3, uv: Vec2, norm: Vec3A) -> VertexId {
        self.buffers.positions.push(pos);
        self.buffers.uvs.push(uv);
        self.buffers.normals.push(norm);

        self.sides.push(UNSET_SIDE_DATA);

        // TODO Check VertexId size
        (self.buffers.positions.len() - 1) as VertexId
    }

    pub(crate) fn from_bevy_render_mesh(mesh: &RenderMesh) -> Self {
        let buffers = MeshBuffers::from_bevy_render_mesh(mesh);
        let sides = vec![UNSET_SIDE_DATA; buffers.positions().len()];
        Self {
            buffers,
            generation: INITIAL_GENERATION,
            sides,
        }
    }

    pub(crate) fn update_generation(&mut self) {
        if self.generation == Generation::MAX {
            self.generation = INITIAL_GENERATION;
            // Reset sides buffer to unset data
            for s in self.sides.iter_mut() {
                *s = UNSET_SIDE_DATA;
            }
        } else {
            self.generation += 1;
        }
    }
}

#[derive(Debug, Clone)]
pub struct MeshBuffers {
    positions: Vec<Vec3>,
    normals: Vec<Vec3A>,
    uvs: Vec<Vec2>,
}
impl MeshBuffers {
    pub fn new() -> Self {
        Self {
            positions: Vec::new(),
            normals: Vec::new(),
            uvs: Vec::new(),
        }
    }

    pub fn from_buffers(positions: Vec<Vec3>, normals: Vec<Vec3A>, uvs: Vec<Vec2>) -> Self {
        Self {
            positions,
            normals,
            uvs,
        }
    }

    pub fn from_bevy_render_mesh(mesh: &RenderMesh) -> Self {
        // TODO Error & format handling
        let positions: Vec<Vec3> = vertices_from_bevy_mesh(mesh);
        let uvs = uv_from_bevy_mesh(mesh);
        let normals = normal_from_bevy_mesh(mesh);
        Self::from_buffers(positions, normals, uvs)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            positions: Vec::with_capacity(capacity),
            normals: Vec::with_capacity(capacity),
            uvs: Vec::with_capacity(capacity),
        }
    }

    #[inline]
    pub fn positions(&self) -> &Vec<Vec3> {
        &self.positions
    }

    #[inline]
    pub fn normals(&self) -> &Vec<Vec3A> {
        &self.normals
    }

    #[inline]
    pub fn uvs(&self) -> &Vec<Vec2> {
        &self.uvs
    }
}

#[derive(Clone)]
pub struct Submesh {
    indices: Vec<VertexId>,
    cached_aabb: Option<Aabb>,
}
impl Submesh {
    pub fn new() -> Self {
        Self {
            // TODO Capacity hint ?
            indices: Vec::new(),
            cached_aabb: None,
        }
    }

    pub fn from_bevy_render_mesh(mesh: &RenderMesh) -> Self {
        Self {
            // TODO Handle errors
            indices: indices_from_bevy_mesh(mesh),
            cached_aabb: None,
        }
    }

    #[inline]
    pub fn cached_aabb(&self) -> Option<Aabb> {
        self.cached_aabb
    }

    pub fn compute_aabb(&mut self, positions: &Vec<Vec3>) -> Aabb {
        let aabb = Aabb::enclosing(self.indices.iter().map(|i| positions[*i as usize])).unwrap();
        self.cached_aabb = Some(aabb);
        aabb
    }

    #[inline]
    pub fn indices(&self) -> &Vec<VertexId> {
        &self.indices
    }

    #[inline]
    pub fn indices_mut(&mut self) -> &mut Vec<VertexId> {
        &mut self.indices
    }

    pub fn to_bevy_render_mesh(&self, sliced_mesh_data: &SlicedMeshData) -> RenderMesh {
        // TODO Optimization: may share a global allocated LUT for all the fragments generated by multiple slice iterations. (Use an array of custom packed Option type) + generational index
        let mut global_to_local_index_mapping = HashMap::<VertexId, u32>::new();

        // TODO Capacity is a bit overboard here for pos, uvs & normals
        let mut positions = Vec::with_capacity(self.indices.len());
        let mut uvs = Vec::with_capacity(self.indices.len());
        let mut normals = Vec::with_capacity(self.indices.len());
        let mut local_indices: Vec<u32> = Vec::with_capacity(self.indices.len());
        for global_index in self.indices.iter() {
            // TODO Some type safety for u32 indices
            match global_to_local_index_mapping.get(global_index) {
                None => {
                    // Not present yet, insert all vertex data
                    positions.push(sliced_mesh_data.pos(*global_index).to_array());
                    uvs.push(sliced_mesh_data.uv(*global_index).to_array());
                    normals.push(sliced_mesh_data.normal(*global_index).to_array());
                    local_indices.push(positions.len() as u32 - 1);
                    global_to_local_index_mapping.insert(*global_index, positions.len() as u32 - 1);
                }
                Some(local_index) => {
                    local_indices.push(*local_index);
                }
            };
        }

        let mesh = RenderMesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(RenderMesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(RenderMesh::ATTRIBUTE_UV_0, uvs)
        .with_inserted_attribute(RenderMesh::ATTRIBUTE_NORMAL, normals)
        // TODO May sometimes use U16 ?
        .with_inserted_indices(Indices::U32(local_indices));
        mesh
    }
}

pub fn vertices_from_bevy_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<Vec3> {
    // TODO Handle unwraps
    mesh.attribute(bevy::render::mesh::Mesh::ATTRIBUTE_POSITION)
        .unwrap()
        .as_float3()
        .unwrap()
        .iter()
        .map(|v| Vec3::from_array(*v))
        .collect()
}

pub fn uv_from_bevy_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<Vec2> {
    // TODO Handle unwrap
    let uv = match mesh
        .attribute(bevy::render::mesh::Mesh::ATTRIBUTE_UV_0)
        .unwrap()
    {
        VertexAttributeValues::Float32x2(values) => Some(values),
        _ => None,
    }
    .unwrap();
    uv.iter().map(|v| Vec2::from_array(*v)).collect()
}

pub fn normal_from_bevy_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<Vec3A> {
    // TODO Handle unwrap
    let normal = match mesh
        .attribute(bevy::render::mesh::Mesh::ATTRIBUTE_NORMAL)
        .unwrap()
    {
        VertexAttributeValues::Float32x3(values) => Some(values),
        _ => None,
    }
    .unwrap();
    normal.iter().map(|v| Vec3A::from_array(*v)).collect()
}

pub fn indices_from_bevy_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<VertexId> {
    // TODO Handle unwrap
    let indices = mesh.indices().unwrap();
    indices.iter().map(|i| i as VertexId).collect()
}
