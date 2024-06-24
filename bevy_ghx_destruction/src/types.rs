use bevy::{
    math::{Vec2, Vec3A},
    prelude::Entity,
    render::{
        mesh::{Indices, PrimitiveTopology, VertexAttributeValues},
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

    pub fn origin(&self) -> Vec3A {
        *&self.origin_point
    }

    pub fn normal(&self) -> Vec3A {
        *&self.normal_vec
    }
}

#[derive(Debug, Clone, Copy)]
pub struct MeshBuilderVertex {
    // TODO struct of arrays would probably be better
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
pub struct SlicedMesh {
    vertices: Vec<MeshBuilderVertex>,
    indices: Vec<VertexId>,
    sliced_face_vertices: Vec<MeshBuilderVertex>,
    /// Mapping which gives the id of a vertex in the mesh slice from the id of the vertex in the mesh being sliced. Sparse array
    index_map: Vec<VertexId>,
    constraints: Vec<Edge>,
}

impl SlicedMesh {
    pub fn new(
        vertices: Vec<MeshBuilderVertex>,
        indices: Vec<VertexId>,
        index_map: Vec<VertexId>,
    ) -> SlicedMesh {
        SlicedMesh {
            vertices,
            indices,
            index_map,
            sliced_face_vertices: Vec::new(), // TODO Capacity ?
            constraints: Vec::new(),          // TODO Capacity ?
        }
    }

    pub fn initialize_with(mesh: &SlicedMesh) -> SlicedMesh {
        SlicedMesh::new(
            Vec::new(), // TODO Capacity ?
            Vec::new(), // TODO Capacity ?
            // TODO Do we need a custom type to use as an option ?
            // Index map is sparse
            vec![0; mesh.vertices().len() + mesh.sliced_face_vertices().len()],
        )
    }

    pub fn from_bevy_mesh(mesh: &bevy::render::mesh::Mesh) -> SlicedMesh {
        let pos: Vec<Vec3A> = Self::vertices_from_mesh(mesh); // TODO Do we really need to copy it ?
        let uv = Self::uv_from_mesh(mesh);
        let normal = Self::normal_from_mesh(mesh);

        let mut vertices = Vec::new();
        for index in 0..pos.len() {
            vertices.push(MeshBuilderVertex::new(pos[index], uv[index], normal[index]));
        }

        let index_map: Vec<VertexId> = vec![0; vertices.len()];
        let indices: Vec<VertexId> = Self::triangles_from_mesh(mesh);

        SlicedMesh::new(vertices, indices, index_map)
    }

    pub fn constraints(&self) -> &Vec<Edge> {
        &self.constraints
    }
    pub fn constraints_mut(&mut self) -> &mut Vec<Edge> {
        &mut self.constraints
    }

    pub fn vert(&self, vert_index: VertexId) -> &MeshBuilderVertex {
        &self.vertices[vert_index as usize]
    }

    pub fn vertices(&self) -> &Vec<MeshBuilderVertex> {
        &self.vertices
    }

    pub fn vertices_mut(&mut self) -> &mut Vec<MeshBuilderVertex> {
        &mut self.vertices
    }

    pub fn sliced_face_vertices(&self) -> &Vec<MeshBuilderVertex> {
        &self.sliced_face_vertices
    }

    // pub fn sliced_vertices_mut(&mut self, sliced_vertex: &mut MeshBuilderVertex, id: VertexId) {
    //     *&mut self.sliced_vertices[id] = *sliced_vertex;
    // }

    pub fn sliced_face_vertices_mut(&mut self) -> &mut Vec<MeshBuilderVertex> {
        &mut self.sliced_face_vertices
    }

    pub fn indices(&self) -> &Vec<VertexId> {
        &self.indices
    }

    pub fn index_map(&self) -> &Vec<VertexId> {
        &self.index_map
    }

    pub fn add_sliced_vertex(&mut self, pos: Vec3A, uv: Vec2, normal: Vec3A) {
        let vertex = MeshBuilderVertex::new(pos, uv, normal);
        self.vertices.push(vertex);
        self.sliced_face_vertices.push(vertex);
    }

    pub fn push_mapped_vertex(&mut self, vertex: MeshBuilderVertex, original_vertex_id: usize) {
        self.vertices.push(vertex);
        self.index_map[original_vertex_id] = self.vertices.len() as VertexId - 1;
    }

    pub fn push_mapped_triangle(&mut self, v1: VertexId, v2: VertexId, v3: VertexId) {
        self.indices.push(self.index_map[v1 as usize]);
        self.indices.push(self.index_map[v2 as usize]);
        self.indices.push(self.index_map[v3 as usize]);
    }

    pub fn push_triangle(&mut self, v1: VertexId, v2: VertexId, v3: VertexId) {
        self.indices.push(v1);
        self.indices.push(v2);
        self.indices.push(v3);
    }

    pub fn triangles_from_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<VertexId> {
        let mut triangles: Vec<VertexId> = Vec::new();
        let indices = mesh.indices().unwrap();

        for index in 0..indices.len() {
            triangles.push(indices.at(index) as VertexId);
        }

        triangles
    }

    pub fn vertices_from_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<Vec3A> {
        mesh.attribute(bevy::render::mesh::Mesh::ATTRIBUTE_POSITION)
            .unwrap()
            .as_float3()
            .unwrap()
            .iter()
            .map(|v| Vec3A::from_array(*v))
            .collect()
    }

    pub fn uv_from_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<Vec2> {
        let uv = match mesh
            .attribute(bevy::render::mesh::Mesh::ATTRIBUTE_UV_0)
            .unwrap()
        {
            VertexAttributeValues::Float32x2(values) => Some(values),
            _ => None,
        }
        .unwrap();
        uv.clone().iter().map(|v| Vec2::from_array(*v)).collect()
    }

    pub fn normal_from_mesh(mesh: &bevy::render::mesh::Mesh) -> Vec<Vec3A> {
        let normal = match mesh
            .attribute(bevy::render::mesh::Mesh::ATTRIBUTE_NORMAL)
            .unwrap()
        {
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

    // //todo: use ordered float
    // pub fn shrink_sliced_vertices(&mut self) {
    //     let mut shrink_vertices: Vec<MeshBuilderVertex> =
    //         Vec::with_capacity(self.sliced_vertices.len());

    //     let mut index_map = vec![0; self.sliced_vertices.len()];

    //     let mut k = 0;

    //     for i in 0..self.sliced_vertices.len() {
    //         let mut duplicate = false;
    //         for j in 0..shrink_vertices.len() {
    //             if self.sliced_vertices[i].pos == shrink_vertices[j].pos {
    //                 index_map[i] = j;
    //                 duplicate = true;
    //                 break;
    //             }
    //         }

    //         if !duplicate {
    //             shrink_vertices.push(self.sliced_vertices[i].clone());
    //             index_map[i] = k;
    //             k += 1;
    //         }
    //     }

    //     for edge in self.constraints.iter_mut() {
    //         edge.from = index_map[edge.from as usize] as VertexId;
    //         edge.to = index_map[edge.to as usize] as VertexId;
    //     }

    //     shrink_vertices.shrink_to_fit();

    //     self.sliced_vertices = shrink_vertices;
    // }

    pub fn to_bevy_mesh(&self) -> bevy::render::mesh::Mesh {
        let fragment_mesh = bevy::render::mesh::Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(
            bevy::render::mesh::Mesh::ATTRIBUTE_POSITION,
            SlicedMesh::into_bevy_vertices(&self.vertices(), &self.sliced_face_vertices()),
        )
        .with_inserted_attribute(
            bevy::render::mesh::Mesh::ATTRIBUTE_UV_0,
            SlicedMesh::into_bevy_uvs(&self.vertices(), &self.sliced_face_vertices()),
        )
        .with_inserted_attribute(
            bevy::render::mesh::Mesh::ATTRIBUTE_NORMAL,
            SlicedMesh::into_bevy_normal(&self.vertices(), &self.sliced_face_vertices()),
        )
        .with_inserted_indices(Indices::U32(SlicedMesh::into_bevy_indices(&self.indices())));
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
        &self.indices
    }
}
