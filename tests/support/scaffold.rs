//! Raw scaffolds carrying an ownership labelling, built by hand.
//!
//! Every fixture here drives the core's own edit primitives — `add_dart`,
//! `link` and `sew` inside one transaction — and registers no domain attribute
//! at all. What it adds beside the map is a [`Embedding`]: one record per raw
//! cell saying which logical entity's interior contains it. That pairing is the
//! whole subject of these fixtures, so the shapes are the smallest ones that
//! still have the feature being proved.
//!
//! The solid fixtures classify from the map alone: an `alpha3`-sewn face has
//! material on both sides and is a cut the solid owns, a free one bounds it.
#![allow(dead_code)]

use std::collections::{HashMap, HashSet};

use ngk::model::Model;
use ngk::topology::ModelEdit;
use ngk::topology::embedding::{
    Embedding, EmbeddingError, EmbeddingIndex, EntityOwner, LogicalRegion, recover_region,
};
use ngk::topology::gmap::{Dart, Dim, GMap};
use ngk::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use ngk::topology::{ModelEditError, StandardPayload};
use slotmap::SlotMap;

/// A model whose raw cells are labelled but which carries no geometry.
pub struct Scaffold {
    /// The model: a pure map, its labelling, and nothing else.
    pub model: Model<StandardPayload>,
    /// Every logical entity, in the order it was created.
    pub entities: Vec<EntityOwner>,
    /// Where each entity's own cell is.
    ///
    /// Held apart from `staged` for the reason a real model holds it apart from
    /// its embedding: a record says a cell is embedded in something larger,
    /// which an entity's own cell never is. A `Model` reads this half off its
    /// attribute stores; a fixture has none, so it keeps the list itself.
    anchors: Vec<(Dim, Dart, EntityOwner)>,
    staged: Embedding,
    vertices: SlotMap<VertexKey, ()>,
    edges: SlotMap<EdgeKey, ()>,
    faces: SlotMap<FaceKey, ()>,
    solids: SlotMap<SolidKey, ()>,
}

impl Scaffold {
    /// Wraps a built model, with nothing labelled yet.
    pub fn wrap(model: Model<StandardPayload>) -> Self {
        Self {
            model,
            entities: Vec::new(),
            anchors: Vec::new(),
            staged: Embedding::new(),
            vertices: SlotMap::with_key(),
            edges: SlotMap::with_key(),
            faces: SlotMap::with_key(),
            solids: SlotMap::with_key(),
        }
    }

    /// Allocates a logical vertex.
    pub fn vertex(&mut self) -> EntityOwner {
        let key = self.vertices.insert(());
        self.remember(EntityOwner::Vertex(key))
    }

    /// Allocates a logical edge.
    pub fn edge(&mut self) -> EntityOwner {
        let key = self.edges.insert(());
        self.remember(EntityOwner::Edge(key))
    }

    /// Allocates a logical face.
    pub fn face(&mut self) -> EntityOwner {
        let key = self.faces.insert(());
        self.remember(EntityOwner::Face(key))
    }

    /// Allocates a logical solid.
    pub fn solid(&mut self) -> EntityOwner {
        let key = self.solids.insert(());
        self.remember(EntityOwner::Solid(key))
    }

    /// Declares which entity the raw `dimension`-cell containing `dart` belongs
    /// to, as an anchor or as an embedded cell according to the dimensions.
    ///
    /// An entity named on a cell of its own dimension is anchored there; one
    /// named on a smaller cell is a record saying that cell is embedded in it.
    /// Callers say the same thing either way, and the split follows.
    pub fn own(&mut self, dimension: Dim, dart: Dart, owner: EntityOwner) {
        if owner.dimension() == dimension {
            self.anchors
                .retain(|&(held, at, _)| held != dimension || at != dart);
            self.anchors.push((dimension, dart, owner));
        } else {
            self.staged.own(dimension, dart, owner);
        }
    }

    /// Returns every anchor, for a caller building an index of its own.
    pub fn anchors(&self) -> impl Iterator<Item = (Dim, Dart, EntityOwner)> + '_ {
        self.anchors.iter().copied()
    }

    /// Stages a label for every raw cell of `dimension`.
    pub fn own_all(&mut self, dimension: Dim, owner: EntityOwner) {
        for cell in self.map().cells(dimension).collect::<Vec<_>>() {
            self.own(dimension, cell, owner);
        }
    }

    /// Gives each still unlabelled raw cell of `dimension` an entity of its own.
    pub fn own_each_remaining(&mut self, dimension: Dim, make: fn(&mut Self) -> EntityOwner) {
        let declared: Vec<Dart> = self
            .staged
            .records()
            .filter(|record| record.dimension == dimension)
            .map(|record| record.representative)
            .chain(
                self.anchors
                    .iter()
                    .filter(|&&(held, ..)| held == dimension)
                    .map(|&(_, dart, _)| dart),
            )
            .collect();
        let labelled: HashSet<Dart> = declared
            .into_iter()
            .flat_map(|dart| self.map().orbit(dart, self.map().orbit_indices(dimension)))
            .collect();
        let bare: Vec<Dart> = self
            .map()
            .cells(dimension)
            .filter(|cell| !labelled.contains(cell))
            .collect();

        for cell in bare {
            let owner = make(self);
            self.own(dimension, cell, owner);
        }
    }

    /// Commits every staged label into the model in one transaction.
    pub fn seal(mut self) -> Self {
        let staged = std::mem::take(&mut self.staged);
        self.model
            .transaction(|edit| {
                for record in staged.records() {
                    edit.own_cell(record.dimension, record.representative, record.owner);
                }
                Ok::<_, ModelEditError>(())
            })
            .expect("a fixture's labelling should commit");
        self
    }

    /// Returns the pure map under the model.
    pub fn map(&self) -> &GMap {
        self.model.topology()
    }

    /// Returns the committed labelling.
    pub fn embedding(&self) -> &Embedding {
        self.model.embedding()
    }

    /// Expands the committed labelling into the lookup the walkers read.
    pub fn index(&self) -> EmbeddingIndex {
        self.index_of(self.embedding())
            .expect("a fixture's labelling should describe its own map")
    }

    /// Expands `embedding` against this fixture's anchors.
    ///
    /// A bare [`Embedding`] holds only embedded cells, so indexing one on its
    /// own would leave every entity's own cell unclassified. Tests that build a
    /// deliberately wrong labelling index it through here so that the fault
    /// they wrote is the only thing wrong with it.
    pub fn index_of(&self, embedding: &Embedding) -> Result<EmbeddingIndex, EmbeddingError> {
        EmbeddingIndex::build_with_anchors(self.map(), embedding, self.anchors())
    }

    /// Returns the entities of one dimension, in creation order.
    pub fn owners(&self, dimension: Dim) -> Vec<EntityOwner> {
        self.entities
            .iter()
            .copied()
            .filter(|owner| owner.dimension() == dimension)
            .collect()
    }

    /// Returns the cell this entity is anchored at.
    pub fn anchor(&self, owner: EntityOwner) -> Dart {
        self.anchors
            .iter()
            .find(|&&(dimension, _, held)| held == owner && dimension == owner.dimension())
            .expect("every entity should be anchored at a cell of its own dimension")
            .1
    }

    /// Recovers one entity's region by walking the map.
    pub fn region(&self, owner: EntityOwner) -> LogicalRegion {
        recover_region(self.map(), &self.index(), owner, self.anchor(owner))
            .expect("a fixture's region should be recoverable")
    }

    /// Recovers every entity's region, one orbit walk from each anchor.
    pub fn regions(&self) -> Vec<LogicalRegion> {
        let index = self.index();
        self.entities
            .iter()
            .map(|&owner| {
                recover_region(self.map(), &index, owner, self.anchor(owner))
                    .expect("a fixture's region should be recoverable")
            })
            .collect()
    }

    /// Counts the raw cells of one dimension.
    pub fn raw_cell_count(&self, dimension: Dim) -> usize {
        self.map().cells(dimension).count()
    }

    /// Copies the labelling with every record naming the cell at `dart` dropped.
    ///
    /// This is how a test writes a wrong label: replace what the fixture said
    /// about one cell rather than adding a second, conflicting claim to it.
    pub fn relabelled(&self, dimension: Dim, dart: Dart, owner: EntityOwner) -> Embedding {
        let orbit: HashSet<Dart> = self
            .map()
            .orbit(dart, self.map().orbit_indices(dimension))
            .collect();
        let mut copy = Embedding::new();
        for record in self.embedding().records() {
            if record.dimension == dimension && orbit.contains(&record.representative) {
                continue;
            }
            copy.own(record.dimension, record.representative, record.owner);
        }
        copy.own(dimension, dart, owner);
        copy
    }

    fn remember(&mut self, owner: EntityOwner) -> EntityOwner {
        self.entities.push(owner);
        owner
    }
}

/// Builds a map from a closure that only adds and links darts.
fn raw_map(
    build: impl FnOnce(&mut ModelEdit<'_, StandardPayload>) -> Result<Vec<Dart>, ModelEditError>,
) -> (Model<StandardPayload>, Vec<Dart>) {
    let mut map = Model::<StandardPayload>::new();
    let darts = map
        .transaction(build)
        .expect("a hand-built scaffold should commit");
    (map, darts)
}

/// One open edge between two logical vertices.
pub fn segment() -> Scaffold {
    let (map, darts) = raw_map(|edit| {
        let a = edit.add_dart();
        let b = edit.add_dart();
        edit.link(Dim::Zero, a, b)?;
        Ok(vec![a, b])
    });

    let mut scaffold = Scaffold::wrap(map);
    let edge = scaffold.edge();
    let start = scaffold.vertex();
    let end = scaffold.vertex();
    scaffold.own(Dim::One, darts[0], edge);
    scaffold.own(Dim::Zero, darts[0], start);
    scaffold.own(Dim::Zero, darts[1], end);
    scaffold.seal()
}

/// One closed edge whose closure point is interior to it.
///
/// The two darts are swapped by both `alpha0` and `alpha1`, which is the whole
/// circle: one edge, one raw vertex, and that vertex inside the edge.
pub fn circle() -> Scaffold {
    let (map, darts) = raw_map(|edit| {
        let a = edit.add_dart();
        let b = edit.add_dart();
        edit.link(Dim::Zero, a, b)?;
        edit.link(Dim::One, a, b)?;
        Ok(vec![a, b])
    });

    let mut scaffold = Scaffold::wrap(map);
    let edge = scaffold.edge();
    scaffold.own(Dim::One, darts[0], edge);
    scaffold.own(Dim::Zero, darts[0], edge);
    scaffold.seal()
}

/// A capped cylinder: a seamed wall tube with a monogon disc on each rim.
///
/// The wall is one quad whose left and right sides are `alpha2`-linked into the
/// seam; each cap is the two-dart face bounded by a single closed edge, sewn
/// onto a rim. The seam belongs to the wall and each rim's closure point
/// belongs to its rim, so the shape has three logical faces, two logical edges
/// and no logical vertex at all.
pub fn cylinder() -> Scaffold {
    let (map, darts) = raw_map(|edit| {
        let wall = quad(edit)?;
        // Close the tube: the quad's right side onto its left side.
        edit.link(Dim::Two, wall[3][1], wall[1][0])?;
        edit.link(Dim::Two, wall[3][0], wall[1][1])?;

        let mut out: Vec<Dart> = wall.iter().flatten().copied().collect();
        for side in [2usize, 0] {
            let cap = monogon(edit)?;
            edit.sew(Dim::Two, wall[side][0], cap[1])?;
            out.extend(cap);
        }
        Ok(out)
    });

    // `quad` lays its darts out edge by edge: 0,1 bottom; 2,3 right; 4,5 top;
    // 6,7 left. The caps follow, top first.
    let (bottom, seam, top) = (darts[0], darts[2], darts[4]);
    let (top_cap, bottom_cap) = (darts[8], darts[10]);

    let mut scaffold = Scaffold::wrap(map);
    let wall = scaffold.face();
    let top_rim = scaffold.edge();
    let bottom_rim = scaffold.edge();
    let top_disc = scaffold.face();
    let bottom_disc = scaffold.face();

    scaffold.own(Dim::Two, bottom, wall);
    scaffold.own(Dim::One, seam, wall);
    scaffold.own(Dim::Two, top_cap, top_disc);
    scaffold.own(Dim::Two, bottom_cap, bottom_disc);
    scaffold.own(Dim::One, top, top_rim);
    scaffold.own(Dim::Zero, top, top_rim);
    scaffold.own(Dim::One, bottom, bottom_rim);
    scaffold.own(Dim::Zero, bottom, bottom_rim);
    scaffold.seal()
}

/// A planar face with `holes` holes, each reached by its own artificial bridge.
///
/// The face is one raw 2-cell: its boundary walk leaves the outer square along
/// a bridge, goes round a hole, comes back along the same bridge, and carries
/// on. A bridge is one edge the walk uses twice, with the two uses
/// `alpha2`-linked to each other, and it belongs to the face.
pub fn holed_face(holes: usize) -> Scaffold {
    assert!(holes <= 4, "each hole is bridged from its own outer corner");
    // Slot layout, repeated per hole: bridge out, four hole edges, bridge back,
    // then one outer edge. Any outer edges left over close the square.
    let slots = holes * 7 + (4 - holes);
    let bridges: Vec<(usize, usize)> = (0..holes).map(|hole| (hole * 7, hole * 7 + 5)).collect();

    let (map, darts) = raw_map(|edit| {
        let uses = boundary_word(edit, slots)?;
        for &(out, back) in &bridges {
            edit.link(Dim::Two, uses[out][0], uses[back][1])?;
            edit.link(Dim::Two, uses[out][1], uses[back][0])?;
        }
        Ok(uses.iter().flatten().copied().collect())
    });

    let mut scaffold = Scaffold::wrap(map);
    let face = scaffold.face();
    scaffold.own(Dim::Two, darts[0], face);
    for &(out, _) in &bridges {
        scaffold.own(Dim::One, darts[out * 2], face);
    }
    scaffold.own_each_remaining(Dim::One, Scaffold::edge);
    scaffold.own_each_remaining(Dim::Zero, Scaffold::vertex);
    scaffold.seal()
}

/// A whole sphere: one bigon with its two edges identified.
///
/// Four darts, every raw cell of them classified in the same logical face:
/// one edge and two poles, all embedded. Nothing is left on the boundary, so
/// the face has no loop at all — and it still occupies exactly one raw 2-cell,
/// which is what a face covering a closed support is required to stand on.
pub fn sphere() -> Scaffold {
    let (map, _) = raw_map(|edit| {
        bigon(edit)?;
        Ok(Vec::new())
    });

    let mut scaffold = Scaffold::wrap(map);
    let face = scaffold.face();
    for dimension in [Dim::Two, Dim::One, Dim::Zero] {
        scaffold.own_all(dimension, face);
    }
    scaffold.seal()
}

/// A whole torus: one quad with both pairs of opposite sides identified.
///
/// The two cuts and the single corner they cross at all belong to the face, so
/// the torus is one logical face with no edge and no vertex.
pub fn torus() -> Scaffold {
    let (map, _) = raw_map(|edit| {
        let quad = quad(edit)?;
        // Bottom onto top, then right onto left: a periodic square.
        edit.link(Dim::Two, quad[0][0], quad[2][1])?;
        edit.link(Dim::Two, quad[0][1], quad[2][0])?;
        edit.link(Dim::Two, quad[1][0], quad[3][1])?;
        edit.link(Dim::Two, quad[1][1], quad[3][0])?;
        Ok(Vec::new())
    });

    let mut scaffold = Scaffold::wrap(map);
    let face = scaffold.face();
    for dimension in [Dim::Two, Dim::One, Dim::Zero] {
        scaffold.own_all(dimension, face);
    }
    scaffold.seal()
}

/// A solid with a spherical cavity in it, as the one raw 3-cell it must be.
///
/// The material between two spheres cannot be a polyhedron on its own: a ball
/// with one pair of boundary faces identified always leaves a boundary of genus
/// one. What makes two boundary spheres instead is a **cut face** the solid
/// owns — the face swept by the outer sphere's one edge as it travels inward —
/// used twice by the solid's boundary and `alpha3`-linked to itself there.
///
/// The polyhedron is a pillow: an outer bigon, an inner bigon, and two squares
/// joining them along two vertical edges. Identifying the two squares is what
/// closes each bigon into a sphere, and the boundary walk turns across the cut
/// rather than crossing it, so the two spheres stay two shells.
pub fn cavity_solid() -> Scaffold {
    let (map, _) = raw_map(|edit| {
        // Each face is wound counter-clockwise seen from outside the pillow, so
        // every edge is walked one way by one face and the other way by the
        // other -- which is what lets `alpha2` pair start dart with end dart.
        let outer = boundary_word(edit, 2)?; // T1 -t1-> T2 -t2-> T1
        let inner = boundary_word(edit, 2)?; // B2 -b1-> B1 -b2-> B2
        let front = boundary_word(edit, 4)?; // T2 -t1-> T1 -v1-> B1 -b1-> B2 -v2-> T2
        let back = boundary_word(edit, 4)?; // T1 -t2-> T2 -v2-> B2 -b2-> B1 -v1-> T1

        for (one, other) in [
            (outer[0], front[0]), // t1
            (outer[1], back[0]),  // t2
            (inner[0], front[2]), // b1
            (inner[1], back[2]),  // b2
            (front[1], back[3]),  // v1
            (front[3], back[1]),  // v2
        ] {
            edit.link(Dim::Two, one[0], other[1])?;
            edit.link(Dim::Two, one[1], other[0])?;
        }

        // The two squares are the same cut face seen from its two sides. The
        // seed pairs the darts at one corner: the front's use of `t1` and the
        // back's use of `t2` both arrive at T2, and the rest follows.
        edit.sew(Dim::Three, front[0][0], back[0][1])?;
        Ok(Vec::new())
    });

    solid_from_buried_cells(map)
}

/// A solid with a handle, as the one raw 3-cell it must be.
///
/// A cube with its two opposite x-faces identified: the four faces left over
/// close into a torus, so the solid has one shell of genus one. The identified
/// face is a cut the solid owns, exactly as a cavity's is — one pair of faces
/// glued to each other is what a handle and a cavity each cost.
pub fn handle_solid() -> Scaffold {
    let (map, _) = raw_map(|edit| {
        let cube = cube_surface(edit)?;
        let (here, there) = alpha3_seed(&cube, &cube, [0, 0, 0], 0);
        edit.sew(Dim::Three, here, there)?;
        Ok(Vec::new())
    });

    solid_from_buried_cells(map)
}

/// Labels one solid over `map`: what has material on both sides is inside it.
///
/// A face the builder `alpha3`-sewed is the solid's own cut, and so are the
/// edges and corners buried behind it; everything the solid does not own is
/// left on its boundary as a face, an edge or a vertex of its own.
fn solid_from_buried_cells(map: Model<StandardPayload>) -> Scaffold {
    let mut scaffold = Scaffold::wrap(map);
    let solid = scaffold.solid();
    scaffold.own_all(Dim::Three, solid);
    for dimension in [Dim::Two, Dim::One, Dim::Zero] {
        let buried_cells: Vec<Dart> = scaffold
            .map()
            .cells(dimension)
            .filter(|&cell| buried(scaffold.map(), cell, dimension))
            .collect();
        for cell in buried_cells {
            scaffold.own(dimension, cell, solid);
        }
    }
    scaffold.own_each_remaining(Dim::Two, Scaffold::face);
    scaffold.own_each_remaining(Dim::One, Scaffold::edge);
    scaffold.own_each_remaining(Dim::Zero, Scaffold::vertex);
    scaffold.seal()
}

/// Reports whether a raw cell has material all the way around it.
///
/// A face the builder sewed is `alpha3`-linked on both its darts; a face left on
/// the boundary is free. A cell of any dimension is buried exactly when no dart
/// of it lies on a free face.
fn buried(map: &GMap, cell: Dart, dimension: Dim) -> bool {
    map.orbit(cell, map.orbit_indices(dimension))
        .all(|dart| !map.is_free(dart, Dim::Three))
}

/// Adds one quad face: four edges in a cycle, two darts each.
///
/// Returns the darts by edge then by end, so `quad[i][0]` starts edge `i` and
/// `quad[i][1]` ends it.
fn quad(edit: &mut ModelEdit<'_, StandardPayload>) -> Result<[[Dart; 2]; 4], ModelEditError> {
    let uses = boundary_word(edit, 4)?;
    Ok([uses[0], uses[1], uses[2], uses[3]])
}

/// Adds the four-dart 2-cell a whole sphere is.
///
/// Two edge uses between the same two corners, then each paired with the use
/// reading the other edge the same way round: that is the sphere written as a
/// bigon. Pairing them the other way round identifies the corners too and gives
/// a projective plane.
fn bigon(edit: &mut ModelEdit<'_, StandardPayload>) -> Result<[[Dart; 2]; 2], ModelEditError> {
    let uses = boundary_word(edit, 2)?;
    edit.link(Dim::Two, uses[0][0], uses[1][1])?;
    edit.link(Dim::Two, uses[0][1], uses[1][0])?;
    Ok([uses[0], uses[1]])
}

/// Adds one face bounded by a single closed edge through a single vertex.
fn monogon(edit: &mut ModelEdit<'_, StandardPayload>) -> Result<[Dart; 2], ModelEditError> {
    let a = edit.add_dart();
    let b = edit.add_dart();
    edit.link(Dim::Zero, a, b)?;
    edit.link(Dim::One, a, b)?;
    Ok([a, b])
}

/// Adds one face whose boundary walks `slots` edge uses in a single cycle.
fn boundary_word(
    edit: &mut ModelEdit<'_, StandardPayload>,
    slots: usize,
) -> Result<Vec<[Dart; 2]>, ModelEditError> {
    let uses: Vec<[Dart; 2]> = (0..slots)
        .map(|_| [edit.add_dart(), edit.add_dart()])
        .collect();
    for slot in &uses {
        edit.link(Dim::Zero, slot[0], slot[1])?;
    }
    for i in 0..slots {
        edit.link(Dim::One, uses[i][1], uses[(i + 1) % slots][0])?;
    }
    Ok(uses)
}

/// The six outward faces of a unit cube, each listing the corners it uses.
///
/// Each face is wound so the right-hand rule points away from the cube, which
/// is what makes two touching cubes present opposite windings to each other and
/// so lets `alpha3` pair their darts by shared corner.
const CUBE_FACES: [[[i32; 3]; 4]; 6] = [
    [[0, 0, 0], [0, 0, 1], [0, 1, 1], [0, 1, 0]],
    [[1, 0, 0], [1, 1, 0], [1, 1, 1], [1, 0, 1]],
    [[0, 0, 0], [1, 0, 0], [1, 0, 1], [0, 0, 1]],
    [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
    [[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]],
    [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]],
];

/// Adds the six quads of one cube and sews them into a closed surface.
fn cube_surface(
    edit: &mut ModelEdit<'_, StandardPayload>,
) -> Result<[[[Dart; 2]; 4]; 6], ModelEditError> {
    let mut faces = [[[Dart::new(0); 2]; 4]; 6];
    for face in &mut faces {
        *face = quad(edit)?;
    }

    // Every directed corner pair occurs on exactly one face and its reverse on
    // exactly one other; linking those two uses closes the cube.
    let mut directed: HashMap<([i32; 3], [i32; 3]), (usize, usize)> = HashMap::new();
    for (face, corners) in CUBE_FACES.iter().enumerate() {
        for edge in 0..4 {
            directed.insert((corners[edge], corners[(edge + 1) % 4]), (face, edge));
        }
    }
    for (&(from, to), &(face, edge)) in &directed {
        if from > to {
            continue;
        }
        let (other, other_edge) = directed[&(to, from)];
        edit.link(Dim::Two, faces[face][edge][0], faces[other][other_edge][1])?;
        edit.link(Dim::Two, faces[face][edge][1], faces[other][other_edge][0])?;
    }
    Ok(faces)
}

/// Picks the dart pair that starts an `alpha3` sew between two touching cubes.
///
/// The pair must sit at the same corner of the same shared edge: the faces wind
/// opposite ways, so the dart that starts an edge on one is the dart that ends
/// the matching edge on the other.
fn alpha3_seed(
    here: &[[[Dart; 2]; 4]; 6],
    there: &[[[Dart; 2]; 4]; 6],
    cell: [i32; 3],
    axis: usize,
) -> (Dart, Dart) {
    let positive = 2 * axis + 1;
    let negative = 2 * axis;
    let mut neighbour = cell;
    neighbour[axis] += 1;

    let shift = |corner: [i32; 3], origin: [i32; 3]| {
        [
            corner[0] + origin[0],
            corner[1] + origin[1],
            corner[2] + origin[2],
        ]
    };
    let from = shift(CUBE_FACES[positive][0], cell);
    let to = shift(CUBE_FACES[positive][1], cell);
    let matching = (0..4)
        .find(|&edge| {
            shift(CUBE_FACES[negative][edge], neighbour) == to
                && shift(CUBE_FACES[negative][(edge + 1) % 4], neighbour) == from
        })
        .expect("touching cubes should share an edge in both windings");

    (here[positive][0][0], there[negative][matching][1])
}
