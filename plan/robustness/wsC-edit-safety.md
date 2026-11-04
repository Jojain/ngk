# WS C: no panic on an open edit, and post-conditions at commit

**Owns:** `src/model.rs`, `src/topology/edit*`, `src/topology/validation.rs`.
**Decision:** D5. **Size:** small to medium. Enables B and F.

## Symptoms

- `Model::embedding_index` panics ("a committed model's labelling should
  describe its own map") when a builder reads a view (`Face::loops`) on a model
  whose transaction is still open and whose labelling a previous step left
  half updated. Seen in `sweep::sew_cap` and in
  `faces::split_face_by_imprints` under the Boolean. The panic escapes the
  transaction, so nothing is rolled back and no error reaches the caller.
- `Frame::from_xz` and similar constructors `expect` their preconditions and
  panic deep inside builders.
- A revolve by a negative angle commits a solid whose shell has zero signed
  volume: commit validates topology and embedding, not orientation.

## The class

Any builder bug that leaves the model inconsistent mid-edit, or produces an
invalid solid, must surface as an error from that builder with the model
restored, never as a panic or a committed invalid solid.

## Approach (D5)

Today a transaction works like a database transaction with deferred
constraints: it restores a snapshot on failure, and `validate_embedding`
checks the labelling only at commit. The embedding index, though, is built
lazily from the labelling and assumes those deferred constraints already
hold, so a read between two writes can meet a state no database index
would ever be in, and panics. The options, by analogy:

- **(a) Fallible reads.** The index build returns `Result` and views on an
  open edit propagate it; the transaction rolls back. A safety net, not a
  design: the error surfaces far from the step that caused it.
- **(b) Immediate constraints.** `own_cell` and the other labelling
  primitives refuse a conflicting claim when it is made (a 0-cell claimed by
  a vertex while an edge owns it as scaffold), and the index is maintained
  incrementally on each write, as a database maintains its indexes. The
  error then names the faulty step, and reads are always valid.
- **(c) Snapshot reads.** Builders plan from the transaction-start snapshot,
  which is immutable and valid, and only write into the edit; the blend
  engine already works this way (everything up to `assemble` reads the
  model as the call found it, only `execute` writes). Enforced by types:
  the plan phase gets `&Model`, the apply phase a `ModelEdit` with no views.
- **(d) Savepoints.** An operation that must read its own writes (the
  Boolean splits faces, then classifies the split model) validates at the
  end of each stage, and the next stage reads that validated state. A
  failure still restores the transaction-start snapshot.

Proposed, as one rule for every builder to come: (b), built as a small set
of **invariant-preserving operators** in the spirit of Euler operators
(Mäntylä): cut an edge (marking it and relabelling its 0-cell in one step),
insert an edge across a face, split and merge faces, sew and unsew along an
edge, make a face on a loop, and so on. Each leaves the model valid, and the
embedding index is maintained on each write. Builders compose them and may
read their own writes at any point. Raw dart and labelling calls stay
inside the operators. Surgery that has to pass through invalid states (the
blend's cut, re-embed and fill) runs in an explicit deferred block that
exposes no views, validated when it closes, like a database's deferred
constraints. (a) remains as the last net; (c) remains good practice; (d)
becomes unnecessary.

**Prior art, for orientation only (NGK writes its own).** CGoGN, the
combinatorial-maps library from Strasbourg, splits every operator in two: a
protected `*_topo` half that only rewires darts (`cut_edge_topo` unsews,
splits, re-sews) and a public half that calls it and updates every embedded
orbit in the same call (`cut_edge` in `cmap2.h`: new vertex and edge
embeddings, face and volume embeddings copied). The raw sews and `_topo`
operations are reachable only through a separate builder class
(`CMap2Builder_T`, a friend of the map) used by importers and surgery, and
`check_map_integrity` validates topology and embeddings afterwards. In
CGoGN 3 the same split is a `set_indices` flag on each operator, with
`check_integrity` for GMaps. The proposal above is the same shape: public
operators that keep labelling and index in step, a builder-like deferred
block for raw surgery, and a check when the block closes.

Whatever is chosen, fix the builders that leave the labelling inconsistent
(WS B1 for unmarked edges).

## Post-conditions at commit

Add a cheap orientation and closedness check for every solid a transaction
creates or touches (`validate_solid_orientation` already exists), on in
debug builds and in tests always, optionally in release. A builder that fails
it returns an error instead of committing.

## Panics audit

List every `expect`, `unwrap` and `panic!` reachable from a public builder,
and give each one an error path or a comment proving it unreachable.
`Frame::from_xz` gets a fallible twin used everywhere inside builders.

## Tests

- Every crash from the bench (`sweep/circle-along-arc`,
  `sweep/mitred-pipe-on-cube-edges`, `parts/v-belt-pulley`,
  `spline/pipe-on-planar-cubic`) as a test asserting an `Err`, until WS B
  turns it into the right solid.
- A property test: random sequences of builders on random inputs never
  panic, and every `Ok` solid passes the orientation check.
