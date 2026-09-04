---
name: gmap-reference
description: Use when working in the ngk codebase on combinatorial maps, generalized maps (GMap/gmap), darts, involutions, alpha links, cells, or sew/unsew topology. Consult the private combinatorial maps book as the authoritative theory source, reading only targeted chunks rather than the whole file.
---

# GMap Reference

Use the private combinatorial maps book as the source of truth when reasoning about map/GMap theory, definitions, algorithms, and topology in this repo.

## Authoritative Source

The canonical reference is:

`private_doc/Combinatorial_Maps_Book/Combinatorial_Maps_Book.md`

Prefer this book over memory when answering theory questions or making topology-sensitive implementation decisions involving darts, involutions, alpha links, cells, or sewing.

## Read In Chunks

The book is too large to load whole. Do not read or paste the full file in one shot.

Use targeted search and small reads:

1. Formulate the specific definition, theorem, construction, or algorithm needed.
2. Search for focused terms such as `generalized map`, `involution`, `sew`, `sewable`, `alpha`, `dart`, `orbit`, `cell`, `dimension`, `boundary`, or the relevant algorithm name.
3. Read only the matching section or a small adjacent chunk.
4. If the excerpt is insufficient, refine the search or read the next nearby chunk.

## Practical Workflow

When editing code:

1. Inspect the local implementation first to understand current abstractions and naming.
2. Use the book only for the theoretical point that affects the change.
3. Translate book terms into the repo's existing API and style instead of inventing new concepts.
4. Keep raw topology operations in lower-level APIs; prefer higher-level modeling/building helpers from scripts and examples.

When explaining or reviewing:

1. Cite the book conceptually as the project authority, but avoid long quotations.
2. Distinguish book-backed facts from implementation observations.
3. Mention when a conclusion is inferred from the current code rather than directly stated in the book.
