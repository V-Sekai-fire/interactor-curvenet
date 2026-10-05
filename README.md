# interactor-curvenet

The curvenet stage as a sandboxed engine guest: sketched pen strokes become a curve network, a surfaced mesh and its sketch graph.

## What it is for

It carries the CASSIE curve-network code and the geometry subsets it depends on, adapted to run inside the engine's sandbox, with a Lean specification of its kernels.

## Build and run

It has no build of its own. `transport-meshing-pen` builds the guest and finds this repository and its dependencies as sibling checkouts at their paths in the goal manifest, `contract-manifest-taskweft`.

## Licence

The Lean specification's CITATION.cff names MIT. The rest of the repository states no licence, and vendored code keeps its own.
