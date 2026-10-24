# interactor-curvenet

The curvenet stage as a sandboxed engine guest: sketched pen strokes become a curve network, a surfaced mesh and its sketch graph.

## What it is for

It carries the CASSIE curve-network code and the geometry subsets it depends on, adapted to run inside the engine's sandbox, with a Lean specification of its kernels.

## Build and run

It has no build of its own. `transport-meshing-pen` builds the guest and finds this repository and its dependencies as sibling checkouts at their paths in the goal manifest, `contract-manifest-taskweft`.

## Checking the graph port against CASSIE

`tools/cassie_oracle` runs CASSIE's original C# graph code (`vendor/cassie-cs`) under dotnet, with a small stand-in for the game engine math types it was written against. `tools/cassie_oracle_compare.py` replays recorded sessions through it and through `graph_port_replay --events`, and reports per event where the two disagree and how each matches the faces the session log records.

    dotnet build tools/cassie_oracle
    c++ -std=c++17 -O2 tools/graph_port_replay.cpp vendor/cassie/src/graph_port/*.cpp -o graph_port_replay
    python tools/cassie_oracle_compare.py --oracle tools/cassie_oracle/bin/Debug/net10.0/CassieOracle.dll \
        --replay ./graph_port_replay --self-test dress.json session.json...

`--self-test` switches off the oracle's mirrored tap on a session that taps with mirroring on, and fails unless the comparison catches it.

The port has no distance thresholds: a snap lands on a node only when the log says it did (`isAtExistingNode`) or the curve parameters are equal, and the closest point on a curve is found exactly, so a point on a stroke's end projects to t = 0 or 1. A tapped face is rebuilt from the strokes the log names for it. `tools/test_tap_replay.py` checks that.

    python3 tools/test_tap_replay.py ./graph_port_replay

## Licence

MIT. See [LICENSE](LICENSE).
