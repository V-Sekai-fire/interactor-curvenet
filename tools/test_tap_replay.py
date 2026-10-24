"""Replay test: a tapped face is rebuilt from the strokes the log names, not from where the hand was.

The fixture is four strokes of a recorded session (32, 34, 66, 74 with their mirror twins). After the last
stroke a user taps the face bounded by all four, with the hand far from it. The log names the face by its
strokes, so the replay must rebuild it: at least one user face (the face and its mirror twin) and tap_misses=0. A replay that searches from the hand
position alone finds nothing, so the same session reports user=0 and tap_misses=1.

    c++ -std=c++17 -O2 tools/graph_port_replay.cpp vendor/cassie/src/graph_port/*.cpp -o graph_port_replay
    python3 tools/test_tap_replay.py ./graph_port_replay
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
FACE = [32, 34, 66, 74]


def session():
    s = json.load(open(os.path.join(HERE, "fixtures", "tap_four_strokes.json")))
    tap = dict(s["systemStates"][-1])
    tap.update(interactionType=3, elementID=900, primaryHandPos={"x": 3.0, "y": 5.0, "z": 3.0})
    s["systemStates"].append(tap)
    s["allCreatedPatches"].append({"id": 900, "foundByAlgo": False, "strokesID": FACE})
    return s


def summary(exe, data):
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(data, f)
    try:
        out = subprocess.run([exe, f.name], capture_output=True, text=True).stdout.splitlines()
    finally:
        os.unlink(f.name)
    head = out[0].split() if out else []
    return {k: int(v) for k, v in (x.split("=") for x in head[1:] if "=" in x)}


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    got = summary(argv[1], session())
    ok = got.get("user", 0) >= 1 and got.get("tap_misses") == 0
    print(("PASS" if ok else "FAIL"), "tapped face rebuilt from its logged strokes:", got)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
