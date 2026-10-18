"""Replays CASSIE sessions through CASSIE's own C# graph (tools/cassie_oracle) and through
the C++ port (graph_port_replay --events), and compares the cycles each event adds, against
each other and against the faces the session log records.

  python tools/cassie_oracle_compare.py --oracle <CassieOracle.dll> --replay <graph_port_replay> \
      [--port-fix] [--self-test MIRRORED.json] [--show N] session.json...

The port runs with --match-cassie unless --port-fix is given. --self-test first replays the
given session, which must tap a face with mirroring on, with the oracle's mirrored tap switched
off, and fails unless that planted defect shows up as a C#/C++ difference.
"""

import argparse
import json
import subprocess
import sys


def ids(strokes):
    return sorted(set(int(x) for x in strokes))


def logged(session):
    """Per stroke or tap state: the faces the log records for it."""
    patches = {int(p["id"]): p for p in session["allCreatedPatches"]}
    states = session["systemStates"]
    out, pending, i = {}, [], 0
    while i < len(states):
        st = states[i]
        kind, nxt = int(st["interactionType"]), i + 1
        patch = patches[int(st["elementID"])] if kind in (3, 4) else None
        if kind == 3 and patch is not None and patch["foundByAlgo"]:
            pending.append(ids(patch["strokesID"]))
        elif kind == 3 and patch is not None:
            group = [ids(patch["strokesID"])]
            while (nxt < len(states) and int(states[nxt]["interactionType"]) == 3
                   and not patches[int(states[nxt]["elementID"])]["foundByAlgo"]
                   and states[nxt]["time"] == st["time"]):
                group.append(ids(patches[int(states[nxt]["elementID"])]["strokesID"]))
                nxt += 1
            out[i] = sorted(group)
        elif kind in (1, 2):
            if kind == 1:
                out[i] = sorted(pending)
            pending = []
        i = nxt
    return out


def parse(stdout):
    got = {}
    for line in stdout.splitlines():
        if "\t" not in line or line.startswith("FINAL"):
            continue
        state, _, cycles = line.partition("\t")
        got[int(state)] = sorted(list(map(int, c.split(","))) for c in cycles.split(";")) if cycles else []
    return got


def run(cmd):
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not proc.stdout.startswith("ok"):
        sys.exit(f"failed: {' '.join(cmd)}\n{proc.stdout[:300]}{proc.stderr[-600:]}")
    return proc.stdout


def compare(args, path, control=False):
    want = logged(json.load(open(path)))
    oracle = ["dotnet", args.oracle, path] + (["--control-no-mirror-tap"] if control else [])
    replay = [args.replay, path, "--events"] + ([] if args.port_fix else ["--match-cassie"])
    cs, cpp = parse(run(oracle)), parse(run(replay))
    differ = [k for k in sorted(want) if cs.get(k, []) != cpp.get(k, [])]
    match_cs = sum(1 for k in want if want[k] == cs.get(k, []))
    match_cpp = sum(1 for k in want if want[k] == cpp.get(k, []))
    return want, cs, cpp, differ, match_cs, match_cpp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--oracle", required=True)
    ap.add_argument("--replay", required=True)
    ap.add_argument("--port-fix", action="store_true")
    ap.add_argument("--self-test", metavar="MIRRORED_SESSION")
    ap.add_argument("--show", type=int, default=0)
    ap.add_argument("sessions", nargs="+")
    args = ap.parse_args()

    if args.self_test:
        differ = compare(args, args.self_test, control=True)[3]
        if not differ:
            sys.exit("self-test FAILED: the planted defect (no mirrored tap) was not detected")
        print(f"self-test: planted defect detected in {len(differ)} events")

    totals = [0, 0, 0, 0]
    for path in args.sessions:
        want, cs, cpp, differ, match_cs, match_cpp = compare(args, path)
        totals = [totals[0] + match_cs, totals[1] + match_cpp, totals[2] + len(want), totals[3] + len(differ)]
        print(f"{path}\tC# {match_cs}/{len(want)}\tC++ {match_cpp}/{len(want)}\tdiffer {len(differ)}")
        for k in differ[:args.show]:
            print(f"  state {k}: log {want.get(k)}\n    C#  {cs.get(k)}\n    C++ {cpp.get(k)}")
    print(f"TOTAL {len(args.sessions)} sessions: C# {totals[0]}/{totals[2]}, C++ {totals[1]}/{totals[2]}, "
          f"{totals[3]} events where C# and C++ differ")


if __name__ == "__main__":
    main()
