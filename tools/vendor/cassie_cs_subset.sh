#!/usr/bin/env bash
# Vendor CASSIE's original C# graph and curve code into vendor/cassie-cs/src for
# tools/cassie_oracle, which runs it under dotnet to check the C++ graph port.
#
#   tools/vendor/cassie_cs_subset.sh
#
# Source: V-Sekai/interactor-cassie at 3a3178e73e (override the checkout with
# CASSIE_CS). Files are copied unedited except two adaptations:
# ConstraintCandidate is cut out of ConstraintSolver.cs (the solver needs
# MathNet's sparse solvers the oracle does not run), and
# patches/cassie-cs.patch hands FinalStroke its graph instead of finding it on
# the parent DrawingCanvas in Awake.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CS="${CASSIE_CS:-$ROOT/../interactor-cassie}"
PIN="3a3178e73e1218f4f38e24fba4b6d72cd18e962b"
DST="$ROOT/vendor/cassie-cs/src"
S="Assets/Scripts"

show() { git -C "$CS" show "$PIN:$S/$1"; }
rm -rf "$DST"
mkdir -p "$DST"
for f in Data/Graph/Internal/Cycle.cs Data/Graph/CycleDetection.cs Data/Graph/Graph.cs Data/Graph/GraphData.cs \
	Data/Graph/ICycle.cs Data/Graph/INode.cs Data/Graph/ISegment.cs \
	Data/Graph/HalfSegment.cs Data/Graph/Internal/Node.cs Data/Graph/Internal/Segment.cs \
	Data/Graph/Internal/SegmentCycles.cs \
	Curves/BezierCurve.cs Curves/Curve.cs Curves/FrenetFrame.cs Curves/LineCurve.cs Curves/PointOnCurve.cs \
	Curves/RamerDouglasPeucker.cs Utils.cs Data/Strokes/FinalStroke.cs \
	Data/Constraints/Constraint.cs Data/Constraints/IntersectionConstraint.cs \
	Data/Constraints/MirrorPlaneConstraint.cs Data/Constraints/SurfaceConstraint.cs \
	Study/StudyLog.cs Study/StudyUtils.cs; do
	show "$f" > "$DST/$(basename "$f")"
done
{
	show Create/Sketch/Beautify/ConstraintSolver.cs | sed -n 1,13p
	show Create/Sketch/Beautify/ConstraintSolver.cs | sed -n 55,76p
	echo "}"
} > "$DST/ConstraintCandidate.cs"
grep -q 'public class ConstraintCandidate' "$DST/ConstraintCandidate.cs"
patch -s -d "$ROOT" -p1 < "$HERE/patches/cassie-cs.patch"
echo "vendored $(ls "$DST" | wc -l | tr -d ' ') files from $PIN"
