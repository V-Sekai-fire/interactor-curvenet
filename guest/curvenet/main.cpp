// curvenet.elf -- the curvenet stage: Cassie's pen -> curvenet -> mesh and
// mesh -> curvenet paths (plan Cut 4).
//
// The godot-lite split: this TU sees the sandbox's api.hpp and
// curvenet_api.h (std types) and nothing of Cassie's; curvenet_api.cpp and
// checks.cpp see Cassie (on godot-lite) and nothing of the sandbox's. So all
// marshalling is here: PackedArray <-> std::vector, String <-> std::string.
// Arrays follow guest/common/mesh_wire.h. Every ADD_API_FUNCTION has a
// no-argument wrapper in project/main.gd (AGENTS.md rule 8).
//
// No GPU here: the stage is CPU-only Cassie, so no RenderingDevice is opened
// and rule 4 does not arise. Single calls are sized to stay inside the
// sandbox's execution_timeout (Gate 4 records the heaviest).

#include <api.hpp>

#include <string>
#include <vector>

#include "curvenet_api.h"

static Variant text(const std::string &s) {
	return Variant(String(s));
}

template <typename T>
static Variant packed(const std::vector<T> &v) {
	return Variant(PackedArray<T>(v));
}

// --- state --------------------------------------------------------------------

static Variant cn_reset() {
	return text(cn::reset());
}

static Variant cn_set_param(String name, double value) {
	return text(cn::set_param(name.utf8(), value));
}

static Variant cn_get_param(String name) {
	return Variant(cn::get_param(name.utf8()));
}

static Variant cn_set_body(PackedArray<float> vertices, PackedArray<int32_t> triangles) {
	return text(cn::set_body(vertices.fetch(), triangles.fetch()));
}

// --- pen ----------------------------------------------------------------------

static Variant pen_begin(double x, double y, double z, double pressure) {
	return Variant(int64_t(cn::pen_begin(float(x), float(y), float(z), float(pressure))));
}

static Variant pen_point(int id, double x, double y, double z, double pressure) {
	return text(cn::pen_point(id, float(x), float(y), float(z), float(pressure)));
}

static Variant pen_end(int id) {
	return text(cn::pen_end(id));
}

// Finalize the stroke against crossings the collision guest found (3 floats a
// point); empty crossings fall back to pen_end's own solve.
static Variant pen_end_with_crossings(int id, PackedArray<float> crossings) {
	return text(cn::pen_end_with_crossings(id, crossings.fetch()));
}

// A whole stroke in one call: xyzp holds 4 floats per sample (x, y, z,
// pressure); answers pen_end's line.
static Variant pen_stroke(PackedArray<float> xyzp) {
	const std::vector<float> s = xyzp.fetch();
	if (s.size() < 8 || s.size() % 4 != 0) {
		return text("FAIL: pen_stroke wants 4 floats per sample and at least 2 samples");
	}
	const int id = cn::pen_begin(s[0], s[1], s[2], s[3]);
	if (id < 0) {
		return text("FAIL: pen_begin");
	}
	for (size_t i = 4; i < s.size(); i += 4) {
		cn::pen_point(id, s[i], s[i + 1], s[i + 2], s[i + 3]);
	}
	return text(cn::pen_end(id));
}

static Variant patch_count() {
	return Variant(int64_t(cn::patch_count()));
}

static Variant patch_vertices(int i) {
	return packed(cn::patch_vertices(i));
}

static Variant patch_indices(int i) {
	return packed(cn::patch_indices(i));
}

// --- curvenet -----------------------------------------------------------------

static Variant curvenet_build() {
	return text(cn::curvenet_build());
}

static Variant curvenet_extract(PackedArray<float> vertices, PackedArray<int32_t> triangles, int target,
		double rdp_error, double fit_error, double curvature_weight) {
	return text(cn::curvenet_extract(vertices.fetch(), triangles.fetch(), target, rdp_error, fit_error,
			curvature_weight));
}

static Variant curvenet_curves() {
	return packed(cn::curvenet_curves());
}

static Variant curvenet_knots() {
	return packed(cn::curvenet_knots());
}

// --- mesh ---------------------------------------------------------------------

static Variant mesh_build(double target_edge_length, double weld_eps) {
	return text(cn::mesh_build(target_edge_length, weld_eps));
}

static Variant mesh_vertices() {
	return packed(cn::mesh_vertices());
}

static Variant mesh_indices() {
	return packed(cn::mesh_indices());
}

static Variant mesh_boundary_loops() {
	return packed(cn::mesh_boundary_loops());
}

static Variant mesh_rims() {
	return packed(cn::mesh_rims());
}

static Variant mesh_patch_ids() {
	return packed(cn::mesh_patch_ids());
}

static Variant session_replay(String json) {
	return text(cn::session_replay(json.utf8()));
}

static Variant boundary_patches(PackedArray<float> points, PackedArray<int32_t> counts, double target_edge_length,
		double remesh_edge_length) {
	return text(cn::boundary_patches(points.fetch(), counts.fetch(), target_edge_length, remesh_edge_length));
}

static Variant parts_vertices() {
	return packed(cn::parts_vertices());
}

static Variant parts_triangles() {
	return packed(cn::parts_triangles());
}

static Variant parts_counts() {
	return packed(cn::parts_counts());
}

static Variant set_parts(PackedArray<float> vertices, PackedArray<int32_t> triangles, PackedArray<int32_t> counts) {
	return text(cn::set_parts(vertices.fetch(), triangles.fetch(), counts.fetch()));
}

// --- checks -------------------------------------------------------------------

static Variant check(String name) {
	return text(cn::check(name.utf8()));
}

static Variant check_all() {
	return text(cn::check_all());
}

static Variant check_names() {
	std::string out;
	for (const std::string &n : cn::check_names()) {
		out += (out.empty() ? "" : " ") + n;
	}
	return text(out);
}

int main() {
	ADD_API_FUNCTION(cn_reset, "String", "", "Drop strokes, patches, curvenet and mesh; keep params and body");
	ADD_API_FUNCTION(cn_set_param, "String", "String name, float value",
			"snap_radius, surface_offset, target_edge_length, split_closed, merge_eps, mirror");
	ADD_API_FUNCTION(cn_get_param, "float", "String name", "A param's value (NaN if unknown)");
	ADD_API_FUNCTION(cn_set_body, "String", "PackedFloat32Array vertices, PackedInt32Array triangles",
			"The body the pen snaps to (mesh_wire; empty clears)");
	ADD_API_FUNCTION(pen_begin, "int", "float x, float y, float z, float pressure", "Start a stroke; its id or -1");
	ADD_API_FUNCTION(pen_point, "String", "int id, float x, float y, float z, float pressure", "Add a sample");
	ADD_API_FUNCTION(pen_end, "String", "int id",
			"Commit: ok valid closed new_patches patches edges nodes cycles");
	ADD_API_FUNCTION(pen_end_with_crossings, "String", "int id, PackedFloat32Array crossings",
			"End a stroke, splitting at guest-supplied crossings (3 floats a point)");
	ADD_API_FUNCTION(pen_stroke, "String", "PackedFloat32Array xyzp", "A whole stroke, 4 floats per sample");
	ADD_API_FUNCTION(patch_count, "int", "", "Active surface patches");
	ADD_API_FUNCTION(patch_vertices, "PackedFloat32Array", "int i", "Patch i's vertices (mesh_wire)");
	ADD_API_FUNCTION(patch_indices, "PackedInt32Array", "int i", "Patch i's triangles, CCW-outward");
	ADD_API_FUNCTION(curvenet_build, "String", "", "Sketch graph -> curvenet");
	ADD_API_FUNCTION(curvenet_extract, "String",
			"PackedFloat32Array vertices, PackedInt32Array triangles, int target, float rdp_error, float fit_error, float curvature_weight",
			"Mesh -> curvenet");
	ADD_API_FUNCTION(curvenet_curves, "PackedFloat32Array", "", "The curvenet's curves (mesh_wire)");
	ADD_API_FUNCTION(curvenet_knots, "PackedFloat32Array", "", "The curvenet's knots (mesh_wire)");
	ADD_API_FUNCTION(mesh_build, "String", "float target_edge_length, float weld_eps",
			"Merge + weld the patches, optionally PMP-remesh");
	ADD_API_FUNCTION(mesh_vertices, "PackedFloat32Array", "", "The built mesh's vertices");
	ADD_API_FUNCTION(mesh_indices, "PackedInt32Array", "", "The built mesh's triangles");
	ADD_API_FUNCTION(mesh_boundary_loops, "PackedInt32Array", "", "The built mesh's boundary loops");
	ADD_API_FUNCTION(mesh_rims, "PackedInt32Array", "", "The drawn surface's boundary loops, open or closed into the shell");
	ADD_API_FUNCTION(mesh_patch_ids, "PackedInt32Array", "", "Source patch per triangle (-1 after a remesh)");
	ADD_API_FUNCTION(session_replay, "String", "String json",
			"Replay a CASSIE session through the graph port: ok cycles=N user=M, then a stroke-id line per cycle");
	ADD_API_FUNCTION(boundary_patches, "String",
			"PackedFloat32Array points, PackedInt32Array counts, float target_edge_length, float remesh_edge_length",
			"Triangulate closed boundaries (counts[i] xyz points each), optionally remesh each, into the patches mesh_build merges");
	ADD_API_FUNCTION(parts_vertices, "PackedFloat32Array", "", "The kept patches' vertices, flat");
	ADD_API_FUNCTION(parts_triangles, "PackedInt32Array", "", "The kept patches' triangles, each patch's own indices");
	ADD_API_FUNCTION(parts_counts, "PackedInt32Array", "", "Per kept patch: vertex count, index count");
	ADD_API_FUNCTION(set_parts, "String", "PackedFloat32Array vertices, PackedInt32Array triangles, PackedInt32Array counts",
			"Replace the kept patches (parts_* of another sandbox)");
	ADD_API_FUNCTION(check, "String", "String name", "One Gate 4 check");
	ADD_API_FUNCTION(check_all, "String", "", "Every Gate 4 check");
	ADD_API_FUNCTION(check_names, "String", "", "The Gate 4 check names");
	halt();
}
