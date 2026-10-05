/**************************************************************************/
/*  cassie_graph_port.h                                                   */
/**************************************************************************/
/* Ported from CASSIE (Yu, Arora, Stanko, Baerentzen, Singh, Bousseau),   */
/* MIT. Copyright (c) 2021 Emilie Yu and the CASSIE authors.              */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include <vector>

namespace cassie_graph_port {

// World coordinates stay double at the boundary; graph math is float in canvas-local space.
using cassie_world = double;

struct WorldPoint {
	cassie_world x = 0.0;
	cassie_world y = 0.0;
	cassie_world z = 0.0;
};

struct RecordedConstraint {
	WorldPoint position;
	bool is_intersection = false;
	bool is_at_existing_node = false;
	bool is_at_new_endpoint = false;
};

struct ReplayStats {
	int unresolved_constraints = 0;
	int mirror_seam_constraints = 0;
	int caught_exceptions = 0;
	int on_mirror_strokes = 0;
	int tap_misses = 0; // logged taps whose replayed search found no cycle
	int missing_patch_deletes = 0;
};

struct LoggedPatch {
	int id = -1;
	std::vector<int> strokes;
	bool found_by_algo = true;
};

// A stretch of one stroke's curve, from parameter t_from to t_to in walking order.
struct BoundarySpan {
	int stroke = -1;
	double t_from = 0.0;
	double t_to = 0.0;
};

struct CycleBoundary {
	std::vector<int> strokes; // sorted, unique
	std::vector<BoundarySpan> spans; // the cycle's half-segments, in order
	bool user_created = false;
	bool broken_walk = false; // the segments do not chain node to node
};

class Replay {
public:
	Replay(WorldPoint p_canvas_origin, WorldPoint p_mirror_point, WorldPoint p_mirror_normal);
	~Replay();
	Replay(const Replay &) = delete;
	Replay &operator=(const Replay &) = delete;

	// Patches logged since the last stroke event; new cycles take their ids.
	void SetPendingPatches(const std::vector<LoggedPatch> &p_patches);
	void AddStroke(int p_id, const std::vector<WorldPoint> &p_ctrl_points, const std::vector<RecordedConstraint> &p_constraints, bool p_closed_loop, bool p_mirroring, float p_canvas_scale);
	void DeleteStroke(int p_id, bool p_mirroring);
	// p_tap is the logged hand position mapped into canvas space.
	bool AddUserPatches(const std::vector<LoggedPatch> &p_group, WorldPoint p_tap, bool p_mirroring);
	bool DeletePatch(int p_id);
	std::vector<std::vector<int>> Cycles(bool p_include_user) const;
	// Every live cycle, sorted by strokes, with its boundary as curve-parameter spans.
	std::vector<CycleBoundary> Boundaries(bool p_include_user) const;
	ReplayStats Stats() const;

private:
	struct Impl;
	Impl *impl = nullptr;
};

} // namespace cassie_graph_port
