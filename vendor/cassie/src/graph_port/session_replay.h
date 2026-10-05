/**************************************************************************/
/*  session_replay.h                                                      */
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

#include "cassie_graph_port.h"

#include <string>
#include <utility>
#include <vector>

namespace cassie_graph_port {

struct SessionResult {
	bool ok = false;
	std::string error;
	// Live algorithm cycles at the end, each a sorted stroke-id list.
	std::vector<std::vector<int>> cycles;
	// cycles[i]'s boundary: half-segments as curve-parameter spans, in walking order.
	std::vector<CycleBoundary> boundaries;
	int user_cycles = 0;
	ReplayStats stats;
	// (system state index, every cycle incl. user) after each graph event, when asked for.
	std::vector<std::pair<int, std::vector<std::vector<int>>>> trace;
};

// p_json is a CASSIE session export, or its compact subset: systemStates
// (interactionType, elementID, mirroring, canvasScale, time), allSketchedStrokes
// (id, ctrlPts, appliedPositionConstraints, rejectedPositionConstraints,
// closedLoop) and allCreatedPatches (id, foundByAlgo, strokesID).
SessionResult ReplaySession(const std::string &p_json, bool p_trace);

// "ok cycles=N user=M ..." then one line per cycle: its stroke ids, " |", then
// each half-segment as "stroke:t_from:t_to" in walking order.
std::string FormatSessionResult(const SessionResult &p_result);

} // namespace cassie_graph_port
