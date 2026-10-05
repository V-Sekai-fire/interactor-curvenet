/**************************************************************************/
/*  session_replay.cpp                                                    */
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


#include "session_replay.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>

namespace cassie_graph_port {

namespace {

struct Json {
	enum Kind { NUL, BOOL, NUM, STR, ARR, OBJ } kind = NUL;
	bool b = false;
	double num = 0.0;
	std::string str;
	std::vector<Json> arr;
	std::vector<std::pair<std::string, Json>> obj;
	const Json &operator[](const char *k) const {
		static const Json none;
		for (const std::pair<std::string, Json> &kv : obj) {
			if (kv.first == k) {
				return kv.second;
			}
		}
		return none;
	}
};

struct Parser {
	const char *p = nullptr;
	bool bad = false;
	void ws() {
		while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') {
			p++;
		}
	}
	std::string str() {
		std::string s;
		p++;
		while (*p != '"' && *p != '\0') {
			if (*p == '\\' && p[1] != '\0') {
				p++;
			}
			s += *p++;
		}
		if (*p == '\0') {
			bad = true;
			return s;
		}
		p++;
		return s;
	}
	Json value() {
		ws();
		Json j;
		if (bad || *p == '\0') {
			bad = true;
		} else if (*p == '{') {
			j.kind = Json::OBJ;
			p++;
			ws();
			while (!bad && *p != '}') {
				if (*p != '"') {
					bad = true;
					break;
				}
				std::string k = str();
				ws();
				if (*p != ':') {
					bad = true;
					break;
				}
				p++;
				Json v = value();
				j.obj.push_back(std::make_pair(k, v));
				ws();
				if (*p == ',') {
					p++;
				}
				ws();
			}
			if (!bad) {
				p++;
			}
		} else if (*p == '[') {
			j.kind = Json::ARR;
			p++;
			ws();
			while (!bad && *p != ']') {
				j.arr.push_back(value());
				ws();
				if (*p == ',') {
					p++;
				}
				ws();
			}
			if (!bad) {
				p++;
			}
		} else if (*p == '"') {
			j.kind = Json::STR;
			j.str = str();
		} else if (*p == 't' || *p == 'f') {
			j.kind = Json::BOOL;
			j.b = *p == 't';
			p += j.b ? 4 : 5;
		} else if (*p == 'n') {
			p += 4;
		} else {
			j.kind = Json::NUM;
			char *end = nullptr;
			// Unity parsed these as float; round once to float, then widen.
			j.num = (double)std::strtof(p, &end);
			if (end == p) {
				bad = true;
			} else {
				p = end;
			}
		}
		return j;
	}
};

WorldPoint Point(const Json &j) {
	WorldPoint w;
	w.x = j["x"].num;
	w.y = j["y"].num;
	w.z = j["z"].num;
	return w;
}

// Transform.InverseTransformPoint of the canvas the state logged: q^-1 (p - t) / s.
bool CanvasPoint(const Json &st, WorldPoint &r_local) {
	const Json &h = st["primaryHandPos"];
	const Json &t = st["canvasPos"];
	const Json &q = st["canvasRot"];
	if (h.kind != Json::OBJ || t.kind != Json::OBJ || q.kind != Json::OBJ || st["canvasScale"].kind != Json::NUM) {
		return false;
	}
	double vx = h["x"].num - t["x"].num;
	double vy = h["y"].num - t["y"].num;
	double vz = h["z"].num - t["z"].num;
	double qx = -q["x"].num;
	double qy = -q["y"].num;
	double qz = -q["z"].num;
	double qw = q["w"].num;
	double tx = 2.0 * (qy * vz - qz * vy);
	double ty = 2.0 * (qz * vx - qx * vz);
	double tz = 2.0 * (qx * vy - qy * vx);
	double s = st["canvasScale"].num;
	r_local.x = (vx + qw * tx + (qy * tz - qz * ty)) / s;
	r_local.y = (vy + qw * ty + (qz * tx - qx * tz)) / s;
	r_local.z = (vz + qw * tz + (qx * ty - qy * tx)) / s;
	return true;
}

// StudyUtils.MirrorModelMapping[sketchModel] offset by InputController.OnModelChange's origin, the
// rig's view point snapped down to 0.25 m. The rig is not logged: the origin per interactionMode is
// what every session's mirror-plane constraints agree on (study x 0 and z 0.75, free creation x 0.25).
bool MirrorPlane(const Json &root, WorldPoint &r_point, WorldPoint &r_normal, std::string &r_error) {
	if (root["sketchModel"].kind != Json::NUM || root["interactionMode"].kind != Json::NUM) {
		r_error = "session has no sketchModel and interactionMode";
		return false;
	}
	int model = (int)root["sketchModel"].num;
	int mode = (int)root["interactionMode"].num;
	static const double kModelPoint[4][3] = { { -0.125, 0.125, 0.125 }, { 0.125, 0.125, 0.0 }, { 0.125, 0.125, 0.0 }, { 0.0, 0.125, 0.125 } };
	static const double kModelNormal[4][3] = { { 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, 1.0 }, { 1.0, 0.0, 0.0 } };
	static const double kStudyOrigin[3] = { 0.0, 1.0, 0.75 };
	static const double kFreeCreationOrigin[3] = { 0.25, 1.0, 0.75 };
	if (model < 0 || model > 3 || mode < 0 || mode > 2) {
		r_error = "session has sketchModel " + std::to_string(model) + ", interactionMode " + std::to_string(mode);
		return false;
	}
	const double *origin = mode == 2 ? kFreeCreationOrigin : kStudyOrigin;
	r_point.x = kModelPoint[model][0] + origin[0];
	r_point.y = kModelPoint[model][1] + origin[1];
	r_point.z = kModelPoint[model][2] + origin[2];
	r_normal.x = kModelNormal[model][0];
	r_normal.y = kModelNormal[model][1];
	r_normal.z = kModelNormal[model][2];
	return true;
}

void Run(const Json &root, bool p_trace, SessionResult &r) {
	std::map<int, const Json *> by_id;
	for (const Json &s : root["allSketchedStrokes"].arr) {
		by_id[(int)s["id"].num] = &s;
	}
	std::map<int, LoggedPatch> patches;
	for (const Json &p : root["allCreatedPatches"].arr) {
		LoggedPatch lp;
		lp.id = (int)p["id"].num;
		lp.found_by_algo = p["foundByAlgo"].b;
		for (const Json &v : p["strokesID"].arr) {
			lp.strokes.push_back((int)v.num);
		}
		patches[lp.id] = lp;
	}
	WorldPoint origin;
	WorldPoint mirror_point;
	WorldPoint mirror_normal;
	if (!MirrorPlane(root, mirror_point, mirror_normal, r.error)) {
		return;
	}
	Replay replay(origin, mirror_point, mirror_normal);
	// StudyUtils.SketchSystem: 0 Baseline, 1 Snap, 2 SnapSurface.
	if (root["sketchSystem"].kind != Json::NUM) {
		r.error = "session has no sketchSystem";
		return;
	}
	replay.SetSurfacing((int)root["sketchSystem"].num == 2);
	std::vector<LoggedPatch> pending_log;
	const std::vector<Json> &states = root["systemStates"].arr;
	std::vector<std::vector<int>> before;
	SessionResult::Event event;
	bool have_event = false;
	size_t i = 0;
	while (i < states.size()) {
		const Json &st = states[i];
		int type = (int)st["interactionType"].num;
		int id = (int)st["elementID"].num;
		bool mirroring = st["mirroring"].b;
		size_t next = i + 1;
		if (type == 3 && patches[id].found_by_algo) {
			pending_log.push_back(patches[id]);
		} else if (type == 3) {
			std::vector<LoggedPatch> grp;
			grp.push_back(patches[id]);
			while (next < states.size() && (int)states[next]["interactionType"].num == 3 && !patches[(int)states[next]["elementID"].num].found_by_algo && states[next]["time"].num == st["time"].num) {
				grp.push_back(patches[(int)states[next]["elementID"].num]);
				next++;
			}
			WorldPoint tap;
			if (!CanvasPoint(st, tap)) {
				r.error = "tapped patch " + std::to_string(id) + " has no primaryHandPos and canvas transform";
				return;
			}
			if (p_trace) {
				before = replay.Cycles(true);
				event = SessionResult::Event();
				event.state = (int)i;
				have_event = true;
			}
			replay.AddUserPatches(grp, tap, mirroring);
		} else if (type == 4) {
			replay.DeletePatch(id);
		} else if (type == 1 || type == 2) {
			if (p_trace && type == 1) {
				before = replay.Cycles(true);
				event = SessionResult::Event();
				event.state = (int)i;
				have_event = true;
			}
			replay.SetPendingPatches(pending_log);
			pending_log.clear();
			if (type == 1) {
				std::map<int, const Json *>::const_iterator found = by_id.find(id);
				if (found == by_id.end()) {
					r.error = "stroke " + std::to_string(id) + " is not in allSketchedStrokes";
					return;
				}
				const Json &s = *found->second;
				std::vector<WorldPoint> ctrl;
				for (const Json &p : s["ctrlPts"].arr) {
					ctrl.push_back(Point(p));
				}
				std::vector<const Json *> recorded;
				for (const Json &c : s["appliedPositionConstraints"].arr) {
					recorded.push_back(&c);
				}
				// A line curve's rejected constraints still land on it within 0.1 r_proximity.
				if (ctrl.size() == 2) {
					for (const Json &c : s["rejectedPositionConstraints"].arr) {
						recorded.push_back(&c);
					}
				}
				std::vector<RecordedConstraint> cons;
				for (const Json *cp : recorded) {
					RecordedConstraint rc;
					rc.position = Point((*cp)["position"]);
					rc.is_intersection = (*cp)["isIntersection"].b;
					rc.is_at_existing_node = (*cp)["isAtExistingNode"].b;
					rc.is_at_new_endpoint = (*cp)["isAtNewEndpoint"].b;
					cons.push_back(rc);
				}
				replay.AddStroke(id, ctrl, cons, s["closedLoop"].b, mirroring, (float)st["canvasScale"].num);
			} else {
				replay.DeleteStroke(id, mirroring);
			}
		}
		if (p_trace && type >= 1 && type <= 4) {
			r.trace.push_back(std::make_pair((int)i, replay.Cycles(true)));
		}
		if (have_event) {
			std::vector<std::vector<int>> after = replay.Cycles(true);
			for (const std::vector<int> &c : after) {
				if (std::find(before.begin(), before.end(), c) == before.end()) {
					event.added.push_back(c);
				}
			}
			std::sort(event.added.begin(), event.added.end());
			r.events.push_back(event);
			have_event = false;
		}
		i = next;
	}
	r.boundaries = replay.Boundaries(false);
	for (const CycleBoundary &b : r.boundaries) {
		r.cycles.push_back(b.strokes);
	}
	r.user_cycles = (int)replay.Cycles(true).size() - (int)r.cycles.size();
	r.stats = replay.Stats();
	r.ok = true;
}

} // namespace

SessionResult ReplaySession(const std::string &p_json, bool p_trace) {
	SessionResult r;
	Parser parser;
	parser.p = p_json.c_str();
	Json root = parser.value();
	if (parser.bad || root.kind != Json::OBJ) {
		r.error = "session is not a JSON object";
		return r;
	}
	if (root["systemStates"].kind != Json::ARR) {
		r.error = "session has no systemStates";
		return r;
	}
	try {
		Run(root, p_trace, r);
	} catch (const std::exception &e) {
		r.ok = false;
		r.error = std::string("replay threw: ") + e.what();
	}
	return r;
}

std::string FormatSessionResult(const SessionResult &p_result) {
	if (!p_result.ok) {
		return "error " + p_result.error + "\n";
	}
	std::string out = "ok cycles=" + std::to_string(p_result.cycles.size()) + " user=" + std::to_string(p_result.user_cycles);
	const ReplayStats &s = p_result.stats;
	char buf[192];
	std::snprintf(buf, sizeof(buf), " unresolved=%d seams=%d on_mirror=%d tap_misses=%d missing_patch_deletes=%d exceptions=%d\n",
			s.unresolved_constraints, s.mirror_seam_constraints, s.on_mirror_strokes, s.tap_misses, s.missing_patch_deletes, s.caught_exceptions);
	out += buf;
	int broken = 0;
	for (const CycleBoundary &b : p_result.boundaries) {
		broken += b.broken_walk ? 1 : 0;
	}
	out.insert(out.size() - 1, " broken_walks=" + std::to_string(broken));
	for (size_t i = 0; i < p_result.cycles.size(); i++) {
		const std::vector<int> &c = p_result.cycles[i];
		for (size_t k = 0; k < c.size(); k++) {
			if (k > 0) {
				out += ' ';
			}
			out += std::to_string(c[k]);
		}
		out += " |";
		for (const BoundarySpan &sp : p_result.boundaries[i].spans) {
			char span[64];
			std::snprintf(span, sizeof(span), " %d:%.7g:%.7g", sp.stroke, sp.t_from, sp.t_to);
			out += span;
		}
		out += '\n';
	}
	return out;
}

std::string FormatSessionEvents(const SessionResult &p_result) {
	if (!p_result.ok) {
		return "error " + p_result.error + "\n";
	}
	std::string out;
	for (const SessionResult::Event &e : p_result.events) {
		out += std::to_string(e.state) + "\t";
		for (size_t c = 0; c < e.added.size(); c++) {
			for (size_t k = 0; k < e.added[c].size(); k++) {
				out += (k ? "," : "") + std::to_string(e.added[c][k]);
			}
			out += c + 1 < e.added.size() ? ";" : "";
		}
		out += "\n";
	}
	return out;
}

} // namespace cassie_graph_port
