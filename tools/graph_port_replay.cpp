// Replays a CASSIE session JSON through the graph port and prints the live cycles as stroke ids.
// MIT. Ported from CASSIE (Yu, Arora, Stanko, Baerentzen, Singh, Bousseau), MIT.

#include "../vendor/cassie/src/graph_port/cassie_graph_port.h"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

struct Json {
	enum Kind { NUL, BOOL, NUM, STR, ARR, OBJ } kind = NUL;
	bool b = false;
	double num = 0.0;
	std::string str;
	std::vector<Json> arr;
	std::vector<std::pair<std::string, Json>> obj;
	const Json &operator[](const char *k) const {
		static Json none;
		for (const std::pair<std::string, Json> &kv : obj) {
			if (kv.first == k) {
				return kv.second;
			}
		}
		return none;
	}
};

struct Parser {
	const char *p;
	void ws() {
		while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') {
			p++;
		}
	}
	std::string str() {
		std::string s;
		p++;
		while (*p != '"') {
			if (*p == '\\') {
				p++;
			}
			s += *p++;
		}
		p++;
		return s;
	}
	Json value() {
		ws();
		Json j;
		if (*p == '{') {
			j.kind = Json::OBJ;
			p++;
			ws();
			while (*p != '}') {
				ws();
				std::string k = str();
				ws();
				p++;
				Json v = value();
				j.obj.push_back(std::make_pair(k, v));
				ws();
				if (*p == ',') {
					p++;
				}
				ws();
			}
			p++;
		} else if (*p == '[') {
			j.kind = Json::ARR;
			p++;
			ws();
			while (*p != ']') {
				j.arr.push_back(value());
				ws();
				if (*p == ',') {
					p++;
				}
				ws();
			}
			p++;
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
			p = end;
		}
		return j;
	}
};

cassie_graph_port::WorldPoint Point(const Json &j) {
	cassie_graph_port::WorldPoint w;
	w.x = j["x"].num;
	w.y = j["y"].num;
	w.z = j["z"].num;
	return w;
}

} // namespace

int main(int argc, char **argv) {
	const char *path = "C:/meshing-pen/6-datasource/cassie/data/raw_data/dress.json";
	const char *dump_path = nullptr;
	for (int a = 1; a < argc; a++) {
		if (std::string(argv[a]) == "--dump" && a + 1 < argc) {
			dump_path = argv[++a];
		} else {
			path = argv[a];
		}
	}
	FILE *f = std::fopen(path, "rb");
	if (f == nullptr) {
		std::fprintf(stderr, "cannot open %s\n", path);
		return 1;
	}
	std::string text;
	char buf[65536];
	size_t n = 0;
	while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
		text.append(buf, n);
	}
	std::fclose(f);
	Parser parser;
	parser.p = text.c_str();
	Json root = parser.value();

	std::map<int, const Json *> by_id;
	for (const Json &s : root["allSketchedStrokes"].arr) {
		by_id[(int)s["id"].num] = &s;
	}
	std::map<int, cassie_graph_port::LoggedPatch> patches;
	for (const Json &p : root["allCreatedPatches"].arr) {
		cassie_graph_port::LoggedPatch lp;
		lp.id = (int)p["id"].num;
		lp.found_by_algo = p["foundByAlgo"].b;
		for (const Json &v : p["strokesID"].arr) {
			lp.strokes.push_back((int)v.num);
		}
		patches[lp.id] = lp;
	}
	cassie_graph_port::WorldPoint origin;
	cassie_graph_port::WorldPoint mirror_point;
	mirror_point.x = 0.125;
	mirror_point.y = 0.125;
	mirror_point.z = 0.125;
	cassie_graph_port::WorldPoint mirror_normal;
	mirror_normal.x = 1.0;
	cassie_graph_port::Replay replay(origin, mirror_point, mirror_normal);
	FILE *dump = dump_path != nullptr ? std::fopen(dump_path, "wb") : nullptr;
	if (dump != nullptr) {
		std::fprintf(dump, "[");
	}
	bool first_dump = true;
	std::vector<cassie_graph_port::LoggedPatch> pending_log;
	const std::vector<Json> &states = root["systemStates"].arr;
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
			std::vector<cassie_graph_port::LoggedPatch> grp;
			grp.push_back(patches[id]);
			while (next < states.size() && (int)states[next]["interactionType"].num == 3 && !patches[(int)states[next]["elementID"].num].found_by_algo && states[next]["time"].num == st["time"].num) {
				grp.push_back(patches[(int)states[next]["elementID"].num]);
				next++;
			}
			replay.AddUserPatches(grp);
		} else if (type == 4) {
			replay.DeletePatch(id);
		} else if (type == 1 || type == 2) {
			replay.SetPendingPatches(pending_log);
			pending_log.clear();
			if (type == 1) {
				const Json &s = *by_id[id];
				std::vector<cassie_graph_port::WorldPoint> ctrl;
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
				std::vector<cassie_graph_port::RecordedConstraint> cons;
				for (const Json *cp : recorded) {
					cassie_graph_port::RecordedConstraint rc;
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
		if (dump != nullptr && type >= 1 && type <= 4) {
			std::vector<std::vector<int>> all = replay.Cycles(true);
			std::fprintf(dump, "%s[%d,[", first_dump ? "" : ",\n", (int)i);
			first_dump = false;
			for (size_t c = 0; c < all.size(); c++) {
				std::fprintf(dump, "%s[", c ? "," : "");
				for (size_t k = 0; k < all[c].size(); k++) {
					std::fprintf(dump, k ? ",%d" : "%d", all[c][k]);
				}
				std::fprintf(dump, "]");
			}
			std::fprintf(dump, "]]");
		}
		i = next;
	}
	if (dump != nullptr) {
		std::fprintf(dump, "]\n");
		std::fclose(dump);
	}
	std::vector<std::vector<int>> cycles = replay.Cycles(false);
	cassie_graph_port::ReplayStats stats = replay.Stats();
	std::printf("cycles %d unresolved %d seams %d on_mirror %d user_fallbacks %d missing_patch_deletes %d exceptions %d\n", (int)cycles.size(),
			stats.unresolved_constraints, stats.mirror_seam_constraints, stats.on_mirror_strokes, stats.user_fallbacks, stats.missing_patch_deletes, stats.caught_exceptions);
	for (const std::vector<int> &c : cycles) {
		for (size_t k = 0; k < c.size(); k++) {
			std::printf(k ? " %d" : "%d", c[k]);
		}
		std::printf("\n");
	}
	return 0;
}
