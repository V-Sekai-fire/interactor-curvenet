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
	const char *path = argc > 1 ? argv[1] : "C:/meshing-pen/6-datasource/cassie/data/raw_data/dress.json";
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
	cassie_graph_port::WorldPoint origin;
	cassie_graph_port::WorldPoint mirror_point;
	mirror_point.x = 0.125;
	cassie_graph_port::WorldPoint mirror_normal;
	mirror_normal.x = 1.0;
	cassie_graph_port::Replay replay(origin, mirror_point, mirror_normal);
	int adds = 0;
	int deletes = 0;
	for (const Json &st : root["systemStates"].arr) {
		int type = (int)st["interactionType"].num;
		int id = (int)st["elementID"].num;
		bool mirroring = st["mirroring"].b;
		if (type == 1) {
			std::map<int, const Json *>::iterator it = by_id.find(id);
			if (it == by_id.end()) {
				continue;
			}
			const Json &s = *it->second;
			std::vector<cassie_graph_port::WorldPoint> ctrl;
			for (const Json &p : s["ctrlPts"].arr) {
				ctrl.push_back(Point(p));
			}
			std::vector<cassie_graph_port::RecordedConstraint> cons;
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
			for (const Json *cp : recorded) {
				const Json &c = *cp;
				cassie_graph_port::RecordedConstraint rc;
				rc.position = Point(c["position"]);
				rc.is_intersection = c["isIntersection"].b;
				rc.is_at_existing_node = c["isAtExistingNode"].b;
				rc.is_at_new_endpoint = c["isAtNewEndpoint"].b;
				cons.push_back(rc);
			}
			replay.AddStroke(id, ctrl, cons, s["closedLoop"].b, mirroring);
			adds++;
		} else if (type == 2) {
			replay.DeleteStroke(id, mirroring);
			deletes++;
		}
	}
	std::vector<std::vector<int>> cycles = replay.Cycles();
	cassie_graph_port::ReplayStats stats = replay.Stats();
	std::printf("adds %d deletes %d cycles %d unresolved %d seams %d on_mirror %d exceptions %d\n", adds, deletes, (int)cycles.size(),
			stats.unresolved_constraints, stats.mirror_seam_constraints, stats.on_mirror_strokes, stats.caught_exceptions);
	for (const std::vector<int> &c : cycles) {
		for (size_t i = 0; i < c.size(); i++) {
			std::printf(i ? " %d" : "%d", c[i]);
		}
		std::printf("\n");
	}
	return 0;
}
