// Native mesh-loops oracle for the CASSIE graph port: a cube split n x n per side, a torus or an
// open tube, each ring cut at random into strokes; the replay must end with exactly the quads.
// MIT. Generator follows transport-meshing-pen tests/cassie_mesh_loops.gd.
//   mesh_loops_oracle [--mesh=cube|torus|tube] [--straight] [--flat] [--control] [--seed=N]

#include "../vendor/cassie/src/graph_port/session_replay.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct P3 {
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};
P3 operator+(P3 a, P3 b) { return P3{ a.x + b.x, a.y + b.y, a.z + b.z }; }
P3 operator-(P3 a, P3 b) { return P3{ a.x - b.x, a.y - b.y, a.z - b.z }; }
P3 operator*(P3 a, double s) { return P3{ a.x * s, a.y * s, a.z * s }; }
double Dot(P3 a, P3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
P3 Unit(P3 a) {
	double m = std::sqrt(Dot(a, a));
	return m > 0.0 ? a * (1.0 / m) : a;
}

using Key = int;
using Loop = std::vector<Key>;

struct Mesh {
	std::map<Key, P3> points;
	std::vector<Loop> rings;
	std::vector<Loop> faces;
	std::vector<Loop> caps;
};

struct Drawing {
	std::string mesh;
	int n = 0;
	double nudge = 0.0;
	Mesh m;
	std::vector<Loop> strokes;
};

const double kSize = 0.2;
const P3 kCentre{ -0.1, 1.1, 0.4 };
const double kTau = 6.283185307179586;

Key Grid3(int i, int j, int k) { return (i * 64 + j) * 64 + k; }
Key Grid2(int u, int v) { return u * 64 + v; }

Mesh Cube(int n) {
	Mesh m;
	for (int i = 0; i <= n; i++) {
		for (int j = 0; j <= n; j++) {
			for (int k = 0; k <= n; k++) {
				if (i == 0 || i == n || j == 0 || j == n || k == 0 || k == n) {
					m.points[Grid3(i, j, k)] = P3{ double(i) / n, double(j) / n, double(k) / n };
				}
			}
		}
	}
	for (int axis = 0; axis < 3; axis++) {
		for (int level = 0; level <= n; level++) {
			std::vector<std::pair<int, int>> perimeter;
			for (int s = 0; s < n; s++) {
				perimeter.push_back({ s, 0 });
			}
			for (int s = 0; s < n; s++) {
				perimeter.push_back({ n, s });
			}
			for (int s = 0; s < n; s++) {
				perimeter.push_back({ n - s, n });
			}
			for (int s = 0; s < n; s++) {
				perimeter.push_back({ 0, n - s });
			}
			perimeter.push_back({ 0, 0 });
			Loop ring;
			for (const std::pair<int, int> &p : perimeter) {
				int c[3] = { 0, 0, 0 };
				c[axis] = level;
				c[(axis + 1) % 3] = p.first;
				c[(axis + 2) % 3] = p.second;
				ring.push_back(Grid3(c[0], c[1], c[2]));
			}
			m.rings.push_back(ring);
		}
	}
	const int d[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
	for (int axis = 0; axis < 3; axis++) {
		for (int side : { 0, n }) {
			for (int u = 0; u < n; u++) {
				for (int v = 0; v < n; v++) {
					Loop corners;
					for (int q = 0; q < 4; q++) {
						int c[3] = { 0, 0, 0 };
						c[axis] = side;
						c[(axis + 1) % 3] = u + d[q][0];
						c[(axis + 2) % 3] = v + d[q][1];
						corners.push_back(Grid3(c[0], c[1], c[2]));
					}
					m.faces.push_back(corners);
				}
			}
		}
	}
	return m;
}

Mesh Torus(int n) {
	Mesh m;
	for (int u = 0; u < n; u++) {
		for (int v = 0; v < n; v++) {
			double a = kTau * u / n;
			double b = kTau * v / n;
			double r = 0.35 + 0.15 * std::cos(b);
			m.points[Grid2(u, v)] = P3{ 0.5 + std::cos(a) * r, 0.5 + 0.15 * std::sin(b), 0.5 + std::sin(a) * r };
		}
	}
	for (int u = 0; u < n; u++) {
		Loop ring;
		for (int v = 0; v <= n; v++) {
			ring.push_back(Grid2(u, v % n));
		}
		m.rings.push_back(ring);
	}
	for (int v = 0; v < n; v++) {
		Loop ring;
		for (int u = 0; u <= n; u++) {
			ring.push_back(Grid2(u % n, v));
		}
		m.rings.push_back(ring);
	}
	for (int u = 0; u < n; u++) {
		for (int v = 0; v < n; v++) {
			m.faces.push_back({ Grid2(u, v), Grid2((u + 1) % n, v), Grid2((u + 1) % n, (v + 1) % n), Grid2(u, (v + 1) % n) });
		}
	}
	return m;
}

Mesh Tube(int n) {
	Mesh m;
	int rows = n - 1;
	for (int u = 0; u < n; u++) {
		for (int v = 0; v <= rows; v++) {
			double a = kTau * u / n;
			m.points[Grid2(u, v)] = P3{ 0.5 + 0.3 * std::cos(a), double(v) / rows, 0.5 + 0.3 * std::sin(a) };
		}
	}
	for (int v = 0; v <= rows; v++) {
		Loop ring;
		for (int u = 0; u <= n; u++) {
			ring.push_back(Grid2(u % n, v));
		}
		m.rings.push_back(ring);
	}
	for (int u = 0; u < n; u++) {
		Loop ring;
		for (int v = 0; v <= rows; v++) {
			ring.push_back(Grid2(u, v));
		}
		m.rings.push_back(ring);
	}
	for (int u = 0; u < n; u++) {
		for (int v = 0; v < rows; v++) {
			m.faces.push_back({ Grid2(u, v), Grid2((u + 1) % n, v), Grid2((u + 1) % n, v + 1), Grid2(u, v + 1) });
		}
	}
	m.caps.push_back(m.rings[0]);
	m.caps.push_back(m.rings[rows]);
	return m;
}

std::pair<Key, Key> Edge(Key a, Key b) { return a < b ? std::make_pair(a, b) : std::make_pair(b, a); }

bool Straight(const std::map<Key, P3> &pts, Key a, Key b, Key c) {
	return Dot(Unit(pts.at(b) - pts.at(a)), Unit(pts.at(c) - pts.at(b))) > 0.999;
}

struct Options {
	std::string only;
	bool straight = false;
	bool flat = false;
	bool control = false;
	// --loops=open splits every stroke that closes on itself; --loops=closed draws each ring as one closed stroke.
	std::string loops;
	uint32_t seed = 20261005u;
	int detail = 0;
	int rung_only = -1;
	std::string dump;
};

Drawing Draw(std::mt19937 &rng, int rung, const Options &o) {
	std::uniform_real_distribution<double> unit(0.0, 1.0);
	Drawing d;
	d.n = rung + 1;
	d.nudge = o.flat ? 0.0 : 0.2 * double(rung) / 2.0 * unit(rng);
	if (o.only.empty()) {
		const char *kinds[3] = { "cube", "torus", "tube" };
		d.mesh = kinds[rng() % 3];
	} else {
		d.mesh = o.only;
	}
	if (d.mesh == "cube") {
		d.m = Cube(d.n);
	} else if (d.mesh == "torus") {
		d.m = Torus(d.n + 7);
	} else {
		d.m = Tube(d.n + 7);
	}
	for (std::pair<const Key, P3> &kv : d.m.points) {
		P3 jitter{ unit(rng) * 2.0 - 1.0, unit(rng) * 2.0 - 1.0, unit(rng) * 2.0 - 1.0 };
		kv.second = kv.second + jitter * (d.nudge / double(d.n));
	}
	std::set<std::pair<Key, Key>> used;
	for (const Loop &ring : d.m.rings) {
		Loop run{ ring[0] };
		for (size_t r = 1; r < ring.size(); r++) {
			std::pair<Key, Key> e = Edge(ring[r - 1], ring[r]);
			if (used.count(e)) {
				if (run.size() > 1) {
					d.strokes.push_back(run);
				}
				run = Loop{ ring[r] };
				continue;
			}
			used.insert(e);
			bool turns = run.size() > 1 && !Straight(d.m.points, run[run.size() - 2], ring[r - 1], ring[r]);
			// Same RNG draws as the GDScript generator: rng() only when the run can be cut.
			bool cut = run.size() > 1 && rng() % 3 == 0;
			if (o.loops == "closed") {
				cut = false;
			}
			if (run.size() > 1 && (cut || (o.straight && turns))) {
				d.strokes.push_back(run);
				run = Loop{ ring[r - 1] };
			}
			run.push_back(ring[r]);
		}
		if (run.size() > 1) {
			d.strokes.push_back(run);
		}
	}
	if (o.loops == "open") {
		std::vector<Loop> split;
		for (const Loop &arc : d.strokes) {
			if (arc.size() > 3 && arc.front() == arc.back()) {
				size_t mid = arc.size() / 2;
				split.push_back(Loop(arc.begin(), arc.begin() + long(mid) + 1));
				split.push_back(Loop(arc.begin() + long(mid), arc.end()));
			} else {
				split.push_back(arc);
			}
		}
		d.strokes = split;
	}
	std::shuffle(d.strokes.begin(), d.strokes.end(), rng);
	for (Loop &s : d.strokes) {
		if (rng() % 2 == 1) {
			std::reverse(s.begin(), s.end());
		}
	}
	return d;
}

std::vector<std::vector<int>> Cycles(const Drawing &d, const std::vector<Loop> &loops) {
	std::map<std::pair<Key, Key>, int> stroke_of;
	for (size_t k = 0; k < d.strokes.size(); k++) {
		const Loop &arc = d.strokes[k];
		for (size_t i = 1; i < arc.size(); i++) {
			stroke_of[Edge(arc[i - 1], arc[i])] = int(k);
		}
	}
	std::vector<std::vector<int>> out;
	for (const Loop &q : loops) {
		std::vector<int> ids;
		for (size_t i = 0; i < q.size(); i++) {
			Key b = q[(i + 1) % q.size()];
			if (q[i] == b) {
				continue;
			}
			int s = stroke_of.at(Edge(q[i], b));
			if (std::find(ids.begin(), ids.end(), s) == ids.end()) {
				ids.push_back(s);
			}
		}
		std::sort(ids.begin(), ids.end());
		out.push_back(ids);
	}
	std::sort(out.begin(), out.end());
	return out;
}

P3 World(const Drawing &d, Key k) { return kCentre + (d.m.points.at(k) - P3{ 0.5, 0.5, 0.5 }) * kSize; }

std::string Json(P3 p) {
	char buf[128];
	std::snprintf(buf, sizeof(buf), "{\"x\":%.9g,\"y\":%.9g,\"z\":%.9g}", p.x, p.y, p.z);
	return buf;
}

std::string Session(const Drawing &d) {
	std::ostringstream states;
	std::ostringstream strokes;
	std::set<Key> held;
	for (size_t k = 0; k < d.strokes.size(); k++) {
		const Loop &arc = d.strokes[k];
		std::ostringstream ctrl;
		for (size_t i = 0; i < arc.size(); i++) {
			P3 p = World(d, arc[i]);
			if (i > 0) {
				P3 q = World(d, arc[i - 1]);
				ctrl << "," << Json(q + (p - q) * (1.0 / 3.0)) << "," << Json(q + (p - q) * (2.0 / 3.0)) << ",";
			}
			ctrl << Json(p);
		}
		bool closed = arc.size() > 2 && arc.front() == arc.back();
		std::ostringstream cons;
		bool first = true;
		size_t limit = arc.size() - (closed ? 1 : 0);
		for (size_t i = 0; i < limit; i++) {
			if (held.count(arc[i])) {
				bool at_end = i == 0 || i == arc.size() - 1;
				cons << (first ? "" : ",") << "{\"position\":" << Json(World(d, arc[i]))
					 << ",\"isIntersection\":true,\"isAtExistingNode\":true,\"isAtNewEndpoint\":" << (at_end ? "true" : "false")
					 << ",\"alignTangents\":false}";
				first = false;
			}
		}
		for (Key key : arc) {
			held.insert(key);
		}
		strokes << (k ? "," : "") << "{\"id\":" << k << ",\"ctrlPts\":[" << ctrl.str() << "],\"closedLoop\":" << (closed ? "true" : "false")
				<< ",\"planar\":false,\"appliedPositionConstraints\":[" << cons.str() << "],\"rejectedPositionConstraints\":[]}";
		states << (k ? "," : "") << "{\"interactionType\":1,\"elementID\":" << k << ",\"mirroring\":false,\"canvasScale\":1.0,\"time\":" << k << ".0}";
	}
	return "{\"sketchSystem\":2,\"sketchModel\":0,\"interactionMode\":2,\"systemStates\":[" + states.str() +
			"],\"allSketchedStrokes\":[" + strokes.str() + "],\"allCreatedPatches\":[]}";
}

struct Score {
	std::vector<std::vector<int>> missing;
	std::vector<std::vector<int>> extras;
	int quads = 0;
	int hit = 0;
	int extra = 0;
	bool ok = true;
	std::string error;
};

Score Run(const Drawing &d, bool control) {
	Score s;
	std::vector<std::vector<int>> want = Cycles(d, d.m.faces);
	if (control) {
		want.push_back({ -1 });
	}
	std::vector<std::vector<int>> caps = Cycles(d, d.m.caps);
	cassie_graph_port::SessionResult r = cassie_graph_port::ReplaySession(Session(d), false);
	if (!r.ok) {
		s.ok = false;
		s.error = r.error;
	}
	std::vector<std::vector<int>> got;
	for (const std::vector<int> &c : r.cycles) {
		if (std::find(caps.begin(), caps.end(), c) == caps.end()) {
			got.push_back(c);
		}
	}
	s.quads = int(want.size());
	for (const std::vector<int> &w : want) {
		if (std::find(got.begin(), got.end(), w) != got.end()) {
			s.hit++;
		} else {
			s.missing.push_back(w);
		}
	}
	for (const std::vector<int> &g : got) {
		if (std::find(want.begin(), want.end(), g) == want.end()) {
			s.extra++;
			s.extras.push_back(g);
		}
	}
	return s;
}

std::string KeyName(const Drawing &d, Key k) {
	char buf[32];
	if (d.mesh == "cube") {
		std::snprintf(buf, sizeof(buf), "%d%d%d", k / 4096, (k / 64) % 64, k % 64);
	} else {
		std::snprintf(buf, sizeof(buf), "%d.%d", k / 64, k % 64);
	}
	return buf;
}

void PrintIds(const Drawing &d, const std::vector<int> &ids) {
	std::printf("  {");
	for (int id : ids) {
		std::printf(" s%d[", id);
		if (id >= 0) {
			const Loop &arc = d.strokes[size_t(id)];
			for (size_t i = 0; i < arc.size(); i++) {
				std::printf("%s%s", i ? " " : "", KeyName(d, arc[i]).c_str());
			}
		}
		std::printf("]");
	}
	std::printf(" }\n");
}

void Detail(const Drawing &d, const Score &s) {
	std::printf("DETAIL %s n=%d nudge=%.3f strokes=%zu quads=%d hit=%d extra=%d\n", d.mesh.c_str(), d.n, d.nudge, d.strokes.size(), s.quads, s.hit,
			s.extra);
	for (const std::vector<int> &m : s.missing) {
		std::printf(" missing");
		PrintIds(d, m);
	}
	for (const std::vector<int> &e : s.extras) {
		std::printf(" extra");
		PrintIds(d, e);
	}
}

} // namespace

int main(int argc, char **argv) {
	Options o;
	for (int a = 1; a < argc; a++) {
		std::string s = argv[a];
		if (s.rfind("--mesh=", 0) == 0) {
			o.only = s.substr(7);
		} else if (s == "--straight") {
			o.straight = true;
		} else if (s == "--flat") {
			o.flat = true;
		} else if (s.rfind("--loops=", 0) == 0) {
			o.loops = s.substr(8);
		} else if (s == "--control") {
			o.control = true;
		} else if (s.rfind("--detail=", 0) == 0) {
			o.detail = std::stoi(s.substr(9));
		} else if (s.rfind("--dump=", 0) == 0) {
			o.dump = s.substr(7);
		} else if (s.rfind("--rung=", 0) == 0) {
			o.rung_only = std::stoi(s.substr(7));
		} else if (s.rfind("--seed=", 0) == 0) {
			o.seed = uint32_t(std::stoul(s.substr(7)));
		}
	}
	std::vector<std::string> kinds;
	if (o.only.empty()) {
		kinds = { "cube", "torus", "tube" };
	} else {
		kinds = { o.only };
	}
	std::mt19937 rng(o.seed);
	long closed_strokes = 0;
	long all_strokes = 0;
	int all_quads = 0;
	int all_hit = 0;
	int all_extra = 0;
	int failed = 0;
	for (const std::string &kind : kinds) {
		Options one = o;
		one.only = kind;
		for (int rung = 0; rung < 3; rung++) {
			if (o.rung_only >= 0 && rung != o.rung_only) {
				continue;
			}
			int quads = 0;
			int hit = 0;
			int extra = 0;
			int exact = 0;
			for (int i = 0; i < 20; i++) {
				Drawing d = Draw(rng, rung, one);
				Score s = Run(d, o.control);
				for (const Loop &arc : d.strokes) {
					all_strokes++;
					closed_strokes += (arc.size() > 2 && arc.front() == arc.back()) ? 1 : 0;
				}
				quads += s.quads;
				hit += s.hit;
				extra += s.extra;
				if (s.hit == s.quads && s.extra == 0) {
					exact++;
				} else {
					failed++;
					if (o.detail > 0) {
						o.detail--;
						Detail(d, s);
						if (!o.dump.empty()) {
							FILE *f = std::fopen(o.dump.c_str(), "wb");
							std::string text = Session(d);
							std::fwrite(text.data(), 1, text.size(), f);
							std::fclose(f);
							o.dump.clear();
						}
					}
				}
			}
			all_quads += quads;
			all_hit += hit;
			all_extra += extra;
			std::printf("RATE %s rung %d: %d of %d quads (%.1f%%), %d extra cycles, %d of 20 drawings exact\n", kind.c_str(), rung, hit, quads,
					100.0 * hit / quads, extra, exact);
		}
	}
	std::printf("STROKES %ld, closed %ld\n", all_strokes, closed_strokes);
	std::printf("TOTAL %d of %d quads (%.2f%%), %d extra, %d drawings not exact\n", all_hit, all_quads, 100.0 * all_hit / all_quads, all_extra,
			failed);
	std::printf("RESULT: %s\n", failed == 0 ? "PASS" : "FAIL");
	return failed == 0 ? 0 : 1;
}
