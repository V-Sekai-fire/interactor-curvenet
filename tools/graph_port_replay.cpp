// Replays a CASSIE session JSON through the graph port and prints the live cycles as stroke ids.
// MIT. Ported from CASSIE (Yu, Arora, Stanko, Baerentzen, Singh, Bousseau), MIT.

#include "../vendor/cassie/src/graph_port/session_replay.h"

#include <cstdio>
#include <string>
#include <vector>

int main(int argc, char **argv) {
	const char *path = "C:/meshing-pen/6-datasource/cassie/data/raw_data/dress.json";
	const char *dump_path = nullptr;
	bool match_cassie = false;
	bool events = false;
	for (int a = 1; a < argc; a++) {
		if (std::string(argv[a]) == "--match-cassie") {
			match_cassie = true;
		} else if (std::string(argv[a]) == "--events") {
			events = true;
		} else if (std::string(argv[a]) == "--dump" && a + 1 < argc) {
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
	cassie_graph_port::SessionResult result = cassie_graph_port::ReplaySession(text, dump_path != nullptr || events, match_cassie);
	if (events) {
		// One "state<TAB>a,b;c,d" line per stroke or tap event: the cycles it added.
		std::printf("%s\n", result.ok ? "ok" : result.error.c_str());
		for (const cassie_graph_port::SessionResult::Event &e : result.events) {
			std::printf("%d\t", e.state);
			for (size_t c = 0; c < e.added.size(); c++) {
				for (size_t k = 0; k < e.added[c].size(); k++) {
					std::printf(k ? ",%d" : "%d", e.added[c][k]);
				}
				std::printf(c + 1 < e.added.size() ? ";" : "");
			}
			std::printf("\n");
		}
		return result.ok ? 0 : 1;
	}
	if (dump_path != nullptr) {
		FILE *dump = std::fopen(dump_path, "wb");
		if (dump != nullptr) {
			std::fprintf(dump, "[");
			for (size_t e = 0; e < result.trace.size(); e++) {
				const std::vector<std::vector<int>> &all = result.trace[e].second;
				std::fprintf(dump, "%s[%d,[", e ? ",\n" : "", result.trace[e].first);
				for (size_t c = 0; c < all.size(); c++) {
					std::fprintf(dump, "%s[", c ? "," : "");
					for (size_t k = 0; k < all[c].size(); k++) {
						std::fprintf(dump, k ? ",%d" : "%d", all[c][k]);
					}
					std::fprintf(dump, "]");
				}
				std::fprintf(dump, "]]");
			}
			std::fprintf(dump, "]\n");
			std::fclose(dump);
		}
	}
	std::fputs(cassie_graph_port::FormatSessionResult(result).c_str(), stdout);
	return result.ok ? 0 : 1;
}
