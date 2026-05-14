/**
 * @file main.cpp
 * @brief Example: using the vrr_vpr C++ API for visual place recognition.
 *
 * Build:
 *   cd cpp && mkdir build && cd build
 *   cmake .. -DPython3_EXECUTABLE=$(which python3)
 *   make -j
 *
 * Run:
 *   # Make sure vrr_vpr Python package is installed / on PYTHONPATH:
 *   export PYTHONPATH=/path/to/Navi/NaviSkill/vpr:$PYTHONPATH
 *   ./vrr_vpr_example /path/to/database /path/to/query.jpg
 */

#include <cstdlib>
#include <iostream>
#include <string>

#include "vrr_vpr/vrr_vpr.h"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <database_folder> <query_image> [top_k]\n";
        return 1;
    }

    const std::string db_folder   = argv[1];
    const std::string query_image = argv[2];
    const int top_k = (argc > 3) ? std::atoi(argv[3]) : 5;

    try {
        // 1. Create the engine
        //    Use "salad-indoor" + color_rerank for cross-floor scenarios,
        //    or "megaloc" + rerank for same-floor geometric matching.
        std::cout << "[vrr] Initialising VPR engine...\n";
        vrr::VPREngine engine(
            "salad-indoor", "cuda",
            /*rerank=*/false, /*max_keypoints=*/1024, /*rerank_resize=*/512,
            /*color_rerank=*/true, /*color_weight=*/5.0f
        );
        std::cout << "[vrr] Descriptor dim: " << engine.descriptorDim() << "\n";

        // 2. Build database
        std::cout << "[vrr] Building database from: " << db_folder << "\n";
        int n = engine.buildDatabase(db_folder);
        std::cout << "[vrr] Indexed " << n << " images\n";

        // 3. Query with color re-scoring (best for cross-floor)
        std::cout << "[vrr] Querying: " << query_image << "  (top-"
                  << top_k << ")\n";
        vrr::QueryResult result = engine.queryWithColorRerank(
            query_image, top_k
        );

        std::cout << "\n--- Results ---\n";
        for (size_t i = 0; i < result.indices.size(); ++i) {
            std::cout << "  #" << i
                      << "  idx=" << result.indices[i]
                      << "  score=" << result.scores[i]
                      << "  path=" << result.paths[i] << "\n";
        }

        // 4. (Optional) Save database for later reuse
        // engine.saveDatabase("/tmp/vrr_db");

    } catch (const std::exception& e) {
        std::cerr << "[vrr] ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
