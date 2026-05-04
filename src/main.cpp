#include "problem_description.h"
#include "radar_algo.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

struct RuntimeOptions {
    bool write_truth_csv = true;
};

RuntimeOptions parseRuntimeOptions(int argc, char **argv) {
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--no-truth" || arg == "--no-truth-csv") {
            options.write_truth_csv = false;
            continue;
        }

        throw std::runtime_error("unknown argument: " + std::string(arg));
    }

    if (const char *disable_truth_csv = std::getenv("VORTEX_DISABLE_TRUTH_CSV")) {
        options.write_truth_csv = std::strcmp(disable_truth_csv, "0") != 0;
    }

    return options;
}

} // namespace

int main(int argc, char **argv) {
    try {
        parseRuntimeOptions(argc, argv);

        const problem::ProblemDescription description =
            problem::kDefaultProblemDescription;
        radar_algo::runRadarAlgorithm(description);

        std::cout << "Optimized simulation complete (Batch I/O Enabled).\n";
    } catch (const std::exception &ex) {
        std::cerr << "Radar demo failed: " << ex.what() << '\n';
        return 1;
    }
    return 0;
}
