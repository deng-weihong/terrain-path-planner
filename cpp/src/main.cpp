#include "terrain_planner/json.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    try {
        std::string input_path, output_path;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--help" || argument == "-h") {
                std::cout
                    << "Usage: terrain-plan [--input request.json] [--output result.json]\n"
                       "Without file arguments, reads JSON from stdin and writes JSON to stdout.\n"
                       "Native C++ graph search and whole-path smoothing; no Python runtime.\n";
                return 0;
            }
            if ((argument != "--input" && argument != "--output") || i + 1 >= argc)
                throw std::invalid_argument("Expected --input FILE or --output FILE");
            (argument == "--input" ? input_path : output_path) = argv[++i];
        }
        std::ifstream input_file;
        if (!input_path.empty()) {
            input_file.open(input_path);
            if (!input_file)
                throw std::runtime_error("Cannot open input file");
        }
        auto& input = input_path.empty() ? std::cin : input_file;
        const auto request = nlohmann::json::parse(input);
        const auto result = terrain_planner::run_request(request);
        if (output_path.empty())
            std::cout << result.dump(2) << '\n';
        else {
            std::ofstream output(output_path);
            if (!output)
                throw std::runtime_error("Cannot open output file");
            output << result.dump(2) << '\n';
            if (!output)
                throw std::runtime_error("Cannot write output file");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << nlohmann::json({{"error", error.what()}}).dump() << '\n';
        return 2;
    }
}
