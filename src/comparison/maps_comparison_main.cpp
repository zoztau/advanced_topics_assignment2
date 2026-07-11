#include <drone_mapper/MapsComparison.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }

    try {
        std::optional<std::filesystem::path> comparison_config;
        if (argc == 4) {
            const std::string arg{argv[3]};
            const std::string prefix{"comparison_config="};
            if (arg.rfind(prefix, 0) != 0) {
                throw std::runtime_error("Optional argument must be comparison_config=<path>.");
            }
            comparison_config = std::filesystem::path{arg.substr(prefix.size())};
        }

        const double score = drone_mapper::MapsComparison::compareFiles(argv[1], argv[2], comparison_config);
        if (score < 0.0) {
            throw std::runtime_error("Map comparison failed.");
        }
        std::cout << score << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cout << "-1\n";
        std::cerr << error.what() << "\n";
        return 1;
    }
}
