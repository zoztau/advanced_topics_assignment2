#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

namespace fs = std::filesystem;

void read_cpp_yaml(const fs::path& input_path) {
    const YAML::Node config = YAML::LoadFile(input_path);
    const YAML::Node national_inner = config["national"];

    std::cout << "national tag has " << national_inner.size() << " elements." << std::endl;
    for (const auto& team : national_inner) {
        std::cout << team << std::endl;
    }
}

void write_cpp_yaml(const fs::path& output_path) {
    YAML::Node root;

    const std::vector<std::string> sequence_values{"first_v", "2nd_v", "v3", "v_fourth"};
    for (const auto& value : sequence_values) {
        root["sequence"].push_back(value);
    }

    root["float_val"] = std::numbers::pi;
    root["int_val"] = 89;

    std::ofstream output(output_path, std::ios::trunc);
    output << root;
}

int main(int argc, char** argv) {
    const fs::path input_path = argc >= 2 ? fs::path{argv[1]} : fs::path{"docs/examples/example.yml"};
    const fs::path output_path = argc >= 3 ? fs::path{argv[2]} : fs::path{"docs/examples/generated.yml"};
    if (!fs::exists(input_path)) {
        std::cout << "file not found" << std::endl;
        return 1;
    }

    read_cpp_yaml(input_path);
    write_cpp_yaml(output_path);
    return 0;
}
