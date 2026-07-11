#include <drone_mapper/ConfigLoader.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactory.h>

#include <filesystem>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    if (argc > 3) {
        std::cerr << "Usage: drone_mapper_simulation [<simulation.yaml>] [<output_path>]\n";
        return 1;
    }

    try {
        const std::filesystem::path composition_file =
            (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
        const std::filesystem::path output_path =
            (argc >= 3) ? drone_mapper::ConfigLoader::resolveInputPath(argv[2])
                        : std::filesystem::current_path();

        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactory>();
        drone_mapper::SimulationManager simulation{std::move(run_factory)};
        const drone_mapper::types::SimulationCompositionData composition =
            drone_mapper::ConfigLoader::loadComposition(composition_file);
        const drone_mapper::types::SimulationManagerReport report = simulation.run(composition, output_path);

        std::cout << "drone_mapper_simulation completed " << report.runs.size()
                  << " run(s). Results were written under " << output_path << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
