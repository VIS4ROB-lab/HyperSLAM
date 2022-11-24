/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>
#include <csignal>
#include <filesystem>
#include <fstream>

#include "hyper/system/system.hpp"
#include "hyper/yaml/yaml.hpp"

namespace {

using namespace hyper;

// Definitions.
using Path = std::filesystem::path;

// Arguments.
constexpr auto kNameArgument = 0;
constexpr auto kInputArgument = kNameArgument + 1;
constexpr auto kOutputArgument = kInputArgument + 1;
constexpr auto kNumArguments = kOutputArgument + 1;

struct Cluster {
  // Definitions.
  using Systems = std::set<std::unique_ptr<System>>;

  Path input_file;       ///< Input file.
  Path output_directory; ///< Output directory.
  Path log_directory;    ///< Logging directory.

  Systems systems; ///< Systems.
};

Cluster cluster; ///< Cluster.

/// Starts all systems.
auto Run() -> int {
  for (const auto& system : cluster.systems) {
    system->start();
  }
  ros::waitForShutdown();
  return EXIT_SUCCESS;
}

/// Shuts down all systems.
auto Shutdown(int sig) -> void {
  if (sig == SIGINT || sig == SIGUSR1) {
    for (const auto& system : cluster.systems) {
      // Write estimation file.
      if (sig == SIGUSR1) {
        for (const auto& [name, backend] : system->backends()) {
          // Create output file.
          constexpr auto kPrecision = 20;
          std::ofstream estimation_file;
          estimation_file.open(cluster.output_directory / "estimation.hyper");
          std::cout<<"estimation.hyper created"<<std::endl;
          estimation_file.precision(kPrecision);

          // Define output format.
          static const auto kFormat = Eigen::IOFormat{kPrecision, Eigen::DontAlignCols, ", ", "\n"};

          std::cout<<"wait backend"<<std::endl;
          // Wait on backend.
          auto lock = backend->wait();
          const auto& optimizer = backend->optimizer();

          if (optimizer) {
            // Write estimates.
            constexpr auto kRate = 100;
            constexpr auto kValueIndex = 0;
            const auto& state = optimizer->state();
            for (const auto& sample : state.range().sample(kRate)) {
              // Evaluate state.
              const auto query = StateQuery{sample, kValueIndex};
              const auto result = state.evaluate(query);

              // Write estimate.
              const auto stamp = optimizer->root() + sample;

              estimation_file << std::scientific << stamp << ", " << result.derivatives[kValueIndex].transpose().format(kFormat) << "\n";
            }
          }

          // Close output file.
          estimation_file.close();
        }
      }

      // System shutdown.
      system->shutdown();
    }

    // ROS shutdown.
    ros::requestShutdown();
    std::cout<<"ros shutdown"<<std::endl;

  } else {
    LOG(FATAL) << "Unknown interrupt signal.";
  }
}

} // namespace

/// Main function for the HyperSLAM pipeline.
/// \param argc Argument count.
/// \param argv Character array.
/// \return EXIT_SUCCESS or EXIT_FAILURE.
auto main(int argc, char* argv[]) -> int {
  // Read program name.
  const auto program_name = argv[kNameArgument];

  // Initialize logging.
  google::InitGoogleLogging(program_name);

  // Read additional arguments.
  if (argc < kNumArguments) {
    LOG(FATAL) << "Usage: " << program_name << " <PATH_TO_INPUT_FILE> <PATH_TO_OUTPUT_DIRECTORY>";

  } else {
    // Read input arguments.
    cluster.input_file = Path{argv[kInputArgument]};
    cluster.output_directory = Path{argv[kOutputArgument]};
    cluster.log_directory = cluster.output_directory / "log";

    // Set logging flags.
    FLAGS_log_dir = cluster.log_directory;
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = 2;

    try {
      // Load YAML file.
      const auto yaml_file = YAML::LoadFile(cluster.input_file);

      // Initialize ROS.
      const auto node_name = yaml::ReadAs<std::string>(yaml_file, "Node");
      ros::init(argc, argv, node_name);

      // Initialize HyperSLAM.
      const auto system_node = yaml::Read(yaml_file, "System");
      auto system = std::make_unique<System>(system_node);
      cluster.systems.insert(std::move(system));

      // Define interrupt actions.
      struct sigaction sigint_action {};
      sigint_action.sa_handler = Shutdown;
      sigemptyset(&sigint_action.sa_mask);
      sigint_action.sa_flags = 0;
      sigaction(SIGINT, &sigint_action, nullptr);

      struct sigaction sigusr1_action {};
      sigusr1_action.sa_handler = Shutdown;
      sigemptyset(&sigusr1_action.sa_mask);
      sigusr1_action.sa_flags = 0;
      sigaction(SIGUSR1, &sigusr1_action, nullptr);

      // Start systems.
      return Run();

    } catch (...) {
      LOG(FATAL) << "YAML file is malformed or does not exist.";
      return EXIT_FAILURE;
    }
  }
}
