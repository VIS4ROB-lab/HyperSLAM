/// This file is subject to the terms and conditions defined in
/// the file 'LICENSE.txt', which is part of this source code package.

#include <glog/logging.h>
#include <csignal>
#include <filesystem>

#include "hyper/system/system.hpp"
#include "hyper/yaml/yaml.hpp"

using namespace hyper;

using Systems = std::set<std::unique_ptr<System>>; ///< System storage container.

Systems systems; ///< Systems.

/// Starts all running systems.
static inline void Start() {
  for (const auto& system : systems) {
    system->start();
  }
}

/// Pauses all running systems.
static inline void Pause(int) {
  for (const auto& system : systems) {
    system->pause();
  }
}

/// Stops all running systems.
static inline void Shutdown(int) {
  for (const auto& system : systems) {
    system->shutdown();
  }
}

/// Main function that starts HyperSLAM pipeline.
/// \param argc Argument count.
/// \param argv Array of characters.
/// \return EXIT_SUCCESS or EXIT_FAILURE.
int main(int argc, char* argv[]) {
  // Initialize Google logging.
  google::InitGoogleLogging(argv[0]);

  // Set flags.
  FLAGS_log_dir = "../log/";
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_stderrthreshold = 2;

  // Read config file path.
  std::filesystem::path config_file_path;
  if (argc < 2) {
    LOG(FATAL) << "Usage: " << argv[0] << " <CONFIG_FILE_PATH>";
  } else {
    config_file_path = argv[1];
  }

  // Read YAML file.
  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(config_file_path);
  } catch (...) {
    LOG(FATAL) << "YAML configuration file not found.";
  }

  // Initialize ROS.
  const auto node_name = yaml::ReadAs<std::string>(yaml_node, "Node");
  ros::init(argc, argv, node_name);

  // Initialize HyperSLAM.
  const auto system_node = yaml::Read(yaml_node, "System");
  auto system = std::make_unique<System>(system_node);
  systems.insert(std::move(system));

  // Define pause action.
  struct sigaction sigusr1_action {};
  sigusr1_action.sa_handler = Pause;
  sigemptyset(&sigusr1_action.sa_mask);
  sigusr1_action.sa_flags = 0;
  sigaction(SIGUSR1, &sigusr1_action, nullptr);

  // Define interrupt action.
  struct sigaction sigint_action {};
  sigint_action.sa_handler = Shutdown;
  sigemptyset(&sigint_action.sa_mask);
  sigint_action.sa_flags = 0;
  sigaction(SIGINT, &sigint_action, nullptr);

  // Start systems.
  Start();
  return EXIT_SUCCESS;
}
