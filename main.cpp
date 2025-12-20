#include "wall_visualizer.hpp"
#include <iostream>
#include <string>

wall_visualizer::BondType ParseBondType(const std::string &bond_type_str) {
  std::string upperBondType = bond_type_str;

  std::transform(upperBondType.begin(), upperBondType.end(),
                 upperBondType.begin(),
                 [](unsigned char c) { return std::toupper(c); });

  if (upperBondType == "STRETCHER") {
    return wall_visualizer::BondType::STRETCHER;
  } else if (upperBondType == "ENGLISH_CROSS_BOND") {
    return wall_visualizer::BondType::ENGLISH_CROSS_BOND;
  } else if (upperBondType == "WILD_BOND") {
    return wall_visualizer::BondType::WILD_BOND;
  } else {
    throw std::invalid_argument(
        "Unknown bond type: " + bond_type_str +
        ". Valid types are: STRETCHER, ENGLISH_CROSS_BOND, WILD_BOND");
  }
}

int main(int argc, char *argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " --config <config_file_path> --bond_type <STRETCHER|...>"
              << std::endl;
    return 1;
  }

  std::string config_file;
  std::string bond_type_str;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_file = argv[++i];
    } else if (arg == "--bond_type" && i + 1 < argc) {
      bond_type_str = argv[++i];
    }
  }

  if (config_file.empty() || bond_type_str.empty()) {
    std::cerr << "Missing required arguments" << std::endl;
    return 1;
  }

  try {
    wall_visualizer::WallVisualizer visualizer(config_file);
    wall_visualizer::BondType bondType = ParseBondType(bond_type_str);
    visualizer.RunInteractiveBuild(bondType);

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}