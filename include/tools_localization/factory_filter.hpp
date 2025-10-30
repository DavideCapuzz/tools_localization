//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_FACTORY_FILTER_HPP
#define TOOLS_LOCALIZATION_FACTORY_FILTER_HPP

#include "UKF.hpp"
#include "estimator_interface.hpp"
#pragma once
#include <memory>
#include <yaml-cpp/yaml.h>

inline std::shared_ptr<Estimator> filter_factory(const std::string &type,
                                                 YAML::Node &config_node) {
  // parse config path to string
  const std::string config_file_path =
      config_node["config_file_path"].as<std::string>();

  if (type == "ukf") {
    config_node["config_file_path"] =
        "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";
    return std::make_shared<UKF>(config_file_path);
  } else {
    throw std::runtime_error("Unknown filter type: " + type);
  }
}

#endif // TOOLS_LOCALIZATION_FACTORY_FILTER_HPP
