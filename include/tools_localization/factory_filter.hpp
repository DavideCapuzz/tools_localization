//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_FACTORY_FILTER_HPP
#define TOOLS_LOCALIZATION_FACTORY_FILTER_HPP

#pragma once
#include <memory>
#include "UKF_wrapper.hpp"
#include "EKF_basic_wrapper.hpp"

inline std::shared_ptr<FilterBase> filter_factory(const std::string &type, YAML::Node& config_node)
{
    if (type == "ukf") {
        config_node["config_file_path"] = "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";
        return std::make_shared<UKFwrapper>(config_node);
    } else if (type == "ekf_basic") {
        config_node["config_file_path"] = "/home/davide/ros_ws/wheele/src/tools_localization/config/config.json";
        return std::make_shared<EKF>(config_node);
    } else {
        throw std::runtime_error("Unknown filter type: " + type);
    }
}

#endif //TOOLS_LOCALIZATION_FACTORY_FILTER_HPP
