//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_FACTORY_FILTER_HPP
#define TOOLS_LOCALIZATION_FACTORY_FILTER_HPP

#pragma once
#include <memory>
#include "UKF_wrapper.hpp"

inline std::shared_ptr<FilterBase> filter_factory(const std::string &type)
{
    if (type == "ukf") {
        return std::make_shared<UKFwrapper>();
    } else {
        throw std::runtime_error("Unknown filter type: " + type);
    }
}

#endif //TOOLS_LOCALIZATION_FACTORY_FILTER_HPP