//
// Created by davide on 10/25/25.
//

#ifndef TOOLS_LOCALIZATION_BASIC_FILTER_HPP
#define TOOLS_LOCALIZATION_BASIC_FILTER_HPP

#pragma once
#include <Eigen/Dense>

class FilterBase
{
public:
    virtual ~FilterBase() = default;

    virtual void predict(const double time, const Eigen::VectorXd &u) = 0;
    virtual void update(const double time, const Eigen::VectorXd &z) = 0;
    virtual Eigen::VectorXd get_state() = 0;
};

#endif //TOOLS_LOCALIZATION_BASIC_FILTER_HPP