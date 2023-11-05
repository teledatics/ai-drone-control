/*
 * Copyright (c) 2021 Rico Studio
 * All rights reserved.
 * License: BSD-0
 * author: Rico Jia
 */
#ifndef __FILTERS_UTIL_HPP__
#define __FILTERS_UTIL_HPP__

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

namespace Filter {
namespace Util {
// generate a vector of uniformly distributed random numbers. Each number falls in the range defined by elements with
// the same indices in upper_lims and lower lims.
std::vector<double> generate_random_num_universal(const std::vector<double>& upper_lims,
                                                  const std::vector<double>& lower_lims) {
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    if (upper_lims.size() != lower_lims.size())
        return {};
    unsigned int sz = upper_lims.size();
    std::vector<double> ret_vec;
    ret_vec.reserve(sz);
    for (unsigned int i = 0; i < upper_lims.size(); ++i) {
        std::uniform_real_distribution<> distribution(lower_lims.at(i), upper_lims.at(i));
        ret_vec.emplace_back(distribution(gen));
    }
    return ret_vec;
}
// generate a random number in [lower_lim, upper_lim] following universal distribution
std::vector<double> generate_random_num_universal(const double& lower_lim, const double& upper_lim,
                                                  const unsigned int& num) {
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> distribution(lower_lim, upper_lim);
    std::vector<double> ret_vec;
    ret_vec.reserve(num);
    for (auto i = 0; i < num; ++i)
        ret_vec.emplace_back(distribution(gen));
    return ret_vec;
}

std::vector<double> generate_random_num_gaussian(const double& mean, const double& std_dev, const unsigned int& num) {
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(mean, std_dev);
    std::vector<double> ret;
    ret.reserve(num);
    for (unsigned int i = 0; i < num; ++i) {
        ret.emplace_back(distribution(generator));
    }
    return ret;
}

bool is_equal(double a, double b) {
    return std::abs(a - b) < std::pow(10, -6);
}
} // namespace Util
} // namespace Filter

#endif /* end of include guard: __FILTERS_UTIL_HPP__ */
