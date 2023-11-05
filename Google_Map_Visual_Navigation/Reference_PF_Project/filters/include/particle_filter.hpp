/*
 * Copyright (c) 2021 Rico Studio
 * All rights reserved.
 * License: BSD-0
 * author: Rico Jia
 */
#ifndef __PARTICLE_FILTER_HPP__
#define __PARTICLE_FILTER_HPP__

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <iostream>
#include <numeric>
#include <random>
#include <thread>
#include <utility>
#include <vector>

#include "filters_util.hpp"
#include "thread_pool.hpp"

namespace Filter {
class Particle_Filter {
  public:
    /**
     * @brief: Constructor for particle filter, initial state is specified
     */
    Particle_Filter(const std::vector<double>& initial_state, const std::vector<std::pair<double, double>>& ranges,
                    double valid_weight_lower_limit, const unsigned int particle_num);

    /**
     * @brief: Register function for send_relief callback, which updates the state with control input in place. See
     * Particle_Filter::update_control_cb_;
     * @param: Callable with signature void (std::vector<double>&).
     */
    void register_control_callback(std::function<void(std::vector<double>&)>);

    /**
     * @brief: Register function for calc_observation callback. See Particle_Filter::calc_observation_cb_
     * @param: Callable with signature double (std::vector<double>&).
     */
    void register_observation_callback(std::function<void(double&, const std::vector<double>&)> calc_observation_cb);

    /**
     * @brief: Main function that contains the main prediction-corrrection loop.
     * @notes: std::runtime_error will be thrown if callbacks are not attached.
     */
    std::vector<double> run();

  private:
    struct State {
        std::vector<double> state_vec_;
        double weight_;
    };
    std::vector<State> states_;
    std::vector<std::pair<double, double>> ranges_;
    const double valid_weight_lower_limit_;

    std::condition_variable observation_ready_;
    std::unique_ptr<ThreadPool> thread_pool_;

    std::function<void(std::vector<double>&)> update_control_cb_;

    // The callable takes in a single predicted state before an observation, and returns the likelihood of observation
    // given the state observation.
    std::function<void(double&, const std::vector<double>&)> calc_observation_cb_;

    // resampling using Russian Rollet
    void resampling();

    /**
     * @brief: Function user can call to reset all states to random, with uniform weight. This is good for quickly bring
     * the focus back once the target gets back into the scene
     */
    void reset_all_states_random();

    // return the average accross all particles over each dimension
    std::vector<double> average_belief();
};

inline Particle_Filter::Particle_Filter(const std::vector<double>& initial_state,
                                        const std::vector<std::pair<double, double>>& ranges,
                                        double valid_weight_lower_limit, const unsigned int particle_num)
    : states_(std::vector<State>(particle_num, State{initial_state, 1.0 / particle_num})),
      valid_weight_lower_limit_(valid_weight_lower_limit), ranges_(ranges) {
    // launch a thread pool for parallelism
    auto max_num_threads = std::thread::hardware_concurrency();
    auto num_threads = std::min((max_num_threads < 2) ? 2 : max_num_threads - 1, particle_num);
    thread_pool_ = std::make_unique<ThreadPool>(num_threads);
}

inline void Particle_Filter::register_control_callback(std::function<void(std::vector<double>&)> update_control_cb) {
    update_control_cb_ = update_control_cb;
}

inline void Particle_Filter::register_observation_callback(
    std::function<void(double&, const std::vector<double>&)> calc_observation_cb) {
    calc_observation_cb_ = calc_observation_cb;
}

inline void Particle_Filter::resampling() {
    // create the CDF of weight at each state.
    auto particle_num = states_.size();
    std::vector<double> weight_cdfs(particle_num, states_.at(0).weight_);
    std::transform(states_.begin() + 1, states_.end(), weight_cdfs.begin(), weight_cdfs.begin() + 1,
                   [](const State& current_state, const double& last_total_weight) {
                       return current_state.weight_ + last_total_weight;
                   });

    double r = Util::generate_random_num_universal(0, 1.0 / particle_num, 1).at(0);
    std::vector<State> new_states;
    new_states.reserve(particle_num);
    auto cdf_index = 0;
    for (int i = 0; i < particle_num; ++i) {
        double current_spoke = r + (i * 1.0) / particle_num;

        for (; cdf_index < particle_num && weight_cdfs.at(cdf_index) < current_spoke; ++cdf_index) {
        }
        new_states.emplace_back(State{states_.at(cdf_index).state_vec_,
                                      1.0 / particle_num}); // all new states have uniform weight, and the same
                                                            // states_.at(cdf_index) can be resampled multiple times.
    }
    states_ = std::move(new_states);
}

inline std::vector<double> Particle_Filter::average_belief() {
    auto num_dim = states_.at(0).state_vec_.size();
    std::vector<double> avg(num_dim);
    for (auto i = 0; i < num_dim; ++i) {
        avg.at(i) = std::accumulate(states_.begin(), states_.end(), 0.0, [&i](double sum, const State& s1) {
            return sum + s1.weight_ * s1.state_vec_.at(i);
        });
    }
    return avg;
}

inline std::vector<double> Particle_Filter::run() {
    if (update_control_cb_ == nullptr) {
        throw std::runtime_error("update_control_cb_ has not been attached. Particle Filter not running");
    }

    if (calc_observation_cb_ == nullptr) {
        throw std::runtime_error("calc_observation_cb_ has not been attached. Particle Filter not running");
    }

    resampling();

    // Parallelized control update and observation
    std::vector<std::future<void>> fut_vec;
    fut_vec.reserve(states_.size());
    for (auto& state : states_) {
        auto fut = thread_pool_->enqueue(update_control_cb_, std::ref<std::vector<double>>(state.state_vec_));
        fut_vec.emplace_back(std::move(fut));
    }
    for (auto& fut : fut_vec)
        fut.get();

    fut_vec.clear();

    for (auto& state : states_) {
        auto fut = thread_pool_->enqueue(calc_observation_cb_, std::ref<double>(state.weight_),
                                         std::ref<std::vector<double>>(state.state_vec_));
        fut_vec.emplace_back(std::move(fut));
    }

    for (auto& fut : fut_vec)
        fut.get();

    // normalize the states
    double sum = std::accumulate(states_.begin(), states_.end(), 0.0,
                                 [](double sum, const State& s) { return sum + s.weight_; });
    if (sum < valid_weight_lower_limit_) {
        reset_all_states_random();
    } else {
        std::for_each(states_.begin(), states_.end(), [sum](State& s) { s.weight_ /= sum; });
    }

    // send average belief
    return average_belief();
}

void Particle_Filter::reset_all_states_random() {
    std::vector<double> upper_lims, lower_lims;
    for (const auto& range : ranges_) {
        upper_lims.emplace_back(range.first);
        lower_lims.emplace_back(range.second);
    }

    for (auto& state : states_) {
        state.state_vec_ = Util::generate_random_num_universal(upper_lims, lower_lims);
        state.weight_ = 1.0 / states_.size();
    }
}

} // namespace Filter
#endif /* end of include guard: __PARTICLE_FILTER_HPP__ */
