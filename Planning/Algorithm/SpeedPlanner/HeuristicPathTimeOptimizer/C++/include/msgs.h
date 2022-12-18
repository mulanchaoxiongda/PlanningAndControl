#pragma once

#include <vector>

typedef enum class ObstacleType {
    actual,
    fictitious
} ObstacleType;

struct ObstacleInfo {
    ObstacleType obs_type = ObstacleType::actual;

    std::vector<double> time_sampled;
    std::vector<double> lower_bound;
    std::vector<double> upper_bound;
};

struct STGraph {
    double total_s;
    double total_time;
    double time_interval;
    double time_dividing;

    std::vector<double> time_sampled;
    std::vector<double> s_dense_sampled;
    std::vector<double> s_sparse_sampled;
    std::vector<double> kappa_dense_sampled;
    std::vector<double> kappa_sparse_sampled;
    std::vector<double> speed_bound_dense_sampled;
    std::vector<double> speed_bound_sparse_sampled; // treat as hard constraint
};

struct SpeedPara {
    double cruise_speed;
    double speed_limit;
    double keep_clean_min_speed;

    bool cruise_state_flag;
    bool keep_clean_flag;
};
