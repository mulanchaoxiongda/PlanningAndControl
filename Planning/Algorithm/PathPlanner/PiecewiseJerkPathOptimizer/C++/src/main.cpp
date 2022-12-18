#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <eigen3/Eigen/Eigen>
#include <vector>

#include "save_data.h"
#include "piecewise_jerk_path_optimizer.h"

int main(int argc, char **argv) {
    double ego_rear2front = 3.70;
    double ego_rear2back = 1.00;
    double ego_width = 2.06;
    double ego_wheelbase = 2.8448;

    PiecewiseJerkPathOptimizer path_planner(
            ego_rear2front, ego_rear2back, ego_width, ego_wheelbase);

    double ref_speed = 5.0 / 3.6;
    bool path_reusable = false;
    double total_time = 30.0;
    double sample_time_interval = 0.5;
    int num_of_knots = std::round(total_time / sample_time_interval) + 1;
    double delta_s = 0.5 * ref_speed;
    std::vector<double> speed_data(num_of_knots, ref_speed);
    std::vector<double> l_lower_bound(num_of_knots, -10.0);
    for (int i = 0; i < l_lower_bound.size(); ++i) {
        if ((double)i * delta_s > 25.0 && (double)i * delta_s < 30.0) {
            l_lower_bound.at(i) = 2.0;
        }
    }
    std::vector<double> l_upper_bound(num_of_knots, 10.0);
    std::vector<double> steer_bound(num_of_knots, 25.0 / 57.3);
    std::vector<double> steer_rate_bound(num_of_knots - 1, 25.0 / 57.3);
    std::vector<double> ref_line_kappa(num_of_knots, 0.0);
    std::vector<double> ref_line_dkappa(num_of_knots, 0.0);
    std::vector<double> init_state = {0.2, 0.0, 0.0};
    std::vector<double> end_state = {0.5, 0,0, 0.0};
    bool has_end_state_constraint = true;
    double path_point_time_interval = 0.1;
    double path_point_interval = ref_speed * path_point_time_interval;
    int num_of_match_points =
            std::round(delta_s / path_point_interval) * (num_of_knots - 1)+ 1;
    std::vector<MatchPoint> match_points(num_of_match_points);
    double turning_radius = 1000000.0;
    double center_corner;
    for (int i = 0; i < num_of_match_points; ++i) {
        center_corner =
                (double)(i) * ref_speed *
                path_point_time_interval / turning_radius;
        match_points.at(i).x = cos(center_corner) * turning_radius;
        match_points.at(i).y = sin(center_corner) * turning_radius;
        match_points.at(i).theta = center_corner + M_PI / 2.0;
        match_points.at(i).kappa = 1.0 / turning_radius;
    }

    std::vector<SLPathPoint> sl_path_points;

    struct timeval t_start, t_end;
    gettimeofday(&t_start, NULL);

    int loop_times = 1;

    for (int i = 0; i < loop_times; ++i) {
        path_planner.Process(
                path_reusable, num_of_knots, delta_s, speed_data,
                l_lower_bound, l_upper_bound, steer_bound, steer_rate_bound,
                ref_line_kappa, ref_line_dkappa, init_state, end_state,
                has_end_state_constraint, path_point_interval,
                match_points, sl_path_points);
    }

    gettimeofday(&t_end, NULL);
    
    std::cout << "Program running time:"
              << ((double)(t_end.tv_sec - t_start.tv_sec) * 1000.0 +
                 (double)(t_end.tv_usec - t_start.tv_usec) / 1000.0) /
                 (double)loop_times
              << "ms." << std::endl;

              std::cout << sl_path_points.size() << std::endl;

    SaveData save_result("../tools/simulation_result.txt");
    for (auto mem : sl_path_points) {
        save_result.file << "[local_path] "
                         << " s " << mem.s
                         <<" l " << mem.l
                         <<" kappa " << mem.kappa
                         << std::endl;
    }

    return 0;
}
