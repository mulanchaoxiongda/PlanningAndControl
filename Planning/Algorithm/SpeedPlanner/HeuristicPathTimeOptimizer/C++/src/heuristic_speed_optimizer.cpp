#include <cmath>
#include <cfloat>

#include "heuristic_speed_optimizer.h"

OptimizeResult HeuristicSpeedOptimizer::Process (
        const std::vector<double>& ego_init_state, // speed & acc
        const std::vector<ObstacleInfo>& obstacle_info,
        const STGraph& st_graph,
        const SpeedPara& speed_para,
        std::vector<STPathPoint>& st_path_points) {
    st_path_points.clear();

    ego_init_state_ = ego_init_state;
    obs_info_ = obstacle_info;
    st_graph_ = st_graph;
    speed_para_ = speed_para;

    nodes_searched_.clear();
    while (!open_list_.empty()) {
        open_list_.pop();
    }
    closed_list_.clear();

    OptimizerParaCfg();

    std::shared_ptr<NodeInfo> ptr_start_node = std::make_shared<NodeInfo>();
    CalculateStartNode(ptr_start_node);

    NodeIdxCost start_node_idx_cost;
    start_node_idx_cost.first = ptr_start_node->idx_st;
    start_node_idx_cost.second = ptr_start_node->f_cost;

    nodes_searched_[ptr_start_node->idx_st] = ptr_start_node;
    open_list_.push(start_node_idx_cost);

    std::string idx_patent_node;

    while (!open_list_.empty()) {
        idx_patent_node = open_list_.top().first;
        if (closed_list_.find(idx_patent_node) != closed_list_.end()) {
            open_list_.pop();
            continue;
        }

        if (nodes_searched_[idx_patent_node]->s >= st_graph_.total_s ||
            nodes_searched_[idx_patent_node]->t >= st_graph_.total_time) {
            break;
        }

        closed_list_.insert(idx_patent_node);
        open_list_.pop();

        std::vector<NodeInfo> child_nodes;
        CalculateChildNodes(nodes_searched_[idx_patent_node], child_nodes);

        for (auto child_node : child_nodes) {
            if(closed_list_.find(child_node.idx_st) != closed_list_.end()){
                continue;
            }

            CalculateTotalCost(nodes_searched_[idx_patent_node], child_node);

            if (nodes_searched_.find(child_node.idx_st) !=
                nodes_searched_.end()) {
                if (child_node.f_cost <
                    nodes_searched_[child_node.idx_st]->f_cost) {
                    nodes_searched_[child_node.idx_st] =
                            std::make_shared<NodeInfo>(child_node);
                    open_list_.push({child_node.idx_st, child_node.f_cost});
                }
            } else {
                    nodes_searched_[child_node.idx_st] =
                            std::make_shared<NodeInfo>(child_node);
                    open_list_.push({child_node.idx_st, child_node.f_cost});
            }
        }
    }

    if (open_list_.empty() ||
        nodes_searched_[idx_patent_node]->f_cost == DBL_MAX) {
        std::cout << "[ERROR] there is no path solution" << std::endl;

        return OptimizeResult::fail;
    }

    CalculateOptimalSTPath(idx_patent_node);

    st_path_points = opt_st_path_;

    return OptimizeResult::success;
}

bool HeuristicSpeedOptimizer::OptimizerParaCfg() {
    planning_para_.w_obstacle = 1000.0;
    planning_para_.s_safe_follow = 20.0;
    planning_para_.s_safe_overtake = 20.0;
    planning_para_.default_safe_dis_penalty = 0.0;
    planning_para_.max_lon_control_err = 2.5 + 1.0;

    planning_para_.w_exceed_speed = 2000.0;
    planning_para_.w_deceed_speed = 100.0;
    planning_para_.w_cruise_speed = 3000.0;
    planning_para_.w_keep_clean_speed = 2.0;
    planning_para_.keep_clean_low_speed_penalty = 50.0;
    planning_para_.cruise_speed_err_ratio= 0.1;
    planning_para_.keep_clean_deceed_ratio = 0.2;
    planning_para_.speed_overrun_ratio = 0.1;

    planning_para_.w_lon_acc = 100.0;
    planning_para_.w_lon_deacc = 100.0;
    planning_para_.w_lat_acc = 1.0;
    planning_para_.safe_lat_acc_coefficient = 0.6;
    planning_para_.lat_acc_max = 2.5;
    planning_para_.acc_max = 4.0;
    planning_para_.acc_min = -6.0;

    planning_para_.w_positive_jerk = 1.0;
    planning_para_.w_negative_jerk = 1.0;
    planning_para_.jerk_max = 5.0;
    planning_para_.jerk_min = -10.0;

    planning_para_.w_spatial = 150.0;
    
    planning_para_.st_graph_time_interval = st_graph_.time_interval;
    planning_para_.opt_path_time_interval = 0.1;

    return true;
}

bool HeuristicSpeedOptimizer::CalculateStartNode(const std::shared_ptr<NodeInfo> ptr_start_node) {
    ptr_start_node->idx_t = 0;
    ptr_start_node->idx_s = st_graph_.s_dense_sampled.size() - 1;
    ptr_start_node->idx_st =
            std::to_string(ptr_start_node->idx_t) + "_" +
            std::to_string(ptr_start_node->idx_s);

    ptr_start_node->g_cost = 0.0;
    ptr_start_node->f_cost =
            planning_para_.w_spatial * st_graph_.total_s /
            ego_init_state_.at(0) / st_graph_.time_interval;
    ptr_start_node->h_cost = 0.0;
    ptr_start_node->obs_cost = 0.0;
    ptr_start_node->edge_cost = 0.0;

    ptr_start_node->t = 0.0;
    ptr_start_node->s = 0.0;
    ptr_start_node->speed = ego_init_state_.at(0);
    ptr_start_node->acc = ego_init_state_.at(1);
    ptr_start_node->kappa = st_graph_.kappa_dense_sampled.back();

    ptr_start_node->idx_t_parent = -1;
    ptr_start_node->idx_s_parent = -1;
    ptr_start_node->idx_st_parent =
            std::to_string(ptr_start_node->idx_t_parent) + "_" +
            std::to_string(ptr_start_node->idx_s_parent);
    
    return true;
}

bool HeuristicSpeedOptimizer::CalculateChildNodes(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        std::vector<NodeInfo>& child_nodes) {
    NodeInfo child_node;

    double time_margin = 0.000001;
    double dt = st_graph_.time_interval + time_margin;
    double dt_square = std::pow(dt, 2.0);
    double dt_cube = std::pow(dt, 3.0);

    double s_min =
            ptr_parent_node->s + ptr_parent_node->speed * dt +
            1.0 / 2.0 * ptr_parent_node->acc * dt_square +
            1.0 / 6.0 * planning_para_.jerk_min * dt_cube;
    double s_max =
            ptr_parent_node->s + ptr_parent_node->speed * dt +
            1.0 / 2.0 * ptr_parent_node->acc * dt_square +
            1.0 / 6.0 * planning_para_.jerk_max * dt_cube;

    int idx_t_dividing = std::round(st_graph_.time_dividing / st_graph_.time_interval);
    int idx_t_child_node = ptr_parent_node->idx_t + 1;

    if (idx_t_child_node <= idx_t_dividing) {
        for (int i = ptr_parent_node->idx_s; i >= 0; --i) {
            if (st_graph_.s_dense_sampled.at(i) <= s_max &&
                st_graph_.s_dense_sampled.at(i) >= s_min) {
                double s0 = ptr_parent_node->s;
                double v0 = ptr_parent_node->speed;
                double a0 = ptr_parent_node->acc;
                double s1 = st_graph_.s_dense_sampled.at(i);

                double jerk =
                        (s1 - s0 - v0 * dt - 0.5 * a0 * dt_square) * 6.0 / dt_cube;
                
                double a1 = a0 + jerk * dt;
                if (a1 > planning_para_.acc_max || a1 < planning_para_.acc_min) {
                    continue;
                }

                double speed_limit =
                        speed_para_.speed_limit *
                        (1 + planning_para_.speed_overrun_ratio); // st_graph_.GetSpeedLimitByS(s_i)
                double v1 = v0 + a0 * dt + 0.5 * jerk * dt_square;
                if (v1 > speed_limit || v1 < 0.0) {
                    continue;
                }

                double centrifugal_acc = std::pow(v1, 2.0) * st_graph_.kappa_dense_sampled.at(i);
                if (std::fabs(centrifugal_acc) > planning_para_.lat_acc_max) {
                    continue;
                }

                child_node.t = ptr_parent_node->t + st_graph_.time_interval;
                child_node.s = s1;
                child_node.speed = v1;
                child_node.acc = a1;
                child_node.kappa = st_graph_.kappa_dense_sampled.at(i);

                child_node.f_cost = DBL_MAX;
                child_node.g_cost = DBL_MAX;
                child_node.h_cost = DBL_MAX;
                child_node.edge_cost = DBL_MAX;
                child_node.obs_cost = DBL_MAX;

                child_node.idx_t = ptr_parent_node->idx_t + 1;
                child_node.idx_s = i;
                child_node.idx_t_parent = ptr_parent_node->idx_t;
                child_node.idx_s_parent = ptr_parent_node->idx_s;
                child_node.idx_st =
                        std::to_string(child_node.idx_t) + "_" +
                        std::to_string(child_node.idx_s);
                child_node.idx_st_parent =
                        std::to_string(child_node.idx_t_parent) + "_" +
                        std::to_string(child_node.idx_s_parent);
                
                child_nodes.push_back(child_node);
            } else if (st_graph_.s_dense_sampled.at(i) > s_max) {
                break;
            }
        }
    } else {
        for (int i = st_graph_.s_sparse_sampled.size() - 1; i >= 0; --i) {
            if (st_graph_.s_sparse_sampled.at(i) <= s_max &&
                st_graph_.s_sparse_sampled.at(i) >= s_min) {
                double s0 = ptr_parent_node->s;
                double v0 = ptr_parent_node->speed;
                double a0 = ptr_parent_node->acc;
                double s1 = st_graph_.s_sparse_sampled.at(i);

                double jerk =
                        (s1 - s0 - v0 * dt - 0.5 * a0 * dt_square) * 6.0 / dt_cube;

                double a1 = a0 + jerk * dt;
                if (a1 > planning_para_.acc_max || a1 < planning_para_.acc_min) {
                    continue;
                }

                double speed_limit =
                        speed_para_.speed_limit *
                        (1.0 + planning_para_.speed_overrun_ratio);
                double v1 = v0 + a0 * dt + 0.5 * jerk * dt_square;
                if (v1 > speed_limit || v1 < 0.0) {
                    continue;
                }

                double centrifugal_acc = std::pow(v1, 2.0) * st_graph_.kappa_sparse_sampled.at(i);
                if (std::fabs(centrifugal_acc) > planning_para_.lat_acc_max) {
                    continue;
                }

                child_node.t = ptr_parent_node->t + st_graph_.time_interval;
                child_node.s = s1;
                child_node.speed = v1;
                child_node.acc = a1;
                child_node.kappa = st_graph_.kappa_sparse_sampled.at(i);

                child_node.f_cost = DBL_MAX;
                child_node.g_cost = DBL_MAX;
                child_node.h_cost = DBL_MAX;
                child_node.edge_cost = DBL_MAX;
                child_node.obs_cost = DBL_MAX;

                child_node.idx_t = ptr_parent_node->idx_t + 1;
                child_node.idx_s = i;
                child_node.idx_t_parent = ptr_parent_node->idx_t;
                child_node.idx_s_parent = ptr_parent_node->idx_s;
                child_node.idx_st =
                        std::to_string(child_node.idx_t) + "_" +
                        std::to_string(child_node.idx_s);
                child_node.idx_st_parent =
                        std::to_string(child_node.idx_t_parent) + "_" +
                        std::to_string(child_node.idx_s_parent);

                child_nodes.push_back(child_node);
            } else if (st_graph_.s_sparse_sampled.at(i) > s_max) {
                break;
            }
        }
    }

    return true;
}

bool HeuristicSpeedOptimizer::CalculateTotalCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        NodeInfo& child_node) {
    if (!CalculateGCost(ptr_parent_node, child_node)) {
        child_node.f_cost = DBL_MAX;
        return false;
    }

    child_node.h_cost =
            planning_para_.w_spatial * (st_graph_.total_s - child_node.s) /
            child_node.speed / st_graph_.time_interval;

    child_node.f_cost = child_node.g_cost + child_node.h_cost;

    return true;
}

bool HeuristicSpeedOptimizer::CalculateGCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        NodeInfo& child_node) {
    if(!CalculateEdgeCost(ptr_parent_node, child_node)) {
        child_node.g_cost = DBL_MAX;
        return false;
    }

    if (!CalculateObstacleCost(ptr_parent_node,child_node)) {
        child_node.g_cost = DBL_MAX;
        return false;
    }

    child_node.g_cost = ptr_parent_node->g_cost + child_node.edge_cost + child_node.obs_cost;

    return true;
}

bool HeuristicSpeedOptimizer::CalculateEdgeCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        NodeInfo& child_node) {
    double speed_cost, speed;
    if (!CalculateSpeedCost(ptr_parent_node, child_node,speed_cost,speed)) {
        child_node.edge_cost = DBL_MAX;
        return false;
    }

    double acc_cost;
    if (!CalculateAccCost(ptr_parent_node, child_node, speed, acc_cost)) {
        child_node.edge_cost = DBL_MAX;
        return false;
    }

    double jerk_cost;
    if(!CalculateJerkCost(ptr_parent_node, child_node, jerk_cost)) {
        child_node.edge_cost = DBL_MAX;
        return false;
    }

    child_node.edge_cost = (speed_cost + acc_cost + jerk_cost) * planning_para_.st_graph_time_interval;
    
    return true;
}

bool HeuristicSpeedOptimizer::CalculateSpeedCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        const NodeInfo& child_node,
        double& speed_cost, double& max_speed) {
    double speed_limit =
        speed_para_.speed_limit *
        (1 + planning_para_.speed_overrun_ratio);

    double speed_max = std::max(child_node.speed, ptr_parent_node->speed);
    double speed_min = std::min(child_node.speed, ptr_parent_node->speed);

    double v0 = ptr_parent_node->speed;
    double a0 = ptr_parent_node->acc;
    double jerk = (child_node.acc - ptr_parent_node->acc) / planning_para_.st_graph_time_interval;

    double step_max = 0.3;

    double step=
            planning_para_.st_graph_time_interval /
            std::ceil(planning_para_.st_graph_time_interval / step_max);

    double t = step;

    while (t < planning_para_.st_graph_time_interval) {
        double v = v0 + a0 * t + 0.5 * jerk * std::pow(t, 2.0);
       
        if (v > speed_limit || v < 0.0) {
            speed_cost = DBL_MAX;
            max_speed = DBL_MAX;

            return false;
        }

        speed_max = std::max(v, speed_max);
        speed_min = std::min(v, speed_min);

        t = t + step;
    }

    double exceed_speed_cost = 0;
    double deceed_speed_cost = 0;

    speed_limit = speed_para_.speed_limit;
    
    double diff_speed = speed_max - speed_limit;
    if (diff_speed > 0.0) {
        double scale_factor =
                1.0 / (speed_limit * planning_para_.speed_overrun_ratio);
        diff_speed = diff_speed * scale_factor;
        
        exceed_speed_cost =
                planning_para_.w_exceed_speed * std::pow(diff_speed, 2.0);
    }

    diff_speed = speed_min - speed_limit;
    if (diff_speed < 0.0 && !speed_para_.cruise_state_flag) {
        double deceed_speed_ratio = 0.2;
     
        double scale_factor = 1.0 / (speed_limit * deceed_speed_ratio);

        diff_speed = diff_speed * scale_factor;

        deceed_speed_cost =
                planning_para_.w_deceed_speed * std::pow(diff_speed, 2.0);
    }

    double cruise_cost = 0.0;

    if (speed_para_.cruise_state_flag) {
        diff_speed =
                std::max(std::fabs(speed_max - speed_para_.cruise_speed),
                std::fabs(speed_para_.cruise_speed - speed_min));

        double scale_factor =
                1.0 / (speed_para_.cruise_speed *
                planning_para_.cruise_speed_err_ratio);

        diff_speed = diff_speed * scale_factor;

        cruise_cost =
                planning_para_.w_cruise_speed * std::pow(diff_speed, 2.0);
                
        if (speed_para_.cruise_speed > speed_para_.speed_limit) {
            std::cout << "[warning]cruise._speed·is·bigger·than·speed_limit"
                      << std::endl;
        }
    }

    double keep_clean_cost = 0;

    if (speed_para_.keep_clean_flag &&
        speed_min < speed_para_.keep_clean_min_speed) {
        diff_speed = -speed_min + speed_para_.keep_clean_min_speed;

        double scale_factor =
                1.0 / (speed_para_.keep_clean_min_speed *
                planning_para_.keep_clean_deceed_ratio);

        diff_speed = diff_speed * scale_factor;

        keep_clean_cost =
                planning_para_.w_keep_clean_speed * std::pow(diff_speed, 2.0) +
                planning_para_.keep_clean_low_speed_penalty;
    }

    speed_cost =
            exceed_speed_cost + deceed_speed_cost +
            cruise_cost + keep_clean_cost;
    max_speed = speed_max;

    return true;
}

bool HeuristicSpeedOptimizer::CalculateAccCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        const NodeInfo& child_node,
        const double& speed, double& acc_cost) {
    double lon_acc_cost;
    if (!CalculateLongitudeAccCost(ptr_parent_node, child_node, lon_acc_cost)) {
        acc_cost = DBL_MAX;
        return false;
    }

    double lat_acc_cost;
    if (!CalculateLatitudeAccCost(ptr_parent_node, child_node, speed, lat_acc_cost)) {
        acc_cost = DBL_MAX;
        return false;
    }

    acc_cost = lon_acc_cost + lat_acc_cost;

    return true;
}

bool HeuristicSpeedOptimizer::CalculateLongitudeAccCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        const NodeInfo& child_node,
        double& lon_acc_cost) {
    double lon_acc_min = std::min(child_node.acc, ptr_parent_node->acc);
    double lon_acc_max = std::max(child_node.acc, ptr_parent_node->acc);
    
    if (lon_acc_max < 0.0) {
        double scale_factor = 1.0 / planning_para_.acc_min;
        lon_acc_cost =
                planning_para_.w_lon_deacc *
                (std::pow(lon_acc_max * scale_factor, 2.0) +
                std::pow(lon_acc_min * scale_factor, 2.0)) / 2.0;
    } else if (lon_acc_min > 0.0) {
        double scale_factor = 1.0 / planning_para_.acc_max;
        lon_acc_cost =
                planning_para_.w_lon_acc *
                (std::pow(lon_acc_max * scale_factor, 2.0) +
                std::pow(lon_acc_min * scale_factor, 2.0)) / 2.0;
    } else {
        double scale_factor = 1.0 / planning_para_.acc_min;
        double deacc_cost =
                planning_para_.w_lon_deacc *
                std::pow(lon_acc_min * scale_factor, 2.0);

        scale_factor = 1.0 / planning_para_.acc_max;
        double acc_cost =
                planning_para_.w_lon_acc *
                std::pow(lon_acc_max * scale_factor, 2.0);

        double acc_time_proportion = lon_acc_max / (lon_acc_max - lon_acc_min);
        lon_acc_cost =
                (deacc_cost * (1.0 - acc_time_proportion) +
                acc_cost * acc_time_proportion) / 2.0;
    }

    return true;
}

bool HeuristicSpeedOptimizer::CalculateLatitudeAccCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        const NodeInfo& child_node,
        const double max_speed, double& lat_acc_cost) {
    double kappa =
            std::max(std::fabs(child_node.kappa), std::fabs(ptr_parent_node->kappa));
            
    double lat_acc = std::pow(max_speed, 2.0) * kappa;

    double safe_lat_acc =
            planning_para_.lat_acc_max *
            planning_para_.safe_lat_acc_coefficient;
            
    if (lat_acc > planning_para_.lat_acc_max) {
        lat_acc_cost = DBL_MAX;
        return false;
    } else if (std::fabs(lat_acc) < safe_lat_acc) {
        lat_acc_cost = 0.0;
        return true;
    } else {
        double scale_factor = 1.0 / planning_para_.lat_acc_max;
        lat_acc = lat_acc * scale_factor;

        lat_acc_cost = planning_para_.w_lat_acc * std::pow(lat_acc, 2.0); // cost function is uncontinuous
    }

    return true;
}

bool HeuristicSpeedOptimizer::CalculateJerkCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        const NodeInfo& child_node, double& jerk_cost) {
    double jerk = (child_node.acc - ptr_parent_node->acc) / st_graph_.time_interval;
    
    if (jerk > 0.0) {
        double scale_factor = 1.0 / planning_para_.jerk_max;
        jerk = jerk * scale_factor;

        jerk_cost =
                planning_para_.w_positive_jerk * std::pow(jerk, 2.0);
    } else {
        double scale_factor = 1.0 / planning_para_.jerk_min;
        jerk = jerk * scale_factor;

        jerk_cost =
                planning_para_.w_negative_jerk * std::pow(jerk, 2.0);
    }

    return true;
}

bool HeuristicSpeedOptimizer::CalculateObstacleCost(
        const std::shared_ptr<NodeInfo> ptr_parent_node,
        NodeInfo& child_node) {
    double obs_cost = 0.0;

    if (obs_info_.empty()) {
        child_node.obs_cost = 0.0;
        return true;
    }

    for (int i = 0; i < (int)obs_info_.size(); ++i) {
        if (obs_info_.at(i).obs_type != ObstacleType::actual) {
            continue;
        }

        double s0 = ptr_parent_node->s;
        double v0 = ptr_parent_node->speed;
        double a0 = ptr_parent_node->acc;
        double jerk =
                (child_node.acc - ptr_parent_node->acc) /
                planning_para_.st_graph_time_interval;

        double step = planning_para_.st_graph_time_interval / 5.0;
        
        double t = step;
        double time_margin = 0.000001;

        while (t <= planning_para_.st_graph_time_interval + time_margin) {
            double ego_t = t + ptr_parent_node->t;
            
            if(ego_t < obs_info_.at(i).time_sampled.front() - time_margin ||
            ego_t > obs_info_.at(i).time_sampled.back() + time_margin) {
                t = t + step;
                continue;
            }

            double s = s0 + v0 * t + 0.5 * a0 * std::pow(t, 2.0) + jerk * std::pow(t, 3.0) / 6.0;
            double ego_s = s;

            double obs_lb =
                    -planning_para_.max_lon_control_err +
                    LinearInterpolation(obs_info_.at(i).time_sampled,
                                        obs_info_.at(i).lower_bound, ego_t);
            double obs_ub =
                    planning_para_.max_lon_control_err +
                    LinearInterpolation(obs_info_.at(i).time_sampled,
                                        obs_info_.at(i).upper_bound, ego_t);

            double s_safe_follow = planning_para_.s_safe_follow;
            double s_safe_overtake = planning_para_.s_safe_overtake;

            t = t + step;

            if (ego_s >= obs_lb && ego_s <= obs_ub) {
                obs_cost = DBL_MAX;
                return false;
            } else if (ego_s <= obs_ub + s_safe_overtake && ego_s > obs_ub) {
                double dis_ego2obs = ego_s - (obs_ub + s_safe_overtake);

                double scale_factor = 1.0 / s_safe_overtake;
                dis_ego2obs = dis_ego2obs * scale_factor;

                obs_cost =
                        obs_cost +
                        planning_para_.w_obstacle * std::pow(dis_ego2obs, 2.0) +
                        planning_para_.default_safe_dis_penalty * step;
            } else if (ego_s <= obs_lb && ego_s > obs_lb - s_safe_follow) {
                double dis_ego2obs = ego_s - (obs_lb - s_safe_follow);

                double scale_factor = 1.0 / s_safe_follow;
                dis_ego2obs = dis_ego2obs * scale_factor;

                obs_cost =
                        obs_cost +
                        planning_para_.w_obstacle * std::pow(dis_ego2obs, 2.0) +
                        planning_para_.default_safe_dis_penalty * step;
            }
        }
    }
            
    child_node.obs_cost = obs_cost;

    return true;
}

double HeuristicSpeedOptimizer::LinearInterpolation(
        const std::vector<double>& x, const std::vector<double>&y,
        double x0) {
    double y0;

    if (x0 > x.back()) {
        x0 = x.back() - 0.000000001;
    } else if (x0 < x.front()) {
        x0 = x.front() + 0.000000001;
    }

    for (int i = 1; i < (int)x.size(); ++i) {
        if (x0 <= x.at(i)) {
            y0 =
                    (y.at(i) - y.at(i - 1)) / (x.at(i) - x.at(i - 1)) *
                    (x0 - x.at(i - 1)) + y.at(i - 1);

            return y0;
        }
    }

    return DBL_MAX;
}

bool HeuristicSpeedOptimizer::CalculateOptimalSTPath(const std::string& idx_end_node) {
    std::vector<STPathPoint> opt_st_path_temp;

    NodeInfoPtr ptr_parent_node = nodes_searched_[idx_end_node];

    while (ptr_parent_node->idx_st_parent != "-1_-1") {
        opt_st_path_temp.push_back(
                {ptr_parent_node->t, ptr_parent_node->s,
                 ptr_parent_node->speed, ptr_parent_node->acc});

        ptr_parent_node =
                nodes_searched_[ptr_parent_node->idx_st_parent];
    }

    opt_st_path_temp.push_back(
            {ptr_parent_node->t, ptr_parent_node->s,
             ptr_parent_node->speed, ptr_parent_node->acc});

    int idx_left = 0;
    int idx_right = opt_st_path_temp.size() - 1;
    
    while (idx_left < idx_right) {
        STPathPoint temp = opt_st_path_temp.at(idx_right);

        opt_st_path_temp.at(idx_right) = opt_st_path_temp.at(idx_left);
        opt_st_path_temp.at(idx_left) = temp;
        
        idx_left = idx_left + 1;
        idx_right = idx_right - 1;
    }

    opt_st_path_.clear();

    for (int i = 0; i < (int)opt_st_path_temp.size() - 1; ++i) {
        double jerk =
                (opt_st_path_temp.at(i + 1).dds - opt_st_path_temp.at(i).dds) /
                st_graph_.time_interval;
        double t0 = opt_st_path_temp.at(i).t;
        double s0 = opt_st_path_temp.at(i).s;
        double v0 = opt_st_path_temp.at(i).ds;
        double a0 = opt_st_path_temp.at(i).dds;

        opt_st_path_.push_back({t0, s0, v0, a0});
        
        double step = planning_para_.opt_path_time_interval;
        int num_of_loop = std::round(st_graph_.time_interval / step) - 1;
        
        double dt = 0.0, dt_square, dt_cube;

        double t, v, a, s;

        for (int i = 1; i <= num_of_loop; ++i) {
            dt = dt + step;
            dt_square = std::pow(dt, 2.0);
            dt_cube = std::pow(dt, 3.0);
            
            t = t0 + dt;
            a = a0 + jerk * dt;
            v = v0 + a0 * dt + 1.0 / 2.0 * jerk * dt_square;
            s =
                    s0 + v0 * dt + 1.0 / 2.0 * a0 * dt_square +
                    1.0 / 6.0 * jerk * dt_cube;

            opt_st_path_.push_back({t, s, v, a});
        }
    }
    
    opt_st_path_.push_back(opt_st_path_temp.back());

    return true;
}
