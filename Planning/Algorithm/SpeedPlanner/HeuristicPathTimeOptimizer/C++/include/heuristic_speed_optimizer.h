#pragma once

#include <vector>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <string>

#include "save_data.h"
#include "msgs.h"

struct STPathPoint {
    double t;

    double s;
    double ds;
    double dds;
};

struct NodeInfo {
    int idx_t;
    int idx_s;
    std::string idx_st;

    double t;
    double s;
    double speed;
    double acc;
    double kappa;

    int idx_t_parent;
    int idx_s_parent;
    std::string idx_st_parent;
    
    double f_cost;
    double g_cost;

    double h_cost;
    double obs_cost;
    double edge_cost;
};

struct PlanningPara {
    double w_edge = 1.0;

    double w_exceed_speed;
    double w_deceed_speed;
    double w_cruise_speed;
    double w_keep_clean_speed;
    double keep_clean_low_speed_penalty;
    double cruise_speed_err_ratio;
    double keep_clean_deceed_ratio;

    double w_lon_acc;
    double w_lon_deacc;
    double w_lat_acc;
    double safe_lat_acc_coefficient;

    double w_positive_jerk;
    double w_negative_jerk;

    double w_obstacle;
    double s_safe_follow;
    double s_safe_overtake;
    double default_safe_dis_penalty;
    double max_lon_control_err;

    double w_spatial;  
    
    double speed_overrun_ratio;
    double lat_acc_max;
    double acc_max;
    double acc_min;
    double jerk_max;
    double jerk_min;

    double st_graph_time_interval;
    double opt_path_time_interval;
};

typedef enum class OptimizeResult {
    success,
    fail
} OptimizeResult;

class HeuristicSpeedOptimizer {
    public:
        HeuristicSpeedOptimizer() = default;
        virtual ~HeuristicSpeedOptimizer() = default;
        
        OptimizeResult Process(
                const std::vector<double>& go_init_state, // speed & acc
                const std::vector<ObstacleInfo>& obstacle_info,
                const STGraph& st_graph,
                const SpeedPara& speed_para,
                std::vector<STPathPoint>& st_path_points);

    private:
        bool OptimizerParaCfg();

        bool CalculateStartNode(const std::shared_ptr<NodeInfo> ptr_start_node);
        
        bool CalculateChildNodes(
                const std::shared_ptr<NodeInfo> parent_node,
                std::vector<NodeInfo>& child_nodes);

        bool CalculateTotalCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                NodeInfo& child_node);
        bool CalculateGCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                NodeInfo& child_node);
        bool CalculateEdgeCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                NodeInfo& child_node);
        bool CalculateSpeedCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                const NodeInfo& child_node,
                double& speed_cost, double& max_speed);
        bool CalculateAccCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                const NodeInfo& child_node,
                const double& speed_max, double& acc_cost);
        bool CalculateLongitudeAccCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                const NodeInfo& child_node,
                double& lon_acc_cost);
        bool CalculateLatitudeAccCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                const NodeInfo& child_node,
                const double max_speed, double& lat_acc_cost);
        bool CalculateJerkCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                const NodeInfo& child_node,
                double& jerk_cost);
        bool CalculateObstacleCost(
                const std::shared_ptr<NodeInfo> ptr_parent_node,
                NodeInfo&child_node);
        double LinearInterpolation(
                const std::vector<double>& x,const std::vector<double>& y,
                double x0);
        
        bool CalculateOptimalSTPath(const std::string& idx_end_node);
                
        std::vector<double> ego_init_state_;
        std::vector<STPathPoint> opt_st_path_;

        PlanningPara planning_para_;
        SpeedPara speed_para_;
        std::vector<ObstacleInfo> obs_info_;
        STGraph st_graph_;

        using NodeInfoPtr = std::shared_ptr<NodeInfo>;
        using NodeIdxCost = std::pair<std::string, double>;

        struct cmp{
            bool operator() (
                const NodeIdxCost& left, const NodeIdxCost& right) const {
                    return left.second >= right.second;
                }
        };

        std::unordered_map<std::string, NodeInfoPtr> nodes_searched_;
        std::priority_queue<NodeIdxCost, std::vector<NodeIdxCost>, cmp> open_list_;
        std::unordered_set<std::string> closed_list_;
};
