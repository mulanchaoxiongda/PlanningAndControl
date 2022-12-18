#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <osqp/osqp.h>

#include "save_data.h"

struct MatchPoint {
    double x;
    double y;
    double theta;
    double kappa;
};

struct SLPathPoint {
    double s;
    double ds;
    double dds;

    double l;
    double dl;
    double ddl;

    double kappa;
};

struct XYPathPoint {
    double x;
    double y;
    double theta;
    double curvature;
};

typedef enum class OptimizeResult {
    success,
    fail,
    reuse
} OptimizeResult;

typedef enum class OptimizerStatus {
    init,
    success,
    fail,
    reuse
} OptimizerStatus;

class PiecewiseJerkPathOptimizer {
    public:
        PiecewiseJerkPathOptimizer() = delete;
        PiecewiseJerkPathOptimizer(
                const double& ego_rear2front, const double& ego_rear2back,
                const double& ego_width, const double& ego_wheelbase);
        virtual ~PiecewiseJerkPathOptimizer() = default;

        OptimizeResult Process(
                const bool path_reusable, const int num_of_knots,
                const double delta_s,
                const std::vector<double>&speed_data,
                const std::vector<double>& l_lower_bound,
                const std::vector<double>& l_upper_bound,
                const std::vector<double>& steer_bound,
                const std::vector<double>& steer_rate_bound,
                const std::vector<double>& ref_line_kappa,
                const std::vector<double>& ref_line_dkappa,
                const std::vector<double> init_state,
                const std::vector<double> end_state,
                bool has_end_state_constraint,
                const double path_point_interval,
                const std::vector<MatchPoint>&match_points,
                std::vector<SLPathPoint>&sl_path_points);

    private:
        void InputInfoPretreatment(
                const int num_of_knots,
                const double delta_s,
                const std::vector<double>& l_lower_bound,
                const std::vector<double>& l_upper_bound,
                const std::vector<double>& steer_bound,
                const std::vector<double>& steet_rate_bound,
                const std::vector<double>& ref_line_kappa,
                const std::vector<double>& ref_line_dkappa,
                const std::vector<double>& speed_data,
                const std::vector<double> init_state,
                const std::vector<double> end_state,
                bool has_end_state_constraint);
        void EstimateLBoundary(
                const std::vector<double>& l_lower_bound,
                const std::vector<double>& l_upper_bound);
        void EstimateDdlBoundary(
                const std::vector<double>& steer_bound,
                const std::vector<double>& ref_line_kappa);
        void EstimateDddlBoundary(
                const std::vector<double>& steet_rate_bound,
                const std::vector<double>& ref_line_dkappa,
                const std::vector<double>& speed_data);
        
        void OptimizerParaCfg();

        void FormulateQpProblem(
                Eigen::MatrixXd& matrix_h,
                Eigen::VectorXd& vector_f, Eigen::MatrixXd& matrix_A,
                Eigen::VectorXd& vector_lb, Eigen::VectorXd& vector_ub) const;
        void CalHessianMatrix(Eigen::MatrixXd& matrix_h) const;    
        void CalGradientVector(Eigen::VectorXd& vector_f)const;    
        void CalVariableConstraint(
                Eigen::MatrixXd& matrix_a_var,
                Eigen::VectorXd& vector_b_var_lb,
                Eigen::VectorXd& vector_b_var_ub) const;
        void CalEqualityConstraint(
                Eigen::MatrixXd& matrix_a_equ,
                Eigen::VectorXd& vector_b_equ) const;
        void CalInequalityConstraint(
                Eigen::MatrixXd& matrix_a_inequ,
                Eigen::VectorXd& vector_b_inequ_lb,
                Eigen::VectorXd& vector_b_inequ_ub) const;

        c_int OptimizationSolver(
                Eigen::VectorXd& optimal_solution,
                const Eigen::MatrixXd matrix_h,
                const Eigen::VectorXd vector_f,
                const Eigen::MatrixXd matrix_A,
                const Eigen::VectorXd vector_lb,
                const Eigen::VectorXd vector_ub,
                const c_int max_iteration, const c_float eps_abs);
        OSQPData* BulidOsqpData(
                const Eigen::MatrixXd matrix_h,
                const Eigen::VectorXd vector_f,
                const Eigen::MatrixXd matrix_A,
                const Eigen::VectorXd vector_lb,
                const Eigen::VectorXd vector_ub);
        OSQPSettings* SolverDefaultSettings() const;
        void FreeData(OSQPData* data) const;
        void MatrixToCCS(
            const Eigen::MatrixXd matrix_a,
  
            std::vector<c_float> *sm_x, c_int &sm_nnz,
            std::vector<c_int> *sm_i, std::vector<c_int> *sm_p) const;
            
        template <typename T>
        T *CopyData(const std::vector<T> &vec) {
            T *data = (T*)c_malloc(vec.size() * sizeof(T));
            memcpy(data, vec.data(), sizeof(T) * vec.size());
            return data;
        }

        void CalculateSLPath(const double path_point_interval);
        
        void CalculateXYPath(
                const std::vector<MatchPoint>& match_points,
                std::vector<XYPathPoint>& xy_path_points);
        XYPathPoint FrenentToCartesian(
                const MatchPoint& match_point,
                const SLPathPoint&sl_path_point,
                const double ref_kappa_rate) const;

        int num_of_knots_;
        double delta_s_;
        double speed_ref_;

        std::vector<double> l_lower_bounds_;
        std::vector<double> l_upper_bounds_;
        std::vector<double> dl_lower_bounds_;
        std::vector<double> dl_upper_bounds_;
        std::vector<double> ddl_lower_bounds_;
        std::vector<double> ddl_upper_bounds_;
        std::vector<double> dddl_lower_bounds_;
        std::vector<double> dddl_upper_bounds_;

        bool has_end_state_constraint_ = false;
        std::vector<double> end_state_ = { 0.0, 0.0, 0.0 };
        std::vector<double> init_state_;

        double ego_rear2front_;
        double ego_rear2back_;
        double ego_width_;
        double ego_wheelbase_;

        double weigth_l_ = 0.2;
        double weigth_dl_  = 40.0;
        double weigth_ddl_ = 1000.0;
        double weigth_dddl_ = 100000.0;

        int num_of_optimization_variables_;
        int num_of_variable_constraints_;
        int num_of_equality_constraints_;
        int num_of_inequality_constraints_;
        int num_of_constraints_;
        
        const c_int osqp_max_iter_ = 2000;
        const c_float osqp_eps_abs_ = 0.00001;

        OptimizerStatus optimizer_status_ = OptimizerStatus::init;

        std::vector<double> opt_l_;
        std::vector<double> opt_dl_;
        std::vector<double> opt_ddl_;

        std::vector<SLPathPoint> sl_path_points_;
        std::vector<XYPathPoint> xy_path_points_;
};
