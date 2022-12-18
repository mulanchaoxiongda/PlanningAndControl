#include <cmath>

#include "piecewise_jerk_path_optimizer.h"

PiecewiseJerkPathOptimizer::PiecewiseJerkPathOptimizer(
        const double& ego_rear2front, const double& ego_rear2back,
        const double& ego_width, const double& ego_wheelbase) {
    ego_width_      = ego_width;
    ego_rear2front_ = ego_rear2front;
    ego_rear2back_  = ego_rear2back;
    ego_wheelbase_ = ego_wheelbase;
}

OptimizeResult PiecewiseJerkPathOptimizer::Process(
        const bool path_reusable,
        const int num_of_knots,
        const double delta_s,
        const std::vector<double>& speed_data,
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
        std::vector<SLPathPoint>&sl_path_points) {
    if( path_reusable) {
        optimizer_status_ = OptimizerStatus::reuse;
        
        sl_path_points = sl_path_points_;

        return OptimizeResult::reuse;
    }

    InputInfoPretreatment(
            num_of_knots, delta_s, l_lower_bound, l_upper_bound, steer_bound,
            steer_rate_bound, ref_line_kappa, ref_line_dkappa, speed_data,
            init_state, end_state, has_end_state_constraint);

    OptimizerParaCfg();

    Eigen::MatrixXd matrix_h =
            Eigen::MatrixXd::Zero(num_of_optimization_variables_,
            num_of_optimization_variables_);
    Eigen::VectorXd vector_f =
            Eigen::VectorXd::Zero(num_of_optimization_variables_, 1);

    Eigen::MatrixXd matrix_A =
            Eigen::MatrixXd::Zero(num_of_constraints_,
            num_of_optimization_variables_);
    Eigen::VectorXd vector_lb =
            Eigen::VectorXd::Zero(num_of_constraints_, 1);
    Eigen::VectorXd vector_ub =
            Eigen::VectorXd::Zero(num_of_constraints_, 1);
    
    FormulateQpProblem(
            matrix_h,vector_f,matrix_A,vector_lb,vector_ub);

    Eigen::VectorXd optimal_solution(num_of_optimization_variables_);

    int osqp_solver_exitflag =
            OptimizationSolver(optimal_solution, matrix_h, vector_f, matrix_A,
            vector_lb, vector_ub, osqp_max_iter_, osqp_eps_abs_);

    if (osqp_solver_exitflag != 0) {
        optimizer_status_= OptimizerStatus::fail;
        return OptimizeResult::fail;
    } else {
        for (int i = 0; i < num_of_knots; ++i) {
            opt_l_.at(i) = optimal_solution(i,0);
            opt_dl_.at(i) = optimal_solution(i+ num_of_knots, 0);
            opt_ddl_.at(i) = optimal_solution(i+ num_of_knots* 2, 0);
        }                  

        CalculateSLPath(path_point_interval);

        sl_path_points.resize(sl_path_points_.size());
        sl_path_points.swap(sl_path_points_);

        // CalculateXYPath(match_points, xy_path_points);
        
        optimizer_status_ = OptimizerStatus::success;
        return OptimizeResult::success;
    }
}

void PiecewiseJerkPathOptimizer::InputInfoPretreatment(
        const int num_of_knots,
        const double delta_s,
        const std::vector<double>& l_lower_bound,
        const std::vector<double>& l_upper_bound,
        const std::vector<double>& steer_bound,
        const std::vector<double>& steer_rate_bound,
        const std::vector<double>& ref_line_kappa,
        const std::vector<double>& ref_line_dkappa,
        const std::vector<double>& speed_data,
        const std::vector<double> init_state,
        const std::vector<double> end_state,
        bool has_end_state_constraint) {
    num_of_knots_ = num_of_knots;
    delta_s_ = delta_s;

    speed_ref_ = speed_data.at(0);  // 建议取speed_data中的最大值

    EstimateLBoundary(l_lower_bound, l_upper_bound);
    EstimateDdlBoundary(steer_bound, ref_line_kappa);
    EstimateDddlBoundary(steer_rate_bound, ref_line_dkappa, speed_data);

    double tan_theta_max = tan(40.0 / 57.3);
    double tan_theta_min = -tan_theta_max;

    dl_lower_bounds_.resize(num_of_knots_);
    for (int i = 0; i < num_of_knots_; ++i) {
        dl_lower_bounds_.at(i) = tan_theta_min;
    }
        
    dl_upper_bounds_.resize(num_of_knots_);
    for (int i = 0; i < num_of_knots_; ++i) {
        dl_upper_bounds_.at(i) = tan_theta_max;
    }

    has_end_state_constraint_ = has_end_state_constraint;

    init_state_.resize(3);
    end_state_.resize(3);
    for (int i = 0; i < 3; ++i) {
        init_state_.at(i) = init_state.at(i);
        end_state_.at(i) = end_state.at(i);
    }
}

void PiecewiseJerkPathOptimizer::EstimateLBoundary(
        const std::vector<double>& l_lower_bound,
        const std::vector<double>& l_upper_bound) {
    double ego_half_width = ego_width_ / 2.0;
    
    l_lower_bounds_.resize(num_of_knots_);
    l_upper_bounds_.resize(num_of_knots_);

    for(int i = 0; i < num_of_knots_; ++i) {
        l_lower_bounds_.at(i) =
                l_lower_bound.at(i) + ego_half_width;
        l_upper_bounds_.at(i) =
                l_upper_bound.at(i) - ego_half_width;
    }
}

void PiecewiseJerkPathOptimizer::EstimateDdlBoundary(
        const std::vector<double>& steer_bound,
        const std::vector<double>& ref_line_kappa) {
    ddl_lower_bounds_.resize(num_of_knots_);
    ddl_upper_bounds_.resize(num_of_knots_);

    for(int i = 0; i < num_of_knots_; ++i) {
        double kappa_bound = std::tan(steer_bound.at(i)) / ego_wheelbase_;

        ddl_lower_bounds_.at(i) = -kappa_bound - ref_line_kappa.at(i);
        ddl_upper_bounds_.at(i) = kappa_bound - ref_line_kappa.at(i);
    }
}

void PiecewiseJerkPathOptimizer::EstimateDddlBoundary(
        const std::vector<double>& steer_rate_bound,
        const std::vector<double>& ref_line_dkappa,
        const std::vector<double>& speed_data) {
    dddl_lower_bounds_.resize(num_of_knots_ - 1);
    dddl_upper_bounds_.resize(num_of_knots_ - 1);

    for(int i = 0; i < num_of_knots_ - 1; ++i) {
        double speed =
                (speed_data.at(i) > 5.0 / 3.6) ?
                speed_data.at(i) : (5.0 / 3.6);
        double dkappa_bound = steer_rate_bound.at(i) / ego_wheelbase_ / speed;
        
        dddl_lower_bounds_.at(i) = -dkappa_bound - ref_line_dkappa.at(i);
        dddl_upper_bounds_.at(i) = dkappa_bound - ref_line_dkappa.at(i);
    }
}

void PiecewiseJerkPathOptimizer::OptimizerParaCfg() {
    if (speed_ref_ <= 15.0 / 3.6) {
        weigth_l_ = 1.0;
    } else {
        weigth_l_ = 0.2;
    }

    num_of_optimization_variables_ = 3 * num_of_knots_;
    num_of_variable_constraints_  = 2 * num_of_knots_;
    num_of_equality_constraints_ =
            3 * (1 + (int)has_end_state_constraint_) + 2 * (num_of_knots_-1);
    num_of_inequality_constraints_ = (num_of_knots_ - 1) + 2 * num_of_knots_;
    num_of_constraints_ =
            num_of_variable_constraints_ + num_of_equality_constraints_ +
            num_of_inequality_constraints_;

    opt_l_.resize(num_of_knots_);
    opt_dl_.resize(num_of_knots_);
    opt_ddl_.resize(num_of_knots_);
}

void PiecewiseJerkPathOptimizer::FormulateQpProblem(
        Eigen::MatrixXd& matrix_h,
        Eigen::VectorXd& vector_f, Eigen::MatrixXd& matrix_A,
        Eigen::VectorXd& vector_lb, Eigen::VectorXd& vector_ub) const {
    CalHessianMatrix(matrix_h);
    CalGradientVector(vector_f);

    Eigen::MatrixXd matrix_a_var =
            Eigen::MatrixXd::Zero(num_of_variable_constraints_,
            num_of_optimization_variables_);
    Eigen::VectorXd vector_b_var_lb =
            Eigen::VectorXd::Zero(num_of_variable_constraints_, 1);
    Eigen::VectorXd vector_b_var_ub =
            Eigen::VectorXd::Zero(num_of_variable_constraints_, 1);
    CalVariableConstraint(matrix_a_var, vector_b_var_lb, vector_b_var_ub);
    
    Eigen::MatrixXd matrix_a_equ =
            Eigen::MatrixXd::Zero(num_of_equality_constraints_,
            num_of_optimization_variables_);
    Eigen::VectorXd vector_b_equ =
            Eigen::VectorXd::Zero(num_of_equality_constraints_, 1);
    CalEqualityConstraint(matrix_a_equ,vector_b_equ);

    Eigen::MatrixXd matrix_a_inequ=
            Eigen::MatrixXd::Zero(num_of_inequality_constraints_,
            num_of_optimization_variables_);
    Eigen::VectorXd vector_b_inequ_lb =
            Eigen::VectorXd::Zero(num_of_inequality_constraints_, 1);
    Eigen::VectorXd vector_b_inequ_ub =
            Eigen::VectorXd::Zero(num_of_inequality_constraints_, 1);
    CalInequalityConstraint(
            matrix_a_inequ, vector_b_inequ_lb, vector_b_inequ_ub);

    matrix_A.block(
            0, 0,
            num_of_variable_constraints_, num_of_optimization_variables_) =
            matrix_a_var;
    matrix_A.block(
            num_of_variable_constraints_, 0,
            num_of_equality_constraints_, num_of_optimization_variables_) =
            matrix_a_equ;
    matrix_A.block(
            num_of_variable_constraints_ + num_of_equality_constraints_, 0,
            num_of_inequality_constraints_, num_of_optimization_variables_) =
            matrix_a_inequ;

    vector_lb << vector_b_var_lb, vector_b_equ, vector_b_inequ_lb;
    vector_ub << vector_b_var_ub, vector_b_equ, vector_b_inequ_ub;
}

void PiecewiseJerkPathOptimizer::CalHessianMatrix(
        Eigen::MatrixXd& matrix_h) const {
    Eigen::MatrixXd matrix_h_l =
            Eigen::MatrixXd::Identity(num_of_knots_, num_of_knots_) *
            weigth_l_;
    
    Eigen::MatrixXd matrix_h_dl =
            Eigen::MatrixXd::Identity(num_of_knots_, num_of_knots_) *
            weigth_dl_;

    Eigen::MatrixXd matrix_h_ddl =
            Eigen::MatrixXd::Identity(num_of_knots_, num_of_knots_) *
            (weigth_ddl_ + 2.0 * weigth_dddl_ / std::pow(delta_s_, 2.0));

    matrix_h_ddl(0, 0) =
            matrix_h_ddl(0, 0) - weigth_dddl_ / std::pow(delta_s_, 2.0);
    matrix_h_ddl(num_of_knots_ - 1, num_of_knots_ - 1) =
            matrix_h_ddl(num_of_knots_ - 1, num_of_knots_ - 1) -
            weigth_dddl_ / std::pow(delta_s_, 2.0);

    for (int i = 1; i < num_of_knots_; ++i) {
        matrix_h_ddl(i, i - 1) = -2.0 * weigth_dddl_ / std::pow(delta_s_, 2.0);
    }
    
    matrix_h.block(0, 0, num_of_knots_, num_of_knots_) = matrix_h_l;
    matrix_h.block(
            num_of_knots_, num_of_knots_,
            num_of_knots_, num_of_knots_) = matrix_h_dl;
    matrix_h.block(
            num_of_knots_ * 2,num_of_knots_ * 2,
            num_of_knots_, num_of_knots_) = matrix_h_ddl;

    Eigen::MatrixXd matrix_h_transpose(matrix_h.rows(), matrix_h.cols());
    matrix_h_transpose = matrix_h.transpose();
    matrix_h = matrix_h + matrix_h_transpose;
}

void PiecewiseJerkPathOptimizer::CalGradientVector(
        Eigen::VectorXd& vector_f) const {
    Eigen::VectorXd vector_f_l = Eigen::VectorXd::Zero(num_of_knots_, 1);
    Eigen::VectorXd vector_f_dl = Eigen::VectorXd::Zero(num_of_knots_, 1);
    Eigen::VectorXd vector_f_ddl = Eigen::VectorXd::Zero(num_of_knots_, 1);
    
    vector_f << vector_f_l, vector_f_dl, vector_f_ddl;
}

void PiecewiseJerkPathOptimizer::CalVariableConstraint(
        Eigen::MatrixXd& matrix_a_var,
        Eigen::VectorXd& vector_b_var_lb,
        Eigen::VectorXd& vector_b_var_ub) const {
    Eigen::MatrixXd matrix_a_var_dl =
            Eigen::MatrixXd::Zero(num_of_knots_,
            num_of_optimization_variables_);
    matrix_a_var_dl.block(0, num_of_knots_, num_of_knots_, num_of_knots_) =
            Eigen::MatrixXd::Identity(num_of_knots_, num_of_knots_);

    Eigen::MatrixXd matrix_a_var_ddl =
            Eigen::MatrixXd::Zero(num_of_knots_,
            num_of_optimization_variables_);
    matrix_a_var_ddl.block(0, num_of_knots_ * 2,
            num_of_knots_, num_of_knots_) =
            Eigen::MatrixXd::Identity(num_of_knots_, num_of_knots_);

    matrix_a_var.block(0, 0, num_of_knots_, num_of_optimization_variables_) =
            matrix_a_var_dl;
    matrix_a_var.block(
            num_of_knots_, 0, num_of_knots_, num_of_optimization_variables_) =
            matrix_a_var_ddl;

    for (int i=0;i<num_of_knots_;++i){
        vector_b_var_lb(i, 0) = dl_lower_bounds_.at(i);
        vector_b_var_lb(i + num_of_knots_, 0) = ddl_lower_bounds_.at(i);
        
        vector_b_var_ub(i, 0) = dl_upper_bounds_.at(i);
        vector_b_var_ub(i + num_of_knots_, 0) = ddl_upper_bounds_.at(i);
    }
}

void PiecewiseJerkPathOptimizer::CalEqualityConstraint(
        Eigen::MatrixXd& matrix_a_equ,
        Eigen::VectorXd& vector_b_equ) const {
    Eigen::MatrixXd matrix_a_equ_init =
            Eigen::MatrixXd::Zero(3, num_of_optimization_variables_);

    matrix_a_equ_init(0, 0) = 1.0;
    matrix_a_equ_init(1, num_of_knots_) = 1.0;
    matrix_a_equ_init(2, num_of_knots_ * 2) = 1.0;
    
    Eigen::VectorXd vector_b_equ_init(3, 1);
    vector_b_equ_init << init_state_.at(0), init_state_.at(1), init_state_.at(2);
    
    Eigen::MatrixXd matrix_a_equ_l =
            Eigen::MatrixXd::Zero(num_of_knots_ - 1,
            num_of_optimization_variables_);

    for (int i = 0; i < matrix_a_equ_l.rows(); ++i) {
        matrix_a_equ_l(i, i) = -1;
        matrix_a_equ_l(i, i + 1) = 1;
        matrix_a_equ_l(i, num_of_knots_ + i) = -delta_s_;
        matrix_a_equ_l(i, num_of_knots_ * 2 + i) = -std::pow(delta_s_, 2.0) / 3.0;
        matrix_a_equ_l(i, num_of_knots_ * 2 + i + 1) =
                -std::pow(delta_s_, 2.0) / 6.0;
    }

    Eigen::VectorXd vector_b_equ_l =
            Eigen::VectorXd::Zero(num_of_knots_ - 1, 1);

    Eigen::MatrixXd matrix_a_equ_dl=
            Eigen::MatrixXd::Zero(num_of_knots_-1,
            num_of_optimization_variables_);

    for (int i = 0; i< matrix_a_equ_dl.rows(); ++i) {
            matrix_a_equ_dl(i, num_of_knots_  + i) = -1;
            matrix_a_equ_dl(i, num_of_knots_  + i+ 1)= 1;
            matrix_a_equ_dl(i, num_of_knots_ * 2 + i) = -delta_s_ / 2.0;
            matrix_a_equ_dl(i, num_of_knots_ * 2 + i + 1) = -delta_s_ / 2.0;
    }
    
    Eigen::VectorXd vector_b_equ_dl =
            Eigen::VectorXd::Zero(num_of_knots_ - 1, 1);
        
    matrix_a_equ.block(0, 0 , 3, num_of_optimization_variables_) =
            matrix_a_equ_init;
    matrix_a_equ.block(
            3, 0, num_of_knots_ - 1, num_of_optimization_variables_) =
            matrix_a_equ_l;
    matrix_a_equ.block(
            3 + num_of_knots_ - 1, 0,
            num_of_knots_ - 1, num_of_optimization_variables_) =
            matrix_a_equ_dl;
    
    if (has_end_state_constraint_) {
        Eigen::MatrixXd matrix_a_equ_end =
                Eigen::MatrixXd::Zero(3, num_of_optimization_variables_);
        
        matrix_a_equ_end(0, num_of_knots_ - 1) = 1.0;
        matrix_a_equ_end(1, num_of_knots_ * 2 - 1) = 1.0;
        matrix_a_equ_end(2, num_of_knots_ * 3 - 1) = 1.0;
        
        Eigen::VectorXd vector_b_equ_end(3, 1);
        vector_b_equ_end <<
                end_state_.at(0), end_state_.at(1), end_state_.at(2);

        matrix_a_equ.block(
                3 + (num_of_knots_ - 1) * 2, 0, 3, num_of_optimization_variables_) =
                matrix_a_equ_end;

        vector_b_equ <<
                vector_b_equ_init, vector_b_equ_l,
                vector_b_equ_dl, vector_b_equ_end;
    } else {
        vector_b_equ << vector_b_equ_init, vector_b_equ_l, vector_b_equ_dl;
    }
}

void PiecewiseJerkPathOptimizer::CalInequalityConstraint(
        Eigen::MatrixXd& matrix_a_inequ,
        Eigen::VectorXd& vector_b_inequ_lb,
        Eigen::VectorXd& vector_b_inequ_ub) const {
    Eigen::MatrixXd matrix_a_inequ_l =
            Eigen::MatrixXd::Zero(num_of_knots_ * 2,
            num_of_optimization_variables_);

    for (int i = 0; i < num_of_knots_; ++i) {
        matrix_a_inequ_l(i, i) = 1.0;
        matrix_a_inequ_l(i, i + num_of_knots_) = ego_rear2front_;
        matrix_a_inequ_l(i + num_of_knots_, i) = 1.0;
        matrix_a_inequ_l(i + num_of_knots_, i + num_of_knots_) =
                -ego_rear2back_;
    }

    Eigen::VectorXd vector_b_inequ_lb_l =
            Eigen::VectorXd::Zero(num_of_knots_ * 2, 1);
    for (int i = 0; i < num_of_knots_; ++i) {
        vector_b_inequ_lb_l(i, 0)= l_lower_bounds_.at(i);
        vector_b_inequ_lb_l(i + num_of_knots_, 0)= l_lower_bounds_.at(i);
    }

    Eigen::VectorXd vector_b_inequ_ub_l =
            Eigen::VectorXd::Zero(num_of_knots_ * 2, 1);
    for(int i = 0; i < num_of_knots_; ++i) {
        vector_b_inequ_ub_l(i, 0) = l_upper_bounds_.at(i);
        vector_b_inequ_ub_l(i + num_of_knots_, 0)= l_upper_bounds_.at(i);
    }

    Eigen::MatrixXd matrix_a_inequ_dddl =
            Eigen::MatrixXd::Zero(num_of_knots_ - 1,
            num_of_optimization_variables_);       
    for (int i = 0; i < matrix_a_inequ_dddl.rows(); ++i) {
        matrix_a_inequ_dddl(i, num_of_knots_ * 2 + i) = -1.0 / delta_s_;
        matrix_a_inequ_dddl(i, num_of_knots_ * 2 + i + 1) = 1.0 / delta_s_;
    }
        
    Eigen::VectorXd vector_b_inequ_lb_dddl =
            Eigen::VectorXd::Zero(num_of_knots_ - 1, 1);
    for(int i = 0; i < vector_b_inequ_lb_dddl.rows(); ++i) {
        vector_b_inequ_lb_dddl(i, 0) = dddl_lower_bounds_.at(i);
    }

    Eigen::VectorXd vector_b_inequ_ub_dddl =
            Eigen::VectorXd::Zero(num_of_knots_ - 1, 1);
    for (int i = 0; i < vector_b_inequ_ub_dddl.rows(); ++i) {
        vector_b_inequ_ub_dddl(i, 0) = dddl_upper_bounds_.at(i);
    }

    matrix_a_inequ.block(
            0, 0, num_of_knots_ * 2, num_of_optimization_variables_) =
            matrix_a_inequ_l;
    matrix_a_inequ.block(
            num_of_knots_ * 2, 0,
            num_of_knots_ - 1, num_of_optimization_variables_) =
            matrix_a_inequ_dddl;

    vector_b_inequ_lb << vector_b_inequ_lb_l, vector_b_inequ_lb_dddl;
    vector_b_inequ_ub << vector_b_inequ_ub_l, vector_b_inequ_ub_dddl;
}

c_int PiecewiseJerkPathOptimizer::OptimizationSolver(
        Eigen::VectorXd &optimal_solution,
        const Eigen::MatrixXd matrix_h,
        const Eigen::VectorXd vector_f,
        const Eigen::MatrixXd matrix_A,
        const Eigen::VectorXd vector_lb,
        const Eigen::VectorXd vector_ub,
        const c_int max_iteration, const c_float eps_abs) {
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
    data = BulidOsqpData(matrix_h, vector_f, matrix_A, vector_lb, vector_ub);

    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    settings = SolverDefaultSettings();

    OSQPWorkspace *work;
    
    c_int exitflag = osqp_setup(&work,data,settings);

    osqp_solve(work);

    auto status = work->info->status_val;
    if (status < 0 || (status != 1 && status != 2)) {
        std::cout << "failed optimization status:" << status << std::endl;

        osqp_cleanup(work);

        FreeData(data);

        c_free(settings);

        return -100;
    } else if(work->solution == nullptr) {
        std::cout << "the solution from OSQP is nullptr" << std::endl;

        osqp_cleanup(work);

        FreeData(data);
        
        c_free(settings);

        return-100;
    }

    c_int n = matrix_h.cols();
    for (int i = 0; i < n; ++i) {
        optimal_solution(i) = work->solution->x[i];
    }

    osqp_cleanup(work);
    
    FreeData(data);
    
    if (settings) {
        c_free(settings);
    }

    return exitflag;
}

void PiecewiseJerkPathOptimizer::MatrixToCCS(
        const Eigen::MatrixXd matrix_a,
        std::vector<c_float> *sm_x, c_int &sm_nnz,
        std::vector<c_int> *sm_i, std::vector<c_int> *sm_p) const {
    sm_p->emplace_back(0);
    
    int num_cols = matrix_a.cols(), num_rows = matrix_a.rows(), nz = 0;
    
    if (num_cols == num_rows) {
        for (int j = 0; j < num_cols; j++) {
            for (int i = 0; i <= j; i++) {
                if (std::fabs(matrix_a(i, j)) > 0.0000001) {
                    sm_x->emplace_back(matrix_a(i, j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }
            
            sm_p->emplace_back(nz);
        }
    } else if(num_cols < num_rows) {
        for (int j = 0; j < num_cols; j++) {
            for (int i = 0; i < num_rows; i++) {
                if(std::fabs(matrix_a(i, j)) > 0.0000001) {
                    sm_x->emplace_back(matrix_a(i, j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    }
    
    sm_nnz = nz;
}

OSQPData* PiecewiseJerkPathOptimizer::BulidOsqpData(
        const Eigen::MatrixXd matrix_h,
        const Eigen::VectorXd vector_f,
        const Eigen::MatrixXd matrix_A,
        const Eigen::VectorXd vector_lb,
        const Eigen::VectorXd vector_ub) {
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

    std::vector<c_float> p_x;
    c_int                p_nnz;
    std::vector<c_int>   p_i;
    std::vector<c_int>   p_p;

    MatrixToCCS(matrix_h, &p_x, p_nnz, &p_i, &p_p);

    std::vector<c_float> A_x;
    c_int                A_nnz;
    std::vector<c_int>   matrix_A_i;
    std::vector<c_int>   A_p;

    MatrixToCCS(matrix_A, &A_x, A_nnz, &matrix_A_i, &A_p);

    int length = vector_f.size();

    std::vector<c_float> q(length);

    for (int i = 0; i < length; ++i) {
        q.at(i) = vector_f(i);
    }

    length = vector_lb.size();
    
    std::vector<c_float> l(length);
    
    for(int i = 0; i < length; ++i) {
        l.at(i) = vector_lb(i);
    }

    length = vector_ub.size();

    std::vector<c_float> u(length);
    
    for (int i = 0; i < length; i++) {
        u.at(i) = vector_ub(i);
    }

    c_int m = matrix_A.rows();
    c_int n = matrix_h.cols();
    
    if (data) {
        data->n = n;
        data->m = m;

        data->P =
                csc_matrix(data->n, data->n, p_nnz, CopyData(p_x),
                CopyData(p_i),CopyData(p_p));
        data->q = CopyData(q);

        data->A =
                csc_matrix(data->m, data->n, A_nnz, CopyData(A_x),
                CopyData(matrix_A_i),CopyData(A_p));

        data->l = CopyData(l);
        data->u = CopyData(u);
    }

    return data;
}

OSQPSettings* PiecewiseJerkPathOptimizer::SolverDefaultSettings() const {
    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    
    if (settings) {
        osqp_set_default_settings(settings);

        settings->polish = true,
        settings->verbose = false;
        settings->max_iter = osqp_max_iter_;
        settings->eps_abs = osqp_eps_abs_;
        settings->alpha = 1.0;
    }

    return settings;
}

void PiecewiseJerkPathOptimizer::FreeData(OSQPData* data) const {
    if(data) {
        if (data->A) {
            c_free(data->A);
        }

        if (data->P) {
            c_free(data->P);
        }

        c_free(data);
    }
}

void PiecewiseJerkPathOptimizer::CalculateSLPath(
        const double path_point_interval) {
    sl_path_points_.clear();

    SLPathPoint path_point;

    double s = 0.0, dddl, ddl, dl, l, kappa;

    int num_of_points_in_fragment = std::round(delta_s_ / path_point_interval) + 1;

    for (int i = 0; i < num_of_knots_ - 1; ++i) {
        s = i * delta_s_;
        l = opt_l_.at(i);
        dl = opt_dl_.at(i);
        ddl = opt_ddl_.at(i);
        dddl = (opt_ddl_.at(i + 1) - opt_ddl_.at(i)) / delta_s_;
        kappa = ddl / std::pow(1.0 + std::pow(dl, 2.0), 1.5);

        path_point = {s, 0.0, 0.0, l, dl, ddl, kappa};

        sl_path_points_.push_back(path_point);

        for (int j = 1; j < num_of_points_in_fragment - 1; ++j) {
            s = s + path_point_interval;
            l =
                    l + dl * path_point_interval +
                    0.5 * ddl * std::pow(path_point_interval, 2.0) +
                    1.0 / 6.0 * dddl * std::pow(path_point_interval, 3.0);                    
            dl =
                    dl + ddl * path_point_interval +
                    0.5 * dddl * std::pow(path_point_interval, 2.0);
            ddl = ddl + dddl * path_point_interval;
            kappa = ddl / std::pow(1.0 + std::pow(dl, 2.0), 1.5);

            path_point = {s, 0.0, 0.0, l, dl, ddl, kappa};

            sl_path_points_.push_back(path_point);
        }
    }

    s = (num_of_knots_ - 1) * delta_s_;
    l = opt_l_.back();
    dl = opt_dl_.back();
    ddl = opt_ddl_.back();
    kappa = ddl / std::pow(1.0 + std::pow(dl, 2.0), 1.5);

    path_point = {s, 0.0, 0.0, l, dl, ddl, kappa};

    sl_path_points_.push_back(path_point);
}

void PiecewiseJerkPathOptimizer::CalculateXYPath(
        const std::vector<MatchPoint>& match_points,
        std::vector<XYPathPoint>& xy_path_points) {
    xy_path_points_.clear();

    if (match_points.size() != sl_path_points_.size()) {
        std::cout << "[piecewise_jerk_path_optimizer][error] :"
                  << "num_of_match _points != num_ofsl_path_points"
                  << std::endl;
        return;
    }

    int num_of_xy_path_points = (int)sl_path_points_.size();
    std::vector<double> ref_kappa_rate(num_of_xy_path_points);

    for (int i = 1; i < num_of_xy_path_points - 1; ++i) {
        ref_kappa_rate.at(i) =
            (match_points.at(i + 1).kappa - match_points.at(i - 1).kappa) /
            (2.0 * delta_s_);
    }
    ref_kappa_rate.front() = ref_kappa_rate.at(1);
    ref_kappa_rate.back() = ref_kappa_rate.at(num_of_xy_path_points - 1);
    
    for (int i = 0; i < num_of_xy_path_points; ++i) {
        xy_path_points_.push_back(
                FrenentToCartesian(match_points.at(i),
                sl_path_points_.at(i), ref_kappa_rate.at(i)));
    }

    xy_path_points.resize(xy_path_points_.size());
    xy_path_points.swap(xy_path_points_);
}

XYPathPoint PiecewiseJerkPathOptimizer::FrenentToCartesian(
        const MatchPoint& match_point,
        const SLPathPoint& sl_path_point,
        const double ref_kappa_rate) const {
    double delta_theta =
            std::atan2(sl_path_point.dl, 1.0 - match_point.kappa * sl_path_point.l);

    double x =
            match_point.x + sl_path_point.l * std::sin(-match_point.theta);
    double y =
            match_point.y + sl_path_point.l * std::cos(-match_point.theta);

    double theta =
            match_point.theta +
            std::atan2(sl_path_point.dl, 1.0 - match_point.kappa * sl_path_point.l);
            
    double curvature =
            (((sl_path_point.ddl + (ref_kappa_rate * sl_path_point.l +
            match_point.kappa * sl_path_point.dl) * std::tan(delta_theta)) *
            std::pow(cos(delta_theta), 2.0)) / (1.0 - match_point.kappa *
            sl_path_point.l) + match_point.kappa) * std::cos(delta_theta) /
            (1.0 - match_point.kappa * sl_path_point.l);

    XYPathPoint res = { x, y, theta, curvature };

    return res;
}
