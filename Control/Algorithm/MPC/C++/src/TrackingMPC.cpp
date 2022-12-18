#include <math.h>
#include <algorithm>
#include <string.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <iomanip>
#include <memory>
#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <sys/time.h>

#include "TrackingMPC.h"
#include "CustomFunction.h"

using namespace std;

TrackingMPC::TrackingMPC(RobotModel *p_RobotModel, SaveData *p_savedata):
        TrackingAlgorithm::TrackingAlgorithm(p_RobotModel, p_savedata)
{
    Nx_ = 4;
    Nu_ = 2;

    ReadInTrajPoints();
    ReadInControlPara();

    u_.resize(Nu_);

    sensor_info_ = GetSensorInfo();

    reference_point_ = FindRefPoint(trajectory_points_, sensor_info_);

    u_ << p_RobotModel_->motion_state_.v-reference_point_.v,
          p_RobotModel_->motion_state_.w-reference_point_.w;

    running_time_sum_ = 0.0;
    running_time_average_ = 0.0;

    loop_counter_ = 0;
}

ControlCommand TrackingMPC::CalControlCommand()
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    sensor_info_ = GetSensorInfo();

    reference_point_ = FindRefPoint(trajectory_points_, sensor_info_);

    int Np, Nc;
    double control_period;

    VectorXd u_min(Nu_, 1), u_max(Nu_, 1), du_min(Nu_, 1), du_max(Nu_, 1);
    MatrixXd q(Nx_, Nx_), r(Nu_, Nu_);

    double call_cycle = 0.02;

    CalControlCoefficient(q, r, Np, Nc, control_period, u_min, u_max,
                          du_min, du_max, p_RobotModel_->motion_state_.v,
                          call_cycle);

    VectorXd x(Nx_ + Nu_);
    MatrixXd a(Nx_, Nx_), b(Nx_, Nu_);

    UpdateErrorModel(x, a, b, control_period);

    MatrixXd A_(Nx_ + Nu_, Nx_ + Nu_), B_(Nx_ + Nu_, Nu_), C_(Nx_, Nx_ + Nu_);

    UpdateIncrementModel(A_, B_, C_, x, a, b);

    MatrixXd PHI(Nx_ * Np, Nx_ + Nu_), THETA(Nx_ * Np, Nu_ * Nc);

    PredictFunc(PHI, THETA, A_, B_, C_, Np, Nc);

    MatrixXd _H(Nc * Nu_, Nc * Nu_), E_(Nx_ * Np, 1);
    VectorXd _g(Nc * Nu_, 1);

    ObjectiveFunc(_H, E_, _g, x, PHI, THETA, q, r, Np, Nc);

    MatrixXd _A(Nc * Nu_ * 2, Nc * Nu_);
    VectorXd _lb(Nc * Nu_ * 2, 1), _ub(Nc * Nu_ * 2, 1);

    ConstraintCondition(_A, _lb, _ub, u_, u_min, u_max, du_min, du_max,
                        Np, Nc);

    int num_constraints = Nc * Nu_ * 2, num_variables = Nc * Nu_;
    VectorXd u_optim(Nc * Nu_);

    c_int max_iteration =200;
    c_float eps_abs = 0.01;

    OptimizationSolver(u_optim, _H, _g, _A, _lb, _ub, num_constraints,
                       num_variables, max_iteration, eps_abs);

    u_(0) = u_(0) + u_optim(0);
    u_(1) = u_(1) + u_optim(1);

    ControlCommand control_command = { u_(0) + reference_point_.v,
                                       u_(1) + reference_point_.w,
                                       p_RobotModel_->motion_state_.t };

    cout << " [DBG] " << " speed_command: " << control_command.SpeedCommand
           << " yawratio_command: " << control_command.YawRateCommand << endl;

    p_savedata_->file << "[control_command] "
                      << " Time "             << control_command.t
                      << " speed_command "    << control_command.SpeedCommand
                      << " yawratio_command " << control_command.YawRateCommand
                      << " t "                << control_command.t << endl;

    gettimeofday(&t_end, NULL);

    loop_counter_++;

    running_time_sum_ = running_time_sum_ + (t_end.tv_sec - t_start.tv_sec) +
                        (double)(t_end.tv_usec - t_start.tv_usec) / 1000000.0;

    running_time_average_ = running_time_sum_ / (double)loop_counter_;

    return control_command;
}

void TrackingMPC::ReadInControlPara()
{
    vector<vector<double>> control_para;

    CustomFunction::txt_to_vectordouble(
            control_para, "../data/ControlParaMPC.txt");

    v_para_ = control_para[0];

    q_delta_x_para_   = control_para[1];
    q_delta_y_para_   = control_para[2];
    q_delta_yaw_para_ = control_para[3];
    q_delta_wz_para_  = control_para[4];

    r_delta_vc_para_ = control_para[5];
    r_delta_wc_para_ = control_para[6];

    Np_para_ = control_para[7];
    Nc_para_ = control_para[8];

    control_period_para_ = control_para[9];

    v_min_para_ = control_para[10];
    v_max_para_ = control_para[11];

    w_min_para_ = control_para[12];
    w_max_para_ = control_para[13];

    dv_min_para_ = control_para[14];
    dv_max_para_ = control_para[15];

    dw_min_para_ = control_para[16];
    dw_max_para_ = control_para[17];
}

void TrackingMPC::CalControlCoefficient(
        MatrixXd &q, MatrixXd &r, int &Np, int &Nc, double &control_period,
        VectorXd &u_min, VectorXd &u_max, VectorXd &du_min, VectorXd &du_max,
        double v_sensor, double call_cycle)
{
    q.setZero(Nx_, Nx_);

    q(0, 0) =
            CustomFunction::interp_linear(v_para_, q_delta_x_para_,   v_sensor);
    q(1, 1) =
            CustomFunction::interp_linear(v_para_, q_delta_y_para_,   v_sensor);
    q(2, 2) =
            CustomFunction::interp_linear(v_para_, q_delta_yaw_para_, v_sensor);
    q(3, 3) =
            CustomFunction::interp_linear(v_para_, q_delta_wz_para_,  v_sensor);

    r.setZero(Nu_, Nu_);

    r(0, 0) =
            CustomFunction::interp_linear(v_para_, r_delta_vc_para_, v_sensor);
    r(1, 1) =
            CustomFunction::interp_linear(v_para_, r_delta_wc_para_, v_sensor);

    Np = (int)CustomFunction::interp_linear(v_para_, Np_para_, v_sensor);
    Nc = (int)CustomFunction::interp_linear(v_para_, Nc_para_, v_sensor);

    control_period = CustomFunction::interp_linear(v_para_,
                                                   control_period_para_,
                                                   v_sensor);

    double v_min, v_max, w_min, w_max;

    v_min = CustomFunction::interp_linear(v_para_, v_min_para_, v_sensor);
    v_max = CustomFunction::interp_linear(v_para_, v_max_para_, v_sensor);

    w_min = CustomFunction::interp_linear(v_para_, w_min_para_, v_sensor);
    w_max = CustomFunction::interp_linear(v_para_, w_max_para_, v_sensor);

    u_min << v_min - reference_point_.v, w_min / 57.3 - reference_point_.w;
    u_max << v_max - reference_point_.v, w_max / 57.3 - reference_point_.w;

    double dv_min, dv_max, dw_min, dw_max;

    dv_min = CustomFunction::interp_linear(v_para_, dv_min_para_, v_sensor);
    dv_max = CustomFunction::interp_linear(v_para_, dv_max_para_, v_sensor);

    dw_min = CustomFunction::interp_linear(v_para_, dw_min_para_, v_sensor);
    dw_max = CustomFunction::interp_linear(v_para_, dw_max_para_, v_sensor);

    du_min << dv_min * call_cycle, dw_min * call_cycle / 57.3;
    du_max << dv_max * call_cycle, dw_max * call_cycle / 57.3;

    Np = 25;
    Nc = 25;

    q.setIdentity(Nx_, Nx_);

    double k_lon, k_lat;

    k_lon = 1.0;
    k_lat = 40.0;

    double psi;

    psi = p_RobotModel_->motion_state_.yaw;

    double psi_trans;

    if (fabs(psi) >= M_PI / 2.0) {
        psi_trans = M_PI - fabs(psi);
    } else {
        psi_trans = fabs(psi);
    }
    
    double k_x, k_y;

    k_x = k_lon * cos(psi_trans) + k_lat * sin(psi_trans);
    k_y = k_lon * sin(psi_trans) + k_lat * cos(psi_trans);
    
    q(0, 0) = k_x;
    q(1, 1) = k_y;
    q(2, 2) = 0.5;
    q(3, 3) = 0.1;

    r.setIdentity(Nu_, Nu_);

    r(0, 0) = 8.0;
    r(1, 1) = 0.2;

    control_period = 0.05;

    u_min << -1.0 - reference_point_.v, -20.0 / 57.3 - reference_point_.w;

    u_max <<  1.0 - reference_point_.v,  20.0 / 57.3 - reference_point_.w;

    du_min << -2.0 * call_cycle,    -80.0 / 57.3 * call_cycle;

    du_max <<  2.0 * call_cycle,     80.0 / 57.3 * call_cycle;
}

void TrackingMPC::UpdateErrorModel(VectorXd &X_, MatrixXd &a, MatrixXd &b,
                                   double dt)
{
    VectorXd x(Nx_), xr(Nx_), kesi(Nx_ + Nu_);

    x << sensor_info_.x, sensor_info_.y, sensor_info_.yaw, sensor_info_.w;

    xr << reference_point_.x,   reference_point_.y,
          reference_point_.yaw, reference_point_.w;

    kesi << x-xr, u_;

    X_ = kesi;

    double T1 = 0.07;

    a << 1.0, 0.0, -reference_point_.v*dt*sin(reference_point_.yaw),  0.0,
         0.0, 1.0,  reference_point_.v*dt*cos(reference_point_.yaw),  0.0,
         0.0, 0.0,  1.0,                                              dt,
         0.0, 0.0,  0.0,                                              1.0 - dt / T1;

    b << dt * cos(reference_point_.yaw), 0.0,
         dt * sin(reference_point_.yaw), 0.0,
         0.0,                           0.0,
         0.0,                           dt / T1;

    p_savedata_->file << " [tracking_error] "
                      << " Time "    << p_RobotModel_->motion_state_.t
                      << " err_x "   << X_(0)
                      << " err_y "   << X_(1)
                      << " err_yaw " << X_(2)
                      << " t "       << p_RobotModel_->motion_state_.t << endl;
}

void TrackingMPC::UpdateIncrementModel(
        MatrixXd &A, MatrixXd &B, MatrixXd &C,
        VectorXd x, MatrixXd a, MatrixXd b)
{
    A.setZero(Nx_ + Nu_, Nx_ + Nu_);
    A.block(0, 0, Nx_, Nx_) = a;
    A.block(0, Nx_, Nx_, Nu_) = b;
    A.block(Nx_, Nx_, Nu_, Nu_) = MatrixXd::Identity(Nu_, Nu_);

    B.setZero(Nx_ + Nu_, Nu_);
    B.block(0, 0, Nx_, Nu_) = b;
    B.block(Nx_, 0, Nu_, Nu_) = MatrixXd::Identity(Nu_, Nu_);

    C.setZero(Nx_, Nx_ + Nu_);
    C.block(0, 0, Nx_, Nx_) = MatrixXd::Identity(Nx_, Nx_);
}

void TrackingMPC::PredictFunc(MatrixXd &phi, MatrixXd &theta,
                              MatrixXd A, MatrixXd B, MatrixXd C,
                              int Np, int Nc)
{
    phi.setZero(Nx_ * Np, Nx_ + Nu_);

    for (int i = 0; i < Np; i++) {
        MatrixXd A_i = MatrixXd::Identity(A.rows(), A.cols());

        for (int j = 0; j <= i; j++) {
            A_i = A_i * A;
        }

        phi.block(i * C.rows(), 0, C.rows(), A_i.cols()) = C * A_i;
    }

    theta.setZero(Nx_ * Np, Nu_ * Nc);

    for (int i = 0; i < Np; i++) {
        for (int j = 0; j < Nc; j++) {
            if (i >= j) {
                MatrixXd A_i_substract_j =
                        MatrixXd::Identity(A.rows(), A.cols());

                for (int k = i - j; k > 0; k--) {
                    A_i_substract_j = A_i_substract_j * A;
                }

                theta.block(i * C.rows(), j * B.cols(), C.rows(), B.cols()) =
                        C * A_i_substract_j * B;
            }
        }
    }
}

void TrackingMPC::ObjectiveFunc(
        MatrixXd &h_, MatrixXd &e_, VectorXd &_g, MatrixXd kesi_, MatrixXd phi_,
        MatrixXd theta_, MatrixXd q_, MatrixXd r_, int Np, int Nc)
{
    MatrixXd Q(Np * Nx_, Np * Nx_), R(Nc * Nu_, Nc * Nu_);

    Q = CustomFunction::KroneckerProduct(MatrixXd::Identity(Np, Np), q_);

    R = CustomFunction::KroneckerProduct(MatrixXd::Identity(Nc, Nc), r_);

    h_ = theta_.transpose() * Q * theta_ + R;
    h_ = (h_+h_.transpose()) * 0.5;

    e_ = phi_ * kesi_;
    _g = ((e_.transpose()) * Q * theta_).transpose();
}

void TrackingMPC::ConstraintCondition(
        MatrixXd &A, VectorXd &lb, VectorXd &ub,VectorXd U, VectorXd umin,
        VectorXd umax, VectorXd delta_umin, VectorXd delta_umax,
        int Np, int Nc)
{
    MatrixXd A_t = MatrixXd::Zero(Nc, Nc);

    for (int i = 0; i < Nc; i++) {
        for (int j = 0; j <= i; j++) {
            A_t(i, j) = 1;
        }
    }

    MatrixXd A_I(Nc * Nu_, Nc * Nu_);

    A_I = CustomFunction::KroneckerProduct(A_t, MatrixXd::Identity(Nu_, Nu_));

    MatrixXd Ut(Nc * Nu_, 1);
    Ut = CustomFunction::KroneckerProduct(MatrixXd::Ones(Nc, 1), U);

    MatrixXd Umin(Nc * Nu_, 1), Umax(Nc * Nu_, 1);

    Umin = CustomFunction::KroneckerProduct(MatrixXd::Ones(Nc, 1), umin);

    Umax = CustomFunction::KroneckerProduct(MatrixXd::Ones(Nc, 1), umax);

    MatrixXd delta_Umin(Nc * Nu_, 1), delta_Umax(Nc * Nu_, 1);

    delta_Umin =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(Nc, 1), delta_umin);

    delta_Umax =
            CustomFunction::KroneckerProduct(MatrixXd::Ones(Nc, 1), delta_umax);

    A.block(0, 0, Nc * Nu_, Nc * Nu_) = A_I;

    A.block(Nc * Nu_, 0, Nc * Nu_, Nc * Nu_) =
            MatrixXd::Identity(Nc * Nu_, Nc * Nu_);

    lb << Umin - Ut, delta_Umin;

    ub << Umax - Ut, delta_Umax;
}

int TrackingMPC::OptimizationSolver(
        VectorXd &optimal_solution, MatrixXd p01, VectorXd q01, MatrixXd Ac,
        VectorXd l01, VectorXd u01, int m01, int n01, c_int max_iteration,
        c_float eps_abs)
{
    vector<c_float> p_x;
    c_int           p_nnz;
    vector<c_int>   p_i;
    vector<c_int>   p_p;

    MatrixToCCS(p01, &p_x, p_nnz, &p_i, &p_p);

    vector<c_float> A_x;
    c_int           A_nnz;
    vector<c_int>   A_i;
    vector<c_int>   A_p;

    MatrixToCCS(Ac, &A_x, A_nnz, &A_i, &A_p);

    int length = q01.size();

    vector<c_float> q(length);

    for (int i = 0; i < length; i++) {
        q.at(i) = q01(i);
    }

    length = l01.size();

    vector<c_float> l(length);

    for (int i = 0; i < length; i++) {
        l.at(i) = l01(i);
    }

    length = u01.size();

    vector<c_float> u(length);

    for (int i = 0; i < length; i++) {
        u.at(i) = u01(i);
    }

    c_int m = Ac.rows();
    c_int n = p01.cols();

    c_int exitflag = 0;

    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    if (data) {
        data->n = n;
        data->m = m;

        data->P =
                csc_matrix(data->n, data->n, p_nnz,CopyData(p_x),
                           CopyData(p_i), CopyData(p_p));
        data->q = CopyData(q);

        data->A =
                csc_matrix(data->m, data->n, A_nnz, CopyData(A_x),
                           CopyData(A_i), CopyData(A_p));

        data->l = CopyData(l);
        data->u = CopyData(u);
    }

    if (settings) {
        osqp_set_default_settings(settings);

        settings->polish   = true;
        settings->verbose  = false;
        settings->max_iter = max_iteration;
        settings->eps_abs  = eps_abs;
        settings->alpha    = 1.0;
    }

    exitflag = osqp_setup(&work, data, settings);

    osqp_solve(work);

    auto status = work->info->status_val;
    if (status < 0 || (status != 1 && status != 2)) {
        cout << "failed optimization status:" << status << endl;

        osqp_cleanup(work);

        if (data) {
            if (data->A) {
                c_free(data->A);
            }

            if (data->P) {
                c_free(data->P);
            }

            c_free(data);
         }

        c_free(settings);

        return -100;
    } else if (work->solution == nullptr) {
        cout << "The solution from OSQP is nullptr" << endl;

        osqp_cleanup(work);

        if (data) {
            if (data->A) {
                c_free(data->A);
            }

            if (data->P) {
                c_free(data->P);
            }

            c_free(data);
        }

        c_free(settings);

        return -100;
    }

    for (int i = 0; i < n; ++i) {
        optimal_solution(i) = work->solution->x[i];
    }

    osqp_cleanup(work);

    if (data) {
        if (data->A) {
            c_free(data->A);
        }
        if (data->P) {
            c_free(data->P);
        }

        c_free(data);
    }

    if (settings) {
        c_free(settings);
    }

    return exitflag;
}

void TrackingMPC::MatrixToCCS(
        MatrixXd matrix_, vector<c_float> *sm_x, c_int &sm_nnz,
        vector<c_int> *sm_i, vector<c_int> *sm_p)
{
    sm_p->emplace_back(0);

    int cols = matrix_.cols(), rows = matrix_.rows(), nz = 0;

    if (cols == rows) { // 方阵且对称，取上三角矩阵元素
        for( int j = 0; j < cols; j++) {
            for (int i = 0; i <= j; i++) {
                if(fabs(matrix_(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_(i,j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    } else if (cols < rows) { // 非方阵，取全部矩阵元素
        for( int j = 0; j < cols; j++) {
            for (int i = 0; i < rows; i++) {
                if(fabs(matrix_(i,j))>0.0000001) {
                    sm_x->emplace_back(matrix_(i,j));
                    sm_i->emplace_back(i);

                    nz++;
                }
            }

            sm_p->emplace_back(nz);
        }
    }

    sm_nnz = nz;
}