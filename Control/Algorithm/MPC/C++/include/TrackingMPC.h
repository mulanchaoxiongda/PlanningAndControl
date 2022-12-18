#pragma once

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <osqp/osqp.h>

#include "RobotModel.h"
#include "InformationFormat.h"
#include "SaveData.h"
#include "TrackingAlgorithm.h"

using namespace std;
using namespace Eigen;

class RobotModel;
class SaveData;

class TrackingMPC: public TrackingAlgorithm
{
 public:
     TrackingMPC(RobotModel *p_RobotModel, SaveData *p_savedata);
     ~TrackingMPC() {};

     ControlCommand CalControlCommand();

     double running_time_average_;


 private:
     vector<double> q_delta_x_para_, q_delta_y_para_;
     vector<double> q_delta_yaw_para_, q_delta_wz_para_;

     vector<double> r_delta_vc_para_, r_delta_wc_para_, v_para_;

     vector<double> Np_para_, Nc_para_, control_period_para_;

     vector<double> v_min_para_, v_max_para_, dv_min_para_, dv_max_para_;
     vector<double> w_min_para_, w_max_para_, dw_min_para_, dw_max_para_;

     VectorXd u_;
     int Nx_, Nu_;

     double running_time_sum_;

     int loop_counter_;

     void ReadInControlPara();

     void CalControlCoefficient(
             MatrixXd &q, MatrixXd &r, int &Np, int &Nc, double &Control_Period,
             VectorXd &u_min, VectorXd &u_max, VectorXd &du_min, VectorXd &du_max,
             double v_sensor, double call_cycle);

     void UpdateErrorModel(
             VectorXd &X_, MatrixXd &a, MatrixXd &b, double dt);

     void UpdateIncrementModel(
             MatrixXd &A, MatrixXd &B, MatrixXd &C,
             VectorXd x, MatrixXd a, MatrixXd b);

     void PredictFunc(
             MatrixXd &phi, MatrixXd &theta, MatrixXd A,
             MatrixXd B, MatrixXd C, int Np, int Nc);

     void ObjectiveFunc(
             MatrixXd &h_, MatrixXd &e_, VectorXd &g_, MatrixXd kesi_,
             MatrixXd phi_, MatrixXd theta_, MatrixXd q_, MatrixXd r_,
             int Np, int Nc);

     void ConstraintCondition(
             MatrixXd &A, VectorXd &lb, VectorXd &ub,VectorXd U, VectorXd umin,
             VectorXd umax, VectorXd delta_umin, VectorXd delta_umax, int Np, int Nc);

     int  OptimizationSolver(
             VectorXd &optimal_solution, MatrixXd p01, VectorXd q01, MatrixXd Ac,
             VectorXd l01, VectorXd u01, int m01, int n01, c_int max_iteration,
             c_float eps_abs);

     void MatrixToCCS(
             MatrixXd matrix_, vector<c_float> *sm_x, c_int &sm_nnz,
             vector<c_int> *sm_i, vector<c_int> *sm_p);

     template <typename T>
     T *CopyData(const std::vector<T> &vec) {
         // T *data = new T[vec.size()];
         T *data = (T*)c_malloc(vec.size()*sizeof(T));
         memcpy(data, vec.data(), sizeof(T) * vec.size());
         return data;
     }
};