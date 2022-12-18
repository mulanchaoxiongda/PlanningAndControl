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

#include "TrackingPID.h"
#include "CustomFunction.h"

using namespace std;

//Completed
TrackingPID::TrackingPID(RobotModel *p_RobotModel, SaveData *p_savedata)
{
    p_RobotModel_ = p_RobotModel;
    p_savedata_ = p_savedata;

    ReadInTrajPoints();
    ReadInControlPara();

    vc_i_ = 0.0;
    wc_i_ = 0.0;

    control_period_ = 0.02;
    gate_vc_begin_ = GateVcBegin::no;
}

//Completed
ControlCommand TrackingPID::CalculateControlCommand()
{
    SensorInfo sensor_info;
    sensor_info = GetSensorInfo();

    RefPoint reference_point = {0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
    reference_point = FindRefPoint(traj_points_, sensor_info);

    TrackingErr tracking_error;
    tracking_error = CalTrackingError(reference_point, sensor_info);

    double kp_v, ki_v, kd_v, kp_w, ki_w, kd_w;
    CalControlCoefficient(
            kp_v, ki_v, kd_v,kp_w, ki_w, kd_w, sensor_info.v);

    double speed_command = CalSpeedCmd(tracking_error,
                                       reference_point,
                                       kp_v, ki_v, kd_v);

    double yaw_ratio_command = CalYawrateCmd(tracking_error,
                                             reference_point,
                                             kp_w, ki_w, kd_w);

    ControlCommand control_command =
            {speed_command, yaw_ratio_command, tracking_error.t};

    p_savedata_->file << "[control_command] "
                      << "Time " << control_command.t              << "  "
                      << "vc "   << control_command.SpeedCommand   << "  "
                      << "wc "   << control_command.YawRateCommand << "  "
                      << "t "    << control_command.t              << endl;

    cout << " [DBG] "
         << " t "      << fixed << setw(5) << setprecision(2) << tracking_error.t
         << " err_sx " << fixed << setw(6) << setprecision(3) << tracking_error.err_sx
         << " err_sy " << fixed << setw(6) << setprecision(3) << tracking_error.err_sy
         << " x_sen "  << fixed << setw(6) << setprecision(3) << sensor_info.x
         << " y_sen "  << fixed << setw(6) << setprecision(3) << sensor_info.y
         << " x_ref "  << fixed << setw(6) << setprecision(4) << reference_point.x
         << " y_ref "  << fixed << setw(5) << setprecision(4) << reference_point.y
         << " vc "     << fixed << setw(6) << setprecision(4) << speed_command
         << " wc "     << fixed << setw(5) << setprecision(4) << yaw_ratio_command << endl;

    return control_command;
}

//Completed
void  TrackingPID::ReadInTrajPoints()
{
    string string_;

    ifstream ReadFile;
    ReadFile.open("../data/TrajectoryPoints.txt",ios::in);

    if (ReadFile.fail()) {
        cout << "[error] failed to open TrajectoryPoints.txt" << endl;
    } else {
        while (getline(ReadFile,string_)) {
            istringstream is(string_);

            double data_;
            vector<double> Temp;
            TrajPoint temp;

            while (!is.eof()) {
                is >> data_;
                Temp.push_back(data_);
            }

            temp.x_ref   = Temp.at(0);
            temp.y_ref   = Temp.at(1);
            temp.yaw_ref = Temp.at(2);
            temp.v_ref   = Temp.at(3);
            temp.w_ref   = Temp.at(4);
            temp.t_ref   = Temp.at(5);

            traj_points_.push_back(temp);

            Temp.clear();
            string_.clear();
        }
    }

    ReadFile.close();

    cout << "[INFO] read in reference trajectory points successfully !" << endl;

    /* cout << "[DBG]" << "traj_points_ : " << endl;
    for (long unsigned int i = 0; i<traj_points_.size(); i++) {
        cout << "[DBG]"
             << traj_points_.at(i).x_ref   << "  "
             << traj_points_.at(i).y_ref   << "  "
             << traj_points_.at(i).yaw_ref << "  "
             << traj_points_.at(i).v_ref   << "  "
             << traj_points_.at(i).w_ref   << "  "
             << traj_points_.at(i).t_ref   << "  " << endl;
    } */
}

//Completed
SensorInfo TrackingPID::GetSensorInfo()
{
    SensorInfo sensor_info;

    memcpy(&sensor_info, &p_RobotModel_->motion_state_, sizeof(SensorInfo));

    return sensor_info;
}

//Completed
RefPoint TrackingPID::FindRefPoint(
        vector<TrajPoint> &traj_points, SensorInfo &sensor_info)
{
    vector<double> RelativeTime;
    vector<double> fabs_RelativeTime;

    int SizeOfRefTraj = traj_points.size();

    for (int i=0; i < SizeOfRefTraj; i++) {
        double delta_t, fabs_delta_t;

        delta_t = sensor_info.t - traj_points.at(i).t_ref;
        fabs_delta_t = -1.0 * fabs(delta_t);

        RelativeTime.push_back(delta_t);
        fabs_RelativeTime.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest = max_element(begin(fabs_RelativeTime),
                                                   end(fabs_RelativeTime));

    int _ID, ID_RefPoint;
    _ID = distance(begin(fabs_RelativeTime), biggest);

    if (RelativeTime.at(_ID) <= 0.0) {
        ID_RefPoint = _ID-1;

        if (ID_RefPoint == -1) {
            ID_RefPoint = 0;
        }
    } else {
        ID_RefPoint = _ID;

        if (ID_RefPoint == (SizeOfRefTraj - 1))
            ID_RefPoint = SizeOfRefTraj - 2;
    }

    double dis1, dis2, dis_total;

    dis1 = fabs_RelativeTime.at(ID_RefPoint);
    dis2 = fabs_RelativeTime.at(ID_RefPoint + 1);
    dis_total = dis1 + dis2;

    RefPoint ref_point;

    ref_point.x   =
            (traj_points.at(ID_RefPoint).x_ref * dis2 +
             traj_points.at(ID_RefPoint + 1).x_ref * dis1) /dis_total;
    ref_point.y   = (traj_points.at(ID_RefPoint).y_ref * dis2 +
            traj_points.at(ID_RefPoint + 1).y_ref * dis1) / dis_total;
    ref_point.yaw = (traj_points.at(ID_RefPoint).yaw_ref * dis2 +
            traj_points.at(ID_RefPoint + 1).yaw_ref * dis1) / dis_total;
    ref_point.v   = (traj_points.at(ID_RefPoint).v_ref * dis2 +
            traj_points.at(ID_RefPoint + 1).v_ref * dis1) / dis_total;
    ref_point.w   = (traj_points.at(ID_RefPoint).w_ref * dis2 +
            traj_points.at(ID_RefPoint + 1).w_ref * dis1) / dis_total;

    p_savedata_->file << "[reference_point] "
                      << "Time "    << ref_point.t   << "  "
                      << "x_ref "   << ref_point.x   << "  "
                      << "y_ref "   << ref_point.y   << "  "
                      << "yaw_ref " << ref_point.yaw << "  "
                      << "v_ref "   << ref_point.v   << "  "
                      << "w_ref "   << ref_point.w   << "  "
                      << "t "       << ref_point.t   << endl;

    return ref_point;
}

//Completed
TrackingErr TrackingPID::CalTrackingError(
        RefPoint &ref_point, SensorInfo &sensor_info)
{
    using namespace CustomFunction;

    vector<double> direction_reference = {cos(ref_point.yaw), sin(ref_point.yaw)};

    vector<double> vecrtor_robotposition2refpoint =
            {ref_point.x-sensor_info.x, ref_point.y - sensor_info.y};

    double err_yaw, err_sx, err_sy;

    err_sx = direction_reference * vecrtor_robotposition2refpoint;
    err_sy = twoDCrossProd(direction_reference, vecrtor_robotposition2refpoint);
    err_yaw = AngleLimiting(ref_point.yaw - sensor_info.yaw);

    TrackingErr tracking_err = {err_sx, err_sy, err_yaw, sensor_info.t};

    p_savedata_->file << " [tracking_error] "
                      << " Time "    << tracking_err.t
                      << " err_sx "  << tracking_err.err_sx
                      << " err_sy "  << tracking_err.err_sy
                      << " err_yaw " << tracking_err.err_yaw
                      << " t "       << tracking_err.t << endl;

    return tracking_err;
}

//Completed
void TrackingPID::ReadInControlPara()
{
    vector<vector<double>> control_para;

    CustomFunction::txt_to_vectordouble(control_para, "../data/ControlPara.txt");

    v_para_vc_  = control_para[0];
    kp_para_vc_ = control_para[1];
    ki_para_vc_ = control_para[2];
    kd_para_vc_ = control_para[3];

    v_para_wc_  = control_para[4];
    kp_para_wc_ = control_para[5];
    ki_para_wc_ = control_para[6];
    kd_para_wc_ = control_para[7];
}

//Comoleted
void TrackingPID::CalControlCoefficient(
        double &kp_v, double &ki_v, double &kd_v,
        double &kp_w, double &ki_w, double &kd_w, double v_sensor)
{
    kp_v = CustomFunction::interp_linear(v_para_vc_, kp_para_vc_, v_sensor); // 查表控制参数
    ki_v = CustomFunction::interp_linear(v_para_vc_, ki_para_vc_, v_sensor);
    kd_v = CustomFunction::interp_linear(v_para_vc_, kd_para_vc_, v_sensor);
    kp_w = CustomFunction::interp_linear(v_para_wc_, kp_para_wc_, v_sensor);
    ki_w = CustomFunction::interp_linear(v_para_wc_, ki_para_wc_, v_sensor);
    kd_w = CustomFunction::interp_linear(v_para_wc_, kd_para_wc_, v_sensor); // 阻尼回路系数

    /* kp_v = 2.0; // 传动比控制参数
    ki_v = 0.0;
    kd_v = 0.0;
    kp_w = 2.0;
    ki_w = 0.01;
    kd_w = 2.5; // 微分项系数 */
}

//Completed
double TrackingPID::CalSpeedCmd(
        TrackingErr tracking_err, RefPoint ref_point,
        double kp, double ki, double kd)
{
    if (gate_vc_begin_ == GateVcBegin::no) {
        err_sx_pre_ = tracking_err.err_sx;
        gate_vc_begin_ = GateVcBegin::yes;
    }

    double dis_i = 0.0;

    if (fabs(ref_point.v) >= 1.0) {
        dis_i = 0.05;
    } else if (fabs(ref_point.v) >= 0.5) {
        dis_i = 0.01 + (0.05 - 0.01) * (fabs(dis_i) - 0.5) / (1.0 - 0.5);
    } else {
        dis_i = 0.01;
    }

    if (fabs(tracking_err.err_sx)<=dis_i) {
        ki = ki;
    } else if (fabs(tracking_err.err_sx) <= dis_i + 0.01) {
        ki = ki - ki * (fabs(tracking_err.err_sx) - dis_i) / 0.01;
    } else {
        ki = 0.0;
    }

    double gate_ki = 1.0;

    if (fabs(tracking_err.err_sx) <= dis_i) {
        gate_ki = 1.0;
    } else if (fabs(tracking_err.err_sx) <= dis_i + 0.01) {
        gate_ki = 1.0 - 1.0 * (fabs(tracking_err.err_sx) - dis_i) / 0.01;
    }
    else {
        gate_ki = 0.0;
    }

    vc_i_ = vc_i_ + ki * tracking_err.err_sx * control_period_;
    vc_i_ = vc_i_ * gate_ki;

    double vc =
            kp * tracking_err.err_sx + vc_i_ +
            kd * (tracking_err.err_sx - err_sx_pre_) /
            control_period_ + ref_point.v;

    err_sx_pre_ = tracking_err.err_sx;

    return vc;
}

//Completed
double TrackingPID::CalYawrateCmd(
        TrackingErr tracking_err, RefPoint ref_point,
        double kp, double ki, double kd)
{
    double dis_i = 0.0;

    if (fabs(ref_point.w) >= 15.0/57.3) {
        dis_i = 0.055;
    } else if (fabs(ref_point.w) >= 5.0/57.3) {
        dis_i =
                0.015 + (0.055 - 0.015) * (fabs(dis_i) - 5.0 / 57.3) /
                (15.0 / 57.3 - 5.0 / 57.3);
    } else {
        dis_i = 0.015;
    }

    if (fabs(tracking_err.err_sy) <= dis_i) {
        ki = ki;
    }
    else if (fabs(tracking_err.err_sy) <= dis_i + 0.02) {
        ki = ki - ki * (fabs(tracking_err.err_sy) - dis_i) / 0.02;
    }
    else {
        ki = 0.0;
    }

    double gate_ki = 1.0;

    if (fabs(tracking_err.err_sy) <= dis_i) {
        gate_ki = 1.0;
    }
    else if (fabs(tracking_err.err_sy) <= dis_i + 0.01) {
        gate_ki = 1.0 - 1.0 * (fabs(tracking_err.err_sy) - dis_i) / 0.01;
    }
    else {
        gate_ki = 0.0;
    }

    wc_i_ = wc_i_ + ki * tracking_err.err_sy * control_period_;
    wc_i_ = wc_i_ * gate_ki;

    double wzc =
            kp * tracking_err.err_sy  + wc_i_ +
            kd * tracking_err.err_yaw + ref_point.w;

    return wzc;
}