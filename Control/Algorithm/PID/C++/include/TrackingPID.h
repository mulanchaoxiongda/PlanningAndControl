#pragma once

#include <vector>
#include "RobotModel.h"
#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;

class RobotModel;
class SaveData;

class TrackingPID
{
 public:
     TrackingPID(RobotModel *p_RobotModel, SaveData *p_savedata);
     ControlCommand CalculateControlCommand();


 private:
     SaveData *p_savedata_;
     RobotModel *p_RobotModel_;

     vector<TrajPoint> traj_points_;

     std::vector<double> kp_para_vc_, ki_para_vc_, kd_para_vc_, v_para_vc_;
     std::vector<double> kp_para_wc_, ki_para_wc_, kd_para_wc_, v_para_wc_;

     double vc_i_, wc_i_, err_sx_pre_, control_period_;

     enum class GateVcBegin {yes, no} gate_vc_begin_;

     void ReadInTrajPoints();

     SensorInfo GetSensorInfo();

     RefPoint FindRefPoint(
            vector<TrajPoint> &traj_points, SensorInfo &sensor_info);

     TrackingErr CalTrackingError(RefPoint &ref_point, SensorInfo &sensor_info);

     void ReadInControlPara();

     void CalControlCoefficient(
            double &kp_v, double &ki_v, double &kd_v,
            double &kp_w, double &ki_w, double &kd_w, double v_sensor);

     double CalSpeedCmd(
            TrackingErr tracking_err, RefPoint ref_point,
            double kp, double ki, double kd);

     double CalYawrateCmd(
            TrackingErr tracking_err, RefPoint ref_point,
            double kp, double ki, double kd);

};