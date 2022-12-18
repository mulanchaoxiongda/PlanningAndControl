#pragma once

#include <vector>

#include "RobotModel.h"
#include "InformationFormat.h"
#include "SaveData.h"

using namespace std;

class RobotModel;
class SaveData;

class TrackingAlgorithm
{
 public:
    TrackingAlgorithm(RobotModel *p_RobotModel, SaveData *p_savedata);
    ~TrackingAlgorithm() {};

    virtual ControlCommand CalControlCommand() = 0;


 protected:
    void ReadInTrajPoints();

    virtual void ReadInControlPara() = 0;

    SensorInfo GetSensorInfo();

    RefPoint FindRefPoint(
             vector<TrajPoint> &trajectory_points, SensorInfo &sensor_info);

    SaveData *p_savedata_;
    RobotModel *p_RobotModel_;

    vector<TrajPoint> trajectory_points_;

    RefPoint reference_point_;
    SensorInfo sensor_info_;
};