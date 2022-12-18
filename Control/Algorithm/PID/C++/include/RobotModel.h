#pragma once

#include "TrackingPID.h"
#include "InformationFormat.h"
#include "SaveData.h"

class TrackingPID;
class SaveData;

class RobotModel
{
    friend class TrackingPID;
    public:
        RobotModel(
                RobotMotionStatePara motion_state,
                double simulation_step, SaveData *p_savedata);
        RobotMotionStatePara UpdateMotionState(ControlCommand control_command);

    private:
        SaveData *p_savedata_;
        RobotMotionStatePara motion_state_;
        double simulation_step_;
};