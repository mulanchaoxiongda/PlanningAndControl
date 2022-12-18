#pragma once

#include "InformationFormat.h"
#include "SaveData.h"

class SaveData;

class TrajectoryCreator
{
 public:
     TrajectoryCreator(SaveData *p_save_traj, SaveData *p_save_result);
        void TrajCreator();

 private:
     SaveData *p_save_result_;
     SaveData *p_save_traj_ref_;
};