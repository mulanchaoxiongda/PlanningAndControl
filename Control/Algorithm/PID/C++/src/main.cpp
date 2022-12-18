#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <string>

#include "SaveData.h"
#include "RobotModel.h"
#include "TrackingPID.h"
#include "TrajectoryCreator.h"

using namespace std;

 int main(int argc, char **argv)
 {
     const char* full_result_path = "../tools/TrackingResult.txt";

     SaveData save_result(full_result_path);
     const char* full_trajectory_reference_path = "../data/TrajectoryPoints.txt";

     SaveData save_trajectory_reference(full_trajectory_reference_path);

     TrajectoryCreator trajectory_creator(&save_trajectory_reference, &save_result);
     trajectory_creator.TrajCreator();

     RobotMotionStatePara motion_state = {0.05, 0.05, 135.0/57.3, 0.0, 0.0, 0.0}; // x, y, yaw, v, w, time

     double simulation_step = 0.01;

     RobotModel robot_model_(motion_state, simulation_step, &save_result);

     TrackingPID tracking_pid_(&robot_model_, &save_result);

     ControlCommand control_command = {0.0, 0.0, 0.0}; // vc, wc, time

     double time = 0.0, control_period = 0.02;

     int Num = round(control_period / simulation_step), loop_counter = 0;

     while (time <= 20.0) {
         if (loop_counter % Num == 0) {
            control_command = tracking_pid_.CalculateControlCommand();
        }

         robot_model_.UpdateMotionState(control_command);

         time = time + simulation_step;
         loop_counter = loop_counter + 1;
     }

     return 0;
 }