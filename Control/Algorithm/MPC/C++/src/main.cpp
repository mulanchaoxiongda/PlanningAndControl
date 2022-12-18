#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <string>
#include <sys/time.h>

#include "SaveData.h"
#include "RobotModel.h"
#include "TrackingMPC.h"
#include "TrajectoryCreator.h"

using namespace std;

int main(int argc, char **argv)
{
    struct timeval t_start, t_end;
    gettimeofday(&t_start,NULL);

    const char* full_result_path = "../tools/SimulationResult.txt";
    SaveData save_result(full_result_path);

    const char* full_trajectory_reference_path = "../data/TrajectoryPoints.txt";
    SaveData save_trajectory_reference(full_trajectory_reference_path);

    TrajectoryCreator TrajCreator(&save_trajectory_reference, &save_result);
    TrajCreator.TrajCreator();

    RobotMotionStatePara motion_state = {0.00, -0.015, -0.0/57.3, // x, y, yaw
                                         0.0,   0.0,   0.0}; // v, w, time

    double simulation_step = 0.01;

    RobotModel robot_model_(motion_state, simulation_step, &save_result);

    TrackingMPC tracking_mpc(&robot_model_, &save_result);

    ControlCommand control_command = {0.0, 0.0, 0.0};

    double time = 0.0, control_period = 0.02;
    int Num = round(control_period / simulation_step), loop_counter = 0;

    while (time <= 2.5+0.5) {
        if (loop_counter % Num == 0) {
            control_command = tracking_mpc.CalControlCommand();
        }

        robot_model_.UpdateMotionState(control_command);

        time = time + simulation_step;
        loop_counter = loop_counter + 1;
    }

    gettimeofday(&t_end,NULL);

    cout << "[tracking] MPC run time(average): " << tracking_mpc.running_time_average_ * 1000.0
            << "ms." <<endl;

    cout << "Program run time: " << (double)(t_end.tv_sec - t_start.tv_sec) * 1000.0 +
                                    (double)(t_end.tv_usec - t_start.tv_usec) / 1000.0
         << "ms." << endl;

    return 0;
}