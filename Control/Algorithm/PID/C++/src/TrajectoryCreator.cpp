#include <iostream>
#include <math.h>
#include <iomanip>

#include "TrajectoryCreator.h"

TrajectoryCreator::TrajectoryCreator(
        SaveData *p_save_traj_ref, SaveData *p_save_result)
{
    p_save_traj_ref_ = p_save_traj_ref;
    p_save_result_ = p_save_result;
}

void TrajectoryCreator::TrajCreator()
{
    TrajPoint traj_point = {0.0, 0.0, 135.0/57.3, 0.0, 0.0, 0.0}; // x, y, yaw, v, w, t

    p_save_result_->file << " Time " << traj_point.t_ref   << "  "
                         << "[reference_trajectory] "
                         << " x "    << traj_point.x_ref   << "  "
                         << " y "    << traj_point.y_ref   << "  "
                         << " yaw "  << traj_point.yaw_ref << "  "
                         << " v "    << traj_point.v_ref   << "  "
                         << " w "    << traj_point.w_ref   << "  "
                         << " t "    << traj_point.t_ref   << endl;

    p_save_traj_ref_->file << traj_point.x_ref   << "  "
                           << traj_point.y_ref   << "  "
                           << traj_point.yaw_ref << "  "
                           << traj_point.v_ref   << "  "
                           << traj_point.w_ref   << "  "
                           << traj_point.t_ref   << endl;

    while (traj_point.t_ref <= 20.0) {
        double speed_command, yaw_ratio_command, radius,
               simulation_step = 0.05;

        speed_command =
                1.3 + 0.05 * sin(2 * M_PI * 0.2 * traj_point.t_ref) * 1.0;
        radius = 1.5;
        yaw_ratio_command = speed_command / radius * 1.0;

        traj_point.x_ref =
                traj_point.x_ref + traj_point.v_ref *
                cos(traj_point.yaw_ref) * simulation_step;
        traj_point.y_ref =
                traj_point.y_ref + traj_point.v_ref *
                sin(traj_point.yaw_ref) * simulation_step;
        traj_point.yaw_ref =
                traj_point.yaw_ref + traj_point.w_ref*simulation_step;
        traj_point.v_ref = speed_command;
        traj_point.w_ref = yaw_ratio_command;
        traj_point.t_ref = traj_point.t_ref + simulation_step;

        p_save_result_->file << " Time " << traj_point.t_ref   << "  "
                             << "[reference_trajectory] "
                             << " x "    << traj_point.x_ref   << "  "
                             << " y "    << traj_point.y_ref   << "  "
                             << " yaw "  << traj_point.yaw_ref << "  "
                             << " v "    << traj_point.v_ref   << "  "
                             << " w "    << traj_point.w_ref   << "  "
                             << " t "    << traj_point.t_ref   << endl;

        p_save_traj_ref_->file << traj_point.x_ref   << "  "
                               << traj_point.y_ref   << "  "
                               << traj_point.yaw_ref << "  "
                               << traj_point.v_ref   << "  "
                               << traj_point.w_ref   << "  "
                               << traj_point.t_ref   << endl;
    }

    cout << "[INFO] creat trajectory successfully !" << endl;
}