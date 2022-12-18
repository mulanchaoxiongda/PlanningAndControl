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

#include "TrackingAlgorithm.h"

TrackingAlgorithm::TrackingAlgorithm(
        RobotModel *p_RobotModel, SaveData *p_savedata)
{
    p_RobotModel_ = p_RobotModel;
    p_savedata_ = p_savedata;
}

void  TrackingAlgorithm::ReadInTrajPoints()
{
    string _string;

    ifstream ReadFile;
    ReadFile.open("../data/TrajectoryPoints.txt", ios::in);

    if (ReadFile.fail()) {
        cout << "[error] failed to open TrajectoryPoints.txt" << endl;
    } else {
        while (getline(ReadFile, _string)) {
            istringstream is(_string);

            double data;

            vector<double> Temp;
            TrajPoint temp;

            while (!is.eof()) {
                is>>data;
                Temp.push_back(data);
            }

            temp.x_ref   = Temp.at(0);
            temp.y_ref   = Temp.at(1);
            temp.yaw_ref = Temp.at(2);
            temp.v_ref   = Temp.at(3);
            temp.w_ref   = Temp.at(4);
            temp.t_ref   = Temp.at(5);

            trajectory_points_.push_back(temp);

            Temp.clear();
            _string.clear();
        }
    }

    ReadFile.close();
    cout << "[INFO] read in reference trajectory points successfully !" << endl;
}

SensorInfo TrackingAlgorithm::GetSensorInfo()
{
    SensorInfo sensor_info;

    memcpy(&sensor_info, &p_RobotModel_->motion_state_, sizeof(SensorInfo));

    return sensor_info;
}

RefPoint TrackingAlgorithm::FindRefPoint(
        vector<TrajPoint> &trajectory_points, SensorInfo &sensor_info)
{
    vector<double> RelativeTime;
    vector<double> fabs_RelativeTime;

    int SizeOfRefTraj = trajectory_points.size();

    for (int i = 0; i < SizeOfRefTraj; i++) {
        double delta_t, fabs_delta_t;

        delta_t = sensor_info.t - trajectory_points.at(i).t_ref;
        fabs_delta_t = -1.0 * fabs(delta_t);

        RelativeTime.push_back(delta_t);
        fabs_RelativeTime.push_back(fabs_delta_t);
    }

    vector<double>::iterator biggest =
            max_element(begin(fabs_RelativeTime), end(fabs_RelativeTime));

    int _ID, ID_RefPoint;

    _ID = distance(begin(fabs_RelativeTime), biggest);

    if (RelativeTime.at(_ID) <= 0.0) {
        ID_RefPoint = _ID - 1;

        if(ID_RefPoint == -1) {
            ID_RefPoint = 0;
        }
    } else {
        ID_RefPoint = _ID;

        if (ID_RefPoint == (SizeOfRefTraj - 1)) {
            ID_RefPoint = SizeOfRefTraj - 2;
        }
    }

    double dis1, dis2, dis_total;

    dis1 = fabs_RelativeTime.at(ID_RefPoint);
    dis2 = fabs_RelativeTime.at(ID_RefPoint + 1);

    dis_total = dis1 + dis2;

    RefPoint reference_point;

    reference_point.x =
            ( trajectory_points.at(ID_RefPoint).x_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).x_ref * dis1 ) /
              dis_total;
    reference_point.y =
            ( trajectory_points.at(ID_RefPoint).y_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).y_ref * dis1 ) /
              dis_total;
    reference_point.yaw =
            ( trajectory_points.at(ID_RefPoint).yaw_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).yaw_ref * dis1 ) /
              dis_total;
    reference_point.v =
            ( trajectory_points.at(ID_RefPoint).v_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).v_ref * dis1 ) /
            dis_total;
    reference_point.w =
            ( trajectory_points.at(ID_RefPoint).w_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).w_ref * dis1 ) /
              dis_total;
    reference_point.t =
            ( trajectory_points.at(ID_RefPoint).t_ref * dis2 +
              trajectory_points.at(ID_RefPoint + 1).t_ref * dis1 ) /
              dis_total;

    p_savedata_->file << "[reference_point] "
                      << " Time "    << reference_point.t
                      << " x_ref "   << reference_point.x
                      << " y_ref "   << reference_point.y
                      << " yaw_ref " << reference_point.yaw
                      << " v_ref "   << reference_point.v
                      << " w_ref "   << reference_point.w
                      << " t "       << reference_point.t << endl;

    return reference_point;
}
