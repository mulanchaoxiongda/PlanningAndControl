#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include "RobotModel.h"
#include "TrackingPID.h"

using namespace std;

class SaveData
{
    public:
        SaveData(const char* FullFilePath);
        ~SaveData();

        ofstream file;
};