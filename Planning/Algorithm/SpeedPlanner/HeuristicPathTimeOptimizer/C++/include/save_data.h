#pragma once

#include <fstream>
#include <iostream>
#include <string>

class SaveData {
    public:
        SaveData() = delete;
        SaveData(const char* FilePath);
        ~SaveData();

        std::ofstream file;
};
