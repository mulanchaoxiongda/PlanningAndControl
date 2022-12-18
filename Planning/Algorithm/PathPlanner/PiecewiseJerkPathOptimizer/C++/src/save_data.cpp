#include "save_data.h"

SaveData::SaveData(const char* FilePath) {
    file.open(FilePath, std::ios::out);

    file.precision(8);
    file.flags(std::ios::right | std::ios::fixed);
    file.fill('0');
}

SaveData::~SaveData() {
    file.close();
}
