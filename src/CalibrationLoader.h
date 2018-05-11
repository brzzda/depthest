//
// Created by peetaa on 10.5.2018.
//

#ifndef DEPTHEST_CALIBRATIONLOADER_H
#define DEPTHEST_CALIBRATIONLOADER_H

//#include <bits/basic_string.h>
#include <fstream>
#include <sstream>
#include "CalibrationConfiguration.h"
/**
 * calibration file loader
 */
class CalibrationLoader {
private:
    CalibrationConfiguration calibrationConfiguration;

    void readFileLine(std::string &line, std::ifstream &calibFile,
                      std::string &paramName, double &value,
                      std::stringstream &ss) const;
public:
    explicit CalibrationLoader(std::string fileName);

    CalibrationConfiguration getConfiguration();
};


#endif //DEPTHEST_CALIBRATIONLOADER_H