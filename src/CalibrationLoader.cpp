//
// Created by peetaa on 10.5.2018.
//

#include <iostream>
#include "CalibrationLoader.h"

CalibrationLoader::CalibrationLoader(std::string fileName) {
    std::string line;
    std::ifstream calibFile(fileName);
    std::cout << calibFile.is_open() << std::endl;
    std::string paramName, temp;
    double value;

    getline(calibFile,line);
    std::stringstream ss(line);
    getline(ss, temp);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setCameraWidth(static_cast<int>(value));

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setCameraHeight(static_cast<int>(value));

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setFx(value);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setFy(value);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setCx(value);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setCy(value);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setD0(value);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setD1(value);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setD2(value);

    readFileLine(line, calibFile, paramName, value, ss);
    calibrationConfiguration.setD3(value);
}

void CalibrationLoader::readFileLine(std::string &line, std::ifstream &calibFile, std::string &paramName, double &value,
                                     std::stringstream &ss) const {
    getline(calibFile, line);
    ss = std::stringstream(line);
    ss >> paramName >> value;
}

CalibrationConfiguration CalibrationLoader::getConfiguration() {
    return calibrationConfiguration;
}
