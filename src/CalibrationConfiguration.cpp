//
// Created by peetaa on 10.5.2018.
//

#include "CalibrationConfiguration.h"

int CalibrationConfiguration::getCameraWidth()  {
    return cameraWidth;
}

void CalibrationConfiguration::setCameraWidth(int cameraWidth) {
    CalibrationConfiguration::cameraWidth = cameraWidth;
}

int CalibrationConfiguration::getCameraHeight()  {
    return cameraHeight;
}

void CalibrationConfiguration::setCameraHeight(int cameraHeight) {
    CalibrationConfiguration::cameraHeight = cameraHeight;
}

double CalibrationConfiguration::getFx()  {
    return fx;
}

void CalibrationConfiguration::setFx(double fx) {
    CalibrationConfiguration::fx = fx;
}

double CalibrationConfiguration::getFy()  {
    return fy;
}

void CalibrationConfiguration::setFy(double fy) {
    CalibrationConfiguration::fy = fy;
}

double CalibrationConfiguration::getCx()  {
    return cx;
}

void CalibrationConfiguration::setCx(double cx) {
    CalibrationConfiguration::cx = cx;
}

double CalibrationConfiguration::getCy()  {
    return cy;
}

void CalibrationConfiguration::setCy(double cy) {
    CalibrationConfiguration::cy = cy;
}

double CalibrationConfiguration::getD0()  {
    return d0;
}

void CalibrationConfiguration::setD0(double d0) {
    CalibrationConfiguration::d0 = d0;
}

double CalibrationConfiguration::getD1()  {
    return d1;
}

void CalibrationConfiguration::setD1(double d1) {
    CalibrationConfiguration::d1 = d1;
}

double CalibrationConfiguration::getD2()  {
    return d2;
}

void CalibrationConfiguration::setD2(double d2) {
    CalibrationConfiguration::d2 = d2;
}

double CalibrationConfiguration::getD3()  {
    return d3;
}

void CalibrationConfiguration::setD3(double d3) {
    CalibrationConfiguration::d3 = d3;
}
