//
// Created by peetaa on 10.5.2018.
//

#ifndef DEPTHEST_CALIBRATIONCONFIGURATION_H
#define DEPTHEST_CALIBRATIONCONFIGURATION_H

/**
 * Camera calibration configuration class
 */
class CalibrationConfiguration {
private:
    int cameraWidth;
    int cameraHeight;
    double fx, fy, cx, cy;
    double d0, d1, d2, d3;
public:
    int getCameraWidth() ;

    void setCameraWidth(int cameraWidth);

    int getCameraHeight() ;

    void setCameraHeight(int cameraHeight);

    double getFx() ;

    void setFx(double fx);

    double getFy() ;

    void setFy(double fy);

    double getCx() ;

    void setCx(double cx);

    double getCy() ;

    void setCy(double cy);

    double getD0() ;

    void setD0(double d0);

    double getD1() ;

    void setD1(double d1);

    double getD2() ;

    void setD2(double d2);

    double getD3() ;

    void setD3(double d3);
};


#endif //DEPTHEST_CALIBRATIONCONFIGURATION_H
