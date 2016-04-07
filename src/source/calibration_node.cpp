
#include "imu_calibration/calibration_ros.h"


int main(int argc,char **argv)
{

    ImuCalibrationRos TheImuCalibration(argc, argv);

    // Open
    TheImuCalibration.open();

    // Run
    TheImuCalibration.run();

    return 0;
}
