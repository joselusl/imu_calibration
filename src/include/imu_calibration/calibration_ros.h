

#ifndef _CALIBRATION_ROS_H
#define _CALIBRATION_ROS_H


// ROS
#include <ros/ros.h>


// IMU Msgs
#include <sensor_msgs/Imu.h>


// Library
#include "imu_calibration/codels.h"



/////////////////////
/// \brief The ImuCalibrationRos class
////////////////////
class ImuCalibrationRos
{
public:
    ImuCalibrationRos(int argc,char **argv);
    ~ImuCalibrationRos();


private:
    ros::NodeHandle* nh;


protected:
    int init();
    int end();


public:
    int open();
    int close();

public:
    int run();



    // Subscriber IMU
protected:
    std::string ImuTopicName;
protected:
    ros::Subscriber ImuTopicSub;
    void imuTopicCallback(const sensor_msgs::ImuConstPtr& msg);
public:
    int setImuTopicName(std::string SensorImuTopicName);



    // Service Start calibration
    // TODO


};



#endif
