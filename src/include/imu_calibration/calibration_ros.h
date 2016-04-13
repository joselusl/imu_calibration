

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
    //int stop();



    // Subscriber IMU
protected:
    std::string imu_topic_name_;
protected:
    ros::Subscriber imu_topic_sub_;
    void imuTopicCallback(const sensor_msgs::ImuConstPtr& msg);
public:
    int setImuTopicName(std::string imu_topic_name);



    // Service Start calibration
    // TODO



    // Calibration
protected:
    bool flag_imu_calibrated_;
    ImuCalibrator imu_calibrator_;

};



#endif
