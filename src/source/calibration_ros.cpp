#include "imu_calibration/calibration_ros.h"


ImuCalibrationRos::ImuCalibrationRos(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());

    // Node handle
    nh=new ros::NodeHandle();

    init();

    return;
}

ImuCalibrationRos::~ImuCalibrationRos()
{
    end();

    return;
}

int ImuCalibrationRos::init()
{
    return 0;
}

int ImuCalibrationRos::end()
{
    return 0;
}

int ImuCalibrationRos::open()
{

    // Subscriber
    ImuTopicSub=nh->subscribe(ImuTopicName, 10, &ImuCalibrationRos::imuTopicCallback, this);


    return 0;
}

int ImuCalibrationRos::close()
{
    return 0;
}

int ImuCalibrationRos::run()
{
    // Spin forever
    while(ros::ok())
        ros::spin();

    return 0;
}

int ImuCalibrationRos::setImuTopicName(std::string ImuTopicName)
{
    this->ImuTopicName=ImuTopicName;
    return 0;
}


void ImuCalibrationRos::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{


    return;
}



