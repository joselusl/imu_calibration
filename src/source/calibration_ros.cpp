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
    flag_imu_calibrated_=false;

    imu_topic_name_="/fuseon/phidgets_imu_1044/imu/data_raw";

    return 0;
}

int ImuCalibrationRos::end()
{
    return 0;
}

int ImuCalibrationRos::open()
{
    // Read parameters
    // TODO


    // Init imu_calibrator_
    imu_calibrator_.init();


    // Subscriber
    imu_topic_sub_=nh->subscribe(imu_topic_name_, 10, &ImuCalibrationRos::imuTopicCallback, this);


    return 0;
}

int ImuCalibrationRos::close()
{
    return 0;
}

int ImuCalibrationRos::run()
{
    // Spin forever
    ros::spin();

    return 0;
}

int ImuCalibrationRos::setImuTopicName(std::string imu_topic_name)
{
    this->imu_topic_name_=imu_topic_name;
    return 0;
}


void ImuCalibrationRos::imuTopicCallback(const sensor_msgs::ImuConstPtr& msg)
{
    if(flag_imu_calibrated_)
        return;

    ImuMeasurement imu_measurement;
    bool flag_new_still_pose_detected=false;
    int32_t still=0;

    // Fill Variable with message
    // Time Stamp
    imu_measurement.ts.sec=msg->header.stamp.sec;
    imu_measurement.ts.nsec=msg->header.stamp.nsec;

    // Acceleration
    imu_measurement.acc._value.ax=msg->linear_acceleration.x;
    imu_measurement.acc._value.ay=msg->linear_acceleration.y;
    imu_measurement.acc._value.az=msg->linear_acceleration.z;
    imu_measurement.acc._present=true;

    // Angular Velocity
    imu_measurement.vel._value.wx=msg->angular_velocity.x;
    imu_measurement.vel._value.wy=msg->angular_velocity.y;
    imu_measurement.vel._value.wz=msg->angular_velocity.z;
    imu_measurement.vel._present=true;

    // Collect data
    int error_collect=imu_calibrator_.collectData(&imu_measurement, still, flag_new_still_pose_detected);

    if(error_collect)
    {
        std::cout<<"error collecting data: "<<error_collect<<std::endl;
        ros::shutdown();
        return;
    }

    if(flag_new_still_pose_detected)
    {
        std::cout<<"Still poses: "<<still<<" out of "<<imu_calibrator_.nposes<<std::endl;
    }

    // Try calibration
    int error_calibration=imu_calibrator_.calibrate();
    if(!error_calibration)
    {
        std::cout<<"IMU calibration succeed!"<<std::endl;
        flag_imu_calibrated_=true;
        std::cout<<imu_calibrator_.imu_calibration_.print()<<std::endl;
        ros::shutdown();
    }

    return;
}



