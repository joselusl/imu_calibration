/*
 * Copyright (c) 2015-2016 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                      Anthony Mallet on Mon Feb 16 2015
 */
#ifndef H_MIKROKOPTER_CODELS
#define H_MIKROKOPTER_CODELS

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>


//I/O stream
//std::cout
#include <iostream>


//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>

#include <Eigen/Dense>




struct or_time_ts
{
    uint32_t sec;
    uint32_t nsec;
};



struct ImuMeasurement
{
    or_time_ts ts;

    struct optional_or_t3d_vel
    {
       bool _present;
       struct or_t3d_vel
       {
        double wx;
        double wy;
        double wz;
       } _value;
    } vel;

    struct optional_or_t3d_acc
    {
       bool _present;
       struct or_t3d_acc
       {
        double ax;
        double ay;
        double az;
       } _value;
    } acc;

};


/* --- local data ---------------------------------------------------------- */

struct mk_calibration_data
{
    // Imu rate (Samples Per Second)
  int32_t sps;

  // Number of required samples per still pose (Samples Still)
  int32_t sstill;

  // Number of required still poses (Number Poses)
  int32_t nposes;

  // All Time stamp
  Eigen::Matrix<double, 1, Eigen::Dynamic> t;

  // All Measurements
  Eigen::Matrix<double, 3, Eigen::Dynamic> gyr;
  Eigen::Matrix<double, 3, Eigen::Dynamic> acc;

  // Number of stored stamples
  int32_t samples;

  // Time Stamp of the last measurement
  or_time_ts ts;

  // To compute acceleration variance over the last second
  Eigen::Matrix<double, 3, 1> sum, sumsq;


  double  accvarth;

  // All Still values
  Eigen::Array<int32_t, 2, Eigen::Dynamic> still;

  // Number of samples since last still detected
  int32_t nstill;
};


class ImuCalibration
{
public:
    ImuCalibration();

    // Accelerometer
protected:
public:
    double ascale[9];
    double abias[3];

    double astddev[3];

    double maxa[3];


    // Gyro
protected:
public:
    double gscale[9];
    double gbias[3];

    double gstddev[3];


    double maxw[3];

public:
    std::string print();

};



static mk_calibration_data *raw_data;

/////////////////////////////////
/// \brief The ImuCalibrator class
////////////////////////////////
class ImuCalibrator
{
public:
    ImuCalibrator();
    ~ImuCalibrator();


    // Data - Input
protected:



    // Configuration parameters
protected:
    // Time still
    double tstill;
    // Number of required poses
public:
    uint32_t nposes;
    // Imu rate
protected:
    double imu_rate;


    // Calibration output
public:
    ImuCalibration imu_calibration_;


    //
protected:
    // Last still pose detected
    int32_t num_still_poses_captured_;



public:
    int init();

public:
    int collectData(ImuMeasurement *imu_data, int32_t& still, bool& flag_new_still_pose_detected);


public:
    int calibrate();




protected:
    int	mk_calibration_collect(ImuMeasurement *imu_data,
                                 int32_t *still);


protected:
    int	mk_calibration_init(uint32_t sstill, uint32_t nposes, uint32_t sps);

protected:
    int	mk_calibration_acc(double ascale[9], double abias[3]);
    int	mk_calibration_gyr(double gscale[9], double gbias[3]);
    void mk_calibration_fini(double stddeva[3], double stddevw[3],
                              double *maxa, double *maxw);


    // Not really needed!
private:
    void mk_calibration_rotate(double r[9], double s[9]);
    void mk_calibration_bias(double b1[3], double s[9], double b[3]);




};



class mk_calibration_acc_errfunc
{
public:

  typedef double Scalar;
  enum
  {
    InputsAtCompileTime = 9,
    ValuesAtCompileTime = Eigen::Dynamic
  };
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> InputType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ValueType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianType;

  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> measurements;
  void measurement(Eigen::Matrix<Scalar, 3, 1> m)
  {
    measurements.conservativeResize(Eigen::NoChange, measurements.cols()+1);
    measurements.col(measurements.cols()-1) << m;
  }

  int operator()(const InputType &theta, ValueType &L) const
  {
    Eigen::Matrix<Scalar, 3, 3> S;
    Eigen::Matrix<Scalar, 3, 1> b;
    int32_t i;

    S <<
      theta(3),  theta(0) * theta(4),  theta(1) * theta(5),
            0.,             theta(4),  theta(2) * theta(5),
            0.,                   0.,             theta(5);
    b <<
      theta(6),
      theta(7),
      theta(8);

    for(i = 0; i < measurements.cols(); i++)
      L(i) = 9.81*9.81 - (S * (measurements.col(i) + b)).squaredNorm();

    return 0;
  }

  int inputs() const { return InputsAtCompileTime; }
  int values() const { return measurements.cols(); }
};




class mk_calibration_gyr_errfunc
{
public:
  typedef double Scalar;
  enum
  {
    InputsAtCompileTime = 9,
    ValuesAtCompileTime = Eigen::Dynamic
  };
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> InputType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ValueType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianType;

  Eigen::Matrix<double, 3, Eigen::Dynamic> acc_dir;

  int operator()(const InputType &theta, ValueType &L) const
  {
    Eigen::Quaternion<double> q(Eigen::Quaternion<double>::Identity());
    Eigen::Quaternion<double> omega_q;
    Eigen::Matrix<double, 3, 3> S;
    Eigen::Matrix<double, 3, 1> w;
    int32_t i, k;
    double dt, a;

    S <<
                 theta(6),  theta(0) * theta(7),  theta(1) * theta(8),
      theta(2) * theta(6),             theta(7),  theta(3) * theta(8),
      theta(4) * theta(6),  theta(5) * theta(7),             theta(8);

    for(i = 0; i < raw_data->still.cols()-1; i++)
    {

      /* integrate gyro over the ith motion interval */
      q = Eigen::Quaternion<double>::Identity();
      for(k = raw_data->still(0, i); k <= raw_data->still(1, i+1); k++)
      {
        dt = raw_data->t(k) - raw_data->t(k-1);
        w.noalias() = dt * (S * raw_data->gyr.col(k));
        a = w.norm();

        if (a < 1e-3) {
          omega_q.w() = 1 - a*a/8 /*std::cos(a/2)*/;
          omega_q.vec() = - (0.5 - a*a/48 /*std::sin(a/2)/a*/) * w;
        } else {
          omega_q.w() = std::cos(a/2);
          omega_q.vec() = - std::sin(a/2)/a * w;
        }

        q = omega_q * q;
      }

      /* compute ith error */
      L.block<3, 1>(3*i, 0) =
        acc_dir.col(i+1) - q._transformVector(acc_dir.col(i));
    }

    return 0;
  }

  int inputs() const { return InputsAtCompileTime; }
  int values() const { return 3 * (raw_data->still.cols() - 1); }
};









#endif /* H_MIKROKOPTER_CODELS */
