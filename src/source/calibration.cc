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
 *                                      Anthony Mallet on Wed Jan 27 2016
 */
//#include "imu_calibration/acmikrokopter.h"

#include <cfloat>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

#include "imu_calibration/codels.h"

/*
 * This file implements the work described in:
 *
 * Tedaldi, D.; Pretto, A.; Menegatti, E., "A robust and easy to implement
 * method for IMU calibration without external equipments," in 2014 IEEE
 * International Conference on Robotics and Automation (ICRA), pp.3042-3049,
 * May 31 2014-June 7 2014.
 */


//static mk_calibration_data *raw_data;





ImuCalibration::ImuCalibration()
{

      this->gscale[0] = 1.;
      this->gscale[1] = 0.;
      this->gscale[2] = 0.;
      this->gscale[3] = 0.;
      this->gscale[4] = 1.;
      this->gscale[5] = 0.;
      this->gscale[6] = 0.;
      this->gscale[7] = 0.;
      this->gscale[8] = 1.;
      this->gbias[0] = 0.;
      this->gbias[1] = 0.;
      this->gbias[2] = 0.;
      this->gstddev[0] = 1e-1;
      this->gstddev[1] = 1e-1;
      this->gstddev[2] = 1e-1;
    this->maxw[0]=0;
    this->maxw[1]=0;
    this->maxw[2]=0;

      this->ascale[0] = 1.;
      this->ascale[1] = 0.;
      this->ascale[2] = 0.;
      this->ascale[3] = 0.;
      this->ascale[4] = 1.;
      this->ascale[5] = 0.;
      this->ascale[6] = 0.;
      this->ascale[7] = 0.;
      this->ascale[8] = 1.;
      this->abias[0] = 0.;
      this->abias[1] = 0.;
      this->abias[2] = 0.;
      this->astddev[0] = 5e-1;
      this->astddev[1] = 5e-1;
      this->astddev[2] = 5e-1;
    this->maxa[0]=0;
    this->maxa[1]=0;
    this->maxa[2]=0;

      //ids->imu_calibration_updated = true;



    return;
}

std::string ImuCalibration::print()
{
    std::ostringstream imu_calibration_results;

    // Gyro:
    imu_calibration_results<<"gyro: "<<std::endl;
    imu_calibration_results<<"-biases: "<<this->gbias[0]<<"; "<<this->gbias[1]<<"; "<<this->gbias[2]<<std::endl;
    imu_calibration_results<<"-sensitivity: "<<std::endl;
    imu_calibration_results<<"\t"<<this->gscale[0]<<" "<<this->gscale[1]<<" "<<this->gscale[2]<<";"<<std::endl;
    imu_calibration_results<<"\t"<<this->gscale[3]<<" "<<this->gscale[4]<<" "<<this->gscale[5]<<";"<<std::endl;
    imu_calibration_results<<"\t"<<this->gscale[6]<<" "<<this->gscale[7]<<" "<<this->gscale[8]<<std::endl;
    imu_calibration_results<<"-stddev: "<<this->gstddev[0]<<"; "<<this->gstddev[1]<<"; "<<this->gstddev[2]<<std::endl;

    imu_calibration_results<<"-calibration max angular velocity: x "<<maxw[0] * 180./M_PI<<"⁰/s, y "<<maxw[1] * 180./M_PI<<"⁰/s, z "<<maxw[2] * 180./M_PI<<"⁰/s"<<std::endl;


    // Accelerometer:
    imu_calibration_results<<"accel: "<<std::endl;
    imu_calibration_results<<"-biases: "<<this->abias[0]<<"; "<<this->abias[1]<<"; "<<this->abias[2]<<std::endl;
    imu_calibration_results<<"-sensitivity: "<<std::endl;
    imu_calibration_results<<"\t"<<this->ascale[0]<<" "<<this->ascale[1]<<" "<<this->ascale[2]<<";"<<std::endl;
    imu_calibration_results<<"\t"<<this->ascale[3]<<" "<<this->ascale[4]<<" "<<this->ascale[5]<<";"<<std::endl;
    imu_calibration_results<<"\t"<<this->ascale[6]<<" "<<this->ascale[7]<<" "<<this->ascale[8]<<std::endl;
    imu_calibration_results<<"-stddev: "<<this->astddev[0]<<"; "<<this->astddev[1]<<"; "<<this->astddev[2]<<std::endl;

    imu_calibration_results<<"-calibration max acceleration: x "<<maxa[0]<<"m/s², y "<<maxa[1]<<"m/s², z "<<maxa[2]<<"m/s²"<<std::endl;


    return imu_calibration_results.str();

}





ImuCalibrator::ImuCalibrator()
{

    // Time Still
    tstill=1.5;

    // Number of required poses
    nposes=15;

    // Imu rate
    imu_rate=250;




    //flag_new_still_pose_detected_=false;
    num_still_poses_captured_=0;




}

ImuCalibrator::~ImuCalibrator()
{
    return;
}



int ImuCalibrator::init()
{

    uint32_t sps;
    int s;

    sps = imu_rate;

    // Number of samples still
    uint32_t sstill=static_cast<uint32_t>(tstill * sps);

    s = mk_calibration_init(sstill, nposes, sps);

    if (s)
    {
        errno = s;
        return -1;
    }

    return 0;

}

int ImuCalibrator::collectData(ImuMeasurement *imu_data, int32_t& still, bool& flag_new_still_pose_detected)
{
    //
    flag_new_still_pose_detected=false;

    // Collect data
    int error_collect=mk_calibration_collect(imu_data, &still);

    if(!error_collect)
    {
        if(num_still_poses_captured_<still)
        {
            num_still_poses_captured_=still;
            flag_new_still_pose_detected=true;
        }
    }

    return error_collect;
}

int ImuCalibrator::calibrate()
{
    if(raw_data->still.cols() < raw_data->nposes)
    {
        return -1;
    }
    else
    {
        std::cout<<"Calibrating IMU!"<<std::endl;
    }


  int s;

  s = mk_calibration_acc(imu_calibration_.ascale, imu_calibration_.abias);
  if (s)
  {
    mk_calibration_fini(NULL, NULL, NULL, NULL);
    errno = s;
    return -1;
  }

  s = mk_calibration_gyr(imu_calibration_.gscale, imu_calibration_.gbias);
  if (s)
  {
    mk_calibration_fini(NULL, NULL, NULL, NULL);
    errno = s;
    return -1;
  }

  mk_calibration_fini(imu_calibration_.astddev, imu_calibration_.gstddev,
                      imu_calibration_.maxa, imu_calibration_.maxw);


  return 0;
}



/* --- mk_calibration_init ------------------------------------------------- */

int ImuCalibrator::mk_calibration_init(uint32_t sstill, uint32_t nposes, uint32_t sps)
{
  raw_data = new(mk_calibration_data);
  if (!raw_data)
      return ENOMEM;

  raw_data->sps = sps; // Imu rate
  raw_data->sstill = sstill; // Number of samples per still pose
  raw_data->nposes = nposes; // Number of required still poses

  raw_data->gyr.resize(Eigen::NoChange, sps);
  raw_data->acc.resize(Eigen::NoChange, sps);
  raw_data->samples = 0;
  raw_data->ts.sec = raw_data->ts.nsec = 0;

  raw_data->sum << 0., 0., 0.;
  raw_data->sumsq << 0., 0., 0.;
  raw_data->accvarth = DBL_MAX;

  raw_data->still.resize(Eigen::NoChange, 0);
  raw_data->nstill = 0;

  return 0;
}


/* --- mk_calibration_collect ---------------------------------------------- */

int ImuCalibrator::mk_calibration_collect(ImuMeasurement *imu_data, int32_t *still)
{
  Eigen::Matrix<double, 3, 1> acc, accvar;
  double m;

  *still = -1;

  /* check data */
  if (!imu_data->vel._present || !imu_data->acc._present)
      return EIO;
  if (imu_data->ts.sec == raw_data->ts.sec &&
      imu_data->ts.nsec == raw_data->ts.nsec)
      return EAGAIN;


  /* collect raw sample */

  // Store time stamps
  if (raw_data->t.cols() <= raw_data->samples)
    raw_data->t.conservativeResize(raw_data->t.cols() + raw_data->sps);

  raw_data->t(raw_data->samples) =
    imu_data->ts.sec + 1e-9 * imu_data->ts.nsec;

  // Store Gyro
  if (raw_data->gyr.cols() <= raw_data->samples)
    raw_data->gyr.conservativeResize(Eigen::NoChange,
                                    raw_data->gyr.cols() + raw_data->sps);
  raw_data->gyr.col(raw_data->samples) <<
    imu_data->vel._value.wx, imu_data->vel._value.wy, imu_data->vel._value.wz;

  // Store Accelerom
  if (raw_data->acc.cols() <= raw_data->samples)
    raw_data->acc.conservativeResize(Eigen::NoChange,
                                    raw_data->acc.cols() + raw_data->sps);
  raw_data->acc.col(raw_data->samples) <<
    imu_data->acc._value.ax, imu_data->acc._value.ay, imu_data->acc._value.az;


  /* compute accelerometer variance over the last second */

  acc = raw_data->acc.col(raw_data->samples);
  raw_data->sum += acc;
  raw_data->sumsq += acc.cwiseProduct(acc);

  if (raw_data->samples >= raw_data->sps)
  {
    acc = raw_data->acc.col(raw_data->samples - raw_data->sps);
    raw_data->sum -= acc;
    raw_data->sumsq -= acc.cwiseProduct(acc);

    accvar =
      (raw_data->sumsq -
       raw_data->sum.cwiseProduct(raw_data->sum) / raw_data->sps) /
      raw_data->sps;
  }


  /* detect still poses */
  if (raw_data->samples > raw_data->sps)
  {
    m = accvar.maxCoeff();
    if (raw_data->accvarth > m)
        raw_data->accvarth = m;

    if ((accvar.array() < 20 * raw_data->accvarth).all())
    {

      if (!raw_data->nstill)
        *still = 0;

      raw_data->nstill++;
      if (raw_data->nstill == raw_data->sstill)
      {
        raw_data->still.conservativeResize(
          Eigen::NoChange, raw_data->still.cols() + 1);
        raw_data->still(0, raw_data->still.cols()-1) =
          raw_data->samples - raw_data->sstill - raw_data->sps/2;

        *still = raw_data->still.cols();
        std::cout<<"Still pose"<<std::endl;

        // Reset: JL addition
        raw_data->accvarth=DBL_MAX;

      }
      if (raw_data->nstill > 100 * raw_data->sstill)
      {
          std::cout<<"ERROR!!"<<std::endl;
        return EFBIG;
      }
      else if (raw_data->nstill >= raw_data->sstill)
      {
        raw_data->still(1, raw_data->still.cols()-1) =
          raw_data->samples - raw_data->sps/2;
        //std::cout<<"aqui"<<std::endl;
      }
    }
    else
    {
      raw_data->nstill = 0;
      if (raw_data->still.cols() > 0)
      {
        if (raw_data->samples - raw_data->still(1, raw_data->still.cols()-1) >
            100 * raw_data->sstill)
        {
          return EFBIG;
        }
      }
      else
      {
        if (raw_data->samples > 100 * raw_data->sstill)
        {
          return EFBIG;
        }
      }
    }
  }


  /* next sample */
  raw_data->samples++;
  raw_data->ts = imu_data->ts;

  return 0;
  //return (raw_data->still.cols() < raw_data->nposes) ? EAGAIN : 0;
}


/* --- mk_calibration_acc -------------------------------------------------- */



int ImuCalibrator::mk_calibration_acc(double ascale[9], double abias[3])
{
  Eigen::NumericalDiff<mk_calibration_acc_errfunc> errfunc;
  int32_t i, k;

  /* average accelerometer data over all still periods  */
  for(i = 0; i < raw_data->still.cols(); i++)
  {
    raw_data->sum << 0., 0., 0.;
    for(k = raw_data->still(0, i); k <= raw_data->still(1, i); k++)
      raw_data->sum += raw_data->acc.col(k);
    raw_data->sum /= raw_data->still(1, i) - raw_data->still(0, i) + 1;

    errfunc.measurement(raw_data->sum);
  }

  /* compute optimal parameters */
  Eigen::Matrix<double, Eigen::Dynamic, 1> theta(9);
  Eigen::LevenbergMarquardt<
    Eigen::NumericalDiff<mk_calibration_acc_errfunc> > lm(errfunc);
  Eigen::Matrix<double, 3, 3> S1;
  Eigen::Matrix<double, 3, 1> b1;
  int s;

  theta <<
    0., 0., 0.,
    1., 1., 1.,
    0., 0., 0.;

  s = lm.minimize(theta);
  if (s <= 0) return EINVAL;
  if (s > 3) return ERANGE;

  S1 <<
      theta(3),  theta(0) * theta(4),  theta(1) * theta(5),
            0.,             theta(4),  theta(2) * theta(5),
            0.,                   0.,             theta(5);
  b1 <<
      theta(6),
      theta(7),
      theta(8);

  /* apply correction to all raw accelerometer data */
  for(i = 0; i < raw_data->samples; i++)
    raw_data->acc.col(i) = S1 * ( raw_data->acc.col(i) + b1 );

  /* update old scale S0 and bias b0 with new S1 and b1 so that we now read
   * S1.( S0.(a + b0) + b1 ), i.e. S = S1.S0 and b = b0 + S0^-1.b1 */
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > S(ascale);
  Eigen::Map<Eigen::Matrix<double, 3, 1> > b(abias);

  b += S.inverse() * b1;
  S = S1 * S;

  return 0;
}


/* --- mk_calibration_gyr -------------------------------------------------- */



int ImuCalibrator::mk_calibration_gyr(double gscale[9], double gbias[3])
{
  Eigen::NumericalDiff<mk_calibration_gyr_errfunc> errfunc;
  int32_t i, k, n;

  /* average gyroscope data over all still periods to get bias */
  Eigen::Matrix<double, 3, 1> b1;
  b1 << 0., 0., 0.;
  n = 0;
  for(i = 0; i < raw_data->still.cols(); i++)
  {
    for(k = raw_data->still(0, i); k <= raw_data->still(1, i); k++)
      b1 -= raw_data->gyr.col(k);
    n += raw_data->still(1, i) - raw_data->still(0, i) + 1;
  }
  b1 /= n;

  /* apply bias correction to all raw gyro data */
  raw_data->gyr.colwise() += b1;

  /* compute gravity direction over all static intervals */
  Eigen::Matrix<double, 3, 1> g;

  errfunc.acc_dir.resize(Eigen::NoChange, raw_data->still.cols());
  for(i = 0; i < raw_data->still.cols(); i++)
  {
    g << 0., 0., 0.;
    for(k = raw_data->still(0, i); k <= raw_data->still(1, i); k++)
      g += raw_data->acc.col(k);
    g.normalize();

    errfunc.acc_dir.col(i) = g;
  }

  /* compute optimal parameters */
  Eigen::Matrix<double, Eigen::Dynamic, 1> theta(9);
  Eigen::LevenbergMarquardt<
    Eigen::NumericalDiff<mk_calibration_gyr_errfunc> > lm(errfunc);
  int s;

  theta <<
    0., 0., 0., 0., 0., 0.,
    1., 1., 1.;

  s = lm.minimize(theta);
  if (s <= 0) return EINVAL;
  if (s > 3) return ERANGE;

  /* apply correction to all raw gyroscope data */
  Eigen::Matrix<double, 3, 3> S1;
  S1 <<
               theta(6),  theta(0) * theta(7),  theta(1) * theta(8),
    theta(2) * theta(6),             theta(7),  theta(3) * theta(8),
    theta(4) * theta(6),  theta(5) * theta(7),             theta(8);

  for(i = 0; i < raw_data->samples; i++)
    raw_data->gyr.col(i) = S1 * raw_data->gyr.col(i);

  /* update old scale S0 and bias b0 with new S1 and b1 so that we now read
   * S1.( S0.(a + b0) + b1 ), i.e. S = S1.S0 and b = b0 + S0^-1.b1 */
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > S(gscale);
  Eigen::Map<Eigen::Matrix<double, 3, 1> > b(gbias);

  b += S.inverse() * b1;
  S = S1 * S;

  return 0;
}


/* --- mk_calibration_fini ------------------------------------------------- */

void ImuCalibrator::mk_calibration_fini(double stddeva[3], double stddevw[3],
                    double maxa[3], double maxw[3])
{
  Eigen::Matrix<double, 3, 1> s, v;
  int32_t i, k, n, l;

  /* stddev over all still intervals */
  if (stddeva)
  {
    s << 0., 0., 0.;
    n = 0;
    for(i = 0; i < raw_data->still.cols(); i++)
    {
      raw_data->sum << 0., 0., 0.;
      raw_data->sumsq << 0., 0., 0.;
      for(k = raw_data->still(0, i); k <= raw_data->still(1, i); k++)
      {
        v = raw_data->acc.col(k);
        raw_data->sum += v;
        raw_data->sumsq += v.cwiseProduct(v);
      }
      l = raw_data->still(1, i) - raw_data->still(0, i) + 1;
      s += raw_data->sumsq - raw_data->sum.cwiseProduct(raw_data->sum)/l;
      n += l;
    }
    s /= n;
    stddeva[0] = std::sqrt(s(0));
    stddeva[1] = std::sqrt(s(1));
    stddeva[2] = std::sqrt(s(2));
  }
  if (stddevw)
  {
    raw_data->sum << 0., 0., 0.;
    raw_data->sumsq << 0., 0., 0.;
    n = 0;
    for(i = 0; i < raw_data->still.cols(); i++) {
      for(k = raw_data->still(0, i); k <= raw_data->still(1, i); k++)
      {
        v = raw_data->gyr.col(k);
        raw_data->sum += v;
        raw_data->sumsq += v.cwiseProduct(v);
      }
      n += raw_data->still(1, i) - raw_data->still(0, i) + 1;
    }
    s = (raw_data->sumsq - raw_data->sum.cwiseProduct(raw_data->sum)/n)/n;
    stddevw[0] = std::sqrt(s(0));
    stddevw[1] = std::sqrt(s(1));
    stddevw[2] = std::sqrt(s(2));
  }

  /* max absolute */
  if (maxa)
  {
    s = raw_data->acc.rowwise().maxCoeff();
    v = raw_data->acc.rowwise().minCoeff();
    if (s(0) < -v(0)) maxa[0] = -v(0); else maxa[0] = s(0);
    if (s(1) < -v(1)) maxa[1] = -v(1); else maxa[1] = s(1);
    if (s(2) < -v(2)) maxa[2] = -v(2); else maxa[2] = s(2);
  }
  if (maxw)
  {
    s = raw_data->gyr.rowwise().maxCoeff();
    v = raw_data->gyr.rowwise().minCoeff();
    if (s(0) < -v(0)) maxw[0] = -v(0); else maxw[0] = s(0);
    if (s(1) < -v(1)) maxw[1] = -v(1); else maxw[1] = s(1);
    if (s(2) < -v(2)) maxw[2] = -v(2); else maxw[2] = s(2);
  }

  if (raw_data) delete raw_data;
}


/* --- mk_calibration_rotate ----------------------------------------------- */

void ImuCalibrator::mk_calibration_rotate(double r[9], double s[9])
{
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > R(r), S(s);

  S = R * S;
}


/* --- mk_calibration_bias ------------------------------------------------- */

void ImuCalibrator::mk_calibration_bias(double b1[3], double s[9], double b[3])
{
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > S(s);
  Eigen::Map<Eigen::Matrix<double, 3, 1> > B1(b1), B(b);

  B += S.inverse() * B1;
}
