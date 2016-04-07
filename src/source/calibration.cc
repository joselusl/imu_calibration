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
#include "imu_calibration/acmikrokopter.h"

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

/* --- local data ---------------------------------------------------------- */

struct mk_calibration_data {
  int32_t sps, sstill, nposes;

  Eigen::Matrix<double, 1, Eigen::Dynamic> t;
  Eigen::Matrix<double, 3, Eigen::Dynamic> gyr;
  Eigen::Matrix<double, 3, Eigen::Dynamic> acc;
  int32_t samples;
  or_time_ts ts;

  Eigen::Matrix<double, 3, 1> sum, sumsq;
  double  accvarth;

  Eigen::Array<int32_t, 2, Eigen::Dynamic> still;
  int32_t nstill;
};

static mk_calibration_data *raw_data;


/* --- mk_calibration_init ------------------------------------------------- */

int
mk_calibration_init(uint32_t sstill, uint32_t nposes, uint32_t sps)
{
  raw_data = new(mk_calibration_data);
  if (!raw_data) return ENOMEM;

  raw_data->sps = sps;
  raw_data->sstill = sstill;
  raw_data->nposes = nposes;

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

int
mk_calibration_collect(or_pose_estimator_state *imu_data, int32_t *still)
{
  Eigen::Matrix<double, 3, 1> acc, accvar;
  double m;

  *still = -1;

  /* check data */
  if (!imu_data->vel._present || !imu_data->acc._present) return EIO;
  if (imu_data->ts.sec == raw_data->ts.sec &&
      imu_data->ts.nsec == raw_data->ts.nsec) return EAGAIN;


  /* collect raw sample */

  if (raw_data->t.cols() <= raw_data->samples)
    raw_data->t.conservativeResize(raw_data->t.cols() + raw_data->sps);
  raw_data->t(raw_data->samples) =
    imu_data->ts.sec + 1e-9 * imu_data->ts.nsec;

  if (raw_data->gyr.cols() <= raw_data->samples)
    raw_data->gyr.conservativeResize(Eigen::NoChange,
                                    raw_data->gyr.cols() + raw_data->sps);
  raw_data->gyr.col(raw_data->samples) <<
    imu_data->vel._value.wx, imu_data->vel._value.wy, imu_data->vel._value.wz;

  if (raw_data->acc.cols() <= raw_data->samples)
    raw_data->acc.conservativeResize(Eigen::NoChange,
                                    raw_data->acc.cols() + raw_data->sps);
  raw_data->acc.col(raw_data->samples) <<
    imu_data->acc._value.ax, imu_data->acc._value.ay, imu_data->acc._value.az;


  /* compute accelerometer variance over the last second */

  acc = raw_data->acc.col(raw_data->samples);
  raw_data->sum += acc;
  raw_data->sumsq += acc.cwiseProduct(acc);

  if (raw_data->samples >= raw_data->sps) {
    acc = raw_data->acc.col(raw_data->samples - raw_data->sps);
    raw_data->sum -= acc;
    raw_data->sumsq -= acc.cwiseProduct(acc);

    accvar =
      (raw_data->sumsq -
       raw_data->sum.cwiseProduct(raw_data->sum) / raw_data->sps) /
      raw_data->sps;
  }


  /* detect still poses */
  if (raw_data->samples > raw_data->sps) {
    m = accvar.maxCoeff();
    if (raw_data->accvarth > m) raw_data->accvarth = m;

    if ((accvar.array() < 2 * raw_data->accvarth).all()) {
      if (!raw_data->nstill)
        *still = 0;

      raw_data->nstill++;
      if (raw_data->nstill == raw_data->sstill) {
        raw_data->still.conservativeResize(
          Eigen::NoChange, raw_data->still.cols() + 1);
        raw_data->still(0, raw_data->still.cols()-1) =
          raw_data->samples - raw_data->sstill - raw_data->sps/2;

        *still = raw_data->still.cols();
      }
      if (raw_data->nstill > 10 * raw_data->sstill)
        return EFBIG;
      else if (raw_data->nstill >= raw_data->sstill) {
        raw_data->still(1, raw_data->still.cols()-1) =
          raw_data->samples - raw_data->sps/2;
      }
    } else {
      raw_data->nstill = 0;
      if (raw_data->still.cols() > 0) {
        if (raw_data->samples - raw_data->still(1, raw_data->still.cols()-1) >
            10 * raw_data->sstill)
          return EFBIG;
      } else {
        if (raw_data->samples > 10 * raw_data->sstill)
          return EFBIG;
      }
    }
  }


  /* next sample */
  raw_data->samples++;
  raw_data->ts = imu_data->ts;

  return (raw_data->still.cols() < raw_data->nposes) ? EAGAIN : 0;
}


/* --- mk_calibration_acc -------------------------------------------------- */

struct mk_calibration_acc_errfunc {
  typedef double Scalar;
  enum {
    InputsAtCompileTime = 9,
    ValuesAtCompileTime = Eigen::Dynamic
  };
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> InputType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ValueType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianType;

  Eigen::Matrix<Scalar, 3, Eigen::Dynamic> measurements;
  void measurement(Eigen::Matrix<Scalar, 3, 1> m) {
    measurements.conservativeResize(Eigen::NoChange, measurements.cols()+1);
    measurements.col(measurements.cols()-1) << m;
  }

  int operator()(const InputType &theta, ValueType &L) const {
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

int
mk_calibration_acc(double ascale[9], double abias[3])
{
  Eigen::NumericalDiff<mk_calibration_acc_errfunc> errfunc;
  int32_t i, k;

  /* average accelerometer data over all still periods  */
  for(i = 0; i < raw_data->still.cols(); i++) {
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

struct mk_calibration_gyr_errfunc {
  typedef double Scalar;
  enum {
    InputsAtCompileTime = 9,
    ValuesAtCompileTime = Eigen::Dynamic
  };
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> InputType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ValueType;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianType;

  Eigen::Matrix<double, 3, Eigen::Dynamic> acc_dir;

  int operator()(const InputType &theta, ValueType &L) const {
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

    for(i = 0; i < raw_data->still.cols()-1; i++) {

      /* integrate gyro over the ith motion interval */
      q = Eigen::Quaternion<double>::Identity();
      for(k = raw_data->still(0, i); k <= raw_data->still(1, i+1); k++) {
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

int
mk_calibration_gyr(double gscale[9], double gbias[3])
{
  Eigen::NumericalDiff<mk_calibration_gyr_errfunc> errfunc;
  int32_t i, k, n;

  /* average gyroscope data over all still periods to get bias */
  Eigen::Matrix<double, 3, 1> b1;
  b1 << 0., 0., 0.;
  n = 0;
  for(i = 0; i < raw_data->still.cols(); i++) {
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
  for(i = 0; i < raw_data->still.cols(); i++) {
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

void
mk_calibration_fini(double stddeva[3], double stddevw[3],
                    double maxa[3], double maxw[3])
{
  Eigen::Matrix<double, 3, 1> s, v;
  int32_t i, k, n, l;

  /* stddev over all still intervals */
  if (stddeva) {
    s << 0., 0., 0.;
    n = 0;
    for(i = 0; i < raw_data->still.cols(); i++) {
      raw_data->sum << 0., 0., 0.;
      raw_data->sumsq << 0., 0., 0.;
      for(k = raw_data->still(0, i); k <= raw_data->still(1, i); k++) {
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
  if (stddevw) {
    raw_data->sum << 0., 0., 0.;
    raw_data->sumsq << 0., 0., 0.;
    n = 0;
    for(i = 0; i < raw_data->still.cols(); i++) {
      for(k = raw_data->still(0, i); k <= raw_data->still(1, i); k++) {
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
  if (maxa) {
    s = raw_data->acc.rowwise().maxCoeff();
    v = raw_data->acc.rowwise().minCoeff();
    if (s(0) < -v(0)) maxa[0] = -v(0); else maxa[0] = s(0);
    if (s(1) < -v(1)) maxa[1] = -v(1); else maxa[1] = s(1);
    if (s(2) < -v(2)) maxa[2] = -v(2); else maxa[2] = s(2);
  }
  if (maxw) {
    s = raw_data->gyr.rowwise().maxCoeff();
    v = raw_data->gyr.rowwise().minCoeff();
    if (s(0) < -v(0)) maxw[0] = -v(0); else maxw[0] = s(0);
    if (s(1) < -v(1)) maxw[1] = -v(1); else maxw[1] = s(1);
    if (s(2) < -v(2)) maxw[2] = -v(2); else maxw[2] = s(2);
  }

  if (raw_data) delete raw_data;
}


/* --- mk_calibration_rotate ----------------------------------------------- */

void
mk_calibration_rotate(double r[9], double s[9])
{
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > R(r), S(s);

  S = R * S;
}


/* --- mk_calibration_bias ------------------------------------------------- */

void
mk_calibration_bias(double b1[3], double s[9], double b[3])
{
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > S(s);
  Eigen::Map<Eigen::Matrix<double, 3, 1> > B1(b1), B(b);

  B += S.inverse() * B1;
}
