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
 *					Anthony Mallet on Fri Feb 13 2015
 */
//#include "imu_calibration/acmikrokopter.h"

//#include <sys/time.h>
//#include <float.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>

//#include "imu_calibration/mikrokopter_c_types.h"
//#include "imu_calibration/codels.h"


///* --- Attribute set_sensor_rate ---------------------------------------- */

///** Validation codel mk_set_sensor_rate of attribute set_sensor_rate.
// *
// * Returns genom_ok.
// * Throws .
// */
//genom_event
//mk_set_sensor_rate(const mikrokopter_ids_sensor_time_s_rate_s *rate,
//                   const mikrokopter_conn_s *conn,
//                   mikrokopter_ids_sensor_time_s *sensor_time,
//                   genom_context self)
//{
//  int mainc, auxc;
//  uint32_t p;
//  int i;

//  if (rate->imu < 0. || rate->imu > 2000. ||
//      rate->motor < 0. || rate->motor > 2000. ||
//      rate->battery < 0. || rate->battery > 2000.)
//    return mikrokopter_e_range(self);

//  if (sensor_time) {
//    sensor_time->imu.seq = 0;
//    sensor_time->imu.ts = 0.;
//    sensor_time->imu.offset = -DBL_MAX;
//    for(i = 0; i < mikrokopter_max_rotors; i++) {
//      sensor_time->motor[i].seq = 0;
//      sensor_time->motor[i].ts = 0.;
//      sensor_time->motor[i].offset = -DBL_MAX;
//    }
//    sensor_time->battery.seq = 0;
//    sensor_time->battery.ts = 0.;
//    sensor_time->battery.offset = -DBL_MAX;
//  }

//  /* reconfigure existing connection */
//  if (!conn) return genom_ok;
//  mainc = auxc = -1;
//  if (conn->chan[0].fd >= 0) { mainc = 0; auxc = 0; }
//  if (conn->chan[1].fd >= 0) auxc = 1;
//  if (mainc < 0 && auxc < 0) return genom_ok;

//  p = rate->battery > 0. ? 1000000/rate->battery : 0;
//  mk_send_msg(&conn->chan[auxc], "b%4", p);
//  p = rate->motor > 0. ? 1000000/rate->motor : 0;
//  mk_send_msg(&conn->chan[auxc], "m%4", p);
//  p = rate->motor > 0. ? 1000000/rate->imu : 0;
//  mk_send_msg(&conn->chan[mainc], "i%4", p);

//  return genom_ok;
//}


///* --- Attribute set_imu_calibration ------------------------------------ */

///** Validation codel mk_set_imu_calibration of attribute set_imu_calibration.
// *
// * Returns genom_ok.
// * Throws .
// */
//genom_event
//mk_set_imu_calibration(bool *imu_calibration_updated,
//                       genom_context self)
//{
//  *imu_calibration_updated = true;
//  return genom_ok;
//}


///* --- Function disable_motor ------------------------------------------- */

///** Codel mk_disable_motor of function disable_motor.
// *
// * Returns genom_ok.
// */
//genom_event
//mk_disable_motor(uint16_t motor, sequence8_boolean *disabled_motors,
//                 genom_context self)
//{
//  if (motor < 1 || motor > disabled_motors->_maximum)
//    return mikrokopter_e_range(self);

//  if (motor > disabled_motors->_length) disabled_motors->_length = motor;
//  disabled_motors->_buffer[motor-1] = true;
//  return genom_ok;
//}


///* --- Function enable_motor -------------------------------------------- */

///** Codel mk_enable_motor of function enable_motor.
// *
// * Returns genom_ok.
// */
//genom_event
//mk_enable_motor(uint16_t motor, sequence8_boolean *disabled_motors,
//                genom_context self)
//{
//  if (motor < 1 || motor > disabled_motors->_maximum)
//    return mikrokopter_e_range(self);

//  if (motor > disabled_motors->_length) disabled_motors->_length = motor;
//  disabled_motors->_buffer[motor-1] = false;
//  return genom_ok;
//}


///* --- Function set_wrench ---------------------------------------------- */

///** Codel mk_set_wrench of function set_wrench.
// *
// * Returns genom_ok.
// */
//genom_event
//mk_set_wrench(const or_rb3d_wrench *wrench,
//              or_rotorcraft_ts_wrench *target, genom_context self)
//{
//  struct timeval tv;

//  gettimeofday(&tv, NULL);
//  target->ts.sec = tv.tv_sec;
//  target->ts.nsec = tv.tv_usec * 1000;

//  target->w = *wrench;
//  return genom_ok;
//}


///* --- Function stop ---------------------------------------------------- */

///** Codel mk_stop of function stop.
// *
// * Returns genom_ok.
// */
//genom_event
//mk_stop(const mikrokopter_conn_s *conn, genom_context self)
//{
//  if (!conn) return mikrokopter_e_connection(self);
//  mk_send_msg(&conn->chan[0], "x");
//  return genom_ok;
//}


///* --- Function log ----------------------------------------------------- */

///** Codel mk_log_start of function log.
// *
// * Returns genom_ok.
// * Throws mikrokopter_e_sys.
// */
//genom_event
//mk_log_start(const char path[64], mikrokopter_log_s **log,
//             genom_context self)
//{
//  FILE *f;

//  mk_log_stop(log, self);

//  f = fopen(path, "w");
//  if (!f) return mk_e_sys_error("log", self);
//  fprintf(f, mikrokopter_log_header "\n");

//  *log = malloc(sizeof(**log));
//  if (!*log) {
//    fclose(f);
//    unlink(path);
//    errno = ENOMEM;
//    return mk_e_sys_error("log", self);
//  }

//  (*log)->logf = f;
//  return genom_ok;
//}


///* --- Function log_stop ------------------------------------------------ */

///** Codel mk_log_stop of function log_stop.
// *
// * Returns genom_ok.
// */
//genom_event
//mk_log_stop(mikrokopter_log_s **log, genom_context self)
//{
//  if (!*log) return genom_ok;

//  fclose((*log)->logf);
//  free(*log);
//  *log = NULL;
//  return genom_ok;
//}
