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

#include "imu_calibration/mikrokopter_c_types.h"

struct mikrokopter_log_s {
  FILE *logf;

# define mikrokopter_logfmt	" %2.6f "
# define mikrokopter_log_header                         \
  "ts imu_wx imu_wy imu_wz  imu_ax imu_ay imu_az "      \
  "cmd_ts cmd_fz cmd_tx cmd_ty cmd_tz"
# define mikrokopter_log_imu                                            \
  "%d.%09d"                                                             \
  mikrokopter_logfmt mikrokopter_logfmt mikrokopter_logfmt              \
  mikrokopter_logfmt mikrokopter_logfmt mikrokopter_logfmt              \
  "? ? ? ? ?"
};


//struct mk_channel_s {
//  char path[1024];
//  int fd;

//  uint8_t buf[64], r, w; /* read ring buffer */

//  bool start;
//  bool escape;
//  uint8_t msg[64], len; /* last message */
//};

//struct mikrokopter_conn_s {
//  struct mk_channel_s chan[2];
//};

//static inline size_t
//mk_channels(void)
//{
//  mikrokopter_conn_s *c;
//  return sizeof(c->chan)/sizeof(c->chan[0]);
//}

//static inline genom_event
//mk_e_sys_error(const char *s, genom_context self)
//{
//  mikrokopter_e_sys_detail d;
//  size_t l = 0;

//  d.code = errno;
//  if (s) {
//    strncpy(d.what, s, sizeof(d.what) - 3);
//    l = strlen(s);
//    strcpy(d.what + l, ": ");
//    l += 2;
//  }
//  strerror_r(d.code, d.what + l, sizeof(d.what) - l);
//  return mikrokopter_e_sys(&d, self);
//}

//int	mk_open_tty(const char *device, speed_t baud);
//int	mk_wait_msg(const struct mk_channel_s *channels, int n);
//int	mk_recv_msg(struct mk_channel_s *chan, bool block);
//int	mk_send_msg(const struct mk_channel_s *chan, const char *fmt, ...);

#ifdef __cplusplus
extern "C" {
#endif

  int	mk_calibration_init(uint32_t sstill, uint32_t nposes, uint32_t sps);
  int	mk_calibration_collect(or_pose_estimator_state *imu_data,
                               int32_t *still);
  int	mk_calibration_acc(double ascale[9], double abias[3]);
  int	mk_calibration_gyr(double gscale[9], double gbias[3]);
  void	mk_calibration_fini(double stddeva[3], double stddevw[3],
                            double *maxa, double *maxw);

  void	mk_calibration_rotate(double r[9], double s[9]);
  void	mk_calibration_bias(double b1[3], double s[9], double b[3]);

#ifdef __cplusplus
}
#endif

#endif /* H_MIKROKOPTER_CODELS */
