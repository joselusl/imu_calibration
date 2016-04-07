/*
 * Copyright (c) 2015 LAAS/CNRS
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
//#include <sys/stat.h>
//#include <sys/types.h>
//#include <sys/uio.h>

//#include <err.h>
//#include <errno.h>
//#include <fcntl.h>
//#include <poll.h>
//#include <stdarg.h>
//#include <stdio.h>
//#include <termios.h>
//#include <unistd.h>

//#include "imu_calibration/codels.h"


///* --- mk_open_tty --------------------------------------------------------- */

///* Open a serial port, configure to the given baud rate */
//int
//mk_open_tty(const char *device, uint32_t speed)
//{
//  struct termios t;
//  speed_t baud;
//  int fd;

//  /* select baud rate */
//#ifndef B57600
//# define B57600 57600U
//#endif
//#ifndef B115200
//# define B115200 115200U
//#endif
//#ifndef B500000
//# define B500000 500000U
//#endif
//  switch(speed) {
//    case 57600:		baud = B57600; break;
//    case 115200:	baud = B115200; break;
//    case 500000:	baud = B500000; break;

//    default: errno = EINVAL; return -1;
//  }

//  /* open non-blocking */
//  fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
//  if (fd < 0) return fd;
//  if (!isatty(fd)) {
//    errno = ENOTTY;
//    return -1;
//  }

//  /* configure line discipline */
//  if (tcgetattr(fd, &t)) return -1;

//  t.c_iflag = IGNBRK;
//  t.c_oflag = 0;
//  t.c_lflag = 0;
//  t.c_cflag = CS8 | CREAD | CLOCAL;
//  t.c_cc[VMIN] = 0;
//  t.c_cc[VTIME] = 0;

//  if (cfsetospeed(&t, baud)) return -1;
//  if (cfsetispeed(&t, baud)) return -1;

//  if (tcsetattr(fd, TCSANOW, &t)) return -1;

//  /* discard any pending data */
//  tcflush(fd, TCIOFLUSH);

//  return fd;
//}


///* --- mk_wait_msg --------------------------------------------------------- */

//int
//mk_wait_msg(const struct mk_channel_s *channels, int n)
//{
//  struct pollfd pfds[n];
//  int i, s;

//  for(i = 0; i < n; i++) {
//    pfds[i].fd = channels[i].fd;
//    pfds[i].events = POLLIN;
//  }

//  s = poll(pfds, n, 500/*ms*/);

//  for(i = 0; i < n; i++) {
//    if (pfds[i].revents & POLLHUP) {
//      close(pfds[i].fd);

//      /* cheating with const. Oh well... */
//      ((struct mk_channel_s *)channels)[i].fd = -1;
//      warnx("disconnected from %s", channels[i].path);
//    }
//  }

//  return s;
//}


///* --- mk_recv_msg --------------------------------------------------------- */

///* returns: 0: timeout/incomplete, -1: error, 1: complete msg */

//int
//mk_recv_msg(struct mk_channel_s *chan, bool block)
//{
//  struct iovec iov[2];
//  ssize_t s;
//  uint8_t c;

//  if (chan->fd < 0) return -1;

//  do {
//    /* feed the ring  buffer */
//    iov[0].iov_base = chan->buf + chan->w;
//    iov[1].iov_base = chan->buf;

//    if (chan->r > chan->w) {
//      iov[0].iov_len = chan->r - chan->w - 1;
//      iov[1].iov_len = 0;
//    } else if (chan->r > 0) {
//      iov[0].iov_len = sizeof(chan->buf) - chan->w;
//      iov[1].iov_len = chan->r - 1;
//    } else {
//      iov[0].iov_len = sizeof(chan->buf) - chan->w - 1;
//      iov[1].iov_len = 0;
//    }

//    if (iov[0].iov_len || iov[1].iov_len) {
//      do {
//        s = readv(chan->fd, iov, 2);
//      } while(s < 0 && errno == EINTR);

//      if (s < 0)
//        return -1;
//      else if (s == 0 && chan->start && block) {
//        struct pollfd fd = { .fd = chan->fd, .events = POLLIN };

//        s = poll(&fd, 1, 500/*ms*/);
//        if (fd.revents & POLLHUP) return -1;
//      } else if (s == 0)
//        return 0;
//      else
//        chan->w = (chan->w + s) % sizeof(chan->buf);
//    }

//    while(chan->r != chan->w) {
//      c = chan->buf[chan->r];
//      chan->r = (chan->r + 1) % sizeof(chan->buf);

//      switch(c) {
//        case '^':
//        chan->start = true;
//        chan->escape = false;
//        chan->len = 0;
//        break;

//        case '$':
//          if (!chan->start) break;

//          chan->start = false;
//          return 1;

//        case '!':
//          chan->start = false;
//          break;

//        case '\\':
//          chan->escape = true;
//          break;

//        default:
//          if (!chan->start) break;
//          if (chan->len >= sizeof(chan->msg)) {
//            chan->start = false; break;
//          }

//          if (chan->escape) {
//            c = ~c; chan->escape = false;
//          }
//          chan->msg[chan->len++] = c;
//          break;
//      }
//    }
//  } while(1);

//  return 0;
//}


///* --- mk_send_msg --------------------------------------------------------- */

//static void	mk_encode(char x, char **buf);

//int
//mk_send_msg(const struct mk_channel_s *chan, const char *fmt, ...)
//{
//  va_list ap;
//  ssize_t s;
//  char buf[64], *w, *r;
//  char c;

//  if (chan->fd < 0) return -1;

//  w = buf;

//  va_start(ap, fmt);
//  *w++ = '^';
//  while((c = *fmt++)) {
//    while (w - buf > sizeof(buf)-8 /* 8 = worst case (4 bytes escaped) */) {
//      do {
//        s = write(chan->fd, buf, w - buf);
//      } while (s < 0 && errno == EINTR);
//      if (s < 0) return -1;

//      if (s > 0 && s < w - buf) memmove(buf, buf + s, w - buf - s);
//      w -= s;
//    }

//    switch(c) {
//      case '%': {
//        switch(*fmt++) {
//          case '1':
//            mk_encode(va_arg(ap, int/*promotion*/), &w);
//            break;

//          case '2': {
//            uint16_t x = va_arg(ap, int/*promotion*/);
//            mk_encode((x >> 8) & 0xff, &w);
//            mk_encode(x & 0xff, &w);
//            break;
//          }

//          case '4': {
//            uint32_t x = va_arg(ap, uint32_t);
//            mk_encode((x >> 24) & 0xff, &w);
//            mk_encode((x >> 16) & 0xff, &w);
//            mk_encode((x >> 8) & 0xff, &w);
//            mk_encode(x & 0xff, &w);
//            break;
//          }
//        }
//        break;
//      }

//      default:
//        mk_encode(c, &w);
//    }
//  }
//  *w++ = '$';
//  va_end(ap);

//  r = buf;
//  while (w > r) {
//    do {
//      s = write(chan->fd, r, w - r);
//    } while (s < 0 && errno == EINTR);
//    if (s < 0) return -1;

//    r += s;
//  }

//  return 0;
//}

//static void
//mk_encode(char x, char **buf)
//{
//  switch (x) {
//    case '^': case '$': case '\\': case '!':
//      *(*buf)++ = '\\';
//      x = ~x;
//  }
//  *(*buf)++ = x;
//}
