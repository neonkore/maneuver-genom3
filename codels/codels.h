/*
 * Copyright (c) 2016,2018 LAAS/CNRS
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
 *					Anthony Mallet on Tue Dec  6 2016
 */

#ifndef H_MANEUVER_CODELS
#define H_MANEUVER_CODELS

#include <aio.h>

#include <cstdio>
#include <cstring>

#include "libkdtp.h"

#include "maneuver_c_types.h"

struct maneuver_planner_s {
  kdtp::Robot robot;

  maneuver_planner_s(): robot("rotorcraft") {}
};

static inline genom_event
mv_e_sys_error(const char *s, genom_context self)
{
  maneuver_e_sys_detail d;
  char buf[64], *p;

  d.code = errno;
#ifdef STRERROR_R_CHAR_P
  /* glibc managed to mess up with this function */
  p = strerror_r(d.code, buf, sizeof(buf));
#else
  strerror_r(d.code, buf, sizeof(buf));
  p = buf;
#endif
  snprintf(d.what, sizeof(d.what), "%s%s%s", s ? s : "", s ? ": " : "", p);

  return maneuver_e_sys(&d, self);
}

struct maneuver_log_s {
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define mv_logfmt	" %g "
# define mv_log_header_fmt                                              \
  "ts x y z yaw vx vy vz wz ax ay az"
# define mv_log_fmt                                                     \
  "%d.%09d "                                                            \
  mv_logfmt mv_logfmt mv_logfmt mv_logfmt                               \
  mv_logfmt mv_logfmt mv_logfmt mv_logfmt                               \
  mv_logfmt mv_logfmt mv_logfmt
# define mv_replay_fmt                                                  \
  "%d.%09d "                                                            \
  "%lf %lf %lf %lf"                                                     \
  "%lf %lf %lf %lf"                                                     \
  "%lf %lf %lf"
};

#endif /* H_MANEUVER_CODELS */
