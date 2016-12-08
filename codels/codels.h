/*
 * Copyright (c) 2016 LAAS/CNRS
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
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  if (strerror_r(d.code, d.what + l, sizeof(d.what) - l)) /* ignore error*/;
  return maneuver_e_sys(&d, self);
}

struct maneuver_log_s {
  FILE *f;

# define mv_logfmt	" %e "
# define mv_log_header_fmt                                              \
  "ts x y z yaw vx vy vz wz ax ay az"
# define mv_log_fmt                                                     \
  "%d.%09d "                                                            \
  mv_logfmt mv_logfmt mv_logfmt mv_logfmt                               \
  mv_logfmt mv_logfmt mv_logfmt mv_logfmt                               \
  mv_logfmt mv_logfmt mv_logfmt
};

#endif /* H_MANEUVER_CODELS */
