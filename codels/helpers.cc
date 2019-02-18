/*
 * Copyright (c) 2018-2019 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Tue Feb 13 2018
 */
#include "acmaneuver.h"

#include <cmath>

#include "maneuver_c_types.h"
#include "codels.h"


/*
 * --- consider limits -----------------------------------------------------
 */
genom_event
mv_check_duration(const kdtp::LocalPath &p, const double duration,
                  const genom_context self)
{
  /* consider limits */
  if (duration > 0. &&
      p.duration() > duration + maneuver_control_period_ms/1000.)
    return maneuver_e_limits(self);

  return genom_ok;
}


/*
 * --- sample local path according to the exec task period -----------------
 */
genom_event
mv_sample_path(const kdtp::LocalPath &p,
               sequence_or_rigid_body_state *path, genom_context self)
{
  static const double dt = maneuver_control_period_ms/1000.;

  or_rigid_body_state s;
  double q[4][5];
  size_t i;

  /* set path length, including start & end configurations */
  i = 2 + p.duration()/dt;
  if (path->_maximum < i || path->_maximum > 2 * i)
    if (genom_sequence_reserve(path, i))
      return mv_e_sys_error(NULL, self);
  path->_length = i;


  /* fill constant fields along path */
  s.ts.sec = 0;
  s.ts.nsec = 0;
  s.intrinsic = false;

  s.pos._present = true;
  s.att._present = true;
  s.att._value.qx = 0.;
  s.att._value.qy = 0.;

  s.vel._present = true;
  s.avel._present = true;
  s.avel._value.wx = 0;
  s.avel._value.wy = 0;

  s.acc._present = true;
  s.aacc._present = true;
  s.aacc._value.awx = 0.;
  s.aacc._value.awy = 0.;

  s.jerk._present = true;

  s.snap._present = true;


  /* interpolate path */
  for(i = 0; i < path->_length; i++) {
    p.getAllAt(i * dt, q);

    s.pos._value.x = q[0][0];
    s.pos._value.y = q[1][0];
    s.pos._value.z = q[2][0];
    s.att._value.qw = std::cos(q[3][0]/2.);
    s.att._value.qz = std::sin(q[3][0]/2.);

    s.vel._value.vx = q[0][1];
    s.vel._value.vy = q[1][1];
    s.vel._value.vz = q[2][1];
    s.avel._value.wz = q[3][1];

    s.acc._value.ax = q[0][2];
    s.acc._value.ay = q[1][2];
    s.acc._value.az = q[2][2];
    s.aacc._value.awz = q[3][2];

    s.jerk._value.jx = q[0][3];
    s.jerk._value.jy = q[1][3];
    s.jerk._value.jz = q[2][3];

    s.snap._value.sx = q[0][4];
    s.snap._value.sy = q[1][4];
    s.snap._value.sz = q[2][4];

    path->_buffer[i] = s;
  }

  return genom_ok;
}


/*
 * --- sample velocity profile according to the exec task period -----------
 */
genom_event
mv_sample_velocity(const optional_or_t3d_pos &fromp,
                   const optional_or_t3d_att &fromq, const kdtp::LocalPath &p,
                   sequence_or_rigid_body_state *path, genom_context self)
{
  static const double dt = maneuver_control_period_ms/1000.;
  static const double dt2_2 = dt*dt/2.;
  static const double dt3_6 = dt*dt2_2/3.;

  or_rigid_body_state s;
  double q[4][5];
  double dyaw, qw, qz, dqw, dqz;
  size_t i;

  /* set path length, including start & end configurations */
  i = 2 + p.duration()/dt;
  if (path->_maximum < i || path->_maximum > 2 * i)
    if (genom_sequence_reserve(path, i)) return mv_e_sys_error(NULL, self);
  path->_length = i;


  /* fill constant fields along path */
  s.ts.sec = 0;
  s.ts.nsec = 0;
  s.intrinsic = false;

  s.pos = fromp;
  s.att = fromq;

  s.vel._present = true;
  s.avel._present = true;
  s.avel._value.wx = 0;
  s.avel._value.wy = 0;

  s.acc._present = true;
  s.aacc._present = true;
  s.aacc._value.awx = 0.;
  s.aacc._value.awy = 0.;

  s.jerk._present = true;

  s.snap._present = true;

  for(i = 0; i < path->_length; i++) {
    p.getAllAt(i * dt, q);

    s.vel._value.vx = q[0][0];
    s.vel._value.vy = q[1][0];
    s.vel._value.vz = q[2][0];
    s.avel._value.wz = q[3][0];

    s.acc._value.ax = q[0][1];
    s.acc._value.ay = q[1][1];
    s.acc._value.az = q[2][1];
    s.aacc._value.awz = q[3][1];

    s.jerk._value.jx = q[0][2];
    s.jerk._value.jy = q[1][2];
    s.jerk._value.jz = q[2][2];

    s.snap._value.sx = q[0][3];
    s.snap._value.sy = q[1][3];
    s.snap._value.sz = q[2][3];

    path->_buffer[i] = s;

    /* integrate position if needed */
    if (s.pos._present) {
      s.pos._value.x +=
        dt*s.vel._value.vx + dt2_2*s.acc._value.ax + dt3_6*s.jerk._value.jx;
      s.pos._value.y +=
        dt*s.vel._value.vy + dt2_2*s.acc._value.ay + dt3_6*s.jerk._value.jy;
      s.pos._value.z +=
        dt*s.vel._value.vz + dt2_2*s.acc._value.az + dt3_6*s.jerk._value.jz;

      /* XXX assumes roll/pitch == 0 */
      qw = s.att._value.qw;
      qz = s.att._value.qz;

      dyaw = dt*s.avel._value.wz + dt2_2*s.aacc._value.awz;
      if (fabs(dyaw) < 0.25) {
        double dyaw2 = dyaw * dyaw;
        dqw = 1 - dyaw2/8;		/* cos(dyaw/2) ± 1e-5 */
        dqz = (0.5 - dyaw2/48) * dyaw;	/* sin(dyaw/2) ± 1e-6 */
      } else {
        dqw = std::cos(dyaw/2);
        dqz = std::sin(dyaw/2);
      }

      s.att._value.qw = dqw*qw - dqz*qz;
      s.att._value.qz = dqw*qz + dqz*qw;
    }
  }

  return genom_ok;
}
