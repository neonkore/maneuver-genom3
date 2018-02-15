/*
 * Copyright (c) 2018 LAAS/CNRS
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
  if (duration > 0. && p.duration() > duration)
    return maneuver_e_limits(self);

  return genom_ok;
}


/*
 * --- sample local path according to the exec task period -----------------
 */
genom_event
mv_sample_path(const kdtp::LocalPath &p,
               sequence_maneuver_configuration_s *path, genom_context self)
{
  static const double dt = maneuver_control_period_ms/1000.;

  maneuver_configuration_s s;
  double q[4][5];
  size_t i;

  i = 2 + p.duration()/dt;
  if (path->_maximum < i || path->_maximum > 2 * i)
    if (genom_sequence_reserve(path, i))
      return mv_e_sys_error(NULL, self);
  path->_length = i;

  s.pos._present = true;
  s.pos._value.qx = 0.;
  s.pos._value.qy = 0.;

  s.vel[3] = 0.;
  s.vel[4] = 0.;

  s.acc[3] = 0.;
  s.acc[4] = 0.;

  s.jer[3] = 0.;
  s.jer[4] = 0.;

  for(i = 0; i < path->_length; i++) {
    p.getAllAt(i * dt, q);

    s.pos._value.x = q[0][0];
    s.pos._value.y = q[1][0];
    s.pos._value.z = q[2][0];
    s.pos._value.qw = std::cos(q[3][0]/2.);
    s.pos._value.qz = std::sin(q[3][0]/2.);

    s.vel[0] = q[0][1];
    s.vel[1] = q[1][1];
    s.vel[2] = q[2][1];
    s.vel[5] = q[3][1];

    s.acc[0] = q[0][2];
    s.acc[1] = q[1][2];
    s.acc[2] = q[2][2];
    s.acc[5] = q[3][2];

    s.jer[0] = q[0][3];
    s.jer[1] = q[1][3];
    s.jer[2] = q[2][3];
    s.jer[5] = q[3][3];

    path->_buffer[i] = s;
  }

  return genom_ok;
}


/*
 * --- sample velocity profile according to the exec task period -----------
 */
genom_event
mv_sample_velocity(const optional_or_t3d_pos &from, const kdtp::LocalPath &p,
                   sequence_maneuver_configuration_s *path, genom_context self)
{
  static const double dt = maneuver_control_period_ms/1000.;
  static const double dt2_2 = dt*dt/2.;
  static const double dt3_6 = dt*dt2_2/3.;

  maneuver_configuration_s s;
  double q[4][5];
  double dyaw, qw, qz, dqw, dqz;
  size_t i;

  i = 2 + p.duration()/dt;
  if (path->_maximum < i || path->_maximum > 2 * i)
    if (genom_sequence_reserve(path, i)) return maneuver_e_sys(NULL, self);
  path->_length = i;

  printf("samples %zu\n", i);
  s.pos = from;

  s.vel[3] = 0.;
  s.vel[4] = 0.;

  s.acc[3] = 0.;
  s.acc[4] = 0.;

  s.jer[3] = 0.;
  s.jer[4] = 0.;

  for(i = 0; i < path->_length; i++) {
    p.getAllAt(i * dt, q);

    s.vel[0] = q[0][0];
    s.vel[1] = q[1][0];
    s.vel[2] = q[2][0];
    s.vel[5] = q[3][0];

    s.acc[0] = q[0][1];
    s.acc[1] = q[1][1];
    s.acc[2] = q[2][1];
    s.acc[5] = q[3][1];

    s.jer[0] = q[0][2];
    s.jer[1] = q[1][2];
    s.jer[2] = q[2][2];
    s.jer[5] = q[3][2];

    path->_buffer[i] = s;

    /* integrate position if needed */
    if (s.pos._present) {
      s.pos._value.x += dt*s.vel[0] + dt2_2*s.acc[0] + dt3_6*s.jer[0];
      s.pos._value.y += dt*s.vel[1] + dt2_2*s.acc[1] + dt3_6*s.jer[1];
      s.pos._value.z += dt*s.vel[2] + dt2_2*s.acc[2] + dt3_6*s.jer[2];

      /* XXX assumes roll/pitch == 0 */
      qw = s.pos._value.qw;
      qz = s.pos._value.qz;

      dyaw = dt*s.vel[5] + dt2_2*s.acc[5] + dt3_6*s.jer[5];
      if (fabs(dyaw) < 0.25) {
        double dyaw2 = dyaw * dyaw;
        dqw = 1 - dyaw2/8;		/* cos(dyaw/2) ± 1e-5 */
        dqz = (0.5 - dyaw2/48) * dyaw;	/* sin(dyaw/2) ± 1e-6 */
      } else {
        dqw = std::cos(dyaw/2);
        dqz = std::sin(dyaw/2);
      }

      s.pos._value.qw = dqw*qw - dqz*qz;
      s.pos._value.qz = dqw*qz + dqz*qw;
    }
  }

  return genom_ok;
}
