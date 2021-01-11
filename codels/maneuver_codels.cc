/*
 * Copyright (c) 2016-2021 LAAS/CNRS
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
 *                                      Anthony Mallet on Mon Aug 22 2016
 */
#include "acmaneuver.h"

#include <sys/time.h>

#include <fcntl.h>
#include <float.h>
#include <unistd.h>

#include <cstdio>
#include <cmath>

#include "maneuver_c_types.h"
#include "codels.h"



/* --- Activity take_off ------------------------------------------------ */

/** Validation codel mv_plan_cancel of activity take_off.
 *
 * Returns genom_ok.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
genom_event
mv_plan_cancel(maneuver_ids_trajectory_t *trajectory,
               or_rigid_body_state *reference,
               const genom_context self)
{
  /* (re)start from current state if a trajectory is being executed */
  if (trajectory->i + 1 < trajectory->t._length) {
    trajectory->t._buffer[0] = trajectory->t._buffer[trajectory->i];
    *reference = trajectory->t._buffer[trajectory->i + 1];
    trajectory->t._length = 1;
    trajectory->i = 0;
  }

  return genom_ok;
}


/* --- Activity goto ---------------------------------------------------- */

/** Validation codel mv_plan_cancel of activity goto.
 *
 * Returns genom_ok.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off validation */



/* --- Function velocity ------------------------------------------------ */

/** Validation codel mv_plan_cancel of function velocity.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off validation */



/* --- Activity replay -------------------------------------------------- */

/** Validation codel mv_plan_cancel of activity replay.
 *
 * Returns genom_ok.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off validation */



/* --- Activity stop ---------------------------------------------------- */

/** Validation codel mv_plan_cancel of activity stop.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off validation */



/* --- Activity set_horizon --------------------------------------------- */

/** Validation codel mv_set_horizon of activity set_horizon.
 *
 * Returns genom_ok.
 * Throws maneuver_e_sys.
 */
genom_event
mv_set_horizon(double *horizon, uint32_t *samples, uint32_t *horizondt,
               const genom_context self)
{
  uint32_t hdt;

  /* disable horizon if requested or raise an error if parameters are
   * inconsistent */
  if (!*samples && fabs(*horizon) <= DBL_MIN) {
    *horizondt = 0;
    return genom_ok;
  }
  if (*horizon <= DBL_MIN) {
    errno = EDOM;
    return mv_e_sys_error("negative horizon", self);
  }

  /* adapt parameters to the exec task period */
  if (!*samples)
    hdt = 1;
  else
    hdt = 1000. * *horizon / *samples / maneuver_control_period_ms;
  if (!hdt) {
    errno = EDOM;
    return mv_e_sys_error("too many samples", self);
  }

  /* return updated parameters */
  *horizondt = hdt;
  *samples = ceil(1000. * *horizon / hdt / maneuver_control_period_ms);
  *horizon = *samples * hdt * maneuver_control_period_ms / 1000.;

  return genom_ok;
}


/* --- Function set_bounds ---------------------------------------------- */

/** Codel mv_set_bounds of function set_bounds.
 *
 * Returns genom_ok.
 */
genom_event
mv_set_bounds(maneuver_planner_s **planner, double xmin, double xmax,
              double ymin, double ymax, double zmin, double zmax,
              double yawmin, double yawmax, const genom_context self)
{
  (*planner)->robot.getDof(0).setPositionMinMax(xmin, xmax);
  (*planner)->robot.getDof(1).setPositionMinMax(ymin, ymax);
  (*planner)->robot.getDof(2).setPositionMinMax(zmin, zmax);
  (*planner)->robot.getDof(3).setPositionMinMax(yawmin, yawmax);
  return genom_ok;
}


/* --- Function set_velocity_limit -------------------------------------- */

/** Codel mv_set_velocity_limit of function set_velocity_limit.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits.
 */
genom_event
mv_set_velocity_limit(maneuver_planner_s **planner,
                      maneuver_planner_s **vplanner, double v,
                      double w, const genom_context self)
{
  int i;

  if (v < 0. || w < 0.) return maneuver_e_limits(self);

  for(i = 0; i < 3; i++)
    (*planner)->robot.getDof(i).setVelocityMax(v);
  (*planner)->robot.getDof(3).setVelocityMax(w);

  for(i = 0; i < 3; i++) {
    (*vplanner)->robot.getDof(i).setPositionMin(-v);
    (*vplanner)->robot.getDof(i).setPositionMax(v);
  }
  (*vplanner)->robot.getDof(3).setPositionMin(-w);
  (*vplanner)->robot.getDof(3).setPositionMax(w);

  return genom_ok;
}


/* --- Function set_acceleration_limit ---------------------------------- */

/** Codel mv_set_acceleration_limit of function set_acceleration_limit.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits.
 */
genom_event
mv_set_acceleration_limit(maneuver_planner_s **planner,
                          maneuver_planner_s **vplanner, double a,
                          double dw, const genom_context self)
{
  int i;

  if (a < 0. || dw < 0.) return maneuver_e_limits(self);

  for(i = 0; i < 3; i++)
    (*planner)->robot.getDof(i).setAccelerationMax(a);
  (*planner)->robot.getDof(3).setAccelerationMax(dw);

  for(i = 0; i < 3; i++)
    (*vplanner)->robot.getDof(i).setVelocityMax(a);
  (*vplanner)->robot.getDof(3).setVelocityMax(dw);

  return genom_ok;
}


/* --- Function set_jerk_limit ------------------------------------------ */

/** Codel mv_set_jerk_limit of function set_jerk_limit.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits.
 */
genom_event
mv_set_jerk_limit(maneuver_planner_s **planner,
                  maneuver_planner_s **vplanner, double j, double ddw,
                  const genom_context self)
{
  int i;

  if (j < 0. || ddw < 0.) return maneuver_e_limits(self);

  for(i = 0; i < 3; i++)
    (*planner)->robot.getDof(i).setJerkMax(j);
  (*planner)->robot.getDof(3).setJerkMax(ddw);

  for(i = 0; i < 3; i++)
    (*vplanner)->robot.getDof(i).setAccelerationMax(j);
  (*vplanner)->robot.getDof(3).setAccelerationMax(ddw);

  return genom_ok;
}


/* --- Function set_snap_limit ------------------------------------------ */

/** Codel mv_set_snap_limit of function set_snap_limit.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits.
 */
genom_event
mv_set_snap_limit(maneuver_planner_s **planner,
                  maneuver_planner_s **vplanner, double s, double dddw,
                  const genom_context self)
{
  int i;

  if (s < 0. || dddw < 0.) return maneuver_e_limits(self);

  for(i = 0; i < 3; i++)
    (*planner)->robot.getDof(i).setSnapMax(s);
  (*planner)->robot.getDof(3).setSnapMax(dddw);

  for(i = 0; i < 3; i++) {
    (*vplanner)->robot.getDof(i).setJerkMax(s);
    (*vplanner)->robot.getDof(i).setSnapMax(100000. * s);
  }
  (*vplanner)->robot.getDof(3).setJerkMax(dddw);
  (*vplanner)->robot.getDof(3).setSnapMax(100000. * dddw);

  return genom_ok;
}


/* --- Function get_limits ---------------------------------------------- */

/** Codel mv_get_limits of function get_limits.
 *
 * Returns genom_ok.
 */
genom_event
mv_get_limits(maneuver_planner_s **planner, double *xmin, double *xmax,
              double *ymin, double *ymax, double *zmin, double *zmax,
              double *yawmin, double *yawmax, double *v, double *w,
              double *a, double *dw, double *j, double *ddw, double *s,
              double *dddw, const genom_context self)
{
  *xmin = (*planner)->robot.getDof(0).getPositionMin();
  *xmax = (*planner)->robot.getDof(0).getPositionMax();
  *ymin = (*planner)->robot.getDof(1).getPositionMin();
  *ymax = (*planner)->robot.getDof(1).getPositionMax();
  *zmin = (*planner)->robot.getDof(2).getPositionMin();
  *zmax = (*planner)->robot.getDof(2).getPositionMax();
  *yawmin = (*planner)->robot.getDof(3).getPositionMin();
  *yawmax = (*planner)->robot.getDof(3).getPositionMax();

  *v = (*planner)->robot.getDof(0).getVelocityMax();
  *w = (*planner)->robot.getDof(3).getVelocityMax();

  *a = (*planner)->robot.getDof(0).getAccelerationMax();
  *dw = (*planner)->robot.getDof(3).getAccelerationMax();

  *j = (*planner)->robot.getDof(0).getJerkMax();
  *ddw = (*planner)->robot.getDof(3).getJerkMax();

  *s = (*planner)->robot.getDof(0).getSnapMax();
  *dddw = (*planner)->robot.getDof(3).getSnapMax();

  return genom_ok;
}


/* --- Function set_state ----------------------------------------------- */

/** Codel mv_set_state of function set_state.
 *
 * Returns genom_ok.
 */
genom_event
mv_set_state(double x, double y, double z, double yaw,
             or_rigid_body_state *reference, const genom_context self)
{
  reference->pos._value.x = x;
  reference->pos._value.y = y;
  reference->pos._value.z = z;
  reference->pos._present = true;

  reference->att._value.qw = std::cos(yaw/2.);
  reference->att._value.qx = 0.;
  reference->att._value.qy = 0.;
  reference->att._value.qz = std::sin(yaw/2.);
  reference->att._present = true;

  /* reset velocity and derivatives */
  reference->vel._present = false;
  reference->avel._present = false;
  reference->acc._present = false;
  reference->aacc._present = false;
  reference->jerk._present = false;

  return genom_ok;
}


/* --- Function velocity ------------------------------------------------ */

/** Codel mv_plan_velocity of function velocity.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits, maneuver_e_sys.
 */
genom_event
mv_plan_velocity(const maneuver_planner_s *vplanner,
                 const or_rigid_body_state *reference, double vx,
                 double vy, double vz, double wz, double ax, double ay,
                 double az, double duration,
                 sequence_or_rigid_body_state *path,
                 const genom_context self)
{
  kdtp::State from(vplanner->robot);
  kdtp::State to(vplanner->robot);
  genom_event e;

  /* uses the planner for velocity: position() is actually velocity etc. */
  if (reference->vel._present) {
    from.position()[0] = reference->vel._value.vx;
    from.position()[1] = reference->vel._value.vy;
    from.position()[2] = reference->vel._value.vz;
  }
  if (reference->avel._present) {
    from.position()[3] = reference->avel._value.wz;
  }

  if (reference->acc._present) {
    from.velocity()[0] = reference->acc._value.ax;
    from.velocity()[1] = reference->acc._value.ay;
    from.velocity()[2] = reference->acc._value.az;
  }
  if (reference->aacc._present) {
    from.velocity()[3] = reference->aacc._value.awz;
  }

  if (reference->jerk._present) {
    from.acceleration()[0] = reference->jerk._value.jx;
    from.acceleration()[1] = reference->jerk._value.jy;
    from.acceleration()[2] = reference->jerk._value.jz;
  }

  to.position()[0] = vx;
  to.position()[1] = vy;
  to.position()[2] = vz;
  to.position()[3] = wz;

  to.velocity()[0] = ax;
  to.velocity()[1] = ay;
  to.velocity()[2] = az;

  kdtp::LocalPath lpath(vplanner->robot, from, to, duration);

  e = mv_check_duration(lpath, duration, self);
  if (e) return e;
  e = mv_sample_velocity(reference->pos, reference->att, lpath, path, self);
  if (e) return e;

  return genom_ok;
}

/** Codel mv_push_path of function velocity.
 *
 * Returns genom_ok.
 * Throws maneuver_e_limits, maneuver_e_sys.
 */
genom_event
mv_push_path(const sequence_or_rigid_body_state *path,
             or_rigid_body_state *reference,
             maneuver_ids_trajectory_t *trajectory,
             const genom_context self)
{
  size_t i, l;

  if (!path->_length) return genom_ok;

  l = trajectory->t._length + path->_length;
  if (trajectory->t._maximum < l || trajectory->t._maximum > 2 * l)
    if (genom_sequence_reserve(&trajectory->t, l))
      return mv_e_sys_error(NULL, self);

  for(i = 0; i < path->_length; i++)
    trajectory->t._buffer[trajectory->t._length++] = path->_buffer[i];

  *reference = path->_buffer[path->_length-1];

  return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel mv_log of function log.
 *
 * Returns genom_ok.
 * Throws maneuver_e_sys.
 */
genom_event
mv_log(const char path[64], uint32_t decimation, maneuver_log_s **log,
       const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return mv_e_sys_error(path, self);

  if (write(fd, mv_log_header_fmt "\n", sizeof(mv_log_header_fmt)) < 0)
    return mv_e_sys_error(path, self);

  if ((*log)->req.aio_fildes >= 0) {
    close((*log)->req.aio_fildes);

    if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
        /* empty body */;
  }
  (*log)->req.aio_fildes = fd;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel mv_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
mv_log_stop(maneuver_log_s **log, const genom_context self)
{
  if (*log && (*log)->req.aio_fildes >= 0)
    close((*log)->req.aio_fildes);
  (*log)->req.aio_fildes = -1;

  return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel mv_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
mv_log_info(const maneuver_log_s *log, uint32_t *miss, uint32_t *total,
            const genom_context self)
{
  *miss = *total = 0;
  if (log) {
    *miss = log->missed;
    *total = log->total;
  }

  return genom_ok;
}
