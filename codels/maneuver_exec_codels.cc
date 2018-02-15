/*
 * Copyright (c) 2016-2018 LAAS/CNRS
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
#include <aio.h>
#include <err.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <limits>

#include "maneuver_c_types.h"
#include "codels.h"

static void	mv_exec_log(const or_time_ts &ts,
                            const maneuver_configuration_s &s,
                            maneuver_log_s *log);


/* --- Task exec -------------------------------------------------------- */


/** Codel mv_exec_start of task exec.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_wait.
 */
genom_event
mv_exec_start(maneuver_configuration_s *reference,
              maneuver_ids_trajectory_t *trajectory,
              const maneuver_desired *desired,
              const genom_context self)
{
  or_pose_estimator_state *ddata;
  struct timeval tv;
  int i;

  gettimeofday(&tv, NULL);

  ddata = desired->data(self);
  ddata->ts.sec = tv.tv_sec;
  ddata->ts.nsec = 1000*tv.tv_usec;
  ddata->intrinsic = false;
  ddata->pos._present = false;
  ddata->pos_cov._present = false;
  ddata->vel._present = false;
  ddata->vel_cov._present = false;
  ddata->acc._present = false;
  ddata->acc_cov._present = false;
  desired->write(self);

  reference->pos._present = false;
  for(i = 0; i < 6; i++) {
    reference->vel[i] = 0.;
    reference->acc[i] = 0.;
    reference->jer[i] = 0.;
  }

  trajectory->t._length = 0;
  trajectory->i = 0;

  return maneuver_wait;
}


/** Codel mv_exec_wait of task exec.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_wait, maneuver_path,
 *           maneuver_servo.
 */
genom_event
mv_exec_wait(const maneuver_ids_trajectory_t *trajectory,
             const maneuver_configuration_s *reference,
             const maneuver_desired *desired,
             const genom_context self)
{
  or_pose_estimator_state *ddata;
  int i;

  /* if there is a trajectory, play it */
  if (trajectory->t._length > 0)
    return maneuver_path;


  /* servo on a reference vel/acc/jer */
  for(i = 0; i < 6; i++)
    if (fabs(reference->vel[i]) > 1e-4) return maneuver_servo;
  for(i = 0; i < 6; i++)
    if (fabs(reference->acc[i]) > 1e-3) return maneuver_servo;
  for(i = 0; i < 6; i++)
    if (fabs(reference->jer[i]) > 1e-2) return maneuver_servo;


  /* no motion */
  ddata = desired->data(self);
  if (ddata->vel._present || ddata->acc._present) {
    struct timeval tv;

    gettimeofday(&tv, NULL);

    ddata->ts.sec = tv.tv_sec;
    ddata->ts.nsec = 1000*tv.tv_usec;

    ddata->vel._present = false;
    ddata->acc._present = false;

    desired->write(self);

    /* return once with no pause to wake up plan task (if needed) */
    return maneuver_wait;
  }

  return maneuver_pause_wait;
}


/** Codel mv_exec_path of task exec.
 *
 * Triggered by maneuver_path.
 * Yields to maneuver_pause_path, maneuver_wait.
 */
genom_event
mv_exec_path(maneuver_ids_trajectory_t *trajectory,
             const maneuver_configuration_s *reference,
             const maneuver_desired *desired, maneuver_log_s **log,
             const genom_context self)
{
  or_pose_estimator_state *ddata;
  struct timeval tv;

  /* done? */
  if (trajectory->i >= trajectory->t._length) {
    trajectory->t._length = 0;
    trajectory->i = 0;
    return maneuver_wait;
  }
  maneuver_configuration_s &s = trajectory->t._buffer[trajectory->i];

  /* publish */
  ddata = desired->data(self);

  gettimeofday(&tv, NULL);
  ddata->ts.sec = tv.tv_sec;
  ddata->ts.nsec = 1000*tv.tv_usec;

  ddata->pos = s.pos;

  ddata->vel._present = true;
  ddata->vel._value.vx = s.vel[0];
  ddata->vel._value.vy = s.vel[1];
  ddata->vel._value.vz = s.vel[2];
  ddata->vel._value.wx = s.vel[3];
  ddata->vel._value.wy = s.vel[4];
  ddata->vel._value.wz = s.vel[5];

  ddata->acc._present = true;
  ddata->acc._value.ax = s.acc[0];
  ddata->acc._value.ay = s.acc[1];
  ddata->acc._value.az = s.acc[2];

  desired->write(self);

  /* logging */
  mv_exec_log(ddata->ts, s, *log);

  /* next */
  if (++trajectory->i >= trajectory->t._length) {
    trajectory->t._length = 0;
    trajectory->i = 0;
    return maneuver_wait;
  }

  return maneuver_pause_path;
}


/** Codel mv_exec_servo of task exec.
 *
 * Triggered by maneuver_servo.
 * Yields to maneuver_pause_wait.
 */
genom_event
mv_exec_servo(maneuver_configuration_s *reference,
              const maneuver_desired *desired, maneuver_log_s **log,
              const genom_context self)
{
  static const double dt = maneuver_control_period_ms/1000.;
  static const double dt2_2 = dt*dt/2.;
  static const double dt3_6 = dt*dt2_2/3.;

  or_pose_estimator_state *ddata;
  struct timeval tv;
  int i;

  /* integration */
  or_t3d_pos &p = reference->pos._value;
  maneuver_v6d &v = reference->vel;
  maneuver_v6d &a = reference->acc;
  maneuver_v6d &j = reference->jer;

  if (reference->pos._present) {
    double dyaw, qw, qz, dqw, dqz;

    p.x += dt*v[0] + dt2_2*a[0] + dt3_6*j[0];
    p.y += dt*v[1] + dt2_2*a[1] + dt3_6*j[1];
    p.z += dt*v[2] + dt2_2*a[2] + dt3_6*j[2];

    /* XXX assumes roll/pitch == 0 */
    qw = p.qw;
    qz = p.qz;

    dyaw = dt*v[5] + dt2_2*a[5] + dt3_6*j[5];
    if (fabs(dyaw) < 0.25) {
      double dyaw2 = dyaw * dyaw;

      dqw = 1 - dyaw2/8;		/* cos(dyaw/2) ± 1e-5 */
      dqz = (0.5 - dyaw2/48) * dyaw;	/* sin(dyaw/2) ± 1e-6 */
    } else {
      dqw = std::cos(dyaw/2);
      dqz = std::sin(dyaw/2);
    }

    p.qw = dqw*qw - dqz*qz;
    p.qx = 0.;
    p.qy = 0.;
    p.qz = dqw*qz + dqz*qw;
  }

  for(i = 0; i < 6; i++) {
    v[i] += dt*a[i] + dt2_2*j[i];
    a[i] += dt*j[i];
  }

  /* publish */
  ddata = desired->data(self);

  gettimeofday(&tv, NULL);
  ddata->ts.sec = tv.tv_sec;
  ddata->ts.nsec = 1000*tv.tv_usec;

  ddata->pos = reference->pos;

  ddata->vel._present = true;
  ddata->vel._value.vx = v[0];
  ddata->vel._value.vy = v[1];
  ddata->vel._value.vz = v[2];
  ddata->vel._value.wx = v[3];
  ddata->vel._value.wy = v[4];
  ddata->vel._value.wz = v[5];

  ddata->acc._present = true;
  ddata->acc._value.ax = a[0];
  ddata->acc._value.ay = a[1];
  ddata->acc._value.az = a[2];

  desired->write(self);

  /* logging */
  mv_exec_log(ddata->ts, *reference, *log);

  /* next */
  return maneuver_pause_wait;
}


/** Codel mv_exec_stop of task exec.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 */
genom_event
mv_exec_stop(const genom_context self)
{
  return maneuver_ether;
}


/*
 * --- log -----------------------------------------------------------------
 */
static void
mv_exec_log(const or_time_ts &ts,
            const maneuver_configuration_s &s, maneuver_log_s *log)
{
  if (log->req.aio_fildes >= 0) {
    log->total++;
    if (log->total % log->decimation == 0) {
      if (log->pending) {
        if (aio_error(&log->req) != EINPROGRESS) {
          log->pending = false;
          if (aio_return(&log->req) <= 0) {
            warn("log");
            close(log->req.aio_fildes);
            log->req.aio_fildes = -1;
          }
        } else {
          log->skipped = true;
          log->missed++;
        }
      }
    }
  }

  if (log->req.aio_fildes >= 0 && !log->pending) {
    const double
      qw = s.pos._value.qw,
      qx = s.pos._value.qx,
      qy = s.pos._value.qy,
      qz = s.pos._value.qz;
    const double yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

    log->req.aio_nbytes = snprintf(
      log->buffer, sizeof(log->buffer),
      "%s" mv_log_fmt "\n",
      log->skipped ? "\n" : "",
      ts.sec, ts.nsec,
      s.pos._value.x, s.pos._value.y, s.pos._value.z, 0., 0., yaw,
      s.vel[0], s.vel[1], s.vel[2], s.vel[3], s.vel[4], s.vel[5],
      s.acc[0], s.acc[1], s.acc[2], s.acc[3], s.acc[4], s.acc[5],
      s.jer[0], s.jer[1], s.jer[2], s.jer[3], s.jer[4], s.jer[5]);

    if (aio_write(&log->req)) {
      warn("log");
      close(log->req.aio_fildes);
      log->req.aio_fildes = -1;
    } else
      log->pending = true;

    log->skipped = false;
  }
}
