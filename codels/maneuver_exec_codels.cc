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
#include <aio.h>
#include <err.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <limits>

#include "maneuver_c_types.h"
#include "codels.h"

static void	mv_extrapolate(or_rigid_body_state &s, uint32_t n);
static void	mv_exec_log(const or_time_ts &ts,
                            const or_rigid_body_state &s,
                            maneuver_log_s *log);


/* --- Task exec -------------------------------------------------------- */


/** Codel mv_exec_start of task exec.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_wait.
 */
genom_event
mv_exec_start(maneuver_ids *ids, const maneuver_desired *desired,
              const maneuver_horizon *horizon,
              const genom_context self)
{
  or_rigid_body_state *ddata;

  ddata = desired->data(self);
  ddata->ts.sec = 0;
  ddata->ts.nsec = 0;
  ddata->intrinsic = false;
  ddata->pos._present = false;
  ddata->att._present = false;
  ddata->vel._present = false;
  ddata->avel._present = false;
  ddata->acc._present = false;
  ddata->aacc._present = false;
  ddata->jerk._present = false;
  ddata->snap._present = false;
  desired->write(self);

  ids->reference.ts.sec = 0;
  ids->reference.ts.nsec = 0;
  ids->reference.intrinsic = false;
  ids->reference.pos._present = false;
  ids->reference.att._present = false;
  ids->reference.vel._present = false;
  ids->reference.avel._present = false;
  ids->reference.acc._present = false;
  ids->reference.aacc._present = false;
  ids->reference.jerk._present = false;
  ids->reference.snap._present = false;

  ids->trajectory.t._length = 0;
  ids->trajectory.i = 0;

  ids->horizondt = 0;

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
             const or_rigid_body_state *reference,
             const maneuver_desired *desired,
             const genom_context self)
{
  or_rigid_body_state *ddata;

  /* if there is a trajectory, play it */
  if (trajectory->t._length > 0)
    return maneuver_path;


  /* servo on a reference vel/acc/jer */
  if (reference->vel._present) return maneuver_servo;
  if (reference->avel._present) return maneuver_servo;
  if (reference->acc._present) return maneuver_servo;
  if (reference->aacc._present) return maneuver_servo;
  if (reference->jerk._present) return maneuver_servo;
  if (reference->snap._present) return maneuver_servo;


  /* no motion - erase remaining _present flags */
  ddata = desired->data(self);
  if (ddata->vel._present || ddata->avel._present || ddata->acc._present ||
      ddata->aacc._present || ddata->jerk._present || ddata->snap._present) {
    struct timeval tv;

    gettimeofday(&tv, NULL);

    ddata->ts.sec = tv.tv_sec;
    ddata->ts.nsec = 1000*tv.tv_usec;

    ddata->vel._present = false;
    ddata->avel._present = false;
    ddata->acc._present = false;
    ddata->aacc._present = false;
    ddata->jerk._present = false;
    ddata->snap._present = false;

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
             const or_rigid_body_state *reference,
             const maneuver_desired *desired, maneuver_log_s **log,
             uint32_t horizondt, const maneuver_horizon *horizon,
             const genom_context self)
{
  or_rigid_body_state *ddata;
  struct timeval tv;

  /* done? */
  if (trajectory->i >= trajectory->t._length) {
    trajectory->t._length = 0;
    trajectory->i = 0;
    return maneuver_wait;
  }
  or_rigid_body_state &s = trajectory->t._buffer[trajectory->i];

  /* publish */
  gettimeofday(&tv, NULL);
  ddata = desired->data(self);

  *ddata = s;
  ddata->ts.sec = tv.tv_sec;
  ddata->ts.nsec = 1000*tv.tv_usec;

  desired->write(self);

  /* if horizon is configured, update port */
  if (horizondt > 0) {
    or_rigid_body_trajectory *hdata = horizon->data(self);
    unsigned int i, j;

    /* copy existing trajectory samples - at least one by construction */
    for (j = 0, i = trajectory->i;
         j < hdata->_length && i < trajectory->t._length; j++, i += horizondt)
      hdata->_buffer[j] = trajectory->t._buffer[i];

    /* fill remaining bits with integrated values - same as mv_exec_wait */
    for (; j < hdata->_length - 1; j++) {
      hdata->_buffer[j + 1] = hdata->_buffer[j];
      mv_extrapolate(hdata->_buffer[j + 1], horizondt);
    }

    /* create timestamps */
    for (j = 0; j < hdata->_length; j++) {
      hdata->_buffer[j].ts.sec = tv.tv_sec;
      hdata->_buffer[j].ts.nsec = 1000*tv.tv_usec;

      tv.tv_usec += horizondt * maneuver_control_period_ms * 1000;
      if (tv.tv_usec >= 1000000) {
        tv.tv_sec += tv.tv_usec / 1000000;
        tv.tv_usec %= 1000000;
      }
    }

    horizon->write(self);
  }

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
mv_exec_servo(or_rigid_body_state *reference,
              const maneuver_desired *desired, maneuver_log_s **log,
              uint32_t horizondt, const maneuver_horizon *horizon,
              const genom_context self)
{
  or_rigid_body_state *ddata;
  struct timeval tv;

  /* integration */
  mv_extrapolate(*reference, 1);

  /* publish */
  gettimeofday(&tv, NULL);
  ddata = desired->data(self);

  *ddata = *reference;
  ddata->ts.sec = tv.tv_sec;
  ddata->ts.nsec = 1000*tv.tv_usec;

  desired->write(self);

  /* if horizon is configured, update port */
  if (horizondt > 0) {
    or_rigid_body_trajectory *hdata = horizon->data(self);
    unsigned int j;

    /* fill with integrated values */
    hdata->_buffer[0] = *reference;
    for (j = 0; j < hdata->_length - 1; j++) {
      hdata->_buffer[j + 1] = hdata->_buffer[j];
      mv_extrapolate(hdata->_buffer[j + 1], horizondt);
    }

    /* create timestamps */
    for (j = 0; j < hdata->_length; j++) {
      hdata->_buffer[j].ts.sec = tv.tv_sec;
      hdata->_buffer[j].ts.nsec = 1000*tv.tv_usec;

      tv.tv_usec += horizondt * maneuver_control_period_ms * 1000;
      if (tv.tv_usec >= 1000000) {
        tv.tv_sec += tv.tv_usec / 1000000;
        tv.tv_usec %= 1000000;
      }
    }

    horizon->write(self);
  }

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


/* --- Activity set_horizon --------------------------------------------- */

/** Codel mv_update_horizon of activity set_horizon.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_ether.
 * Throws maneuver_e_sys.
 */
genom_event
mv_update_horizon(uint32_t samples, const maneuver_horizon *horizon,
                  const genom_context self)
{
  if (horizon->data(self)->_maximum < samples)
    if (genom_sequence_reserve(horizon->data(self), samples))
      return mv_e_sys_error("genom_sequence_reserve", self);

  horizon->data(self)->_length = samples;
  return maneuver_ether;
}


/* --- mv_extrapolate ------------------------------------------------------ */

static void
mv_extrapolate(or_rigid_body_state &s, uint32_t n)
{
  const double dt = n * maneuver_control_period_ms/1000.;
  const double dt2_2 = dt*dt/2.;
  const double dt3_6 = dt*dt2_2/3.;

  or_t3d_pos &p   = s.pos._value;
  or_t3d_att &q   = s.att._value;
  or_t3d_vel &v   = s.vel._value;
  or_t3d_avel &w  = s.avel._value;
  or_t3d_acc &a   = s.acc._value;
  or_t3d_aacc &aw = s.aacc._value;
  or_t3d_jerk &j  = s.jerk._value;

  if (!s.vel._present) {
    s.vel._present = true;
    v.vx = v.vy = v.vz = 0.;
  }
  if (!s.avel._present) {
    s.avel._present = true;
    w.wx = w.wy = w.wz = 0.;
  }
  if (!s.acc._present) {
    s.acc._present = true;
    a.ax = a.ay = a.az = 0.;
  }
  if (!s.aacc._present) {
    s.aacc._present = true;
    aw.awx = aw.awy = aw.awz = 0.;
  }
  if (!s.jerk._present) {
    s.jerk._present = true;
    j.jx = j.jy = j.jz = 0.;
  }

  /* integration */
  if (s.pos._present && s.att._present) {
    double dyaw, qw, qz, dqw, dqz;

    p.x += dt*v.vx + dt2_2*a.ax + dt3_6*j.jx;
    p.y += dt*v.vy + dt2_2*a.ay + dt3_6*j.jy;
    p.z += dt*v.vz + dt2_2*a.az + dt3_6*j.jz;

    /* XXX assumes roll/pitch == 0 */
    qw = q.qw;
    qz = q.qz;

    dyaw = dt*w.wz + dt2_2*aw.awz;
    if (fabs(dyaw) < 0.25) {
      double dyaw2 = dyaw * dyaw;

      dqw = 1 - dyaw2/8;		/* cos(dyaw/2) ± 1e-5 */
      dqz = (0.5 - dyaw2/48) * dyaw;	/* sin(dyaw/2) ± 1e-6 */
    } else {
      dqw = std::cos(dyaw/2);
      dqz = std::sin(dyaw/2);
    }

    q.qw = dqw*qw - dqz*qz;
    q.qx = 0.;
    q.qy = 0.;
    q.qz = dqw*qz + dqz*qw;
  }

  v.vx += dt*a.ax + dt2_2*j.jx;
  v.vy += dt*a.ay + dt2_2*j.jy;
  v.vz += dt*a.az + dt2_2*j.jz;
  w.wz += dt*aw.awz;

  a.ax += dt*j.jx;
  a.ay += dt*j.jy;
  a.az += dt*j.jz;

  /* reset 0 values */
  if (fabs(v.vx) < 1e-4 && fabs(v.vy) < 1e-4 && fabs(v.vz) < 1e-4)
    s.vel._present = false;
  if (fabs(w.wx) < 1e-4 && fabs(w.wy) < 1e-4 && fabs(w.wz) < 1e-4)
    s.avel._present = false;

  if (fabs(a.ax) < 1e-3 && fabs(a.ay) < 1e-3 && fabs(a.az) < 1e-3)
    s.acc._present = false;
  if (fabs(aw.awx) < 1e-3 && fabs(aw.awy) < 1e-3 && fabs(aw.awz) < 1e-3)
    s.aacc._present = false;

  if (fabs(j.jx) < 1e-2 && fabs(j.jy) < 1e-2 && fabs(j.jz) < 1e-2)
    s.jerk._present = false;

  s.snap._present = false;
}


/*
 * --- log -----------------------------------------------------------------
 */
static void
mv_exec_log(const or_time_ts &ts,
            const or_rigid_body_state &s, maneuver_log_s *log)
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
      qw = s.att._value.qw,
      qx = s.att._value.qx,
      qy = s.att._value.qy,
      qz = s.att._value.qz;
    const double yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));
    const double
      vx = s.vel._present ? s.vel._value.vx : 0.,
      vy = s.vel._present ? s.vel._value.vy : 0.,
      vz = s.vel._present ? s.vel._value.vz : 0.,
      wx = s.avel._present ? s.avel._value.wx : 0.,
      wy = s.avel._present ? s.avel._value.wy : 0.,
      wz = s.avel._present ? s.avel._value.wz : 0.;
    const double
      ax = s.acc._present ? s.acc._value.ax : 0.,
      ay = s.acc._present ? s.acc._value.ay : 0.,
      az = s.acc._present ? s.acc._value.az : 0.,
      awx = s.aacc._present ? s.aacc._value.awx : 0.,
      awy = s.aacc._present ? s.aacc._value.awy : 0.,
      awz = s.aacc._present ? s.aacc._value.awz : 0.;
    const double
      jx = s.jerk._present ? s.jerk._value.jx : 0.,
      jy = s.jerk._present ? s.jerk._value.jy : 0.,
      jz = s.jerk._present ? s.jerk._value.jz : 0.;

    log->req.aio_nbytes = snprintf(
      log->buffer, sizeof(log->buffer),
      "%s" mv_log_fmt "\n",
      log->skipped ? "\n" : "",
      ts.sec, ts.nsec,
      s.pos._value.x, s.pos._value.y, s.pos._value.z, 0., 0., yaw,
      vx, vy, vz, wx, wy, wz,
      ax, ay, az, awx, awy, awz,
      jx, jy, jz, 0., 0., 0.);

    if (aio_write(&log->req)) {
      warn("log");
      close(log->req.aio_fildes);
      log->req.aio_fildes = -1;
    } else
      log->pending = true;

    log->skipped = false;
  }
}
