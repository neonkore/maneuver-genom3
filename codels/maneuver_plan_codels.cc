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

#include <cmath>

#include "maneuver_c_types.h"
#include "codels.h"


/* --- Task plan -------------------------------------------------------- */


/** Codel mv_plan_start of task plan.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_ether.
 */
genom_event
mv_plan_start(maneuver_ids *ids, const genom_context self)
{
  static const double vmax = 1.;
  static const double amax = 1.;
  static const double jmax = 5.;
  static const double wmax = 1.;

  /* the trajectory planner */
  ids->planner = new maneuver_planner_s;
  ids->planner->robot.addDof(
    kdtp::Dof(-3.5, 3.5, vmax, amax, jmax, 50*jmax, false));
  ids->planner->robot.addDof(
    kdtp::Dof(-1.5, 1.5, vmax, amax, jmax, 50*jmax, false));
  ids->planner->robot.addDof(
    kdtp::Dof( 0.,  5.,  vmax, amax, jmax, 50*jmax, false));
  ids->planner->robot.addDof(
    kdtp::Dof(-3*M_PI, 3*M_PI, wmax, 10*wmax, 100*wmax, 1000*wmax, true));

  /* the velocity planner */
  ids->vplanner = new maneuver_planner_s;
  ids->vplanner->robot.addDof(
    kdtp::Dof(-vmax, vmax, amax, jmax, 50*jmax, 1000*jmax, false));
  ids->vplanner->robot.addDof(
    kdtp::Dof(-vmax, vmax, amax, jmax, 50*jmax, 1000*jmax, false));
  ids->vplanner->robot.addDof(
    kdtp::Dof(-vmax, vmax, amax, jmax, 50*jmax, 1000*jmax, false));
  ids->vplanner->robot.addDof(
    kdtp::Dof(-wmax, wmax, 10*wmax, 100*wmax, 1000*wmax, 10000*wmax, true));

  /* init logging */
  ids->log = new maneuver_log_s;
  if (!ids->log) abort();

  ids->log->req.aio_fildes = -1;
  ids->log->req.aio_offset = 0;
  ids->log->req.aio_buf = ids->log->buffer;
  ids->log->req.aio_nbytes = 0;
  ids->log->req.aio_reqprio = 0;
  ids->log->req.aio_sigevent.sigev_notify = SIGEV_NONE;
  ids->log->req.aio_lio_opcode = LIO_NOP;
  ids->log->pending = false;
  ids->log->skipped = false;
  ids->log->decimation = 1;
  ids->log->missed = 0;
  ids->log->total = 0;

  return maneuver_ether;
}


/** Codel mv_plan_stop of task plan.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 */
genom_event
mv_plan_stop(maneuver_ids *ids, const genom_context self)
{
  mv_log_stop(&ids->log, self);

  delete ids->planner;
  delete ids->vplanner;
  delete ids->log;

  return maneuver_ether;
}


/* --- Activity set_current_state --------------------------------------- */

/** Codel mv_current_state_read of activity set_current_state.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate.
 */
genom_event
mv_current_state_read(const maneuver_state *state,
                      or_rigid_body_state *reference,
                      const genom_context self)
{
  or_pose_estimator_state *sdata;

  if (state->read(self) != genom_ok) return maneuver_e_nostate(self);
  sdata = state->data(self);
  if (!sdata) return maneuver_e_nostate(self);

  reference->pos = sdata->pos;
  reference->att = sdata->att;
  if (!sdata->pos._present) return maneuver_e_nostate(self);
  if (!sdata->att._present) return maneuver_e_nostate(self);

  /* disregard current velocity and acceleration - the residual noise
   * in those quantities generates weird trajectories */
  reference->vel._present = false;
  reference->avel._present = false;
  reference->acc._present = false;
  reference->aacc._present = false;
  reference->jerk._present = false;

  return maneuver_ether;
}


/* --- Activity take_off ------------------------------------------------ */

/** Codel mv_plan_take_off of activity take_off.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_take_off(const maneuver_planner_s *planner,
                 const or_rigid_body_state *reference, double height,
                 double duration, sequence_or_rigid_body_state *path,
                 const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!reference->pos._present) return maneuver_e_nostate(self);
  if (!reference->att._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &reference->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;

  const or_t3d_att *q = &reference->att._value;
  from.position()[3] = atan2(
    2 * (q->qw*q->qz + q->qx*q->qy), 1 - 2 * (q->qy*q->qy + q->qz*q->qz));

  if (reference->vel._present) {
    from.velocity()[0] = reference->vel._value.vx;
    from.velocity()[1] = reference->vel._value.vy;
    from.velocity()[2] = reference->vel._value.vz;
  }
  if (reference->avel._present) {
    from.velocity()[3] = reference->avel._value.wz;
  }

  if (reference->acc._present) {
    from.acceleration()[0] = reference->acc._value.ax;
    from.acceleration()[1] = reference->acc._value.ay;
    from.acceleration()[2] = reference->acc._value.az;
  }
  if (reference->aacc._present) {
    from.acceleration()[3] = reference->aacc._value.awz;
  }

  to.position() = from.position();
  to.position()[2] = height;

  kdtp::LocalPath lpath(planner->robot, from, to, duration);

  e = mv_check_duration(lpath, duration, self);
  if (e) return e;
  e = mv_sample_path(lpath, path, self);
  if (e) return e;

  return maneuver_exec;
}

/** Codel mv_plan_exec of activity take_off.
 *
 * Triggered by maneuver_exec.
 * Yields to maneuver_wait.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_exec(const sequence_or_rigid_body_state *path,
             or_rigid_body_state *reference,
             maneuver_ids_trajectory_t *trajectory,
             const genom_context self)
{
  genom_event e;

  e = mv_push_path(path, reference, trajectory, self);
  if (e) return e;

  return maneuver_wait;
}

/** Codel mv_plan_wait of activity take_off.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_wait(const maneuver_ids_trajectory_t *trajectory,
             const genom_context self)
{
  return trajectory->t._length > 0 ? maneuver_pause_wait : maneuver_ether;
}


/* --- Activity goto ---------------------------------------------------- */

/** Codel mv_plan_goto of activity goto.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_goto(const maneuver_planner_s *planner,
             const or_rigid_body_state *reference, double x, double y,
             double z, double yaw, double duration,
             sequence_or_rigid_body_state *path,
             const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!reference->pos._present) return maneuver_e_nostate(self);
  if (!reference->att._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &reference->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;

  const or_t3d_att *q = &reference->att._value;
  from.position()[3] = atan2(
    2 * (q->qw*q->qz + q->qx*q->qy), 1 - 2 * (q->qy*q->qy + q->qz*q->qz));

  if (reference->vel._present) {
    from.velocity()[0] = reference->vel._value.vx;
    from.velocity()[1] = reference->vel._value.vy;
    from.velocity()[2] = reference->vel._value.vz;
  }
  if (reference->avel._present) {
    from.velocity()[3] = reference->avel._value.wz;
  }

  if (reference->acc._present) {
    from.acceleration()[0] = reference->acc._value.ax;
    from.acceleration()[1] = reference->acc._value.ay;
    from.acceleration()[2] = reference->acc._value.az;
  }
  if (reference->aacc._present) {
    from.acceleration()[3] = reference->aacc._value.awz;
  }

  to.position()[0] = x;
  to.position()[1] = y;
  to.position()[2] = z;
  to.position()[3] = yaw;

  kdtp::LocalPath lpath(planner->robot, from, to, duration);

  e = mv_check_duration(lpath, duration, self);
  if (e) return e;
  e = mv_sample_path(lpath, path, self);
  if (e) return e;

  return maneuver_exec;
}

/** Codel mv_plan_exec of activity goto.
 *
 * Triggered by maneuver_exec.
 * Yields to maneuver_wait.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
/* already defined in service take_off */


/** Codel mv_plan_wait of activity goto.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
/* already defined in service take_off */


/* --- Activity waypoint ------------------------------------------------ */

/** Codel mv_plan_waypoint of activity waypoint.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_waypoint(const maneuver_planner_s *planner,
                 const or_rigid_body_state *reference, double x,
                 double y, double z, double yaw, double vx, double vy,
                 double vz, double wz, double ax, double ay, double az,
                 double duration, sequence_or_rigid_body_state *path,
                 const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!reference->pos._present) return maneuver_e_nostate(self);
  if (!reference->att._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &reference->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;

  const or_t3d_att *q = &reference->att._value;
  from.position()[3] = atan2(
    2 * (q->qw*q->qz + q->qx*q->qy), 1 - 2 * (q->qy*q->qy + q->qz*q->qz));

  if (reference->vel._present) {
    from.velocity()[0] = reference->vel._value.vx;
    from.velocity()[1] = reference->vel._value.vy;
    from.velocity()[2] = reference->vel._value.vz;
  }
  if (reference->avel._present) {
    from.velocity()[3] = reference->avel._value.wz;
  }

  if (reference->acc._present) {
    from.acceleration()[0] = reference->acc._value.ax;
    from.acceleration()[1] = reference->acc._value.ay;
    from.acceleration()[2] = reference->acc._value.az;
  }
  if (reference->aacc._present) {
    from.acceleration()[3] = reference->aacc._value.awz;
  }

  to.position()[0] = x;
  to.position()[1] = y;
  to.position()[2] = z;
  to.position()[3] = yaw;

  to.velocity()[0] = vx;
  to.velocity()[1] = vy;
  to.velocity()[2] = vz;
  to.velocity()[3] = wz;

  to.acceleration()[0] = ax;
  to.acceleration()[1] = ay;
  to.acceleration()[2] = az;

  kdtp::LocalPath lpath(planner->robot, from, to, duration);

  e = mv_check_duration(lpath, duration, self);
  if (e) return e;
  e = mv_sample_path(lpath, path, self);
  if (e) return e;

  return maneuver_exec;
}

/** Codel mv_plan_exec of activity waypoint.
 *
 * Triggered by maneuver_exec, maneuver_stop.
 * Yields to maneuver_wait.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
/* already defined in service take_off */


/** Codel mv_no_wait of activity waypoint.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_no_wait(const genom_context self)
{
  return maneuver_ether;
}


/* --- Activity replay -------------------------------------------------- */

/** Codel mv_replay_read of activity replay.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
genom_event
mv_replay_read(const maneuver_planner_s *planner,
               const char filename[128],
               sequence_or_rigid_body_state *path,
               const genom_context self)
{
  sequence_or_rigid_body_state interp;
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  or_rigid_body_state s;

  char line[1024];
  double now, dt;
  genom_event e;
  double roll, pitch, yaw;
  size_t i;
  char *l;
  FILE *f;
  int n, c;

  f = fopen(filename, "r");
  if (!f) return mv_e_sys_error(filename, self);

  path->_length = 0;
  now = 0; /* false positive -Werror=maybe-uninitialized */

  interp._maximum = 0;
  interp._length = 0;
  interp._buffer = NULL;
  interp._release = NULL;

  s.ts.sec = 0;
  s.ts.nsec = 0;
  s.intrinsic = false;
  s.pos._present = true;
  s.att._present = true;
  s.att._value.qx = 0.;
  s.att._value.qy = 0.;
  s.jerk._present = false;
  s.snap._present = false;

  do {
    /* get next line */
    l = fgets(line, sizeof(line), f);
    if (!l) {
      n = errno;
      if (feof(f)) break;

      fclose(f);
      errno = n;
      return mv_e_sys_error(filename, self);
    }

    /* read timestamp */
    n = sscanf(l, "%lf%n", &dt, &c);
    if (n != 1) continue; else l += c;

    /* read position */
    n = sscanf(l, "%lf %lf %lf %lf %lf %lf%n",
               &s.pos._value.x, &s.pos._value.y, &s.pos._value.z,
               &roll, &pitch, &yaw,
               &c);
    if (n != 6) continue; else l += c;
    s.att._value.qw = std::cos(yaw/2.);
    s.att._value.qz = std::sin(yaw/2.);

    /* read optional velocity */
    n = sscanf(l, "%lf %lf %lf %lf %lf %lf%n",
               &s.vel._value.vx, &s.vel._value.vy, &s.vel._value.vz,
               &s.avel._value.wx, &s.avel._value.wy, &s.avel._value.wz,
               &c);
    if (n == 6) {
      l += c;
      s.vel._present = s.avel._present = true;

      /* read optional acceleration */
      n = sscanf(l, "%lf %lf %lf %lf %lf %lf%n",
                 &s.acc._value.ax, &s.acc._value.ax, &s.acc._value.ax,
                 &s.aacc._value.awx, &s.aacc._value.awx, &s.aacc._value.awx,
                 &c);
      if (n != 6) {
        s.acc._present = s.aacc._present = false;
      }
    } else {
      s.vel._present = s.avel._present = false;
      s.acc._present = s.aacc._present = false;
    }

    /* resize path if needed */
    if (path->_length >= path->_maximum)
      if (genom_sequence_reserve(path, path->_length + 256))
        return mv_e_sys_error(NULL, self);

    /* output first sample directly */
    if (!path->_length) {
      path->_buffer[path->_length++] = s;
      now = dt + maneuver_control_period_ms / 1000.;
      continue;
    }

    /* time offset of current sample */
    dt -= now;

    /* skip too high frequency samples */
    if (dt < - maneuver_control_period_ms / 1000. / 2.) continue;

    /* output sample at right frequency directly */
    if (dt < maneuver_control_period_ms / 1000. / 2.) {
      path->_buffer[path->_length++] = s;
      now += maneuver_control_period_ms / 1000.;
      continue;
    }

    /* interpolate sample at too low frequency */
    const or_rigid_body_state &s0 = path->_buffer[path->_length - 1];
    from.position()[0] = s0.pos._value.x;
    from.position()[1] = s0.pos._value.y;
    from.position()[2] = s0.pos._value.z;
    from.position()[3] = 2 * atan2(s0.att._value.qz, s0.att._value.qw);

    if (s0.vel._present) {
      from.velocity()[0] = s0.vel._value.vx;
      from.velocity()[1] = s0.vel._value.vy;
      from.velocity()[2] = s0.vel._value.vz;
    }
    if (s0.avel._present) {
      from.velocity()[3] = s0.avel._value.wz;
    }

    if (s0.acc._present) {
      from.acceleration()[0] = s0.acc._value.ax;
      from.acceleration()[1] = s0.acc._value.ay;
      from.acceleration()[2] = s0.acc._value.az;
    }
    if (s0.aacc._present) {
      from.acceleration()[3] = s0.aacc._value.awz;
    }

    to.position()[0] = s.pos._value.x;
    to.position()[1] = s.pos._value.y;
    to.position()[2] = s.pos._value.z;
    to.position()[3] = 2 * atan2(s.att._value.qz, s.att._value.qw);

    if (s.vel._present) {
      to.velocity()[0] = s.vel._value.vx;
      to.velocity()[1] = s.vel._value.vy;
      to.velocity()[2] = s.vel._value.vz;
    }
    if (s.avel._present) {
      to.velocity()[3] = s.avel._value.wz;
    }

    if (s.acc._present) {
      to.acceleration()[0] = s.acc._value.ax;
      to.acceleration()[1] = s.acc._value.ay;
      to.acceleration()[2] = s.acc._value.az;
    }
    if (s.aacc._present) {
      to.acceleration()[3] = s.aacc._value.awz;
    }

    kdtp::LocalPath lpath(planner->robot, from, to, dt);

    e = mv_sample_path(lpath, &interp, self);
    if (e) return e;

    if (path->_length + interp._length >= path->_maximum)
      if (genom_sequence_reserve(path, path->_length + interp._length))
        return maneuver_e_sys(NULL, self);
    for(i = 0; i < interp._length; i++)
      path->_buffer[path->_length++] = interp._buffer[i];

    now += (1 + interp._length) * maneuver_control_period_ms / 1000.;
  } while(!feof(f));

  fclose(f);

  if (!path->_length) {
    errno = ENOMSG;
    return mv_e_sys_error(filename, self);
  }

  return maneuver_exec;
}

/** Codel mv_plan_exec of activity replay.
 *
 * Triggered by maneuver_exec.
 * Yields to maneuver_wait.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off */


/** Codel mv_plan_wait of activity replay.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off */


/* --- Activity wait ---------------------------------------------------- */

/** Codel mv_plan_wait of activity wait.
 *
 * Triggered by maneuver_start, maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 */
/* already defined in service take_off */



/* --- Activity stop ---------------------------------------------------- */

/** Codel mv_plan_zero of activity stop.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_exec, maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_zero(const maneuver_planner_s *vplanner,
             const or_rigid_body_state *reference,
             sequence_or_rigid_body_state *path,
             const genom_context self)
{
  genom_event e;

  /* goto 0 velocity in minimum time */
  e = mv_plan_velocity(vplanner, reference,
                       0., 0., 0., 0., 0., 0., 0., 0.,
                       path, self);
  if (e) return e;

  return maneuver_exec;
}

/** Codel mv_plan_exec of activity stop.
 *
 * Triggered by maneuver_exec.
 * Yields to maneuver_wait.
 */
/* already defined in service take_off */


/** Codel mv_plan_wait of activity stop.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 */
/* already defined in service take_off */
