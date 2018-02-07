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
#include <cstdio>
#include <iostream>

#include "maneuver_c_types.h"
#include "codels.h"


/* --- Task plan -------------------------------------------------------- */

static genom_event	mv_check_duration(const kdtp::LocalPath &p,
                                          const double duration,
                                          const genom_context self);
static genom_event	mv_sample_path(const kdtp::LocalPath &p,
                                       sequence_or_pose_estimator_state *path,
                                       genom_context self);

/** Codel mv_plan_start of task plan.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_ether.
 */
genom_event
mv_plan_start(maneuver_ids *ids, const genom_context self)
{
  static const double vmax = 1;
  static const double amax = 5;
  static const double wmax = 1.5;

  ids->planner = new maneuver_planner_s;

  ids->planner->robot.addDof(
    kdtp::Dof(-3.5, 3.5, vmax, amax, amax, 10*amax, false));
  ids->planner->robot.addDof(
    kdtp::Dof(-1.5, 1.5, vmax, amax, amax, 10*amax, false));
  ids->planner->robot.addDof(
    kdtp::Dof( 0.,  5.,  vmax, amax, amax, 10*amax, false));
  ids->planner->robot.addDof(
    kdtp::Dof(-3*M_PI, 3*M_PI, wmax, 10*wmax, 100*wmax, 100*wmax, true));

  ids->current.pos._present = false;
  ids->current.pos_cov._present = false;
  ids->current.vel._present = false;
  ids->current.vel_cov._present = false;
  ids->current.acc._present = false;
  ids->current.acc_cov._present = false;

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
  delete ids->log;

  return maneuver_ether;
}


/* --- Activity set_current_state --------------------------------------- */

/** Codel mv_current_state_start of activity set_current_state.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_ether.
 */
genom_event
mv_current_state_start(const maneuver_state *state,
                       or_pose_estimator_state *current,
                       const genom_context self)
{
  if (state->read(self) != genom_ok) return maneuver_e_nostate(self);
  if (!state->data(self)->pos._present) return maneuver_e_nostate(self);
  *current = *state->data(self);

  /* disregard current velocity and acceleration - the residual noise
   * in those quantities generates weird trajectories */
  current->vel._present = false;
  current->acc._present = false;

  return maneuver_ether;
}


/* --- Activity take_off ------------------------------------------------ */

/** Codel mv_plan_cancel of activity take_off.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_plan.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_cancel(or_pose_estimator_state *current,
               maneuver_ids_trajectory_s *trajectory,
               const genom_context self)
{
  /* cancel any trajectory being executed */
  if (trajectory->t._length > 0) {
    *current = trajectory->t._buffer[trajectory->i];
    trajectory->t._length = 0;
  }

  return maneuver_plan;
}

/** Codel mv_take_off_plan of activity take_off.
 *
 * Triggered by maneuver_plan.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_take_off_plan(const maneuver_planner_s *planner,
                 const or_pose_estimator_state *current, double height,
                 double duration,
                 sequence_or_pose_estimator_state *path,
                 const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!current->pos._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &current->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;
  from.position()[3] = atan2(
    2 * (p->qw*p->qz + p->qx*p->qy), 1 - 2 * (p->qy*p->qy + p->qz*p->qz));
  if (current->vel._present) {
    const or_t3d_vel *v = &current->vel._value;
    from.velocity()[0] = v->vx;
    from.velocity()[1] = v->vy;
    from.velocity()[2] = v->vz;
    from.velocity()[3] = v->wz;
  }
  if (current->acc._present) {
    const or_t3d_acc *a = &current->acc._value;
    from.acceleration()[0] = a->ax;
    from.acceleration()[1] = a->ay;
    from.acceleration()[2] = a->az;
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
 * Yields to maneuver_pause_exec, maneuver_wait.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_exec(const maneuver_planner_s *planner,
             const sequence_or_pose_estimator_state *path,
             or_pose_estimator_state *current,
             maneuver_ids_trajectory_s *trajectory,
             const genom_context self)
{
  size_t i;

  if (trajectory->t._length > 0) return maneuver_pause_exec;

  if (genom_sequence_reserve(&trajectory->t, path->_length))
    return mv_e_sys_error(NULL, self);

  for(i = 0; i < path->_length; i++)
    trajectory->t._buffer[i] = path->_buffer[i];
  trajectory->t._length = path->_length;

  *current = path->_buffer[path->_length-1];

  return maneuver_wait;
}

/** Codel mv_plan_exec_wait of activity take_off.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_exec_wait(const maneuver_ids_trajectory_s *trajectory,
                  const genom_context self)
{
  if (trajectory->t._length > 0) return maneuver_pause_wait;
  return maneuver_ether;
}

/** Codel mv_plan_exec_stop of activity take_off.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_plan_exec_stop(maneuver_ids_trajectory_s *trajectory,
                  or_pose_estimator_state *current,
                  const genom_context self)
{
  /* (re)start from current state if a trajectory is being executed */
  if (trajectory->t._length > 0) {
    *current = trajectory->t._buffer[trajectory->i];
    current->vel._present = false;
    current->acc._present = false;
    trajectory->t._length = 0;
  }

  return maneuver_ether;
}


/* --- Activity goto ---------------------------------------------------- */

/** Codel mv_plan_cancel of activity goto.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_plan.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
/* already defined in service take_off */


/** Codel mv_goto_plan of activity goto.
 *
 * Triggered by maneuver_plan.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_goto_plan(const maneuver_planner_s *planner,
             const or_pose_estimator_state *current, double x,
             double y, double z, double yaw, double duration,
             sequence_or_pose_estimator_state *path,
             const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!current->pos._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &current->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;
  from.position()[3] = atan2(
    2 * (p->qw*p->qz + p->qx*p->qy), 1 - 2 * (p->qy*p->qy + p->qz*p->qz));
  if (current->vel._present) {
    const or_t3d_vel *v = &current->vel._value;
    from.velocity()[0] = v->vx;
    from.velocity()[1] = v->vy;
    from.velocity()[2] = v->vz;
    from.velocity()[3] = v->wz;
  }
  if (current->acc._present) {
    const or_t3d_acc *a = &current->acc._value;
    from.acceleration()[0] = a->ax;
    from.acceleration()[1] = a->ay;
    from.acceleration()[2] = a->az;
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
 * Yields to maneuver_pause_exec, maneuver_wait.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
/* already defined in service take_off */


/** Codel mv_plan_exec_wait of activity goto.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
/* already defined in service take_off */


/** Codel mv_plan_exec_stop of activity goto.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
/* already defined in service take_off */



/* --- Activity waypoint ------------------------------------------------ */

/** Codel mv_waypoint_plan of activity waypoint.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_waypoint_plan(const maneuver_planner_s *planner,
                 const or_pose_estimator_state *current, double x,
                 double y, double z, double yaw, double vx, double vy,
                 double vz, double wz, double ax, double ay, double az,
                 double duration,
                 sequence_or_pose_estimator_state *path,
                 const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!current->pos._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &current->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;
  from.position()[3] = atan2(
    2 * (p->qw*p->qz + p->qx*p->qy), 1 - 2 * (p->qy*p->qy + p->qz*p->qz));
  if (current->vel._present) {
    const or_t3d_vel *v = &current->vel._value;
    from.velocity()[0] = v->vx;
    from.velocity()[1] = v->vy;
    from.velocity()[2] = v->vz;
    from.velocity()[3] = v->wz;
  }
  if (current->acc._present) {
    const or_t3d_acc *a = &current->acc._value;
    from.acceleration()[0] = a->ax;
    from.acceleration()[1] = a->ay;
    from.acceleration()[2] = a->az;
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

/** Codel mv_waypoint_add of activity waypoint.
 *
 * Triggered by maneuver_exec, maneuver_stop.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits.
 */
genom_event
mv_waypoint_add(const maneuver_planner_s *planner,
                const sequence_or_pose_estimator_state *path,
                or_pose_estimator_state *current,
                maneuver_ids_trajectory_s *trajectory,
                const genom_context self)
{
  size_t i, l;

  l = trajectory->t._length;
  if (genom_sequence_reserve(&trajectory->t, l + path->_length))
    return maneuver_e_sys(NULL, self);

  for(i = 0; i < path->_length; i++)
    trajectory->t._buffer[l + i] = path->_buffer[i];
  trajectory->t._length = l + path->_length;

  *current = path->_buffer[path->_length-1];

  return maneuver_ether;
}


/* --- Activity wait ---------------------------------------------------- */

/** Codel mv_plan_exec_wait of activity wait.
 *
 * Triggered by maneuver_start, maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 */
/* already defined in service take_off */



/* --- Activity replay -------------------------------------------------- */

/** Codel mv_replay_read of activity replay.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
genom_event
mv_replay_read(const maneuver_planner_s *planner,
               const or_pose_estimator_state *current,
               const char filename[128],
               sequence_or_pose_estimator_state *path,
               const genom_context self)
{
  sequence_or_pose_estimator_state interp;
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  or_pose_estimator_state s;
  char line[1024];
  double now, dt;
  genom_event e;
  double yaw;
  size_t i;
  char *l;
  FILE *f;
  int n;

  f = fopen(filename, "r");
  if (!f) return mv_e_sys_error(filename, self);

  path->_length = 0;

  interp._maximum = 0;
  interp._length = 0;
  interp._buffer = NULL;
  interp._release = NULL;

  s.intrinsic = false;

  s.pos._present = true;
  s.pos._value.qx = 0.;
  s.pos._value.qy = 0.;
  s.pos_cov._present = false;

  s.vel._present = true;
  s.vel._value.wx = 0.;
  s.vel._value.wy = 0.;
  s.vel_cov._present = false;

  s.acc._present = true;
  s.acc_cov._present = false;

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

    n = sscanf(line, mv_replay_fmt,
               &s.ts.sec, &s.ts.nsec,
               &s.pos._value.x, &s.pos._value.y, &s.pos._value.z, &yaw,
               &s.vel._value.vx, &s.vel._value.vy, &s.vel._value.vz,
               &s.vel._value.wz,
               &s.acc._value.ax, &s.acc._value.ay, &s.acc._value.az);
    if (n != 13) continue;

    s.pos._value.qw = std::cos(yaw/2.);
    s.pos._value.qz = std::sin(yaw/2.);

    /* resize path if needed */
    if (path->_length >= path->_maximum)
      if (genom_sequence_reserve(path, path->_length + 256))
        return maneuver_e_sys(NULL, self);

    /* output first sample directly */
    if (!path->_length) {
      path->_buffer[path->_length++] = s;
      now = s.ts.sec + s.ts.nsec * 1e-9 + maneuver_control_period_ms / 1000.;
      continue;
    }

    /* time offset of current sample */
    dt = s.ts.sec + s.ts.nsec * 1e-9 - now;

    /* skip too high frequency samples */
    if (dt < - maneuver_control_period_ms / 1000. / 2.) continue;

    /* output sample at right frequency directly */
    if (dt < maneuver_control_period_ms / 1000. / 2.) {
      path->_buffer[path->_length++] = s;
      now += maneuver_control_period_ms / 1000.;
      continue;
    }

    /* interpolate sample at too low frequency */
    const or_t3d_pos *p0 = &path->_buffer[path->_length - 1].pos._value;
    from.position()[0] = p0->x;
    from.position()[1] = p0->y;
    from.position()[2] = p0->z;
    from.position()[3] = 2 * atan2(p0->qz, p0->qw);

    const or_t3d_vel *v0 = &path->_buffer[path->_length - 1].vel._value;
    from.velocity()[0] = v0->vx;
    from.velocity()[1] = v0->vy;
    from.velocity()[2] = v0->vz;
    from.velocity()[3] = v0->wz;

    const or_t3d_acc *a0 = &path->_buffer[path->_length - 1].acc._value;
    from.acceleration()[0] = a0->ax;
    from.acceleration()[1] = a0->ay;
    from.acceleration()[2] = a0->az;

    const or_t3d_pos *p1 = &s.pos._value;
    to.position()[0] = p1->x;
    to.position()[1] = p1->y;
    to.position()[2] = p1->z;
    to.position()[3] = 2 * atan2(p1->qz, p1->qw);

    const or_t3d_vel *v1 = &s.vel._value;
    to.velocity()[0] = v1->vx;
    to.velocity()[1] = v1->vy;
    to.velocity()[2] = v1->vz;
    to.velocity()[3] = v1->wz;

    const or_t3d_acc *a1 = &s.acc._value;
    to.acceleration()[0] = a1->ax;
    to.acceleration()[1] = a1->ay;
    to.acceleration()[2] = a1->az;

    kdtp::LocalPath lpath(planner->robot, from, to, dt);

    e = mv_sample_path(lpath, &interp, self);
    if (e) return e;

    if (path->_length + interp._length >= path->_maximum)
      if (genom_sequence_reserve(path, path->_length + interp._length))
        return maneuver_e_sys(NULL, self);
    for(i = 0; i < interp._length; i++) {
      dt = i * maneuver_control_period_ms / 1000.;
      interp._buffer[i].ts.sec = floor(dt + now);
      interp._buffer[i].ts.nsec = 1e9*(dt + now - interp._buffer[i].ts.sec);

      path->_buffer[path->_length++] = interp._buffer[i];
    }

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
 * Yields to maneuver_pause_exec, maneuver_wait.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off */


/** Codel mv_plan_exec_wait of activity replay.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off */


/** Codel mv_plan_exec_stop of activity replay.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate, maneuver_e_limits, maneuver_e_sys.
 */
/* already defined in service take_off */


/* --- local functions ----------------------------------------------------- */

/*
 * consider limits
 */
static genom_event
mv_check_duration(const kdtp::LocalPath &p, const double duration,
                  const genom_context self)
{
  /* consider limits */
  if (duration > 0. && p.duration() > duration)
    return maneuver_e_limits(self);

  return genom_ok;
}

/*
 * sample local path according to the exec task period
 */
static genom_event
mv_sample_path(const kdtp::LocalPath &p,
               sequence_or_pose_estimator_state *path, genom_context self)
{
  std::vector<std::vector<double> > q;
  or_pose_estimator_state s;
  double t;
  size_t i;

  i = 1 + 1000 * p.duration()/maneuver_control_period_ms;
  if (genom_sequence_reserve(path, i)) return maneuver_e_sys(NULL, self);

  for(i = 0; i < path->_maximum; i++) {
    t = i * maneuver_control_period_ms/1000.;
    q = p.getAllAt(t);

    s.ts.sec = floor(t);
    s.ts.nsec = 1e9*(t - s.ts.sec);

    s.intrinsic = false;

    s.pos._present = true;
    s.pos._value.x = q[0][0];
    s.pos._value.y = q[1][0];
    s.pos._value.z = q[2][0];
    s.pos._value.qw = std::cos(q[3][0]/2.);
    s.pos._value.qx = 0.;
    s.pos._value.qy = 0.;
    s.pos._value.qz = std::sin(q[3][0]/2.);
    s.pos_cov._present = false;

    s.vel._present = true;
    s.vel._value.vx = q[0][1];
    s.vel._value.vy = q[1][1];
    s.vel._value.vz = q[2][1];
    s.vel._value.wx = 0.;
    s.vel._value.wy = 0.;
    s.vel._value.wz = q[3][1];
    s.vel_cov._present = false;

    s.acc._present = true;
    s.acc._value.ax = q[0][2];
    s.acc._value.ay = q[1][2];
    s.acc._value.az = q[2][2];
    s.acc_cov._present = false;

    path->_buffer[i] = s;
  }
  path->_length = path->_maximum;
  return genom_ok;
}
