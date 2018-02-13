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
 */
genom_event
mv_current_state_read(const maneuver_state *state,
                      maneuver_configuration_s *reference,
                      const genom_context self)
{
  or_pose_estimator_state *sdata;
  int i;

  if (state->read(self) != genom_ok) return maneuver_e_nostate(self);
  sdata = state->data(self);
  if (!sdata) return maneuver_e_nostate(self);

  reference->pos = sdata->pos;
  if (!sdata->pos._present) return maneuver_e_nostate(self);

  /* disregard current velocity and acceleration - the residual noise
   * in those quantities generates weird trajectories */
  for(i = 0; i < 6; i++) {
    reference->vel[i] = 0.;
    reference->acc[i] = 0.;
    reference->jer[i] = 0.;
  }

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
                 const maneuver_configuration_s *reference,
                 double height, double duration,
                 sequence_maneuver_configuration_s *path,
                 const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!reference->pos._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &reference->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;
  from.position()[3] = atan2(
    2 * (p->qw*p->qz + p->qx*p->qy), 1 - 2 * (p->qy*p->qy + p->qz*p->qz));

  from.velocity()[0] = reference->vel[0];
  from.velocity()[1] = reference->vel[1];
  from.velocity()[2] = reference->vel[2];
  from.velocity()[3] = reference->vel[5];

  from.acceleration()[0] = reference->acc[0];
  from.acceleration()[1] = reference->acc[1];
  from.acceleration()[2] = reference->acc[2];
  from.acceleration()[3] = reference->acc[5];

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
mv_plan_exec(const sequence_maneuver_configuration_s *path,
             maneuver_configuration_s *reference,
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
             const maneuver_configuration_s *reference, double x,
             double y, double z, double yaw, double duration,
             sequence_maneuver_configuration_s *path,
             const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!reference->pos._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &reference->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;
  from.position()[3] = atan2(
    2 * (p->qw*p->qz + p->qx*p->qy), 1 - 2 * (p->qy*p->qy + p->qz*p->qz));

  from.velocity()[0] = reference->vel[0];
  from.velocity()[1] = reference->vel[1];
  from.velocity()[2] = reference->vel[2];
  from.velocity()[3] = reference->vel[5];

  from.acceleration()[0] = reference->acc[0];
  from.acceleration()[1] = reference->acc[1];
  from.acceleration()[2] = reference->acc[2];
  from.acceleration()[3] = reference->acc[5];

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
                 const maneuver_configuration_s *reference, double x,
                 double y, double z, double yaw, double vx, double vy,
                 double vz, double wz, double ax, double ay, double az,
                 double duration,
                 sequence_maneuver_configuration_s *path,
                 const genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  if (!reference->pos._present) return maneuver_e_nostate(self);

  const or_t3d_pos *p = &reference->pos._value;
  from.position()[0] = p->x;
  from.position()[1] = p->y;
  from.position()[2] = p->z;
  from.position()[3] = atan2(
    2 * (p->qw*p->qz + p->qx*p->qy), 1 - 2 * (p->qy*p->qy + p->qz*p->qz));

  from.velocity()[0] = reference->vel[0];
  from.velocity()[1] = reference->vel[1];
  from.velocity()[2] = reference->vel[2];
  from.velocity()[3] = reference->vel[5];

  from.acceleration()[0] = reference->acc[0];
  from.acceleration()[1] = reference->acc[1];
  from.acceleration()[2] = reference->acc[2];
  from.acceleration()[3] = reference->acc[5];

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
               sequence_maneuver_configuration_s *path,
               const genom_context self)
{
  sequence_maneuver_configuration_s interp;
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  maneuver_configuration_s s;

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

  interp._maximum = 0;
  interp._length = 0;
  interp._buffer = NULL;
  interp._release = NULL;

  s.pos._present = true;
  s.pos._value.qx = 0.;
  s.pos._value.qy = 0.;
  for(i = 0; i < 6; i++) s.jer[i] = 0.;

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
    s.pos._value.qw = std::cos(yaw/2.);
    s.pos._value.qz = std::sin(yaw/2.);

    /* read optional velocity */
    n = sscanf(l, "%lf %lf %lf %lf %lf %lf%n",
               &s.vel[0], &s.vel[1], &s.vel[2],
               &s.vel[3], &s.vel[4], &s.vel[5],
               &c);
    if (n == 6) {
      l += c;

      /* read optional acceleration */
      n = sscanf(l, "%lf %lf %lf %lf %lf %lf%n",
                 &s.acc[0], &s.acc[1], &s.acc[2],
                 &s.acc[3], &s.acc[4], &s.acc[5],
                 &c);
      if (n != 6) {
        for(i = 0; i < 6; i++) s.acc[i] = 0.;
      }
    } else {
      for(i = 0; i < 6; i++) s.vel[i] = 0.;
      for(i = 0; i < 6; i++) s.acc[i] = 0.;
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
    const maneuver_configuration_s &s0 = path->_buffer[path->_length - 1];
    from.position()[0] = s0.pos._value.x;
    from.position()[1] = s0.pos._value.y;
    from.position()[2] = s0.pos._value.z;
    from.position()[3] = 2 * atan2(s0.pos._value.qz, s0.pos._value.qw);

    from.velocity()[0] = s0.vel[0];
    from.velocity()[1] = s0.vel[1];
    from.velocity()[2] = s0.vel[2];
    from.velocity()[3] = s0.vel[5];

    from.acceleration()[0] = s0.acc[0];
    from.acceleration()[1] = s0.acc[1];
    from.acceleration()[2] = s0.acc[2];
    from.acceleration()[3] = s0.acc[5];

    to.position()[0] = s.pos._value.x;
    to.position()[1] = s.pos._value.y;
    to.position()[2] = s.pos._value.z;
    to.position()[3] = 2 * atan2(s.pos._value.qz, s.pos._value.qw);

    to.velocity()[0] = s.vel[0];
    to.velocity()[1] = s.vel[1];
    to.velocity()[2] = s.vel[2];
    to.velocity()[3] = s.vel[5];

    to.acceleration()[0] = s.acc[0];
    to.acceleration()[1] = s.acc[1];
    to.acceleration()[2] = s.acc[2];
    to.acceleration()[3] = s.acc[5];

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
             const maneuver_configuration_s *reference,
             sequence_maneuver_configuration_s *path,
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
