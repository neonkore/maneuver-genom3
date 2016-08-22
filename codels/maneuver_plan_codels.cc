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
 *                                      Anthony Mallet on Mon Aug 22 2016
 */
#include "acmaneuver.h"

#include <cmath>
#include <iostream>

#include "libkdtp.h"

#include "maneuver_c_types.h"


/* --- Task plan -------------------------------------------------------- */

struct maneuver_planner_s {
  kdtp::Robot robot;

  maneuver_planner_s(): robot("rotorcraft") {}
};

static genom_event	mv_sample_path(const kdtp::LocalPath &p,
                                       sequence_or_pose_estimator_state *path,
                                       genom_context self);

/** Codel mv_plan_start of task plan.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_ether.
 */
genom_event
mv_plan_start(maneuver_ids *ids, genom_context self)
{
  static const double vmax = 0.3;
  static const double wmax = 1.;

  ids->planner = new maneuver_planner_s;

  ids->planner->robot.addDof(
    kdtp::Dof(-3.5, 3.5, vmax, vmax, vmax, vmax, false));
  ids->planner->robot.addDof(
    kdtp::Dof(-1.5, 1.5, vmax, vmax, vmax, vmax, false));
  ids->planner->robot.addDof(
    kdtp::Dof( 0.,  5.,  vmax, vmax, vmax, vmax, false));
  ids->planner->robot.addDof(
    kdtp::Dof(-3*M_PI, 3*M_PI,  wmax, wmax, wmax, wmax, true));

  return maneuver_ether;
}


/** Codel mv_plan_stop of task plan.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 */
genom_event
mv_plan_stop(maneuver_ids *ids, genom_context self)
{
  delete ids->planner;

  return maneuver_ether;
}


/* --- Activity take_off ------------------------------------------------ */

/** Codel mv_current_state_start of activity take_off.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_plan.
 * Throws maneuver_e_nostate.
 */
genom_event
mv_current_state_start(const maneuver_state *state, or_t3d_pos *start,
                       genom_context self)
{
  if (state->read(self) != genom_ok) return maneuver_e_nostate(self);
  if (!state->data(self)->pos._present) return maneuver_e_nostate(self);
  *start = state->data(self)->pos._value;
  return maneuver_plan;
}

/** Codel mv_take_off_plan of activity take_off.
 *
 * Triggered by maneuver_plan.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate.
 */
genom_event
mv_take_off_plan(const maneuver_planner_s *planner,
                 const or_t3d_pos *start, double height,
                 sequence_or_pose_estimator_state *path,
                 genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  from.position()[0] = start->x;
  from.position()[1] = start->y;
  from.position()[2] = start->z;
  from.position()[3] = atan2(
    2 * (start->qw*start->qz + start->qx*start->qy),
    1 - 2 * (start->qy*start->qy + start->qz*start->qz));

  to.position() = from.position();
  to.position()[2] = height;

  kdtp::LocalPath p(planner->robot, from, to);
  e = mv_sample_path(p, path, self);
  if (e) return e;

  return maneuver_exec;
}

/** Codel mv_plan_exec of activity take_off.
 *
 * Triggered by maneuver_exec.
 * Yields to maneuver_pause_exec, maneuver_wait.
 * Throws maneuver_e_nostate.
 */
genom_event
mv_plan_exec(const maneuver_planner_s *planner,
             const sequence_or_pose_estimator_state *path,
             maneuver_ids_trajectory_s *trajectory,
             genom_context self)
{
  size_t i;

  if (trajectory->t._length > 0) return maneuver_pause_exec;

  if (genom_sequence_reserve(&trajectory->t, path->_length))
    return maneuver_e_sys(NULL, self);

  for(i = 0; i < path->_length; i++)
    trajectory->t._buffer[i] = path->_buffer[i];
  trajectory->t._length = path->_length;

  return maneuver_wait;
}

/** Codel mv_plan_exec_wait of activity take_off.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate.
 */
genom_event
mv_plan_exec_wait(const maneuver_ids_trajectory_s *trajectory,
                  genom_context self)
{
  if (trajectory->t._length > 0) return maneuver_pause_wait;
  return maneuver_ether;
}

/** Codel mv_plan_exec_stop of activity take_off.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate.
 */
genom_event
mv_plan_exec_stop(maneuver_ids_trajectory_s *trajectory,
                  genom_context self)
{
  trajectory->t._length = 0;
  return maneuver_ether;
}


/* --- Activity goto ---------------------------------------------------- */

/** Codel mv_current_state_start of activity goto.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_plan.
 * Throws maneuver_e_nostate.
 */
/* already defined in service take_off */


/** Codel mv_goto_plan of activity goto.
 *
 * Triggered by maneuver_plan.
 * Yields to maneuver_exec.
 * Throws maneuver_e_nostate.
 */
genom_event
mv_goto_plan(const maneuver_planner_s *planner,
             const or_t3d_pos *start, double x, double y, double z,
             double yaw, sequence_or_pose_estimator_state *path,
             genom_context self)
{
  kdtp::State from(planner->robot);
  kdtp::State to(planner->robot);
  genom_event e;

  from.position()[0] = start->x;
  from.position()[1] = start->y;
  from.position()[2] = start->z;
  from.position()[3] = atan2(
    2 * (start->qw*start->qz + start->qx*start->qy),
    1 - 2 * (start->qy*start->qy + start->qz*start->qz));

  to.position()[0] = x;
  to.position()[1] = y;
  to.position()[2] = z;
  to.position()[3] = yaw;

  kdtp::LocalPath p(planner->robot, from, to);
  e = mv_sample_path(p, path, self);
  if (e) return e;

  return maneuver_exec;
}

/** Codel mv_plan_exec of activity goto.
 *
 * Triggered by maneuver_exec.
 * Yields to maneuver_pause_exec, maneuver_wait.
 * Throws maneuver_e_nostate.
 */
/* already defined in service take_off */


/** Codel mv_plan_exec_wait of activity goto.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_ether.
 * Throws maneuver_e_nostate.
 */
/* already defined in service take_off */


/** Codel mv_plan_exec_stop of activity goto.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 * Throws maneuver_e_nostate.
 */
/* already defined in service take_off */



/* --- Activity waypoint ------------------------------------------------ */

/** Codel mv_waypoint_start of activity waypoint.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_plan.
 */
genom_event
mv_waypoint_start(const maneuver_state *state,
                  const maneuver_ids_trajectory_s *trajectory,
                  or_t3d_pos *start, genom_context self)
{
  if (trajectory->t._length > 0) {
    *start = trajectory->t._buffer[trajectory->t._length - 1].pos._value;
    return maneuver_plan;
  }

  return mv_current_state_start(state, start, self);
}

/** Codel mv_goto_plan of activity waypoint.
 *
 * Triggered by maneuver_plan.
 * Yields to maneuver_exec.
 */
/* already defined in service goto */


/** Codel mv_waypoint_add of activity waypoint.
 *
 * Triggered by maneuver_exec.
 * Yields to maneuver_ether.
 */
genom_event
mv_waypoint_add(const maneuver_planner_s *planner,
                const sequence_or_pose_estimator_state *path,
                maneuver_ids_trajectory_s *trajectory,
                genom_context self)
{
  size_t i, l;

  l = trajectory->t._length;
  if (genom_sequence_reserve(&trajectory->t, l + path->_length))
    return maneuver_e_sys(NULL, self);

  for(i = 0; i < path->_length; i++)
    trajectory->t._buffer[l + i] = path->_buffer[i];
  trajectory->t._length = l + path->_length;

  return maneuver_ether;
}


/* --- local functions ----------------------------------------------------- */

static genom_event
mv_sample_path(const kdtp::LocalPath &p,
               sequence_or_pose_estimator_state *path, genom_context self)
{
  std::vector<std::vector<double> > q;
  or_pose_estimator_state s;
  size_t i;

  i = 1 + 1000 * p.duration()/maneuver_control_period_ms;
  if (genom_sequence_reserve(path, i)) return maneuver_e_sys(NULL, self);

  for(i = 0; i < path->_maximum; i++) {
    q = p.getAllAt(i * maneuver_control_period_ms/1000.);

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
