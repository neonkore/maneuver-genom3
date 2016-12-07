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

#include <sys/time.h>

#include <cmath>
#include <cstdio>
#include <limits>

#include <Eigen/Geometry>

#include "maneuver_c_types.h"


/* --- Task exec -------------------------------------------------------- */


/** Codel mv_exec_start of task exec.
 *
 * Triggered by maneuver_start.
 * Yields to maneuver_wait.
 */
genom_event
mv_exec_start(const maneuver_desired *desired,
              maneuver_ids_trajectory_s *trajectory,
              genom_context self)
{
  or_pose_estimator_state *ddata;
  struct timeval tv;

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

  trajectory->t._length = 0;
  trajectory->i = 0;

  return maneuver_wait;
}


/** Codel mv_exec_wait of task exec.
 *
 * Triggered by maneuver_wait.
 * Yields to maneuver_pause_wait, maneuver_main.
 */
genom_event
mv_exec_wait(const maneuver_state *state,
             const maneuver_ids_trajectory_s *trajectory,
             genom_context self)
{
  if (trajectory->t._length == 0) return maneuver_pause_wait;

  return maneuver_main;
}


/** Codel mv_exec_main of task exec.
 *
 * Triggered by maneuver_main.
 * Yields to maneuver_wait, maneuver_pause_main, maneuver_start.
 */
genom_event
mv_exec_main(const maneuver_state *state,
             maneuver_ids_trajectory_s *trajectory,
             const maneuver_desired *desired, genom_context self)
{
  const double dt = maneuver_control_period_ms * 1e-3;

  or_pose_estimator_state *sdata;
  struct timeval tv;
  Eigen::Quaternion<double> qcur, qnext;
  Eigen::Vector3d pcur, pnext;
  double pdist, qdist;

  /* done? */
  if (trajectory->i >= trajectory->t._length) {
    trajectory->t._length = 0;
    trajectory->i = 0;
    return maneuver_wait;
  }

  /* current state at next period */
  if (state->read(self)) return maneuver_start;
  sdata = state->data(self);
  if (!sdata || !sdata->pos._present) return maneuver_pause_main;

  gettimeofday(&tv, NULL);
  if (tv.tv_sec + 1e-6 * tv.tv_usec >
      0.5 + sdata->ts.sec + 1e-9 * sdata->ts.nsec)
    return maneuver_pause_main;

  pcur << sdata->pos._value.x, sdata->pos._value.y, sdata->pos._value.z;
  if (sdata->vel._present) {
    pcur += dt * Eigen::Vector3d(
      sdata->vel._value.vx, sdata->vel._value.vy, sdata->vel._value.vz);
  }
  if (sdata->acc._present) {
    pcur += 0.5 * dt * dt * Eigen::Vector3d(
      sdata->acc._value.ax, sdata->acc._value.ay, sdata->acc._value.az);
  }

  qcur.w() = sdata->pos._value.qw;
  qcur.vec() <<
    sdata->pos._value.qx,
    sdata->pos._value.qy,
    sdata->pos._value.qz;

  if (sdata->vel._present) {
    Eigen::Vector3d w;
    Eigen::Quaternion<double> wq;

    w << sdata->vel._value.wx, sdata->vel._value.wy, sdata->vel._value.wz;

    double a2 = dt * dt * w.squaredNorm();
    if (a2 < 1e-1) {
      wq.w() = 1 - a2/8 /*std::cos(a/2)*/;
      wq.vec() = (0.5 - a2/48 /*std::sin(a/2)/a*/) * dt * w;
    } else {
      double a = std::sqrt(a2);
      wq.w() = std::cos(a/2);
      wq.vec() = std::sin(a/2)/a * dt * w;
    }
    qcur = wq * qcur;
  }

  /* get closest next sample */
  for(pdist = qdist = std::numeric_limits<double>::max();
      trajectory->i < trajectory->t._length; trajectory->i++) {
    or_pose_estimator_state *n = &trajectory->t._buffer[trajectory->i];
    double pd, qd;

    pnext << n->pos._value.x, n->pos._value.y, n->pos._value.z;
    qnext.w() = n->pos._value.qw;
    qnext.vec() << n->pos._value.qx, n->pos._value.qy, n->pos._value.qz;

    pd = (pnext-pcur).norm();
    qd = qnext.angularDistance(qcur);
    if (pd > pdist || qd > qdist) break;

    pdist = pd;
    qdist = qd;
  }

  /* publish */
  sdata = desired->data(self);
  if (trajectory->i >= trajectory->t._length)
    *sdata = trajectory->t._buffer[trajectory->t._length - 1];
  else
    *sdata = trajectory->t._buffer[trajectory->i];
  sdata->ts.sec = tv.tv_sec;
  sdata->ts.nsec = 1000*tv.tv_usec;
  desired->write(self);

  /* next */
  if (pdist > 0.1 || qdist > 0.2)
    trajectory->i--;

  return maneuver_pause_main;
}


/** Codel mv_exec_stop of task exec.
 *
 * Triggered by maneuver_stop.
 * Yields to maneuver_ether.
 */
genom_event
mv_exec_stop(genom_context self)
{
  return maneuver_ether;
}
