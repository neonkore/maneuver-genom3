/*
 * Copyright (c) 2016-2017 LAAS/CNRS
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

#include <cstdio>

#include "maneuver_c_types.h"
#include "codels.h"


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
  (*planner)->robot.getDof(0).setPositionMin(xmin);
  (*planner)->robot.getDof(0).setPositionMax(xmax);
  (*planner)->robot.getDof(1).setPositionMin(ymin);
  (*planner)->robot.getDof(1).setPositionMax(ymax);
  (*planner)->robot.getDof(2).setPositionMin(zmin);
  (*planner)->robot.getDof(2).setPositionMax(zmax);
  (*planner)->robot.getDof(3).setPositionMin(yawmin);
  (*planner)->robot.getDof(3).setPositionMax(yawmax);
  return genom_ok;
}


/* --- Function set_velocity -------------------------------------------- */

/** Codel mv_set_velocity of function set_velocity.
 *
 * Returns genom_ok.
 */
genom_event
mv_set_velocity(maneuver_planner_s **planner, double v, double w,
                const genom_context self)
{
  (*planner)->robot.getDof(0).setVelocityMax(v);
  (*planner)->robot.getDof(1).setVelocityMax(v);
  (*planner)->robot.getDof(2).setVelocityMax(v);
  (*planner)->robot.getDof(3).setVelocityMax(w);
  return genom_ok;
}


/* --- Function set_acceleration ---------------------------------------- */

/** Codel mv_set_acceleration of function set_acceleration.
 *
 * Returns genom_ok.
 */
genom_event
mv_set_acceleration(maneuver_planner_s **planner, double a, double dw,
                    const genom_context self)
{
  (*planner)->robot.getDof(0).setAccelerationMax(a);
  (*planner)->robot.getDof(1).setAccelerationMax(a);
  (*planner)->robot.getDof(2).setAccelerationMax(a);
  (*planner)->robot.getDof(3).setAccelerationMax(dw);
  return genom_ok;
}


/* --- Function set_jerk ------------------------------------------------ */

/** Codel mv_set_jerk of function set_jerk.
 *
 * Returns genom_ok.
 */
genom_event
mv_set_jerk(maneuver_planner_s **planner, double j, double ddw,
            const genom_context self)
{
  (*planner)->robot.getDof(0).setJerkMax(j);
  (*planner)->robot.getDof(1).setJerkMax(j);
  (*planner)->robot.getDof(2).setJerkMax(j);
  (*planner)->robot.getDof(3).setJerkMax(ddw);
  return genom_ok;
}


/* --- Function set_snap ------------------------------------------------ */

/** Codel mv_set_snap of function set_snap.
 *
 * Returns genom_ok.
 */
genom_event
mv_set_snap(maneuver_planner_s **planner, double s, double dddw,
            const genom_context self)
{
  (*planner)->robot.getDof(0).setSnapMax(s);
  (*planner)->robot.getDof(1).setSnapMax(s);
  (*planner)->robot.getDof(2).setSnapMax(s);
  (*planner)->robot.getDof(3).setSnapMax(dddw);
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


/* --- Function log ----------------------------------------------------- */

/** Codel mv_log of function log.
 *
 * Returns genom_ok.
 * Throws maneuver_e_sys.
 */
genom_event
mv_log(const char path[64], maneuver_log_s **log,
       const genom_context self)
{
  FILE *f;

  f = fopen(path, "w");
  if (!f) return mv_e_sys_error(path, self);
  fprintf(f, mv_log_header_fmt "\n");

  if ((*log)->f) fclose((*log)->f);
  (*log)->f = f;
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
  if ((*log)->f) fclose((*log)->f);
  (*log)->f = NULL;

  return genom_ok;
}
