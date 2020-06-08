#
# Copyright (c) 2016-2021 LAAS/CNRS
# All rights reserved.
#
# Redistribution and use  in source  and binary  forms,  with or without
# modification, are permitted provided that the following conditions are
# met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
#                                      Anthony Mallet on Mon Aug 22 2016
#
lib_LTLIBRARIES = libmaneuver_codels.la

libmaneuver_codels_la_SOURCES  =	maneuver_c_types.h
libmaneuver_codels_la_SOURCES +=	maneuver_codels.cc
libmaneuver_codels_la_SOURCES +=	maneuver_plan_codels.cc
libmaneuver_codels_la_SOURCES +=	maneuver_exec_codels.cc
libmaneuver_codels_la_SOURCES +=	helpers.cc
libmaneuver_codels_la_SOURCES +=	codels.h

libmaneuver_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libmaneuver_codels_la_LIBADD   =	$(requires_LIBS)
libmaneuver_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libmaneuver_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libmaneuver_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)

libmaneuver_codels_la_LIBADD  +=	-lm


# idl  mappings
BUILT_SOURCES=	maneuver_c_types.h
CLEANFILES=	${BUILT_SOURCES}
maneuver_c_types.h: ${top_srcdir}/maneuver.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/maneuver_c_types.h.d
