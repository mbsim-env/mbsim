SUBDIRS = .
  
noinst_LTLIBRARIES = libcontour.la
libcontour_la_SOURCES = flexible_band.cc

libcontour_la_CPPFLAGS = -I.. $(DEPS_CFLAGS)
libcontour_la_LIBADD = $(DEPS_LIBS)

contourincludedir = $(includedir)/mbsim/contours
contourinclude_HEADERS = flexible_band.h
