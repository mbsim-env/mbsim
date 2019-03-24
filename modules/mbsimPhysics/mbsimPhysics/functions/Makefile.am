SUBDIRS = .

noinst_LTLIBRARIES = libfunction.la
libfunction_la_SOURCES = gravity_function.cc

functionincludedir = $(includedir)/mbsimPhysics/functions

libfunction_la_LIBADD = $(DEPS_LIBS)
libfunction_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS)

functioninclude_HEADERS = gravity_function.h
