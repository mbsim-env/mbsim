noinst_LTLIBRARIES = libnurbs.la
libnurbs_la_SOURCES = nurbs_curve.cc\
                      nurbs_defs.cc\
                      nurbs_surface.cc

nurbsincludedir = $(includedir)/mbsim/numerics/nurbs

INCLUDES = $(FMATVEC_CFLAGS)
LDADD = $(FMATVEC_LIBS)

libnurbs_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS)
libnurbs_la_LIBADD = $(DEPS_LIBS)

nurbsinclude_HEADERS = nurbs_curve.h\
                       nurbs_defs.h\
                       nurbs_surface.h