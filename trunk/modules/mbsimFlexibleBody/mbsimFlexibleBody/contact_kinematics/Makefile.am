noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = circle_nurbsdisk2s.cc\
					point_nurbsdisk2s.cc

contactkinematicsincludedir = $(includedir)/mbsimFlexibleBody/contact_kinematics

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contactkinematicsinclude_HEADERS = circle_nurbsdisk2s.h\
  point_nurbsdisk2s.h
