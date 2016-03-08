noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = circle_flexibleband.cc\
				  point_nurbsdisk2s.cc

contactkinematicsincludedir = $(includedir)/mbsimFlexibleBody/contact_kinematics

libcontactkinematics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libcontactkinematics_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

contactkinematicsinclude_HEADERS = circle_flexibleband.h\
				   point_nurbsdisk2s.h
