noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc\
				  point_line.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

contactkinematicsinclude_HEADERS = contact_kinematics.h \
				   point_line.h
