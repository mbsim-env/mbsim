noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = contact_kinematics.cc

contactkinematicsincludedir = $(includedir)/mbsim/contact_kinematics

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)  $(MBSIMNUMERICS_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)  $(MBSIMNUMERICS_LIBS)

contactkinematicsinclude_HEADERS = contact_kinematics.h
