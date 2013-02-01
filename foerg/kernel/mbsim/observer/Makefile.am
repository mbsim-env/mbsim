noinst_LTLIBRARIES = libobserver.la
libobserver_la_SOURCES = observer.cc \
			 kinematics_observer.cc

observerincludedir = $(includedir)/mbsim/observer

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libobserver_la_CPPFLAGS = -DTIXML_USE_STL

observerinclude_HEADERS = observer.h \
			  kinematics_observer.h
