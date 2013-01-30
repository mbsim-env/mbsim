noinst_LTLIBRARIES = libobserver.la
libobserver_la_SOURCES = observer.cc

observerincludedir = $(includedir)/mbsim/plot_elements

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libobserver_la_CPPFLAGS = -DTIXML_USE_STL

observerinclude_HEADERS = observer.h
