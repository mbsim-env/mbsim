SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimInterface.la
libmbsimInterface_la_LDFLAGS = -version-info @LIBMBSIM_INTERFACE_VERSION_INFO@ -lboost_thread -lboost_system
libmbsimInterface_la_SOURCES = \
  server_integrator.cc


libmbsimInterface_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS)
libmbsimInterface_la_LIBADD = $(DEPS_LIBS)

mbsimInterfaceincludedir = $(includedir)/mbsimInterface
mbsimInterfaceinclude_HEADERS = \
  server_integrator.h \
  server_integrator_messages.h
