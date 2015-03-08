SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimInterface.la
libmbsimInterface_la_LDFLAGS = -version-info @LIBMBSIM_INTERFACE_VERSION_INFO@ -Wl,--disable-new-dtags,-rpath,\$$ORIGIN/../lib
libmbsimInterface_la_SOURCES = \
  interface_integrator.cc \
  mbsim_tcp_server.cc \
  mbsim_udp_server.cc


libmbsimInterface_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS)
libmbsimInterface_la_LIBADD = $(DEPS_LIBS) -l@BOOST_SYSTEM_LIB@ $(EXTRA_LIBS)

mbsimInterfaceincludedir = $(includedir)/mbsimInterface
mbsimInterfaceinclude_HEADERS = \
  interface_messages.h \
  interface_messages.m \
  interface_integrator.h \
  mbsim_server.h


include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target
