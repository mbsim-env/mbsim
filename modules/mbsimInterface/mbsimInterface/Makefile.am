SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimInterface.la
libmbsimInterface_la_LDFLAGS = -version-info @LIBMBSIM_INTERFACE_VERSION_INFO@ -lboost_system
libmbsimInterface_la_SOURCES = \
  interfaceIntegrator.cc \
  mbsimTcpServer.cc \
  mbsimUdpServer.cc


libmbsimInterface_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS)
libmbsimInterface_la_LIBADD = $(DEPS_LIBS)

mbsimInterfaceincludedir = $(includedir)/mbsimInterface
mbsimInterfaceinclude_HEADERS = \
  interfaceMessages.h \
  interfaceMessages.m \
  interfaceIntegrator.h \
  mbsimServer.h
