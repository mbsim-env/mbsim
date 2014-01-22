SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimFmi.la
libmbsimFmi_la_LDFLAGS = -version-info @LIBMBSIMFMI_VERSION_INFO@
libmbsimFmi_la_SOURCES = \
  fmiModelFunctions.cc \
  model_instance.cc \
  system.cc \
  alloc_override.cc 


libmbsimFmi_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS)
libmbsimFmi_la_LIBADD = $(DEPS_LIBS)

mbsimFmiincludedir = $(includedir)/mbsimFmi
mbsimFmiinclude_HEADERS = \
  fmiModelTypes.h \
  fmiModelFunctions.h \
  model_instance.h \
  system.h
  
  
