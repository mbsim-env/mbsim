SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimControl.la
libmbsimControl_la_LDFLAGS = -version-info @LIBMBSIM_CONTROL_VERSION_INFO@
libmbsimControl_la_SOURCES = mectosignal.cc\
nltransfersys.cc\
signalspsys.cc\
spsys.cc\
transfersys.cc

libmbsimControl_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimControlincludedir = $(includedir)/mbsimControl
mbsimControlinclude_HEADERS = mectosignal.h\
nltransfersys.h\
signalspsys.h\
spsys.h\
transfersys.h



