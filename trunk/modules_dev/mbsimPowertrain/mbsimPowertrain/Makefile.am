SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimPowertrain.la
libmbsimPowertrain_la_LDFLAGS = -version-info @LIBMBSIMELECTRONICS_VERSION_INFO@
libmbsimPowertrain_la_SOURCES = rigid_body_with_1d_relative_motion.cc\
			    shaft.cc

libmbsimPowertrain_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimPowertrainincludedir = $(includedir)/mbsimPowertrain
mbsimPowertraininclude_HEADERS = rigid_body_with_1d_relative_motion.h\
			     shaft.h



