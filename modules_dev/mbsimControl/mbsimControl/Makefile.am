SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimControl.la
libmbsimControl_la_LDFLAGS = -version-info @LIBMBSIM_CONTROL_VERSION_INFO@
libmbsimControl_la_SOURCES = \
														 signal_.cc\
														 mechanical_sensors.cc\
														 linear_transfer_system.cc\
														 actuator.cc\
mectosignal.cc\
nltransfersys.cc\
signalspsys.cc\
spsys.cc\
transfersys.cc


libmbsimControl_la_LIBADD = 
INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

mbsimControlincludedir = $(includedir)/mbsimControl
mbsimControlinclude_HEADERS = \
															signal_interface.h\
															signal_.h\
															sensor.h\
															mechanical_sensors.h\
															function_sensors.h\
															signal_manipulation.h\
															signal_processing_system.h\
															linear_transfer_system.h\
															actuator.h\
															signal_processing_system_sensor.h\
mectosignal.h\
nltransfersys.h\
signalspsys.h\
spsys.h\
transfersys.h

