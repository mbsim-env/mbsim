SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimControl.la
libmbsimControl_la_LDFLAGS = -version-info @LIBMBSIM_CONTROL_VERSION_INFO@
libmbsimControl_la_SOURCES = \
														 signal_.cc\
														 object_sensors.cc\
														 frame_sensors.cc\
														 function_sensor.cc\
														 signal_manipulation.cc\
														 signal_processing_system.cc\
														 signal_processing_system_sensor.cc\
														 linear_transfer_system.cc\
														 actuator.cc\
														 objectfactory.cc\
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
														 extern_signal_source.h\
															sensor.h\
															object_sensors.h\
															frame_sensors.h\
															function_sensor.h\
															signal_manipulation.h\
															signal_processing_system.h\
															signal_processing_system_sensor.h\
															linear_transfer_system.h\
															actuator.h\
															objectfactory.h\
mectosignal.h\
nltransfersys.h\
signalspsys.h\
spsys.h\
transfersys.h

