SUBDIRS = .

noinst_LTLIBRARIES = libframe.la
libframe_la_SOURCES = fixed_relative_frame.cc\
		      floating_relative_frame.cc

libframe_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libframe_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

frameincludedir = $(includedir)/mbsim/frames
frameinclude_HEADERS = fixed_relative_frame.h\
		       floating_relative_frame.h
