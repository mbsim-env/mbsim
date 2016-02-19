SUBDIRS = .

noinst_LTLIBRARIES = libframe.la
libframe_la_SOURCES = frame.cc\
		      fixed_relative_frame.cc\
		      floating_relative_frame.cc\
		      contour_frame.cc\
		      floating_relative_contour_frame.cc

libframe_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libframe_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

frameincludedir = $(includedir)/mbsim/frames
frameinclude_HEADERS = frame.h\
		       fixed_relative_frame.h\
		       floating_relative_frame.h\
		       contour_frame.h\
		       floating_relative_contour_frame.h
