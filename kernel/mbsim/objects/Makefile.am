SUBDIRS = .

noinst_LTLIBRARIES = libobject.la
libobject_la_SOURCES = body.cc\
		       rigid_body.cc

libobject_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libobject_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

objectincludedir = $(includedir)/mbsim/objects
objectinclude_HEADERS = body.h\
			rigid_body.h
