SUBDIRS = .

noinst_LTLIBRARIES = libobject.la
libobject_la_SOURCES = object.cc\
		       body.cc\
		       rigid_body.cc

libobject_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) 
libobject_la_LIBADD = $(LDFLAGS_NOUNDEF) $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) 

objectincludedir = $(includedir)/mbsim/objects
objectinclude_HEADERS = object.h\
			body.h\
			rigid_body.h
