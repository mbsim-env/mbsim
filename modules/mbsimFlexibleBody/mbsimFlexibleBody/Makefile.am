SUBDIRS = flexible_body .
  
lib_LTLIBRARIES = libmbsimFlexibleBody.la
libmbsimFlexibleBody_la_LDFLAGS = -version-info @LIBMBSIMFLEXIBLEBODY_VERSION_INFO@
libmbsimFlexibleBody_la_SOURCES = fixed_nodal_frame.cc

libmbsimFlexibleBody_la_LIBADD = flexible_body/libflexiblebody.la
libmbsimFlexibleBody_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

libmbsimFlexibleBody_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)

mbsimFlexibleBodyincludedir = $(includedir)/mbsimFlexibleBody
mbsimFlexibleBodyinclude_HEADERS = flexible_body.h\
				   functions_contact.h\
				   node_frame.h\
				   fixed_nodal_frame.h\
				   pointer.h

include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target
