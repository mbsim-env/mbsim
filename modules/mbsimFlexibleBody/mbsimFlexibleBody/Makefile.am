SUBDIRS = flexible_body contours contact_kinematics utils .
  
lib_LTLIBRARIES = libmbsimFlexibleBody.la
libmbsimFlexibleBody_la_LDFLAGS = -avoid-version
libmbsimFlexibleBody_la_SOURCES = flexible_body.cc\
				  fixed_nodal_frame.cc\
				  node_frame.cc\
				  contour_parameter_frame.cc\
				  floating_frame.cc

libmbsimFlexibleBody_la_LIBADD = flexible_body/libflexiblebody.la
libmbsimFlexibleBody_la_LIBADD += contours/libcontour.la
libmbsimFlexibleBody_la_LIBADD += contact_kinematics/libcontactkinematics.la
libmbsimFlexibleBody_la_LIBADD += utils/libutils.la
libmbsimFlexibleBody_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

libmbsimFlexibleBody_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)

mbsimFlexibleBodyincludedir = $(includedir)/mbsimFlexibleBody
mbsimFlexibleBodyinclude_HEADERS = flexible_body.h\
				   functions_contact.h\
				   frame_ffr.h\
				   fixed_nodal_frame.h\
				   node_frame.h\
				   contour_parameter_frame.h\
				   floating_frame.h\
				   pointer.h

include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target
