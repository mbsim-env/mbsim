SUBDIRS = flexible_body contours contact_kinematics utils .
  
lib_LTLIBRARIES = libmbsimFlexibleBody.la
libmbsimFlexibleBody_la_LDFLAGS = -version-info @LIBMBSIMFLEXIBLEBODY_VERSION_INFO@
libmbsimFlexibleBody_la_SOURCES = flexible_body.cc objectfactory.cc

libmbsimFlexibleBody_la_LIBADD = flexible_body/libflexiblebody.la 
libmbsimFlexibleBody_la_LIBADD += contours/libcontour.la 
libmbsimFlexibleBody_la_LIBADD += contact_kinematics/libcontactkinematics.la 
libmbsimFlexibleBody_la_LIBADD += utils/libutils.la  
libmbsimFlexibleBody_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

libmbsimFlexibleBody_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)

mbsimFlexibleBodyincludedir = $(includedir)/mbsimFlexibleBody
mbsimFlexibleBodyinclude_HEADERS = flexible_body.h\
  functions_contact.h \
  pointer.h
