SUBDIRS = flexible_body contours contact_kinematics utils .
  
lib_LTLIBRARIES = libmbsimFlexibleBody.la
libmbsimFlexibleBody_la_LDFLAGS = -version-info @LIBMBSIMFLEXIBLEBODY_VERSION_INFO@
libmbsimFlexibleBody_la_SOURCES = flexible_body.cc objectfactory.cc dummyof.cc

libmbsimFlexibleBody_la_LIBADD = flexible_body/libflexiblebody.la contours/libcontour.la contact_kinematics/libcontactkinematics.la utils/libutils.la

libmbsimFlexibleBody_la_CPPFLAGS = -DTIXML_USE_STL

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

mbsimFlexibleBodyincludedir = $(includedir)/mbsimFlexibleBody
mbsimFlexibleBodyinclude_HEADERS = flexible_body.h\
  functions_contact.h \
  pointer.h
