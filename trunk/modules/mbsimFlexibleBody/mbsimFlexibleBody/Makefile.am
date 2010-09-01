SUBDIRS = flexible_body contours contact_kinematics .
  
lib_LTLIBRARIES = libmbsimFlexibleBody.la
libmbsimFlexibleBody_la_LDFLAGS = -version-info @LIBMBSIMFLEXIBLEBODY_VERSION_INFO@
libmbsimFlexibleBody_la_SOURCES = \
  objectfactory.cc

libmbsimFlexibleBody_la_LIBADD = flexible_body/libflexiblebody.la contours/libcontour.la contact_kinematics/libcontactkinematics.la

libmbsimFlexibleBody_la_CPPFLAGS = -DTIXML_USE_STL

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

mbsimFlexibleBodyincludedir = $(includedir)/mbsimFlexibleBody
mbsimFlexibleBodyinclude_HEADERS = \
  functions_contact.h \
	objectfactory.h

