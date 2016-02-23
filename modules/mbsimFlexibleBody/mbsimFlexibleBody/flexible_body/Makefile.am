SUBDIRS = finite_elements .

noinst_LTLIBRARIES = libflexiblebody.la
libflexiblebody_la_SOURCES = flexible_body_ffr.cc\
			     flexible_body_1s.cc\
			     flexible_body_1s_21_rcm.cc\
			     flexible_body_1s_21_ancf.cc\
			     flexible_body_1s_33_ancf.cc

libflexiblebody_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libflexiblebody_la_LIBADD = finite_elements/libfe.la
libflexiblebody_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

flexiblebodyincludedir = $(includedir)/mbsimFlexibleBody/flexible_body

flexiblebodyinclude_HEADERS = flexible_body_ffr.h\
			      flexible_body_1s.h\
			      flexible_body_1s_21_rcm.h\
			      flexible_body_1s_21_ancf.h\
			      flexible_body_1s_33_ancf.h
