SUBDIRS = fe .

noinst_LTLIBRARIES = libflexiblebody.la
libflexiblebody_la_SOURCES = generic_flexible_ffr_body.cc\
                             flexible_ffr_body.cc\
                             external_flexible_ffr_body.cc\
			     1s.cc\
			     1s_21_rcm.cc\
			     1s_21_ancf.cc\
			     1s_33_rcm.cc\
			     1s_33_ancf.cc\
			     1s_cosserat.cc\
			     1s_21_cosserat.cc\
			     1s_33_cosserat.cc\
			     1s_23_bta.cc\
			     2s.cc\
			     2s_13.cc\
			     2s_13_disk.cc\
			     2s_13_mfr_mindlin.cc\
			     linear_external_ffr.cc

libflexiblebody_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libflexiblebody_la_LIBADD = fe/libfe.la
libflexiblebody_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

flexiblebodyincludedir = $(includedir)/mbsimFlexibleBody/flexible_body

flexiblebodyinclude_HEADERS = generic_flexible_ffr_body.h\
			      flexible_ffr_body.h\
			      external_flexible_ffr_body.h\
			      1s.h\
			      1s_21_rcm.h\
			      1s_21_ancf.h\
			      1s_33_rcm.h\
			      1s_33_ancf.h\
			      1s_cosserat.h\
			      1s_21_cosserat.h\
			      1s_33_cosserat.h\
			      1s_23_bta.h\
			      2s.h\
			      2s_13.h\
			      2s_13_disk.h\
			      2s_13_mfr_mindlin.h\
			      linear_external_ffr.h
