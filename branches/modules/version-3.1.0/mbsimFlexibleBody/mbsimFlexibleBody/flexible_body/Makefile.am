SUBDIRS = finite_elements .

noinst_LTLIBRARIES = libflexiblebody.la
libflexiblebody_la_SOURCES =	flexible_body_1s_21_rcm.cc\
                                flexible_body_1s_33_rcm.cc\
								flexible_body_1s_21_cosserat.cc\
								flexible_body_1s_33_cosserat.cc\
								flexible_body_2s_13.cc\
								flexible_body_2s_13_disk.cc\
								flexible_body_2s_13_mfr_mindlin.cc\
 								flexible_body_linear_external.cc\
 								flexible_body_1s_23_bta.cc

# libflexiblebody_la_SOURCES =	flexible_body_1s_01_torsion.cc\
# 								flexible_body_1s_21_ancf.cc

libflexiblebody_la_CPPFLAGS = -I.. -Ifinite_elements
libflexiblebody_la_LIBADD=finite_elements/libfe.la
flexiblebodyincludedir = $(includedir)/mbsimFlexibleBody/flexible_body

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

flexiblebodyinclude_HEADERS =	flexible_body_1s_21_rcm.h\
                                flexible_body_1s_33_rcm.h\
								flexible_body_1s_21_cosserat.h\
								flexible_body_1s_33_cosserat.h\
								flexible_body_2s_13.h\
								flexible_body_2s_13_disk.h\
								flexible_body_2s_13_mfr_mindlin.h\
 								flexible_body_linear_external.h\
 								flexible_body_1s_23_bta.h

# flexiblebodyinclude_HEADERS =	flexible_body_1s_01_torsion.h\
# 								flexible_body_1s_21_ancf.h

