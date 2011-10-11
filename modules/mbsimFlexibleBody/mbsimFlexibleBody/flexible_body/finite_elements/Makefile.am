SUBDIRS = finite_element_1s_33_rcm finite_element_2s_13_mfr_mindlin .

noinst_LTLIBRARIES = libfe.la
libfe_la_SOURCES = 	finite_element_1s_21_ancf.cc\
			finite_element_1s_21_rcm.cc\
			finite_element_1s_33_rcm.cc\
			finite_element_1s_33_cosserat.cc\
			finite_element_1s_33_cosserat_rotation.cc\
			finite_element_1s_23_bta.cc\
			finite_element_2s_13_disk.cc\
			finite_element_2s_13_mfr_mindlin.cc\
            superelement_linear_external.cc

libfe_la_CPPFLAGS = -I.. -Ifinite_element_1s_33_rcm -Ifinite_element_2s_13_mfr_mindlin $(DEPS_CFLAGS)
libfe_la_LIBADD = finite_element_1s_33_rcm/libfe1s33rcm.la finite_element_2s_13_mfr_mindlin/libfe2s13mfrmindlin.la $(DEPS_LIBS)

feincludedir = $(includedir)/mbsimFlexibleBody/flexible_body/finite_elements

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

feinclude_HEADERS = finite_element_1s_21_ancf.h\
			finite_element_1s_21_rcm.h\
			finite_element_1s_33_rcm.h\
			finite_element_1s_33_cosserat.h\
			finite_element_1s_33_cosserat_rotation.h\
			finite_element_1s_23_bta.h\
			finite_element_2s_13_disk.h\
			finite_element_2s_13_mfr_mindlin.h\
            superelement_linear_external.h

