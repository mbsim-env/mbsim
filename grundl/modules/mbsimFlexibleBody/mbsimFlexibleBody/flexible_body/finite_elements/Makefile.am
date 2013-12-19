SUBDIRS = finite_element_1s_33_rcm finite_element_2s_13_mfr_mindlin .

noinst_LTLIBRARIES = libfe.la
libfe_la_SOURCES = 	finite_element_1s_21_ancf.cc\
			finite_element_1s_21_rcm.cc\
			finite_element_1s_33_rcm.cc\
			finite_element_1s_21_cosserat_translation.cc\
			finite_element_1s_21_cosserat_rotation.cc\
			finite_element_1s_33_cosserat_translation.cc\
			finite_element_1s_33_cosserat_rotation.cc\
			finite_element_1s_23_bta.cc\
			finite_element_2s_13_disk.cc\
			finite_element_2s_13_mfr_mindlin.cc\
			finite_element_linear_external_lumped_node.cc\
            superelement_linear_external.cc

libfe_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libfe_la_LIBADD = finite_element_1s_33_rcm/libfe1s33rcm.la 
libfe_la_LIBADD += finite_element_2s_13_mfr_mindlin/libfe2s13mfrmindlin.la 
libfe_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

feincludedir = $(includedir)/mbsimFlexibleBody/flexible_body/finite_elements

feinclude_HEADERS = finite_element_1s_21_ancf.h\
			finite_element_1s_21_rcm.h\
			finite_element_1s_33_rcm.h\
			finite_element_1s_21_cosserat_translation.h\
			finite_element_1s_21_cosserat_rotation.h\
			finite_element_1s_33_cosserat_translation.h\
			finite_element_1s_33_cosserat_rotation.h\
			finite_element_1s_23_bta.h\
			finite_element_2s_13_disk.h\
			finite_element_2s_13_mfr_mindlin.h\
			finite_element_linear_external_lumped_node.h\
            superelement_linear_external.h

