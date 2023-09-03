SUBDIRS = 1s_33_rcm 2s_13_mfr_mindlin .

noinst_LTLIBRARIES = libfe.la
libfe_la_SOURCES = 1s_21_rcm.cc\
		   1s_21_ancf.cc\
		   1s_21_cosserat_translation.cc\
		   1s_21_cosserat_rotation.cc\
		   1s_33_rcm.cc\
		   1s_33_ancf.cc\
		   1s_33_cosserat_translation.cc\
		   1s_33_cosserat_rotation.cc\
		   1s_23_bta.cc\
		   2s_13_disk.cc\
		   2s_13_mfr_mindlin.cc\
		   linear_external_lumped_node.cc

libfe_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libfe_la_LIBADD = 1s_33_rcm/libfe1s33rcm.la 
libfe_la_LIBADD += 2s_13_mfr_mindlin/libfe2s13mfrmindlin.la 
libfe_la_LIBADD += $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

feincludedir = $(includedir)/mbsimFlexibleBody/flexible_body/fe

feinclude_HEADERS = 1s_21_rcm.h\
		    1s_21_ancf.h\
		    1s_21_cosserat_translation.h\
		    1s_21_cosserat_rotation.h\
		    1s_33_rcm.h\
		    1s_33_ancf.h\
		    1s_33_cosserat_translation.h\
		    1s_33_cosserat_rotation.h\
		    1s_23_bta.h\
		    2s_13_disk.h\
		    2s_13_mfr_mindlin.h\
		    linear_external_lumped_node.h
