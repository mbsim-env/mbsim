noinst_LTLIBRARIES = libfe.la
libfe_la_SOURCES = 	finite_element_1s_21_ancf.cc\
			finite_element_1s_21_rcm.cc\
			finite_element_1s_23_bta.cc\
            superelement_linear_external.cc

feincludedir = $(includedir)/mbsim

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
