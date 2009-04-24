SUBDIRS = finite_element_1s_33_rcm .

noinst_LTLIBRARIES = libfe.la
libfe_la_SOURCES = 	finite_element_1s_21_ancf.cc\
			finite_element_1s_21_rcm.cc\
			finite_element_1s_33_rcm.cc\
			finite_element_1s_23_bta.cc\
            superelement_linear_external.cc

libfe_la_CPPFLAGS = -I.. -Ifinite_element_1s_33_rcm $(DEPS_CFLAGS)
libfe_la_LIBADD = finite_element_1s_33_rcm/libfe1s33rcm.la $(DEPS_LIBS)

feincludedir = $(includedir)/mbsim/flexible_body/finite_elements

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

feinclude_HEADERS = finite_element_1s_21_ancf.h\
			finite_element_1s_21_rcm.h\
			finite_element_1s_33_rcm.h\
			finite_element_1s_23_bta.h\
            superelement_linear_external.h

