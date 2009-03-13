SUBDIRS = finite_elements .

noinst_LTLIBRARIES = libbodyflexible.la
libbodyflexible_la_SOURCES =	flexible_body_1s_01_torsion.cc\
								flexible_body_1s_21_ancf.cc\
								flexible_body_1s_21_rcm.cc\
								flexible_body_1s_23_bta.cc\
								flexible_body_linear_external.cc

libbodyflexible_la_CPPFLAGS = -I.. -Ifinite_elements
libbodyflexible_la_LIBADD=finite_elements/libfe.la
bodyflexibleincludedir = $(includedir)/mbsim

INCLUDES = $(DEPS_CFLAGS) $(AMVIS_CFLAGS)
LDADD = $(DEPS_LIBS) $(AMVIS_LIBS)

bodyflexibleinclude_HEADERS =	flexible_body_1s_01_torsion.h\
								flexible_body_1s_21_ancf.h\
								flexible_body_1s_21_rcm.h\
								flexible_body_1s_23_bta.h\
								flexible_body_linear_external.h

