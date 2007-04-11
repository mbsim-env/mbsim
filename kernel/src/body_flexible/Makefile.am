SUBDIRS = finite_elements .

noinst_LTLIBRARIES = libbodyflexible.la
libbodyflexible_la_SOURCES =	body_flexible_1s_01_torsion.cc\
				body_flexible_1s_21_ancf.cc\
				body_flexible_1s_21_rcm.cc\
				body_flexible_1s_23_bta.cc\
				body_flexible_linear_external.cc

libbodyflexible_la_CPPFLAGS = -I.. -Ifinite_elements
libbodyflexible_la_LIBADD=finite_elements/libfe.la
bodyflexibleincludedir = $(includedir)/mbsim

INCLUDES = $(DEPS_CFLAGS)
LDADD = $(DEPS_LIBS)

bodyflexibleinclude_HEADERS =	body_flexible_1s_01_torsion.h\
				body_flexible_1s_21_ancf.h\
				body_flexible_1s_21_rcm.h\
				body_flexible_1s_23_bta.h\
				body_flexible_linear_external.h

