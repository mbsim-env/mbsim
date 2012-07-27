
COMPLEXLIB = @complex_lib@

INCLUDES = -I@top_srcdir@/matrix -I@top_srcdir@/numerical

EXTRALIBS = -lm -lc
ALLLIBS = $(EXTRALIBS)

TESTS = tmatrixMat tint tint2 tstatistics
noinst_PROGRAMS = tmatrixMat tint tint2 tstatistics
LDADD = ../../numerical/libmatrixN.la ../../matrix/libmatrix.la $(ALLLIBS)

tmatrixMat_SOURCES = tmatrixMat.cpp
tint_SOURCES = tint.cpp
tint2_SOURCES = tint2.cpp
tstatistics_SOURCES = tstatistics.cpp
