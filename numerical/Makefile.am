EXTRA_DIST = matrixMat.cpp fft.cpp chebexp.cpp intccq.cpp statistic.cpp

pkginclude_HEADERS = matrixMat.h integrate.h statistic.h

INCLUDES = -I@top_srcdir@/include -I@top_srcdir@/matrix

lib_LTLIBRARIES = libmatrixN.la
libmatrixN_la_SOURCES = matrixMat_.cpp fft_.cpp chebexp_.cpp intccq_.cpp statistic_.cpp
libmatrixN_la_LDFLAGS = -version-info 1:0:0

