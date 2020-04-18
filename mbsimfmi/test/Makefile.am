# program to test a FMU
bin_PROGRAMS = mbsimTestFMU

mbsimTestFMU_SOURCES = mbsimTestFMU.cc
mbsimTestFMU_CPPFLAGS = -I$(top_srcdir)/mbsimfmi/3rdparty $(MBXMLUTILSHELPERDEPS_CFLAGS)
mbsimTestFMU_LDADD = $(MBXMLUTILSHELPERDEPS_LIBS) $(LIBDL)
