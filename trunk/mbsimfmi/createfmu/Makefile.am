# program to create a FMU from a MBSim XML project file
bin_PROGRAMS = mbsimCreateFMU

mbsimCreateFMU_SOURCES = mbsimCreateFMU.cc zip.cc
mbsimCreateFMU_LDFLAGS = -Wl,-rpath,\$$ORIGIN/../lib
mbsimCreateFMU_CPPFLAGS = $(CREATEDEPS_CFLAGS) $(LIBARCHIVEDEPS_CFLAGS)
mbsimCreateFMU_LDADD = ../general/libfmivariables.la ../general/libxmlpputils.la $(CREATEDEPS_LIBS) $(LIBARCHIVEDEPS_LIBS)
