if COND_WIN32
  MAYBE_WIN32_mbsimCreateFMU_OBJ = mbsimCreateFMU.o
endif

# windows rc rules
mbsimCreateFMU.o: $(srcdir)/mbsimCreateFMU.rc $(srcdir)/mbsimCreateFMU.manifest
	$(WINDRES) $< -o $@

# program to create a FMU from a MBSim XML project file
bin_PROGRAMS = mbsimCreateFMU

mbsimCreateFMU_SOURCES = mbsimCreateFMU-main.cc zip.cc
mbsimCreateFMU_CPPFLAGS = $(CREATEDEPS_CFLAGS) $(LIBARCHIVEDEPS_CFLAGS)
mbsimCreateFMU_LDFLAGS = $(PYCPPWRAPPER_LIBS_MAIN)
mbsimCreateFMU_LDADD = $(DLFLAGS_NOUNDEF) ../general/libfmivariables.la ../general/libxmlpputils.la $(CREATEDEPS_LIBS) $(LIBARCHIVEDEPS_LIBS) -l@BOOST_DATE_TIME_LIB@ $(MAYBE_WIN32_mbsimCreateFMU_OBJ)
