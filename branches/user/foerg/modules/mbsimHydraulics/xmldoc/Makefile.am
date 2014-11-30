all: .mbsimhydraulics.done

xml/index.xml: $(top_srcdir)/schema/mbsimhydraulics.xsd $(top_srcdir)/mbsimHydraulics/*.h
	doxygen Doxyfile

.mbsimhydraulics.done: xml/index.xml $(top_srcdir)/schema/mbsimhydraulics.xsd $(top_srcdir)/mbsimHydraulics/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimHydraulics/mbsimhydraulics.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimHydraulics MBSim::


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimhydraulics.done
