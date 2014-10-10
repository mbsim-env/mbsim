all: .mbsimhydraulics.done

xml/index.xml: $(top_srcdir)/schema/mbsimhydraulics.xsd $(top_srcdir)/mbsimHydraulics/*.h
	doxygen $(srcdir)/Doxyfile

.mbsimhydraulics.done: xml/index.xml $(top_srcdir)/schema/mbsimhydraulics.xsd $(top_srcdir)/mbsimHydraulics/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimHydraulics/mbsimhydraulics.xsd xml figures:../doc MBSimHydraulics MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimhydraulics.done
