all: .mbsimhydraulics.done

xml/index.xml: $(top_srcdir)/schema/mbsimhydraulics.xsd $(top_srcdir)/mbsimHydraulics/*.h
	doxygen Doxyfile

.mbsimhydraulics.done: xml/index.xml $(top_srcdir)/schema/mbsimhydraulics.xsd $(top_srcdir)/mbsimHydraulics/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimHydraulics/mbsimhydraulics.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimHydraulics MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimControl/mbsimcontrol.xsd


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimhydraulics.done
