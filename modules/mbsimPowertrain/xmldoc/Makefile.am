all: .mbsimpowertrain.done

xml/index.xml: $(top_srcdir)/schema/mbsimpowertrain.xsd $(top_srcdir)/mbsimPowertrain/*.h
	doxygen Doxyfile

.mbsimpowertrain.done: xml/index.xml $(top_srcdir)/schema/mbsimpowertrain.xsd $(top_srcdir)/mbsimPowertrain/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimPowertrain/mbsimpowertrain.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimPowertrain MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimpowertrain.done
