all: .mbsimfcl.done

xml/index.xml: $(top_srcdir)/schema/mbsimfcl.xsd $(top_srcdir)/mbsimFcl/*.h
	doxygen Doxyfile

.mbsimfcl.done: $(top_srcdir)/schema/mbsimfcl.xsd xml/index.xml @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimFcl/mbsimfcl.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimFcl MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd


install-data-local: .mbsimfcl.done
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimfcl.done
