all: .mbsimflexiblebody.done

xml/index.xml: $(top_srcdir)/schema/mbsimflexiblebody.xsd $(top_srcdir)/mbsimFlexibleBody/*.h
	doxygen Doxyfile

.mbsimflexiblebody.done: $(top_srcdir)/schema/mbsimflexiblebody.xsd xml/index.xml @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimFlexibleBody/mbsimflexiblebody.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimFlexibleBody MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd


install-data-local: .mbsimflexiblebody.done
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimflexiblebody.done
