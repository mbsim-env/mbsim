all: .mbsimcontrol.done

xml/index.xml: $(top_srcdir)/schema/mbsimcontrol.xsd $(top_srcdir)/mbsimControl/*.h
	doxygen Doxyfile

.mbsimcontrol.done: $(top_srcdir)/schema/mbsimcontrol.xsd xml/index.xml @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimControl/mbsimcontrol.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimControl MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd


install-data-local: .mbsimcontrol.done
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimcontrol.done
