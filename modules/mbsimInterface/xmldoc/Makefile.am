all: .mbsiminterface.done

xml/index.xml: $(top_srcdir)/schema/mbsiminterface.xsd $(top_srcdir)/mbsimInterface/*.h
	doxygen Doxyfile

.mbsiminterface.done: $(top_srcdir)/schema/mbsiminterface.xsd xml/index.xml @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimControl/mbsimcontrol.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimInterface/mbsiminterface.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimInterface MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimControl/mbsimcontrol.xsd


install-data-local: .mbsiminterface.done
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsiminterface.done
