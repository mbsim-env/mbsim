all: .mbsim.done

xml/index.xml: ../schema/mbsim.xsd $(top_srcdir)/mbsim/*.h
	doxygen Doxyfile

.mbsim.done: $(top_srcdir)/schema/mbsim.xsd xml/index.xml @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSim MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd

install-data-local: .mbsim.done
	cp -r http___* @MBXMLUTILSDOCDIR@

CLEANFILES=xml/index.xml .mbsim.done
