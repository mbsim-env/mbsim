all: .mbsimphysics.done

xml/index.xml: $(top_srcdir)/schema/mbsimphysics.xsd $(top_srcdir)/mbsimPhysics/*.h
	doxygen Doxyfile

.mbsimphysics.done: $(top_srcdir)/schema/mbsimphysics.xsd xml/index.xml @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimPhysics/mbsimphysics.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimPhysics MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd


install-data-local: .mbsimphysics.done
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimphysics.done
