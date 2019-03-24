all: .mbsimphysics.done

xml/index.xml: $(top_srcdir)/schema/mbsimphysics.xsd $(top_srcdir)/mbsimPhysics/*.h
	doxygen Doxyfile

.mbsimphysics.done: xml/index.xml $(top_srcdir)/schema/mbsimphysics.xsd $(top_srcdir)/mbsimPhysics/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimPhysics/mbsimphysics.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimPhysics MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimphysics.done
