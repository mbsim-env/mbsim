all: .mbsim.done .mbsimintegrator.done

xml/index.xml: ../schema/mbsim.xsd $(top_srcdir)/schema/mbsimintegrator.xsd $(top_srcdir)/mbsim/*.h $(top_srcdir)/mbsim/integrators/*.h
	doxygen Doxyfile

.mbsim.done: xml/index.xml $(top_srcdir)/schema/mbsim.xsd $(top_srcdir)/schema/mbsimintegrator.xsd $(top_srcdir)/mbsim/*.h $(top_srcdir)/mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSim MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_OpenMBV/openmbv.xsd

.mbsimintegrator.done: xml/index.xml $(top_srcdir)/schema/mbsim.xsd $(top_srcdir)/schema/mbsimintegrator.xsd $(top_srcdir)/mbsim/*.h $(top_srcdir)/mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimIntegrator/mbsimintegrator.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSim-Integrator MBSim:: @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSim/mbsim.xsd


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsim.done .mbsimintegrator.done
