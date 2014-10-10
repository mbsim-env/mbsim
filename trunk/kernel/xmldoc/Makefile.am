all: .mbsim.done .mbsimintegrator.done

xml/index.xml: ../schema/mbsim.xsd $(top_srcdir)/schema/mbsimintegrator.xsd $(top_srcdir)/mbsim/*.h $(top_srcdir)/mbsim/integrators/*.h
	doxygen $(srcdir)/Doxyfile

.mbsim.done: xml/index.xml $(top_srcdir)/schema/mbsim.xsd $(top_srcdir)/schema/mbsimintegrator.xsd $(top_srcdir)/mbsim/*.h $(top_srcdir)/mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSim/mbsim.xsd xml figures:../doc MBSim MBSim::

.mbsimintegrator.done: xml/index.xml $(top_srcdir)/schema/mbsim.xsd $(top_srcdir)/schema/mbsimintegrator.xsd $(top_srcdir)/mbsim/*.h $(top_srcdir)/mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimIntegrator/mbsimintegrator.xsd xml figures:../doc MBSim-Integrator MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsim.done .mbsimintegrator.done
