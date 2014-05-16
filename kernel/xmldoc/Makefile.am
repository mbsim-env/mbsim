all: .mbsim.done .mbsimintegrator.done

xml/index.xml: ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	doxygen

.mbsim.done: xml/index.xml ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSim/mbsim.xsd xml figures:../doc MBSim MBSim::

.mbsimintegrator.done: xml/index.xml ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimIntegrator/mbsimintegrator.xsd xml figures:../doc MBSim-Integrator MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsim.done .mbsimintegrator.done
