all: mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex

xml/index.xml: ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	doxygen

mbsim/mbsim.tex: xml/index.xml ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSim/mbsim.xsd xml figures:../doc MBSim MBSim::

mbsimintegrator/mbsimintegrator.tex: xml/index.xml ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSim/mbsimintegrator.xsd xml figures:../doc MBSim-Integrator MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex
