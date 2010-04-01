# if doxygen docu should be included
#USEDOXYGEN=true
#USEDOXYGENDEP=xml/index.xml
# if doxygen docu should NOT be included
USEDOXYGEN=false
USEDOXYGENDEP=



all: mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex

xml/index.xml: ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	doxygen

mbsim/mbsim.tex: $(USEDOXYGENDEP) ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSim/mbsim.xsd xml figures:../doc MBSim MBSim:: $(USEDOXYGEN)

mbsimintegrator/mbsimintegrator.tex: $(USEDOXYGENDEP) ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSim/mbsimintegrator.xsd xml figures:../doc MBSim-Integrator MBSim:: $(USEDOXYGEN)


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex
