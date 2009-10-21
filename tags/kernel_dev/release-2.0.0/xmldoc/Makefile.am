# if doxygen docu should be included
#USEDOXYGEN=true
#USEDOXYGENDEP=xml/index.xml
# if doxygen docu should NOT be included
USEDOXYGEN=false
USEDOXYGENDEP=



all: mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex

../schema/mbsim.xsd: ../schema/mbsim.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsim.xsd

../schema/mbsimintegrator.xsd: ../schema/mbsimintegrator.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimintegrator.xsd

xml/index.xml: ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	doxygen

mbsim/mbsim.tex: $(USEDOXYGENDEP) ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsim.xsd xml figures:../doc MBSim MBSim:: $(USEDOXYGEN)

mbsimintegrator/mbsimintegrator.tex: $(USEDOXYGENDEP) ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimintegrator.xsd xml figures:../doc MBSim-Integrator MBSim:: $(USEDOXYGEN)


install:
	cp -r http___* $$(pkg-config mbxmlutils --variable DOCDIR)



CLEANFILES=xml/index.xml mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex
