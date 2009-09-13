all: mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex

../schema/mbsim.xsd: ../schema/mbsim.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsim.xsd

../schema/mbsimintegrator.xsd: ../schema/mbsimintegrator.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimintegrator.xsd

xml/index.xml: ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	doxygen

mbsim/mbsim.tex: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsim.xsd xml figures:../doc MBSim MBSim:: false

mbsimintegrator/mbsimintegrator.tex: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimintegrator.xsd xml figures:../doc MBSim-Integrator MBSim:: false


install:
	cp -r http___* $$(pkg-config mbxmlutils --variable DOCDIR)



CLEANFILES=xml/index.xml mbsim/mbsim.tex mbsimintegrator/mbsimintegrator.tex
