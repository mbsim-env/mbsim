all: mbsimcontrol/mbsimcontrol.tex

xml/index.xml: ../schema/mbsimcontrol.xsd ../mbsimControl/*.h
	doxygen

mbsimcontrol/mbsimcontrol.tex: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html @MBXMLUTILSSCHEMA@/http___mbsim_berlios_de_MBSimControl/mbsimcontrol.xsd xml . MBSimControl MBSim:: false


install:
	cp -r http___* $$(pkg-config mbxmlutils --variable DOCDIR)



CLEANFILES=xml/index.xml mbsimcontrol/mbsimcontrol.tex
