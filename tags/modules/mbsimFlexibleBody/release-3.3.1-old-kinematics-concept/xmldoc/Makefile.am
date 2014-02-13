all: mbsimflexiblebody/mbsimflexiblebody.tex

xml/index.xml: ../schema/mbsimflexiblebody.xsd ../mbsimFlexibleBody/*.h
	doxygen

mbsimflexiblebody/mbsimflexiblebody.tex: xml/index.xml ../schema/mbsimflexiblebody.xsd ../mbsimFlexibleBody/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimFlexibleBody/mbsimflexiblebody.xsd xml figures:../doc MBSimFlexibleBody MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsimflexiblebody/mbsimflexiblebody.tex
