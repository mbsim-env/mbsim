all: mbsimcontrol/mbsimcontrol.tex

../schema/mbsimcontrol.xsd: ../schema/mbsimcontrol.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimcontrol.xsd

xml/index.xml: ../schema/mbsimcontrol.xsd ../mbsimControl/*.h
	doxygen

mbsimcontrol/mbsimcontrol.tex: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimcontrol.xsd xml . MBSimControl MBSim:: false


install:
	cp -r http___* $$(pkg-config mbxmlutils --variable DOCDIR)



CLEANFILES=xml/index.xml mbsimcontrol/mbsimcontrol.tex
