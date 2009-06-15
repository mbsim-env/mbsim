docdir = @datadir@/@PACKAGE@/doc
nodist_doc_DATA = mbsim.html *.png mbsimintegrator.html

../schema/mbsim.xsd: ../schema/mbsim.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsim.xsd

../schema/mbsimintegrator.xsd: ../schema/mbsimintegrator.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimintegrator.xsd

xml/index.xml: ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../../mbsim/*.h ../../mbsim/integrators/*.h
	doxygen

mbsim.html: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsim.xsd xml html MBSim MBSim::
#sed -re "s|$$(pkg-config mbxmlutils --variable DOCDIR)/measurement.html|qrc:measurement.html|" ../doc/openmbv.html > openmbv.qt.html

mbsimintegrator.html: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimintegrator.xsd xml html MBSim-Integrator MBSim::
#sed -re "s|$$(pkg-config mbxmlutils --variable DOCDIR)/measurement.html|qrc:measurement.html|" ../doc/openmbv.html > openmbv.qt.html



schemadir = @datadir@/@PACKAGE@/schema
nodist_schema_DATA = mbsim.doxygen.xsd mbsimintegrator.doxygen.xsd

mbsim.doxygen.xsd: mbsim.html

mbsimintegrator.doxygen.xsd: mbsimintegrator.html
