docdir = @datadir@/@PACKAGE@/doc
nodist_doc_DATA = mbsim.xhtml mbsimintegrator.xhtml

../schema/mbsim.xsd: ../schema/mbsim.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsim.xsd

../schema/mbsimintegrator.xsd: ../schema/mbsimintegrator.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimintegrator.xsd

xml/index.xml: ../schema/mbsim.xsd ../schema/mbsimintegrator.xsd ../mbsim/*.h ../mbsim/integrators/*.h
	doxygen

mbsim.xhtml: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsim.xsd xml .:../doc MBSim MBSim:: false
#sed -re "s|$$(pkg-config mbxmlutils --variable DOCDIR)/measurement.html|qrc:measurement.html|" ../doc/openmbv.html > openmbv.qt.html

mbsimintegrator.xhtml: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimintegrator.xsd xml .:../doc MBSim-Integrator MBSim:: false
#sed -re "s|$$(pkg-config mbxmlutils --variable DOCDIR)/measurement.html|qrc:measurement.html|" ../doc/openmbv.html > openmbv.qt.html



schemadir = @datadir@/@PACKAGE@/schema
nodist_schema_DATA = mbsim.doxygen.xsd mbsimintegrator.doxygen.xsd

mbsim.doxygen.xsd: mbsim.xhtml

mbsimintegrator.doxygen.xsd: mbsimintegrator.xhtml

CLEANFILES=xml/index.xml mbsim.xhtml mbsimintegrator.xhtml
