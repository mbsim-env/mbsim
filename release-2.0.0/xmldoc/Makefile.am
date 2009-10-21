docdir = @MBXMLUTILSDOCDIR@/http___mbsim_berlios_de_MBSimXML
doc_DATA = mbsimxml.xhtml

mbsimxml.xhtml: genContent.xsl ../schema/mbsimxml.xsd
	@XSLTPROC@ --novalid --output mbsimxml.xhtml genContent.xsl ../schema/mbsimxml.xsd
	@XMLLINT@ --noout --schema $$(pkg-config --variable SCHEMADIR mbxmlutils)/xhtml1-transitional.xsd mbsimxml.xhtml || rm mbsimxml.xhtml

CLEANFILES=mbsimxml.xhtml
