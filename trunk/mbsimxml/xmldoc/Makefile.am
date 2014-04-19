docdir = $(MBXMLUTILSDOCDIR)/http___mbsim_berlios_de_MBSimXML
doc_DATA = mbsimxml.xhtml

# none mbsim modules xsd files
XSDFILES = http___openmbv_berlios_de_MBXMLUtils_physicalvariable/physicalvariable.xsd\
  http___mbsim_berlios_de_MBSim/mbsim.xsd\
  http___mbsim_berlios_de_MBSimIntegrator/mbsimintegrator.xsd\
  http___openmbv_berlios_de_OpenMBV/openmbv.xsd

all: mbsimxml.xhtml

# list of all xsd files: variables XSDFILES and all modules (plugins)
.DELETE_ON_ERROR: xsdfiles.txt
xsdfiles.txt: $(MBXMLUTILSBINDIR)/../share/mbsimxml/plugins/*.plugin.xml
	echo "$(XSDFILES)" | tr ' ' '\n' > xsdfiles.txt
	@XSLTPROC@ --novalid getPluginLocation.xsl $(MBXMLUTILSBINDIR)/../share/mbsimxml/plugins/*.plugin.xml >> xsdfiles.txt

# XML representation of all xsd files
.DELETE_ON_ERROR: files.xml
files.xml: xsdfiles.txt
	echo "<files base=\"$(MBXMLUTILSSCHEMA)\">" > files.xml
	for F in $$(cat xsdfiles.txt); do echo "<file base=\"$$(dirname $$F)\" name=\"$$(basename $$F)\"/>" >> files.xml; done
	echo "</files>" >> files.xml

# generate combined schema (root element xs:dummyRoot)
.DELETE_ON_ERROR: combindedSchema.xsd
combindedSchema.xsd: genCombindedSchema.xsl files.xml
	@XSLTPROC@ --novalid --output combindedSchema.xsd genCombindedSchema.xsl files.xml

# generate the documentation
.DELETE_ON_ERROR: mbsimxml.xhtml
mbsimxml.xhtml: genContent.xsl combindedSchema.xsd
	@XSLTPROC@ --novalid --output mbsimxml.xhtml --param DATETIME "'$$(date "+%a %b %d %Y %H:%M:%S")'" --param MBSIMXMLVERSION "'@VERSION@'" genContent.xsl combindedSchema.xsd
# note we convert abs path here to rel ones to avoid xalan/xerces to interprete a abs path as rel when using wine
	@MBXMLUTILSVALIDATE@ $$(pwd | sed -re "s|/[^/]+|/..|g;s|^/||")$(MBXMLUTILSSCHEMA)/http___www_w3_org/xhtml1-transitional.xsd mbsimxml.xhtml

CLEANFILES=mbsimxml.xhtml combindedSchema.xsd files.xml xsdfiles.txt
