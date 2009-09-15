<xsl:stylesheet
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:xs="http://www.w3.org/2001/XMLSchema"
  xmlns="http://www.w3.org/1999/xhtml"
  version="1.0">

  <!-- output method -->
  <xsl:output method="xml"
    encoding="UTF-8"
    doctype-public="-//W3C//DTD XHTML 1.0 Transitional//EN"
    doctype-system="http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd"/>

  <!-- generate html link form a attribute -->
  <xsl:template mode="GENLINK" match="@*">
    <xsl:param name="V1" select="../namespace::*[name()=substring-before(current(),':')]"/>
    <xsl:param name="V2" select="translate($V1,'.:/','___')"/>
    <xsl:text>../</xsl:text><xsl:value-of select="$V2"/><xsl:text>/index.xhtml#</xsl:text>
    <xsl:if test="not(contains(.,':'))">
      <xsl:value-of select="."/>
    </xsl:if>
    <xsl:if test="contains(.,':')">
      <xsl:value-of select="substring-after(.,':')"/>
    </xsl:if>
  </xsl:template>

  <xsl:template match="/">
    <!-- all nodes from MBSim and modules -->
    <xsl:param name="NODES" select="document(/xs:schema/xs:include/@schemaLocation|/xs:schema/xs:import/@schemaLocation)"/>
    <!-- html header -->
    <html xml:lang="en" lang="en">
    <head>
      <title>MBSim XML - XML Documentation</title>
      <style type="text/css">
        *.element { font-family:monospace;font-weight:bold;font-size:1.25em }
        *.namespace { font-style:italic }
        *.namespaceSmall { font-style:italic;font-size:0.6em }
        ul.content { padding-left:3ex;list-style-type:none }
        h2 { margin-top:8ex }
      </style>
    </head>
    <body>
    <h1>MBSim XML - XML Documentation</h1>
    <p>This is the entry point for the documentation of MBSim XML (Kernel and Mechanics) and the installed modules:</p>
    <ul>
      <li><b><a class="namespace" href="../{translate(/xs:schema/@targetNamespace,'.:/','___')}/index.xhtml"><xsl:value-of select="/xs:schema/@targetNamespace"/></a></b></li>
      <xsl:for-each select="/xs:schema/xs:import">
        <xsl:sort select="@namespace"/>
        <li><a class="namespace" href="../{translate(@namespace,'.:/','___')}/index.xhtml"><xsl:value-of select="@namespace"/></a></li>
      </xsl:for-each>
    </ul>

    <h2>All Elements in MBSim XML and in the Modules</h2>
    <ul class="content">
      <xsl:apply-templates mode="CONTENT" select="$NODES/xs:schema/xs:element[not(@substitutionGroup)]">
        <xsl:with-param name="NODES" select="$NODES"/>
        <xsl:sort select="@name"/>
      </xsl:apply-templates>
    </ul>

    <h2>All Simple Types in MBSim XML and in the Modules</h2>
    <ul class="content">
      <xsl:apply-templates mode="CONTENT" select="$NODES/xs:schema/xs:simpleType">
        <xsl:with-param name="NODES" select="$NODES"/>
        <xsl:sort select="@name"/>
      </xsl:apply-templates>
    </ul>
    </body></html>
  </xsl:template>

  <xsl:template mode="CONTENT" match="/xs:schema/xs:element">
    <xsl:param name="NODES"/>
    <xsl:param name="NS" select="namespace::*[name()=substring-before(current()/@name,':')]"/>
    <xsl:param name="NAME" select="translate(substring(@name,string-length(substring-before(@name,':'))+1),':','')"/>
    <li>
      <a class="element">
        <xsl:attribute name="href"><xsl:apply-templates mode="GENLINK" select="@name"/></xsl:attribute>
        &lt;<xsl:value-of select="$NAME"/>&gt;</a>
      <xsl:text> </xsl:text><span class="namespaceSmall"><xsl:value-of select="$NS"/></span>
      <xsl:if test="$NODES/xs:schema/xs:element[concat('{',namespace::*[name()=substring-before(../@substitutionGroup,':')],'}',translate(substring(@substitutionGroup,string-length(substring-before(@substitutionGroup,':'))+1),':',''))=concat('{',$NS,'}',$NAME)]">
        <ul class="content">
          <xsl:apply-templates mode="CONTENT" select="$NODES/xs:schema/xs:element[concat('{',namespace::*[name()=substring-before(../@substitutionGroup,':')],'}',translate(substring(@substitutionGroup,string-length(substring-before(@substitutionGroup,':'))+1),':',''))=concat('{',$NS,'}',$NAME)]">
            <xsl:with-param name="NODES" select="$NODES"/>
            <xsl:sort select="@name"/>
          </xsl:apply-templates>
        </ul>
      </xsl:if>
    </li>
  </xsl:template>

  <xsl:template mode="CONTENT" match="/xs:schema/xs:simpleType">
    <xsl:param name="NS" select="namespace::*[name()=substring-before(current()/@name,':')]"/>
    <xsl:param name="NAME" select="translate(substring(@name,string-length(substring-before(@name,':'))+1),':','')"/>
    <li>
      <a class="element">
        <xsl:attribute name="href"><xsl:apply-templates mode="GENLINK" select="@name"/></xsl:attribute>
        &lt;<xsl:value-of select="$NAME"/>&gt;</a>
      <xsl:text> </xsl:text><span class="namespaceSmall"><xsl:value-of select="$NS"/></span>
    </li>
  </xsl:template>

</xsl:stylesheet>
