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

  <!-- all nodes of all imported schemas -->
  <xsl:param name="ALLNODES" select="document(/xs:schema/xs:import/@schemaLocation)"/>

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

        span.expandcollapsecontent { cursor:nw-resize;color:blue;font-family:monospace;font-weight:bold;font-size:1.25em }
      </style>
      <script type="text/javascript">
        /* <![CDATA[ */
        function expandcollapsecontent(c) {
          var ul=c.parentNode.getElementsByTagName('ul')[0];
          if(ul.style.display=="") {
            ul.style.display="none";
            c.firstChild.data="+ ";
            c.style.cursor="se-resize";
          }
          else {
            ul.style.display="";
            c.firstChild.data="- ";
            c.style.cursor="nw-resize";
          }
        }
        /* ]]> */
      </script>
    </head>
    <body>
    <h1>MBSim XML - XML Documentation</h1>
    <p>This is the entry point for the documentation of MBSim XML (Kernel and Mechanics) and the installed modules:</p>
    <ul>
      <xsl:for-each select="/xs:schema/xs:import">
        <xsl:sort select="@namespace"/>
        <li><a class="namespace" href="../{translate(@namespace,'.:/','___')}/index.xhtml"><xsl:value-of select="@namespace"/></a></li>
      </xsl:for-each>
    </ul>

    <h2>All Elements in MBSim XML and in the Modules</h2>
    <ul class="content">
      <xsl:apply-templates mode="CONTENT" select="$ALLNODES/xs:schema/xs:element[not(@substitutionGroup)]">
        <xsl:sort select="@name"/>
      </xsl:apply-templates>
    </ul>

    <h2>All Simple Types in MBSim XML and in the Modules</h2>
    <ul class="content">
      <xsl:apply-templates mode="CONTENT" select="$ALLNODES/xs:schema/xs:simpleType">
        <xsl:sort select="@name"/>
      </xsl:apply-templates>
    </ul>
    </body></html>
  </xsl:template>

  <xsl:template mode="CONTENT" match="/xs:schema/xs:element">
    <xsl:param name="NS_NAME" select="namespace::*[name()=substring-before(current()/@name,':')]"/>
    <xsl:param name="NAME_NAME" select="translate(substring(@name,string-length(substring-before(@name,':'))+1),':','')"/>
    <li><span class="expandcollapsecontent" onclick="expandcollapsecontent(this)">- </span>
      <a class="element">
        <xsl:attribute name="href"><xsl:apply-templates mode="GENLINK" select="@name"/></xsl:attribute>
        &lt;<xsl:value-of select="$NAME_NAME"/>&gt;</a>
      <xsl:text> </xsl:text><span class="namespaceSmall"><xsl:value-of select="$NS_NAME"/></span>
      <xsl:if test="$ALLNODES/xs:schema/xs:element[concat('{',namespace::*[name()=substring-before(../@substitutionGroup,':')],'}',translate(substring(@substitutionGroup,string-length(substring-before(@substitutionGroup,':'))+1),':',''))=concat('{',$NS_NAME,'}',$NAME_NAME)]">
        <!-- this if is equal to test="/xs:schema/xs:element[@substitutionGroup=current()/@name]" with namespace aware attributes values -->
        <ul class="content">
          <xsl:apply-templates mode="CONTENT" select="$ALLNODES/xs:schema/xs:element[concat('{',namespace::*[name()=substring-before(../@substitutionGroup,':')],'}',translate(substring(@substitutionGroup,string-length(substring-before(@substitutionGroup,':'))+1),':',''))=concat('{',$NS_NAME,'}',$NAME_NAME)]">
            <!-- this apply-templates is equal to select="/xs:schema/xs:element[@substitutionGroup=current()/@name]" with namespace aware attributes values -->
            <xsl:sort select="@name"/>
          </xsl:apply-templates>
        </ul>
      </xsl:if>
    </li>
  </xsl:template>

  <xsl:template mode="CONTENT" match="/xs:schema/xs:simpleType">
    <xsl:param name="NS_NAME" select="namespace::*[name()=substring-before(current()/@name,':')]"/>
    <xsl:param name="NAME_NAME" select="translate(substring(@name,string-length(substring-before(@name,':'))+1),':','')"/>
    <li>
      <a class="element">
        <xsl:attribute name="href"><xsl:apply-templates mode="GENLINK" select="@name"/></xsl:attribute>
        &lt;<xsl:value-of select="$NAME_NAME"/>&gt;</a>
      <xsl:text> </xsl:text><span class="namespaceSmall"><xsl:value-of select="$NS_NAME"/></span>
    </li>
  </xsl:template>

</xsl:stylesheet>
