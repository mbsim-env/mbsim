<xsl:stylesheet
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:xs="http://www.w3.org/2001/XMLSchema"
  version="1.0">

  <xsl:template match="/">
    <xs:dummyRoot>
      <xsl:apply-templates select="/files/file"/>
    </xs:dummyRoot>
  </xsl:template>

  <xsl:template match="/files/file">
    <xsl:copy-of select="document(concat(/files/@base, '/', @base, '/', @name))"/>
  </xsl:template>

</xsl:stylesheet>
