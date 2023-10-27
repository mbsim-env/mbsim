#ifndef _XML_HIGHLIGHTER_H_
#define _XML_HIGHLIGHTER_H_

#include <QSyntaxHighlighter>

namespace MBSimGUI {

  // Taken from https://github.com/d1vanov/basic-xml-syntax-highlighter
  // Modifcations:
  // - renamed class
  // - removed two constructors 
  // - changed formatting
  class XMLHighlighter : public QSyntaxHighlighter {
    public:
      XMLHighlighter(QTextDocument *parent);

    protected:
      virtual void highlightBlock(const QString &text);

    private:
      void highlightByRegex(const QTextCharFormat &format, const QRegExp &regex, const QString &text);

      void setRegexes();
      void setFormats();

    private:
      QTextCharFormat m_xmlKeywordFormat;
      QTextCharFormat m_xmlElementFormat;
      QTextCharFormat m_xmlAttributeFormat;
      QTextCharFormat m_xmlValueFormat;
      QTextCharFormat m_xmlCommentFormat;

      QList<QRegExp> m_xmlKeywordRegexes;
      QRegExp m_xmlElementRegex;
      QRegExp m_xmlAttributeRegex;
      QRegExp m_xmlValueRegex;
      QRegExp m_xmlCommentRegex;
  };

}

#endif
