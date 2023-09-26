/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "octave_highlighter.h"

namespace MBSimGUI {

  OctaveHighlighter::OctaveHighlighter(QTextDocument *parent) : QSyntaxHighlighter(parent) {
    //  QPlainTextEdit dummy;
    bool dark=false;
    //  if(dummy.palette().brush(dummy.backgroundRole()).color().value()<128)
    //    dark=true;

    { // numbers
      QTextCharFormat format;
      QString regex=R"(\b[0-9]+\.?[0-9]*[eE]?[0-9]*\b)";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // numbers
      QTextCharFormat format;
      QString regex=R"(\b[0-9]*\.?[0-9]+[eE]?[0-9]*\b)";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // keywords
      QTextCharFormat format;
      QString regex="\\b(return|case|switch|else|elseif|end|if|otherwise|do|for|while|try|catch|global|persistent)\\b";
      if(dark)
        format.setForeground(QColor(255, 255, 96));
      else
        format.setForeground(QColor(165, 42, 42));
      format.setFontWeight(QFont::Bold);
      rule.emplace_back(QRegExp(regex), format);
    }
    { // functions
      QTextCharFormat format;
      QString regex="\\b(break|zeros|default|margin|round|ones|rand|ceil|floor|size|clear|zeros|eye|mean|std|cov|error|eval|function|abs|acos|atan|asin|cos|cosh|exp|log|prod|sum|log10|max|min|sign|sin|sinh|sqrt|tan|reshape)\\b";
      if(dark)
        format.setForeground(QColor(255, 255, 96));
      else
        format.setForeground(QColor(165, 42, 42));
      format.setFontWeight(QFont::Bold);
      rule.emplace_back(QRegExp(regex), format);
    }
    { // operators
      QTextCharFormat format;
      QString regex=R"([-+*/^=&~'();,[\]]|\.[-+*/^]|==|[<>]=|~=|<>|\.{3})";
      if(dark)
        format.setForeground(QColor(64, 255, 255));
      else
        format.setForeground(QColor(0, 139, 139));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // strings
      QTextCharFormat format;
      QString regex=R"("[^"]*")";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // strings
      QTextCharFormat format;
      QString regex="'[^']*'";
      if(dark)
        format.setForeground(QColor(255, 160, 160));
      else
        format.setForeground(QColor(255, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
    { // comments
      QTextCharFormat format;
      QString regex="%.*";
      if(dark)
        format.setForeground(QColor(128, 160, 255));
      else
        format.setForeground(QColor(0, 0, 255));
      rule.emplace_back(QRegExp(regex), format);
    }
  }

  void OctaveHighlighter::highlightBlock(const QString &text) {
    for(auto & i : rule) {
      int index=0;
      do {
        index=i.first.indexIn(text, index);
        if(index>=0) {
          setFormat(index, i.first.matchedLength(), i.second);
          index+=i.first.matchedLength();
        }
      }
      while(index>=0);
    }
  }

}
