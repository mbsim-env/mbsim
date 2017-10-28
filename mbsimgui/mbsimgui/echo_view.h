/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef __ECHO_VIEW_H_
#define __ECHO_VIEW_H_

#include <QTabWidget>
#include <QTextBrowser>

namespace MBSimGUI {

  class EchoView : public QTabWidget {
    Q_OBJECT
    public:
      EchoView(QWidget *parent);
      void clearOutputAndError();
      void updateOutputAndError();
      QSize sizeHint() const;
      QSize minimumSizeHint() const;
      void setErrorText(const QString &errText_) { errText = errText_; }
      void setOutputText(const QString &outText_) { outText = outText_; }
      void addErrorText(const QString &errText_) { errText += errText_; }
      void addOutputText(const QString &outText_) { outText += outText_; }
    private:
      QTextBrowser *out, *err;
      QString outText, errText;
      static QString convertToHtml(QString &text);
      void linkClicked(const QUrl &link, QTextBrowser *std);
    private slots:
      void outLinkClicked(const QUrl &link);
      void errLinkClicked(const QUrl &link);
  };

}

#endif
