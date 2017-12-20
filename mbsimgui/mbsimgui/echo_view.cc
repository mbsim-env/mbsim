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

#include <config.h>
#include "echo_view.h"
#include "file_editor.h"
#include <iostream>
#include <QTextStream>
#include <QProcessEnvironment>

using namespace std;

namespace MBSimGUI {

  EchoView::EchoView(QWidget *parent) : QTabWidget(parent) {
    out=new QTextBrowser(this);
    err=new QTextBrowser(this);
    out->setOpenLinks(false);
    err->setOpenLinks(false);
    connect(out, SIGNAL(anchorClicked(const QUrl &)), this, SLOT(outLinkClicked(const QUrl &)));
    connect(err, SIGNAL(anchorClicked(const QUrl &)), this, SLOT(errLinkClicked(const QUrl &)));
    addTab(out, "Out");
    addTab(err, "Err");
    setCurrentIndex(0);
    setMinimumHeight(80);
    setTabPosition(QTabWidget::West);
  }

  void EchoView::clearOutputAndError() {
    outText="";
    errText="";
    out->clear();
    err->clear();
    setCurrentIndex(0);
  }

  void EchoView::updateOutputAndError() {
    out->setHtml(convertToHtml(outText));
    out->moveCursor(QTextCursor::End);

    err->setHtml(convertToHtml(errText));
    err->moveCursor(QTextCursor::Start);
  }

  QString EchoView::convertToHtml(QString &text) {
    // the following operations modify the original text

#ifdef _WIN32
    // convert windows line ending to linux line ending (they are later replaced to html line ending)
    text.replace("\x0D\x0A", "\x0A");
#endif
    // from now on use linux line ending => do not use \n, \r in string literals but \x0A, \x0D

    // remove all lines but the last ending with carriage return '\x0D'
    static QRegExp carriageReturn("(^|\x0A)[^\x0A]*\x0D([^\x0A\x0D]*\x0D)");
    text.replace(carriageReturn, "\\1\\2");

    // make some replace on text here
    // (currently nothing since MBXMLUtils can now directly print html style output)

    // the following operations modify only the QString return value
    QString ret=text;

    // newlines '\x0A' to html
    ret.replace("\x0A", "<br/>");

    return ret;
  }

  QSize EchoView::sizeHint() const {
    QSize size=QTabWidget::sizeHint();
    size.setHeight(80);
    return size;
  }

  QSize EchoView::minimumSizeHint() const {
    QSize size=QTabWidget::minimumSizeHint();
    size.setHeight(80);
    return size;
  }

  void EchoView::linkClicked(const QUrl &link, QTextBrowser *std) {
    if(QProcessEnvironment::systemEnvironment().contains("MBSIMGUI_EDITOR")) {
      static QString editorCommand=QProcessEnvironment::systemEnvironment().value("MBSIMGUI_EDITOR", "gvim -R %1 +%2");
      cout<<"Opening file using command '"<<editorCommand.toStdString()<<"'. "<<
        "Where %1 is replaced by the filename and %2 by the line number. "<<
        "Use the environment variable MBSIMGUI_EDITOR to overwrite this command."<<endl;
//      QString comm=editorCommand.arg(link.path()).arg(link.queryItemValue("line").toInt());
//      QProcess::startDetached(comm);
    }
    else {
//      FileEditor *edit = new FileEditor("Model file",link.path(),link.queryItemValue("line").toInt(),QString("Could not open file ")+link.path(),this);
//      edit->setModal(true);
//      edit->show();
    }
  }

  void EchoView::outLinkClicked(const QUrl &link) {
    linkClicked(link, out);
  }

  void EchoView::errLinkClicked(const QUrl &link) {
    linkClicked(link, err);
  }

}
