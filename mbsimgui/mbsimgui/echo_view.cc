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
#include "mainwindow.h"
#include "embedding_view.h"
#include "treemodel.h"
#include "treeitem.h"
#include "embeditemdata.h"
#include <iostream>
#include <QTextStream>
#include <QProcessEnvironment>

using namespace std;
using namespace MBXMLUtils;

namespace MBSimGUI {

  extern MainWindow *mw;

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

  namespace {
    bool walk(QModelIndex index, EmbeddingTreeModel *model, const QUrl &link) {
      // get the filename from the error message
      boost::filesystem::path errorFile(boost::filesystem::absolute(link.path().toStdString()));

      int row = 0;
      // loop over all elements in the current level
      while(index.isValid()) {
        // get the item of the current element (index)
        EmbedItemData *item = dynamic_cast<EmbedItemData*>(model->getItem(index)->getItemData());
        // handle only embed elements but not parameters
        if(item) {
          // get the root element of this embed
          xercesc::DOMElement *e = item->getXMLElement();
          // if the file name matchs than the error is from this embed
          if(errorFile==boost::filesystem::absolute(D(e->getOwnerDocument())->getDocumentFilename())) {
            // evalute the xpath expression of the error message in this embed ...
            xercesc::DOMNode *n=D(e->getOwnerDocument())->evalRootXPathExpression(link.queryItemValue("xpath").toStdString());
            // ... the node n is there the error occured:
            cout<<"MISSING this XML node generated the error addr="<<n<<" name="<<X()%n->getNodeName()<<" value="<<X()%n->getNodeValue()<<endl;
            return true;
          }
        }

        // walk child elements
        if(walk(model->index(0, 0, index), model, link))
          return true;

        // next element on this level
        index = index.sibling(++row, 0);
      }
      return false;
    }
  }

  void EchoView::linkClicked(const QUrl &link, QTextBrowser *std) {
    //MISSING mbsimgui adds new elements e.g. <plotFeatureRecursive value="plotRecursive">false</plotFeatureRecursive>
    //MISSING this break the xpath of the error messages
    // get the model of the embedding
    EmbeddingTreeModel *model = static_cast<EmbeddingTreeModel*>(mw->getEmbeddingView()->model());
    // walk all embeded elements
    if(!walk(model->index(0,0), model, link))
      cout<<"MISSING No XML node found for file="<<link.path().toStdString()<<endl<<
            "        xpath="<<link.queryItemValue("xpath").toStdString()<<endl;
  }

  void EchoView::outLinkClicked(const QUrl &link) {
    linkClicked(link, out);
  }

  void EchoView::errLinkClicked(const QUrl &link) {
    linkClicked(link, err);
  }

}
