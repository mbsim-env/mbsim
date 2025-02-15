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
#include <QtWidgets/QGridLayout>
#include <QDesktopWidget>
#include <QScrollBar>
#include <QToolBar>
#include <boost/dll.hpp>
#include "utils.h"
#include "echo_view.h"
#include "file_editor.h"
#include "mainwindow.h"
#include "parameter_view.h"
#include "treemodel.h"
#include "treeitem.h"
#include "embeditemdata.h"
#include "parameter.h"
#include "dialogs.h"
#include <QTextStream>
#include <QProcessEnvironment>
#include <QUrlQuery>
#include <QMessageBox>
#include <boost/scope_exit.hpp>

using namespace std;
using namespace MBXMLUtils;

namespace MBSimGUI {

  extern MainWindow *mw;

  EchoView::EchoView(QWidget *parent) : QMainWindow(parent) {
    // QMainWindow set the window flag Qt::Window in its ctor but we use this QMainWindow as a widget
    // -> reset the windows flag to Qt::Widget
    setWindowFlags(Qt::Widget); // we cannot do this by : QMainWindow(parent, Qt::Widget) since QMainWindow overwrites this later on

    static boost::filesystem::path installPath(boost::dll::program_location().parent_path().parent_path());

    setIconSize(iconSize()*0.5);

    out=new QTextBrowser(this);
    out->setOpenLinks(false);
    connect(out, &QTextBrowser::anchorClicked, this, &EchoView::linkClicked);
    setCentralWidget(out);
    auto tb=new QToolBar(this);
    addToolBar(Qt::RightToolBarArea, tb);

    showSSE=new QAction(Utils::QIconCached((installPath/"share"/"mbsimgui"/"icons"/"error.svg").string().c_str()),
                        "S", this);
    showSSE->setToolTip("<p>Show/hide subsequent errors message</p>");
    showSSE->setCheckable(true);
    showSSE->setChecked(true);
    connect(showSSE, &QAction::triggered, this, [=](){ this->updateOutput(); });
    tb->addAction(showSSE);

    showWarn=new QAction(Utils::QIconCached((installPath/"share"/"mbsimgui"/"icons"/"warn.svg").string().c_str()),
                         "W", this);
    showWarn->setToolTip("<p>Show/hide warning messages</p>");
    showWarn->setCheckable(true);
    showWarn->setChecked(true);
    connect(showWarn, &QAction::triggered, this, [=](){ this->updateOutput(); });
    tb->addAction(showWarn);

    showInfo=new QAction(Utils::QIconCached((installPath/"share"/"mbsimgui"/"icons"/"info.svg").string().c_str()),
                         "I", this);
    showInfo->setToolTip("<p>Show/hide info messages</p>");
    showInfo->setCheckable(true);
    showInfo->setChecked(true);
    connect(showInfo, &QAction::triggered, this, [=](){ this->updateOutput(); });
    tb->addAction(showInfo);

    showDepr=new QAction(Utils::QIconCached((installPath/"share"/"mbsimgui"/"icons"/"deprecated.svg").string().c_str()),
                         "D", this);
    showDepr->setToolTip("<p>Show/hide messages about deprecated features.</p>");
    showDepr->setCheckable(true);
    showDepr->setChecked(true);
    connect(showDepr, &QAction::triggered, this, [=](){ this->updateOutput(); });
    tb->addAction(showDepr);

    enableDebug=new QAction(Utils::QIconCached((installPath/"share"/"mbsimgui"/"icons"/"debugBlueEnable.svg").string().c_str()),
                          "D", this);
    enableDebug->setToolTip("<p>Enable debug messages. This may decrease the performance.</p>");
    enableDebug->setCheckable(true);
    enableDebug->setChecked(false);
    connect(enableDebug, &QAction::triggered, this, &EchoView::updateDebug);
    tb->addAction(enableDebug);

    showDebug=new QAction(Utils::QIconCached((installPath/"share"/"mbsimgui"/"icons"/"debugBlue.svg").string().c_str()),
                          "D", this);
    showDebug->setToolTip("<p>Show/hide debug messages</p>");
    showDebug->setCheckable(true);
    showDebug->setChecked(false);
    showDebug->setDisabled(true);
    connect(showDebug, &QAction::triggered, this, [=](){ this->updateOutput(); } );
    tb->addAction(showDebug);

    setMinimumHeight(80);
  }

  void EchoView::clearOutput() {
    {
      QMutexLocker lock(&outTextMutex);
      outText="";
    }
    out->setHtml("");
  }

  namespace {
    QColor mergeColor(const QColor &a, double fac, const QColor &b) {
      return QColor(
        static_cast<int>(a.red()  *fac+b.red()  *(1-fac)),
        static_cast<int>(a.green()*fac+b.green()*(1-fac)),
        static_cast<int>(a.blue() *fac+b.blue() *(1-fac))
      );
    }

    void removeSpan(const QString &span, QString &outText) {
      int start;
      while((start=outText.indexOf(span))!=-1) {
        auto end=outText.indexOf("</span>", start);
        outText.replace(start, end-start+7, "");
      }
    }
  }

  void EchoView::updateOutput(bool moveToErrorOrEnd) {
    int currentSBValue;
    if(!moveToErrorOrEnd)
      currentSBValue=out->verticalScrollBar()->value();

    // some colors
    static const QColor bg(QPalette().brush(QPalette::Active, QPalette::Base).color());
    static const QColor fg(QPalette().brush(QPalette::Active, QPalette::Text).color());
    static const QColor errorColor("red");
    static const QColor warnColor("yellow");
    static const QColor debugColor("blue");
    static const QColor deprColor(mergeColor(errorColor, 0.5, warnColor));
    // set the text as html, prefix with a style element and sourounded by a pre element
    QString html;
    {
      QMutexLocker lock(&outTextMutex);

      // the CSS property display is not supported by QTextBrowser. Hence we remove it manually.
      auto outText2=outText;
      if(!showSSE->isChecked()) removeSpan("<span class=\"MBXMLUTILS_ERROROUTPUT MBXMLUTILS_SSE\">", outText2);
      if(!showWarn->isChecked()) removeSpan("<span class=\"MBSIMGUI_WARN\">", outText2);
      if(!showInfo->isChecked()) removeSpan("<span class=\"MBSIMGUI_INFO\">", outText2);
      if(!showDebug->isChecked()) removeSpan("<span class=\"MBSIMGUI_DEBUG\">", outText2);
      if(!showDepr->isChecked()) removeSpan("<span class=\"MBSIMGUI_DEPRECATED\">", outText2);

      // remove trailing newline
      static QRegularExpression re("\n</span>$");
      outText2.replace(re, "</span>");

      // add anchor at first error, if an error is present
      int firstErrorPos=outText.indexOf("<span class=\"MBSIMGUI_ERROR\">");
      if(firstErrorPos!=-1)
        outText2.replace(firstErrorPos, 0, "<a name=\"MBSIMGUI_FIRSTERROROREND\"/>");
      else
        outText2.append("<a name=\"MBSIMGUI_FIRSTERROROREND\"/>");

      html=QString(R"+(
<!DOCTYPE html>
<html>
  <head>
    <meta http-equiv="Content-Type" content="text/html; charset=UTF-8">
    <title>MBSim Echo View</title>
    <style>
      body                 { color:            @bodyfgcolor@; }
      .MBSIMGUI_ERROR      { background-color: @errorbgcolor@; }
      .MBXMLUTILS_MSG      { font-weight:      bold; }
      .MBXMLUTILS_SSE      { background-color: @ssebgcolor@; }
      .MBSIMGUI_WARN       { background-color: @warnbgcolor@; }
      .MBSIMGUI_DEPRECATED { background-color: @deprbgcolor@; }
      .MBSIMGUI_DEBUG      { background-color: @debugbgcolor@; }
    </style>
  </head>
  <body>
    <pre>
)+").
       replace("@bodyfgcolor@", fg.name()).
       replace("@errorbgcolor@", mergeColor(errorColor, 0.3, bg).name()).
       replace("@ssebgcolor@", mergeColor(errorColor, 0.1, bg).name()).
       replace("@warnbgcolor@", mergeColor(warnColor, 0.3, bg).name()).
       replace("@debugbgcolor@", mergeColor(debugColor, 0.3, bg).name()).
       replace("@deprbgcolor@", mergeColor(deprColor, 0.3, bg).name()).
       append(outText2).
       append(
R"+(</pre>
  </body>
</html>
)+");
    }
    out->setHtml(html);


    if(moveToErrorOrEnd)
      out->scrollToAnchor("MBSIMGUI_FIRSTERROROREND");
    else
      out->verticalScrollBar()->setValue(currentSBValue);
  }

  QSize EchoView::sizeHint() const {
    QSize size=QMainWindow::sizeHint();
    size.setHeight(iconSize().height()*5*2);
    return size;
  }

  namespace {
    bool walk(QModelIndex index, ParameterTreeModel *model, const QUrl &link) {
      // get the filename from the error message
      boost::filesystem::path errorFile(boost::filesystem::absolute(link.path().toStdString()));

      int row = 0;
      // loop over all elements in the current level
      while(index.isValid()) {
        // get the item of the current element (index)
        Parameters *item = dynamic_cast<Parameters*>(model->getItem(index)->getItemData());
        // handle only embed elements but not parameters
        if(item) {
          // get the root element of this embed
          xercesc::DOMElement *e = item->getParent()->getXMLElement();
          // if the file name matchs than the error is from this embed
          if(errorFile==boost::filesystem::absolute(D(e->getOwnerDocument())->getDocumentFilename())) {
            // evalute the xpath expression of the error message in this embed ...
            xercesc::DOMNode *n=D(e->getOwnerDocument())->evalRootXPathExpression(QUrlQuery(link).queryItemValue("xpath").toStdString());
            // ... the node n is there the error occured:
            cerr<<"MISSING this XML node generated the error addr="<<n<<" name="<<X()%n->getNodeName()<<" value="<<X()%n->getNodeValue()<<endl;
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

  void EchoView::linkClicked(const QUrl &link) {
    if(showXML) {
      QFile data(link.path());
      if(data.open(QFile::ReadOnly)) {
	QTextStream out(&data);
	SourceCodeDialog dialog(out.readAll(),true,this);
	dialog.highlightLine(QUrlQuery(link).queryItemValue("line").toInt()-1);
	dialog.exec();
      }
    }
    else
      QMessageBox::information(this, "Debug information", "For more debug information press \"Debug model\" and click this link again.");
    //MISSING mbsimgui adds new elements e.g. <plotFeatureRecursive value="plotRecursive">false</plotFeatureRecursive>
    //MISSING this break the xpath of the error messages
    // get the model of the embedding
//    ParameterTreeModel *model = static_cast<ParameterTreeModel*>(mw->getParameterView()->model());
//    // walk all embeded elements
//    if(!walk(model->index(0,0), model, link))
//      cerr<<"MISSING No XML node found for file="<<link.path().toStdString()<<
//            " line="<<QUrlQuery(link).queryItemValue("line").toStdString()<<endl<<
//            "        xpath="<<QUrlQuery(link).queryItemValue("xpath").toStdString()<<endl;
  }

  void EchoView::updateDebug() {
    if(enableDebug->isChecked()) {
      showDebug->setDisabled(false);
      showDebug->setChecked(true);
      mw->refresh();
    }
    else {
      showDebug->setDisabled(true);
      showDebug->setChecked(false);
      updateOutput();
    }
  }

  void EchoView::addOutputText(const QString &outText_) {
    QMutexLocker lock(&outTextMutex);
    outText += outText_;
  }

}
