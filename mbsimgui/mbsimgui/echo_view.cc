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
#include <QClipboard>
#include <boost/dll.hpp>
#include "element_view.h"
#include "file_view.h"
#include "fileitemdata.h"
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
#include "project.h"
#include "xercesc/dom/DOMLSSerializer.hpp"
#include "xercesc/dom/DOMAttr.hpp"
#include <QTextStream>
#include <QProcessEnvironment>
#include <QUrlQuery>
#include <QMessageBox>
#include <boost/scope_exit.hpp>

using namespace std;
using namespace MBXMLUtils;

namespace MBSimGUI {

  extern MainWindow *mw;

  EchoView::EchoView() : QMainWindow() {
    // QMainWindow set the window flag Qt::Window in its ctor but we use this QMainWindow as a widget
    // -> reset the windows flag to Qt::Widget
    setWindowFlags(Qt::Widget); // we cannot do this by : QMainWindow(parent, Qt::Widget) since QMainWindow overwrites this later on

    static boost::filesystem::path installPath(boost::dll::program_location().parent_path().parent_path());

    setIconSize(iconSize()*0.5);

    class EchoViewTextBrowser : public QTextBrowser {
      public:
        EchoViewTextBrowser(EchoView* ev=nullptr) : QTextBrowser(ev), echoView(ev) {}
      protected:
        void contextMenuEvent(QContextMenuEvent *e) override {
          if(auto link = anchorAt(e->pos()); !link.isEmpty()) {
            echoView->linkRightClicked(QUrl(link));
            return;
          }
          QTextBrowser::contextMenuEvent(e);
        }
      private:
        EchoView *echoView;
    };
    out=new EchoViewTextBrowser(this);
    out->setOpenLinks(false);
    out->setHorizontalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAlwaysOn); // if the bar appears/disappears then scrolling to the last line may fail (the very last is hidden by the bar)
    connect(out, &QTextBrowser::anchorClicked, [this](const QUrl &link){
      QSettings settings;
      linkClicked(link, settings.value("mainwindow/options/openPropertyDialogOnErrorLinks", true).toBool());
    });
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
      static QRegularExpression spanStartRE(R"|(<\s*span\s)|");
      static QRegularExpression spanEndRE  (R"|(<\s*/\s*span\s*>)|");
      int start;
      bool stop=false;
      while(!stop && (start=outText.indexOf(span))!=-1) {
        int count=1;
        int s=start;
        while(true) {
          auto spanStartM = spanStartRE.match(outText, s+1);
          auto spanEndM   = spanEndRE  .match(outText, s+1);
          auto spanStartM_capturedStart = spanStartM.capturedStart();
          if(spanStartM_capturedStart==-1)
            spanStartM_capturedStart=numeric_limits<int>::max();
          if(spanEndM.capturedStart()==-1) {
            cerr<<"Internal error: <span> elements do not match"<<endl;
            stop = true; // break also the outer loop (we cannot match the <span> elements
            break;
          }
          auto spanEndM_capturedStart = spanEndM.capturedStart();
          auto spanEndM_capturedEnd = spanEndM.capturedEnd();
          count += spanStartM_capturedStart<spanEndM_capturedStart ? 1 : -1;
          if(count==0) {
            outText.replace(start, spanEndM_capturedEnd-start, "");
            break;
          }
          s=std::min(spanStartM_capturedStart, spanEndM.capturedStart());
        }
      }
    }
  }

  void EchoView::updateOutput(bool moveToErrorOrEnd) {
    int currentSBValue=0;
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

      // the CSS property "display" is not supported by QTextBrowser. Hence, we cannot use "display:none" to remove the content
      // and we need to remove it manually using the removeSpan helper function.
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
        outText2.replace(firstErrorPos, 0, "<a name=\"MBSIMGUI_FIRSTERROROREND\"></a>");
      else
        outText2.append("<a name=\"MBSIMGUI_FIRSTERROROREND\"></a>");

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

  void EchoView::linkClicked(const QUrl &link, bool openPropertyDialog) {
    // get the XML document doc and the rootIndex of this element corresponding the path of the link
    QModelIndex rootIndex;
    std::shared_ptr<xercesc::DOMDocument> doc;
    if(QFileInfo(mw->getProjectFile()).absoluteFilePath()==link.path()) {
      doc = mw->getProjectDocument();
      rootIndex = mw->getProject()->getModelIndex();
    }
    else
      for(auto fileItemData : mw->getFile()) {
        if(fileItemData->getFileInfo().absoluteFilePath()==link.path()) {
          doc = fileItemData->getXMLDocument();
          if(E(fileItemData->getXMLElement()).getTagName() == PV%"Parameter")
            rootIndex = fileItemData->getFileReference(0)->getParameterEmbedItem()->getModelIndex();
          else
            rootIndex = fileItemData->getFileReference(0)->getModelIndex();
          break;
        }
      }

    if(!doc) {
      mw->statusBar()->showMessage("XML document not found: "+link.path());
      return;
    }

    // get the xpath in the re-parsed document
    auto xpath = QUrlQuery(link).queryItemValue("xpath").toStdString();

    if(openPropertyDialog) {
      // open the property dialog of the element which generated the error

      // walk the element tree to find the item with the same xpath as the link
      auto *model = static_cast<const TreeModel*>(rootIndex.model());
      if(model) {
        QModelIndex clickedIndex;
        size_t maxXPathFound = 0;
        function<void(TreeItem *item)> walk;
        walk = [&walk,&xpath,&maxXPathFound,&clickedIndex](TreeItem *item) {
          if(auto e = item->getItemData()->getXMLElement(); e) {
            auto itemXPath = E(e)->getRootXPathExpression();
            if(xpath.substr(0,itemXPath.size())==itemXPath && itemXPath.size()>maxXPathFound) {
              clickedIndex = item->getItemData()->getModelIndex();
              maxXPathFound = itemXPath.size();
            }
          }

          for(int cNr=0; cNr<item->childCount(); ++cNr)
            walk(item->child(cNr));
        };
        walk(model->getItem(rootIndex));
        if(clickedIndex.isValid()) {
          if(dynamic_cast<const ElementTreeModel*>(model)) {
            mw->getElementView()->setCurrentIndex(clickedIndex);
            mw->openElementEditor();
          }
          else if(dynamic_cast<const ParameterTreeModel*>(model)) {
            mw->getParameterView()->setCurrentIndex(clickedIndex);
            mw->openParameterEditor();
          }
        }
        else {
          mw->statusBar()->showMessage("No element/parameter found for XPath in doc: "+
            QString::fromStdString(EmbedDOMLocator::convertToRootHRXPathExpression(xpath)));
          openPropertyDialog = false;
        }
      }
      else
        openPropertyDialog = false;
    }

    if(!openPropertyDialog) {
      // show the XML source code of doc and the line number of the error

      // serialize the document without using the MBXMLUtils_ processing instructions (its a user written XML source file)
      auto rootEle = doc->getDocumentElement();
      auto xml = X()%mw->serializer->writeToString(rootEle);
      // re-parse the serialized document (while recording the line number using the MBXMLUtils_ processing instructions
      // (this generated a equal XML document as the document used by the MainWindow refresh and simulation action)
      stringstream str(std::move(xml));
      auto docReparsed = mw->mbxmlparserNoVal->parse(str);

      xercesc::DOMNode *n;
      try {
        n = D(docReparsed)->evalRootXPathExpression(xpath);
      }
      catch(...) {
        mw->statusBar()->showMessage("XPath not found in doc: "+
          QString::fromStdString(EmbedDOMLocator::convertToRootHRXPathExpression(xpath)));
        return;
      }
      auto e = n->getNodeType() == xercesc::DOMNode::ATTRIBUTE_NODE ?
        static_cast<xercesc::DOMAttr*>(n)->getOwnerElement() :
        static_cast<xercesc::DOMElement*>(n);

      // get the linenr of this re-parsed element
      // (Note that we cannot just take the linenr of the original element this its linenr is usually wrong, at least
      //  when mbsimgui has changed something in the XML DOM tree since it was read from file)
      auto lineNr = E(e)->getLineNumber();

      // show the dialog, highlight lineNr and scroll to it
      auto absPath = link.path();
      auto relPath = QDir(QFileInfo(mw->getProjectFile()).absoluteDir()).relativeFilePath(absPath);
      SourceCodeDialog dialog(xml.c_str(),
        (absPath.size()<relPath.size() || relPath.startsWith("../")) ? absPath : relPath, // filename to show
        EmbedDOMLocator::convertToRootHRXPathExpression(xpath).c_str(), // xpath to show
        this);
      dialog.highlightLine(lineNr-1);
      dialog.exec();
    }
  }

  void EchoView::linkRightClicked(const QUrl &link) {
    QSettings settings;
    bool openProperyDialog = settings.value("mainwindow/options/openPropertyDialogOnErrorLinks", true).toBool();
    QMenu menu;
    menu.addAction(openProperyDialog ? "Open element (or, if not found, XML source)..." : "Open XML source...",
      [this, openProperyDialog, &link](){
        linkClicked(link, openProperyDialog);
    });
    menu.addSeparator();
    menu.addAction("Open element...", [this, &link](){
      linkClicked(link, true);
    }),
    menu.addAction("Open XML source...", [this, &link](){
      linkClicked(link, false);
    }),
    menu.addSeparator();
    menu.addAction("Copy link location", [&link](){
      QGuiApplication::clipboard()->setText(link.url(), QClipboard::Clipboard);
      QGuiApplication::clipboard()->setText(link.url(), QClipboard::Selection);
    });
    menu.exec(QCursor::pos());
  }

  void EchoView::updateDebug() {
    mw->restartProcessRefresh(); // needed to update the --stdout debug~... argument of the refresh mbsimxml process
    mw->restartProcessSimulate(); // needed to update the --stdout debug~... argument of the refresh mbsimxml process
    if(enableDebug->isChecked()) {
      showDebug->setDisabled(false);
      showDebug->setChecked(true);
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
