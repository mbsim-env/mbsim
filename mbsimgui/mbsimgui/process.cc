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
#include "process.h"
#include <iostream>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
#include <mbxmlutils/octeval.h>
#include <mbsimxml/mbsimflatxml.h>
#include <mbxmlutils/preprocess.h>
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsim/integrators/integrator.h"
#include <mbxmlutilshelper/last_write_time.h>
#include <mbxmlutilshelper/getinstallpath.h>

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern bool currentTask;

  Process::Process(QWidget *parent) : QTabWidget(parent) {
    process=new QProcess(this);
    out=new QTextBrowser(this);
    err=new QTextBrowser(this);
    out->setOpenLinks(false);
    err->setOpenLinks(false);
    connect(out, SIGNAL(anchorClicked(const QUrl &)), this, SLOT(outLinkClicked(const QUrl &)));
    connect(err, SIGNAL(anchorClicked(const QUrl &)), this, SLOT(errLinkClicked(const QUrl &)));
    connect(process, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(processFinished(int,QProcess::ExitStatus)));
    addTab(out, "Out");
    addTab(err, "Err");
    setCurrentIndex(0);
    setMinimumHeight(80);
    setTabPosition(QTabWidget::West);
    connect(&timer, SIGNAL(timeout()), this, SLOT(updateOutputAndError()));
  }

  void Process::clearOutputAndStart(const QString &program, const QStringList &arguments) {
    outText="";
    errText="";
    out->clear();
    err->clear();
    setCurrentIndex(0);
    process->start(program, arguments);
    timer.start(250);
  }

  void Process::updateOutputAndError() {
    QByteArray outArray=process->readAllStandardOutput();
    outText+=outArray.data();
    out->setHtml(convertToHtml(outText));
    out->moveCursor(QTextCursor::End);

    QByteArray errArray=process->readAllStandardError();
    errText+=errArray.data();
    err->setHtml(convertToHtml(errText));
    err->moveCursor(QTextCursor::Start);
  }

  QString Process::convertToHtml(QString &text) {
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

  QSize Process::sizeHint() const {
    QSize size=QTabWidget::sizeHint();
    size.setHeight(80);
    return size;
  }

  QSize Process::minimumSizeHint() const {
    QSize size=QTabWidget::minimumSizeHint();
    size.setHeight(80);
    return size;
  }

  void Process::linkClicked(const QUrl &link, QTextBrowser *std) {
    static QString editorCommand=QProcessEnvironment::systemEnvironment().value("MBSIMGUI_EDITOR", "gvim -R %1 +%2");
    cout<<"Opening file using command '"<<editorCommand.toStdString()<<"'. "<<
      "Where %1 is replaced by the filename and %2 by the line number. "<<
      "Use the environment variable MBSIMGUI_EDITOR to overwrite this command."<<endl;
    QString comm=editorCommand.arg(link.path()).arg(link.queryItemValue("line").toInt());
    QProcess::startDetached(comm);
  }

  void Process::outLinkClicked(const QUrl &link) {
    linkClicked(link, out);
  }

  void Process::errLinkClicked(const QUrl &link) {
    linkClicked(link, err);
  }

  void Process::processFinished(int exitCode, QProcess::ExitStatus exitStatus) {
    timer.stop();
    updateOutputAndError();
    if(exitStatus==QProcess::NormalExit && exitCode==0)
      setCurrentIndex(0);
    else
      setCurrentIndex(1);
  }

  void Process::interrupt() {
    errText="Simulation interrupted";
#ifdef _WIN32
    process->kill();
#else
    process->terminate();
#endif
  }
  
  void Process::preprocessFailed(const QString& errText_) {
    errText = errText_;
    processFinished(1,QProcess::NormalExit);
  }

  void MBSimThread::run() {
    try {
      DOMElement *root = doc->getDocumentElement();

      // GUI-XML-Baum mit OriginalFilename ergaenzen
      if(!E(root)->getFirstProcessingInstructionChildNamed("OriginalFilename")) {
        DOMProcessingInstruction *filenamePI=doc->createProcessingInstruction(X()%"OriginalFilename",
            X()%"MBS.mbsimprj.xml");
        root->insertBefore(filenamePI, root->getFirstChild());
      }

      D(doc)->validate();

      vector<boost::filesystem::path> dependencies;
      OctEval dummy(&dependencies);
      Eval &eval=dummy;

      // Praeprozessor starten
      DOMElement *mainxmlele=doc->getDocumentElement();
      Preprocess::preprocess(parser, eval, dependencies, mainxmlele);
    }
    catch(exception &ex) {
      errText = ex.what();
      emit resultReady(1);
      return;
    }
    catch(...) {
      errText = "Unknown exception in preprocess";
      emit resultReady(1);
      return;
    }
    DOMParser::serialize(doc.get(), projectFile.toStdString(), false);

    emit resultReady(0);
  }
}
