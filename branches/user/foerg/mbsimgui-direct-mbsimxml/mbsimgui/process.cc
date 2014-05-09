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
#include <boost/scoped_ptr.hpp>
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objectfactory.h"
#include "mbsim/integrators/integrator.h"
#include <mbxmlutilshelper/last_write_time.h>

using namespace std;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

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
    if(outArray.size()!=0) {
      outText+=outArray.data();
      out->setHtml(convertToHtml(outText));
      out->moveCursor(QTextCursor::End);
    }

    QByteArray errArray=process->readAllStandardError();
    if(errArray.size()!=0) {
      errText+=errArray.data();
      err->setHtml(convertToHtml(errText));
      err->moveCursor(QTextCursor::Start);
    }
  }

  QString Process::convertToHtml(QString &text) {
    // the following operations modify the original text

#ifdef WIN32
    // convert windows line ending to linux line ending (they are later replaced to html line ending)
    text.replace("\x0D\x0A", "\x0A");
#endif
    // from now on use linux line ending => do not use \n, \r in string literals but \x0A, \x0D

    // remove all lines but the last ending with carriage return '\x0D'
    static QRegExp carriageReturn("(^|\x0A)[^\x0A]*\x0D([^\x0A\x0D]*\x0D)");
    text.replace(carriageReturn, "\\1\\2");

    // replace <FILE ...> to html reference
    static QRegExp fileRE("<FILE path=\"([^\"]+)\" line=\"([0-9]+)\">([^<]+)</FILE>");
    text.replace(fileRE, "<a href=\"\\1?line=\\2\">\\3</a>");

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

  void WorkerThread::run() {
    int result=1;
    try {
      DOMElement *root = doc->getDocumentElement();

      // GUI-XML-Baum mit OriginalFilename ergaenzen
      if(!E(root)->getFirstProcessingInstructionChildNamed("OriginalFilename")) {
        DOMProcessingInstruction *filenamePI=doc->createProcessingInstruction(X()%"OriginalFilename",
            X()%"MBS.mbsimprj.xml");
        root->insertBefore(filenamePI, root->getFirstChild());
      }

      cout << "validating" << endl;
      D(doc)->validate();

      vector<boost::filesystem::path> dep;
      OctEval octEval(&dep);
      DOMElement *mainxmlele=doc->getDocumentElement();
      cout << "preprocessing" << endl;
      preprocess(parser, octEval, dep, mainxmlele);

      DOMElement *e=doc->getDocumentElement();

      // create object for DynamicSystemSolver and check correct type
      boost::scoped_ptr<DynamicSystemSolver> dss(ObjectFactory<Element>::createAndInit<DynamicSystemSolver>(e->getFirstElementChild()));

      // create object for Integrator and check correct type
      boost::scoped_ptr<Integrator> integrator(ObjectFactory<Integrator>::createAndInit<Integrator>(e->getFirstElementChild()->getNextElementSibling()));

      dss->initialize();

      if(task) {
        int zSize=dss->getzSize();
        fmatvec::Vec z(zSize);
        if(integrator->getInitialState().size())
          z = integrator->getInitialState();
        else
          dss->initz(z);          
        dss->computeInitialCondition();
        dss->plot(z, 0);
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.xml").c_str(), boost::posix_time::microsec_clock::universal_time());
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.h5" ).c_str(), boost::posix_time::microsec_clock::universal_time());
      }
      else {
        integrator->integrate(*dss);
      }
      result=0;
    }
    catch(const MBSimError &e) {
      cerr<<e.what()<<endl;
    }
    catch(const H5::Exception &e) {
      cerr<<"HDF5 exception: "<<e.getCDetailMsg()<<endl<<
        "function: "<<e.getCFuncName()<<endl;
    }
    catch(const std::exception &e) {
      cerr<<"Exception: "<<e.what()<<endl;
    }
    catch (boost::thread_interrupted&) 
    { 
      cerr<<"interrupted"<<endl;
    } 
    catch(...) {
      cerr<<"Unknown exception"<<endl;
    }

    //   DOMElement *root = doc->getDocumentElement();

    //   // GUI-XML-Baum mit OriginalFilename ergaenzen
    //   if(!E(root)->getFirstProcessingInstructionChildNamed("OriginalFilename")) {
    //     DOMProcessingInstruction *filenamePI=doc->createProcessingInstruction(X()%"OriginalFilename",
    //         X()%"MBS.mbsimprj.xml");
    //     root->insertBefore(filenamePI, root->getFirstChild());
    //   }

    //   try {
    //     cout << "validating" << endl;
    //     D(doc)->validate();

    //     vector<boost::filesystem::path> dep;
    //     OctEval octEval(&dep);
    //     DOMElement *mainxmlele=doc->getDocumentElement();
    //     cout << "preprocessing" << endl;
    //     preprocess(parser, octEval, dep, mainxmlele);

    //     cout << "mbsimflatxml" << endl;

    //     MBSim::MBSimXML::mbsimflatxml(doc,false,task,false);
    //   }
    //   catch(const std::exception &e) {
    //     cerr<<"Exception: "<<e.what()<<endl;
    //   }
    //   catch(...) {
    //     cerr<<"Unknown exception"<<endl;
    //   }
    emit resultReady(result);
  }
}
