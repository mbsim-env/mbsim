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

#ifndef __PROCESS_H_
#define __PROCESS_H_

#include <QProcess>
#include <QTimer>
#include <QTabWidget>
#include <QTextBrowser>
#include <QThread>
#include <xercesc/util/XercesDefs.hpp>
#include <boost/filesystem.hpp>
#include <QDir>

namespace XERCES_CPP_NAMESPACE {
  class DOMNode;
  class DOMElement;
  class DOMDocument;
}

namespace MBXMLUtils {
  class DOMParser;
  class Eval;
}

namespace MBSimGUI {

  class Process : public QTabWidget {
    Q_OBJECT
    public:
      Process(QWidget *parent);
      QProcess *getProcess() { return process; }
      void clearOutputAndStart(const QString &program, const QStringList &arguments);
      QSize sizeHint() const;
      QSize minimumSizeHint() const;
    private:
      QProcess *process;
      QTextBrowser *out, *err;
      QString outText, errText;
      static QString convertToHtml(QString &text);
      void linkClicked(const QUrl &link, QTextBrowser *std);
      QTimer timer;
    private slots:
      void updateOutputAndError();
      void outLinkClicked(const QUrl &link);
      void errLinkClicked(const QUrl &link);
      void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
    public slots:
      void interrupt();
      void preprocessFailed(const QString& errText);
  };

  class MBSimThread : public QThread {
    Q_OBJECT
    public:
      MBSimThread(QObject * parent = 0 ) : QThread(parent) { }
      void run();
      void setParser(const std::shared_ptr<MBXMLUtils::DOMParser> &parser_) { parser = parser_; }
      void setDocument(const std::shared_ptr<xercesc::DOMDocument> &doc_) { doc = doc_; }
      void setProjectFile(const QString &file) { projectFile = file; }
      void setEvaluator(const std::string &evaluator_) { evaluator = evaluator_; }
      const QString& getErrorText() const { return errText; }
    private:
      std::shared_ptr<MBXMLUtils::DOMParser> parser;
      std::shared_ptr<xercesc::DOMDocument> doc;
      QString projectFile, errText;
      std::string evaluator;
    signals:
      void resultReady(int result);
  };

}

#endif
