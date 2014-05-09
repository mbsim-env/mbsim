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
#include <boost/shared_ptr.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/thread/thread.hpp>
#include <QDir>

namespace XERCES_CPP_NAMESPACE {
  class DOMNode;
  class DOMElement;
  class DOMDocument;
}

namespace MBXMLUtils {
  class DOMParser;
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
      QTextBrowser *out, *err;
      static QString convertToHtml(QString &text);
    private:
      QProcess *process;
      QString outText, errText;
      void linkClicked(const QUrl &link, QTextBrowser *std);
      QTimer timer;
      private slots:
        void updateOutputAndError();
      void outLinkClicked(const QUrl &link);
      void errLinkClicked(const QUrl &link);
      void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
  };

  class WorkerThread : public QObject {
    Q_OBJECT
    public:
      WorkerThread() : task(false) { }
      void run();
      void setTask(bool task_) { task = task_; }
      void setParser(const boost::shared_ptr<MBXMLUtils::DOMParser> &parser_) { parser = parser_; }
      void setDocument(const boost::shared_ptr<xercesc::DOMDocument> &doc_) { doc = doc_; }
      void start() { wthread = boost::thread(&WorkerThread::run, this); }
      void join() { wthread.join(); }
      void detach() { wthread.detach(); }
      void terminate() { wthread.interrupt(); }
    private:
      boost::filesystem::path uniqueTempDir;
      boost::shared_ptr<MBXMLUtils::DOMParser> parser;
      boost::shared_ptr<xercesc::DOMDocument> doc;
      bool task;
      boost::thread wthread;    
    signals:
      void resultReady(int result);
  };

}

#endif
