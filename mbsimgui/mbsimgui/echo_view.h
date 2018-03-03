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
#include <QWebView>
#include <QAction>
#include <sstream>
#include <QMainWindow>

namespace MBSimGUI {

  class EchoView : public QMainWindow {
    Q_OBJECT
    public:
      EchoView(QMainWindow *parent);
      void clearOutput();
      QSize sizeHint() const override;
      void addOutputText(const QString &outText_) { outText += outText_; }
      bool debugEnabled() { return enableDebug->isChecked(); }
    public slots:
      void updateOutput(bool moveToErrorOrEnd=false);
    private:
      QWebView *out;
      QString outText;
      QAction *showSSE;
      QAction *showWarn;
      QAction *showInfo;
      QAction *showDepr;
      QAction *enableDebug;
      QAction *showDebug;
    private slots:
      void linkClicked(const QUrl &link);
      void updateDebug();
  };

  class EchoStream : public std::stringbuf {
    public:
      EchoStream(EchoView *ev_, const std::string &className_) : std::stringbuf(std::ios_base::out),
        ev(ev_), className(className_) {}
    protected:
      int sync() override { // overwrite the sync function from stringbuf
        ev->addOutputText(("<span class=\""+className+"\">"+str()+"</span>").c_str());
        // clear the buffer and return
        str("");
        return 0;
      }
      EchoView *ev;
      std::string className;
  };

}

#endif
