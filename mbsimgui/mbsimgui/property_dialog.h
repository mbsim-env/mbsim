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

#ifndef _PROPERYDIALOG_H_
#define _PROPERYDIALOG_H_

#include <QDialog>
#include <boost/function.hpp>
#include <mbxmlutilstinyxml/tinyxml.h>

class MainWindow;

class PropertyDialog : public QDialog {
  Q_OBJECT

  protected:
    PropertyDialog(MainWindow *mw,
                   QWidget *widget,
                   boost::function<TiXmlElement* (TiXmlNode*)> writeXMLFile,
                   boost::function<void (TiXmlElement*)> initializeUsingXML);
    ~PropertyDialog();
    static PropertyDialog *instance;
    boost::function<void (TiXmlElement*)> initializeUsingXML;
    TiXmlElement *savedSettings;
    QWidget *widget;
    MainWindow *mw;
  public:
    static void create(MainWindow *mw,
                       QWidget *widget,
                       boost::function<TiXmlElement* (TiXmlNode*)> writeXMLFile,
                       boost::function<void (TiXmlElement*)> initializeUsingXML);
  protected slots:
    void applyPressed();
    void cancelPressed();
    void okPressed();
};

#endif
