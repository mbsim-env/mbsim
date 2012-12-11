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

#ifndef _PROPERTY_WIDGET_H_
#define _PROPERTY_WIDGET_H_

#include <QScrollArea>
#include <map>

class ExtXMLWidget;
class QVBoxLayout;
class QTabWidget;
class TiXmlElement;
class TiXmlNode;

class PropertyWidget : public QScrollArea {
  Q_OBJECT

    friend class PropertyDialog;
  public:
    PropertyWidget(QObject *obj);
    ~PropertyWidget();
    void setParentObject(QObject *obj);
    void addToTab(const QString &name, ExtXMLWidget* widget_);
    void addTab(const QString &name);
    QObject* getParentObject() { return parentObject; }
    void addStretch();
    void update();
    void initialize();
    void resizeVariables();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    QObject* parentObject;
    std::map<QString,QVBoxLayout*> layout;
    QVBoxLayout *mainLayout;
    std::vector<ExtXMLWidget*> widget;
    QTabWidget *tabWidget;
  signals:
    void dataAccepted();
};

#endif
