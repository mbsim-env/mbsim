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

#ifndef _PARAMETER__H_
#define _PARAMETER__H_

#define PARAMNS_ "http://openmbv.berlios.de/MBXMLUtils/parameter"
#define PARAMNS "{"PARAMNS_"}"

#include <QtGui/QTreeWidgetItem>
#include "mbsimguitinyxml/tinyxml-src/tinyxml.h"
#include "mbsimguitinyxml/tinyxml-src/tinynamespace.h"
#include "editors.h"
#include "utils.h"
#include <string>
#include <set>

class PropertyDialog;

class Parameter : public QObject, public QTreeWidgetItem {
  Q_OBJECT
  friend class Editor;
  friend class MainWindow;
  protected:
    QString newName(const QString &type);
    bool drawThisPath;
    std::string iconFile;
    bool searchMatched;
    PropertyDialog *properties;
    QMenu *contextMenu;
    ExtXMLWidget *name;
  public:
    Parameter(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual ~Parameter();
    std::string &getIconFile() { return iconFile; }
    virtual QString getInfo();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    static Parameter* readXMLFile(const QString &filename, QTreeWidgetItem *parent);
    virtual void writeXMLFile(const QString &name);
    virtual void writeXMLFile() { writeXMLFile(getType()); }
    virtual QString getType() const { return "Parameter"; }
    QMenu* getContextMenu() { return contextMenu; }
    PropertyDialog* getPropertyDialog() { return properties; }
    QString getName() const {return text(0);}
    void setName(const QString &str) {setText(0,str);((NameWidget*)name->getWidget())->setName(str);}
    virtual std::string getValue() const = 0;
  public slots:
    void saveAs();
  protected slots:
    void updateTreeWidgetItem(const QString &str);
    void remove();
  signals:
    void parameterChanged(const QString &str);

};

class DoubleParameter : public Parameter {
  public:
    DoubleParameter(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual QString getType() const { return "scalarParameter"; }
    virtual std::string getValue() const { return ((ParameterValueWidget*)value->getWidget())->getValue(); }
    virtual void initializeUsingXML(TiXmlElement *element);
  protected:
    ExtXMLWidget *value;
};

#endif
