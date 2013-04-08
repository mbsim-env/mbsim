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

#include <QtGui/QTreeWidgetItem>
#include "extended_properties.h"

class PropertyWidget;
class PropertyDialog;
class ExtWidget;
class TiXmlElement;
class TiXmlNode;
class TextWidget;

class Parameter : public QObject, public QTreeWidgetItem {
  Q_OBJECT
  friend class Editor;
  friend class MainWindow;
  protected:
    QString newName(const QString &type);
    bool drawThisPath;
    QString iconFile;
    bool searchMatched;
    PropertyDialog *dialog;
    QMenu *contextMenu;
    QString name;
    TextWidget *textWidget;
  public:
    Parameter(const QString &str, QTreeWidgetItem *parentItem);
    virtual ~Parameter();
    QString &getIconFile() { return iconFile; }
    virtual QString getInfo();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    static Parameter* readXMLFile(const QString &filename, QTreeWidgetItem *parent);
    virtual void writeXMLFile(const QString &name);
    virtual void writeXMLFile() { writeXMLFile(getType()); }
    virtual QString getType() const { return "Parameter"; }
    QMenu* getContextMenu() { return contextMenu; }
    QString getName() const {return name;}
    void setName(const QString &str);
    virtual std::string getValue() const = 0;
    virtual void fromWidget();
    virtual void toWidget();
    virtual void initializeDialog();
  public slots:
    void saveAs();
    void openPropertyDialog();
    void updateElement();
  protected slots:
    void updateTreeWidgetItem(const QString &str);
    void remove();
  signals:
    void parameterChanged(const QString &str);

};

class DoubleParameter : public Parameter {
  public:
    DoubleParameter(const QString &str, QTreeWidgetItem *parentItem);
    virtual QString getType() const { return "scalarParameter"; }
    virtual std::string getValue() const;
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual void fromWidget();
    virtual void toWidget();
    virtual void initializeDialog();
  protected:
    ExtWidget *valueWidget;
    ExtProperty value;
};

#endif
