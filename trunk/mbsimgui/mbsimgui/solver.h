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

#ifndef _SOLVER__H_
#define _SOLVER__H_

#include "group.h"
#include <string>
#include <QtCore/QFileInfo>
#include <QtCore/QDateTime>


class Environment : public QObject {
  Q_OBJECT
  public:
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
    static Environment *getInstance() { return instance?instance:(instance=new Environment); }

  protected:
    Environment();
    virtual ~Environment();
    static Environment *instance;
};

class Solver : public Group {
  Q_OBJECT
  friend class MainWindow;
  friend class Object;
  protected:
    XMLEditor *environment;
  public:
    Solver(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual QString getInfo();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    QString getType() const { return "DynamicSystemSolver"; }
    QString getFileExtension() const { return ".mbsim.xml"; }
    //QString getParameterFile() const { return ((FileWidget*)parameterFile->getXMLWidget())->getFile(); }
    //void setParameterFile(const QString &file) { ((FileWidget*)parameterFile->getXMLWidget())->setFile(file); }

    static Solver* readXMLFile(const QString &filename, QTreeWidgetItem *parent);
    void writeXMLFile(const QString &name);
    void writeXMLFile() { writeXMLFile(getName()); }
    //void writeXMLParamFile(const QString &name) {parameters->writeXMLFile(name);}
};

#endif
