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

#ifndef _GROUP__H_
#define _GROUP__H_

#include "element.h"
#include <string>
#include <QtCore/QFileInfo>
#include <QtCore/QDateTime>
#include <QtGui/QInputDialog>
#include <QtGui/QMessageBox>


class Group : public Element {
  Q_OBJECT
  friend class MainWindow;
  friend class Element;
  protected:
    QString getType() const { return "Group"; }
    QAction *actionPaste;
    QFileInfo *xmlFileInfo, *h5FileInfo;
    QDateTime xmlLastModified, h5LastModified;
    ExtXMLWidget *position, *orientation, *parameterFile, *frameOfReference, *framePos;
    void setActionPasteDisabled(bool flag);

  public:
    Group(const QString &str, QTreeWidgetItem *parentItem, int ind);
    virtual QString getInfo();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual Element *getByPathSearch(std::string path);

  protected slots:
    void addGroup();
    void addFrame();
    void addRigidBody();
    void addJointConstraint();
    void addJoint();
    void addKineticExcitation();
    void addSpringDamper();
    //void remove();

  public slots:
    void addFromFile();
    void paste();
};

#endif
