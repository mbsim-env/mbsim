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
#include "group.h"
#include <QtGui/QMenu>
#include <QtGui/QPushButton>
#include <QtGui/QInputDialog>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include "objectfactory.h"
#include "rigidbody.h"
#include "constraint.h"
#include "spring_damper.h"
#include "joint.h"
#include "kinetic_excitation.h"
#include "frame.h"
#include "editors.h"
#include <string>
#include "utils.h"

using namespace std;

Group::Group(const QString &str, QTreeWidgetItem *parentItem, int ind) : Element(str, parentItem, ind), frameOfReference(0), position(0), orientation(0) {

  setText(1,getType());

  iconFile="group.svg";

  QColor color;
  color.setRgb(200,200,200);
  QBrush brush(color);
  color.setRgb(255,255,255);
  QBrush brush2(color);

  frames = new Container;
  frames->setText(0, "frames");
  frames->setForeground(0,brush);
  frames->setBackground(0,brush2);
  addChild(frames);
  contours = new Container;
  contours->setText(0, "contours");
  contours->setForeground(0,brush);
  contours->setBackground(0,brush2);
  addChild(contours);
  groups = new Container;
  groups->setText(0, "groups");
  groups->setForeground(0,brush);
  groups->setBackground(0,brush2);
  addChild(groups);
  objects = new Container;
  objects->setText(0, "objects");
  objects->setForeground(0,brush);
  objects->setBackground(0,brush2);
  addChild(objects);
  extraDynamics = new Container;
  extraDynamics->setText(0, "extraDynamics");
  extraDynamics->setForeground(0,brush);
  extraDynamics->setBackground(0,brush2);
  addChild(extraDynamics);
  links = new Container;
  links->setText(0, "links");
  links->setForeground(0,brush);
  links->setBackground(0,brush2);
  addChild(links);

  //if(parentItem != treeWidget()->invisibleRootItem())
    //new Frame("I", frames, -1, false);
  //else
    new Frame("I", frames, -1, true);

  QAction *action;

  properties->addTab("Frame positioning");
  if(parentItem != treeWidget()->invisibleRootItem()) {
    properties->addTab("Kinematics");

    vector<PhysicalStringWidget*> input;
    input.push_back(new PhysicalStringWidget(new SVecWidget(3),MBSIMNS"position",lengthUnits(),4));
    position = new ExtPhysicalVarWidget("Position", input);
    properties->addToTab("Kinematics", position);

    input.clear();
    input.push_back(new PhysicalStringWidget(new SMatWidget(getEye<string>(3,3,"1","0")),MBSIMNS"orientation",noUnitUnits(),1));
    orientation = new ExtPhysicalVarWidget("Orientation",input);
    properties->addToTab("Kinematics", orientation);

    frameOfReference=new FrameOfReferenceWidget("Frame of reference",MBSIMNS"frameOfReference",this,((Group*)getParentElement())->getFrame(0));
    properties->addToTab("Kinematics", frameOfReference);
  }

  framePos = new FramePositionsWidget("Position and orientation of frames",this);
  properties->addToTab("Frame positioning", framePos);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFrame()));
  contextMenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Add group", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addGroup()));
  contextMenu->addAction(action);

  //action=new QAction(Utils::QIconCached("newobject.svg"),"Add object", this);
  //connect(action,SIGNAL(triggered()),this,SLOT(addObject()));
  //contextMenu->addAction(action);
  QMenu *submenu = contextMenu->addMenu("Add object");
  action=new QAction(Utils::QIconCached("newobject.svg"),"Rigid body", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addRigidBody()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Joint constraint", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addJointConstraint()));
  submenu->addAction(action);

  //action=new QAction(Utils::QIconCached("newobject.svg"),"Add link", this);
  //connect(action,SIGNAL(triggered()),this,SLOT(addLink()));
  //contextMenu->addAction(action);
  submenu = contextMenu->addMenu("Add link");
  action=new QAction(Utils::QIconCached("newobject.svg"),"Joint", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addJoint()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Kinetic excitation", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addKineticExcitation()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Spring damper", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSpringDamper()));
  submenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Add from file", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFromFile()));
  contextMenu->addAction(action);

  contextMenu->addSeparator();

    action=new QAction(Utils::QIconCached("newobject.svg"),"Save as", this);
    connect(action,SIGNAL(triggered()),this,SLOT(saveAs()));
    contextMenu->addAction(action);

    actionSave=new QAction(Utils::QIconCached("newobject.svg"),"Save", this);
    actionSave->setDisabled(true);
    connect(actionSave,SIGNAL(triggered()),this,SLOT(save()));
    contextMenu->addAction(actionSave);

  if(parentItem != treeWidget()->invisibleRootItem()) {
    action=new QAction(Utils::QIconCached("newobject.svg"),"Copy", this);
    connect(action,SIGNAL(triggered()),this,SLOT(copy()));
    contextMenu->addAction(action);
  }

  actionPaste=new QAction(Utils::QIconCached("newobject.svg"),"Paste", this);
  actionPaste->setDisabled(true);
  connect(actionPaste,SIGNAL(triggered()),this,SLOT(paste()));
  contextMenu->addAction(actionPaste);

  contextMenu->addSeparator();

  if(parentItem != treeWidget()->invisibleRootItem()) {
  action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
  connect(action,SIGNAL(triggered()),this,SLOT(remove()));
  contextMenu->addAction(action);
  }

  properties->addStretch();
}

QString Group::getInfo() {
  return Element::getInfo()+
         QString("<hr width=\"10000\"/>")+
         QString("<b>Number of children:</b> %1").arg(childCount());
}

void Group::addRigidBody() {
  new RigidBody(newName(objects,"RigidBody"), objects, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addJointConstraint() {
  new JointConstraint(newName(objects,"JointConstraint"), objects, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addJoint() {
  new Joint(newName(links,"Joint"), links, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addKineticExcitation() {
  new KineticExcitation(newName(links,"KineticExcitation"), links, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addSpringDamper() {
  new SpringDamper(newName(links,"SpringDamper"), links, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addFrame() {
  new Frame(newName(frames,"P"), frames, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addGroup() {
  new Group(newName(groups,"Group"), groups, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addFromFile() {
  QString file=QFileDialog::getOpenFileName(0, "OpenMBV Files", ".",
      "hdf5 Files (*.xml)");
  if(file!="") {
    MBSimObjectFactory::initialize();
    TiXmlDocument doc;
    assert(doc.LoadFile(file.toAscii().data())==true);
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    TiXml_setLineNrFromProcessingInstruction(e);
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    bool renameObject = false, renameGroup = false;
    string name = e->Attribute("name");
    if(getObject(name,false))
      renameObject = true;
    if(getGroup(e->Attribute("name"),false))
      renameGroup = true;
    Object *o=ObjectFactory::getInstance()->createObject(e, objects, -1);
    if(o) {
      if(renameObject) {
        QString s = e->ValueStr().c_str();
        s.remove(MBSIMNS);
        s += QString::number(objects->childCount());
        QString text = name.c_str();
        do {
          QMessageBox msgBox;
          msgBox.setText(QString("The name ") + text + " does already exist in group.");
          msgBox.exec();
          text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

        } while(getObject(text.toStdString(),false));
        o->setName(text);
      }
      o->initializeUsingXML(e);
      ((Element*)treeWidget()->topLevelItem(0))->update();
      return;
    }
    Group *g=ObjectFactory::getInstance()->createGroup(e, groups, -1);
    if(g) {
      if(renameGroup) {
        QString s = e->ValueStr().c_str();
        s.remove(MBSIMNS);
        s += QString::number(groups->childCount());
        QString text = name.c_str();
        do {
          QMessageBox msgBox;
          msgBox.setText(QString("The name ") + text + " does already exist in group.");
          msgBox.exec();
          text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

        } while(getGroup(text.toStdString(),false));
        g->setName(text);
      }
      g->initializeUsingXML(e);
      ((Element*)treeWidget()->topLevelItem(0))->update();
      return;
    }
  }
}

void Group::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  Element::initializeUsingXML(element);
  e=element->FirstChildElement();

  if(frameOfReference)
    frameOfReference->initializeUsingXML(element);

  // search first element known by Group
  while(e && 
      e->ValueStr()!=MBSIMNS"position" &&
      e->ValueStr()!=MBSIMNS"orientation" &&
      e->ValueStr()!=MBSIMNS"frames")
    e=e->NextSiblingElement();

  if(position)
    position->initializeUsingXML(element);

  if(orientation)
    orientation->initializeUsingXML(element);

  // frames
  TiXmlElement *E=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
  while(E && E->ValueStr()==MBSIMNS"frame") {
    TiXmlElement *ec=E->FirstChildElement();
    Frame *f=new Frame(ec->Attribute("name"), frames, -1);
    f->initializeUsingXML(ec);
    E=E->NextSiblingElement();
  }

  framePos->initializeUsingXML(element->FirstChildElement(MBSIMNS"frames"));

  e=e->NextSiblingElement();

  // contours
  E=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
  while(E && E->ValueStr()==MBSIMNS"contour") {
    //     TiXmlElement *ec=E->FirstChildElement();
    //     Contour *c=ObjectFactory::getInstance()->createContour(ec);
    //     TiXmlElement *contourElement=ec; // save for later initialization
    //     ec=ec->NextSiblingElement();
    //     string refF="I";
    //     if(ec->ValueStr()==MBSIMNS"frameOfReference") {
    //       refF=ec->Attribute("ref");
    //       refF=refF.substr(6, refF.length()-7); // reference frame is allways "Frame[X]"
    //       ec=ec->NextSiblingElement();
    //     }
    //     Vec3 RrRC=getVec3(ec);
    //     ec=ec->NextSiblingElement();
    //     SqrMat3 ARC=getSqrMat3(ec);
    //     addContour(c, RrRC, ARC, refF);
    //     c->initializeUsingXML(contourElement);
    E=E->NextSiblingElement();
  }
  e=e->NextSiblingElement();

  // groups
  E=element->FirstChildElement(MBSIMNS"groups")->FirstChildElement();
  Group *g;
  while(E) {
    g=ObjectFactory::getInstance()->createGroup(E, groups, -1);
    g->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // objects
  E=element->FirstChildElement(MBSIMNS"objects")->FirstChildElement();
  Object *o;
  while(E) {
    o=ObjectFactory::getInstance()->createObject(E, objects, -1);
    o->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // extraDynamics
  E=element->FirstChildElement(MBSIMNS"extraDynamics")->FirstChildElement();
  //ExtraDynamic *ed;
  while(E) {
    //        ed=ObjectFactory::getInstance()->createExtraDynamic(E);
    //        addExtraDynamic(ed);
    //        ed->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // links
  E=element->FirstChildElement(MBSIMNS"links")->FirstChildElement();
  Link *l;
  while(E) {
    l=ObjectFactory::getInstance()->createLink(E, links, -1);
    l->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameI");
  if(e)
    getFrame(0)->initializeUsingXML2(e);

}

TiXmlElement* Group::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Element::writeXMLFile(parent);

  TiXmlElement *ele1;

  if(position) {
    frameOfReference->writeXMLFile(ele0);
    position->writeXMLFile(ele0);
    orientation->writeXMLFile(ele0);
  }

  ele1 = new TiXmlElement( MBSIMNS"frames" );
  framePos->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"contours" );
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"groups" );
  for(int i=0; i<groups->childCount(); i++)
    getGroup(i)->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"objects" );
  for(int i=0; i<objects->childCount(); i++)
    getObject(i)->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"extraDynamics" );
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"links" );
  for(int i=0; i<links->childCount(); i++)
    getLink(i)->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  Frame *I = getFrame(0);
  if(I->openMBVFrame()) {
    ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameI" );
    I->writeXMLFile2(ele1);
    ele0->LinkEndChild(ele1);
  }

  return ele0;
}

Element * Group::getByPathSearch(string path) {
  if (path.substr(0, 1)=="/") { // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.substr(1));
  }
  else if (path.substr(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.substr(3));
  else { // local path
    size_t pos0=path.find_first_of("[");
    string container=path.substr(0, pos0);
    size_t pos1=path.find_first_of("]", pos0);
    string searched_name=path.substr(pos0+1, pos1-pos0-1);
    if(path.length()>pos1+1) { // weiter absteigen
      string rest=path.substr(pos1+2);
      if (container=="Object")
        return getObject(searched_name)->getByPathSearch(rest);
      else if (container=="Link")
        return getLink(searched_name)->getByPathSearch(rest);
      //else if (container=="ExtraDynamic")
      //return getExtraDynamic(searched_name)->getByPathSearch(rest);
      else if (container=="Group")
        return getGroup(searched_name)->getByPathSearch(rest);
      else {
        cout << "Unknown name of container" << endl;
        throw;
      }
    }
    else {
      if (container=="Object")
        return getObject(searched_name);
      else if (container=="Link")
        return getLink(searched_name);
      //        else if (container=="ExtraDynamic")
      //          return getExtraDynamic(searched_name);
      else if (container=="Group")
        return getGroup(searched_name);
      else if (container=="Frame")
        return getFrame(searched_name);
      //        else if (container=="Contour")
      //          return getContour(searched_name);
      else {
        cout << "Unknown name of container" << endl;
        throw;
      }
    }
  }
}

void Group::paste() {
  MBSimObjectFactory::initialize();
  //TiXmlDocument doc;
  //assert(doc.LoadFile(file.toAscii().data())==true);
  //TiXml_PostLoadFile(&doc);
  TiXmlElement *e=copiedElement->FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(copiedElement->FirstChildElement(), dummy);
  bool renameObject = false, renameGroup = false;
  string name = e->Attribute("name");
  if(getObject(name,false))
    renameObject = true;
  if(getGroup(e->Attribute("name"),false))
    renameGroup = true;
  Object *o=ObjectFactory::getInstance()->createObject(e, objects, -1);
  if(o) {
    if(renameObject) {
      QString s = e->ValueStr().c_str();
      s.remove(MBSIMNS);
      s += QString::number(objects->childCount());
      QString text = name.c_str();
      do {
        QMessageBox msgBox;
        msgBox.setText(QString("The name ") + text + " does already exist in group.");
        msgBox.exec();
        text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

      } while(getObject(text.toStdString(),false));
      o->setName(text);
    }
    o->initializeUsingXML(e);
    ((Element*)treeWidget()->topLevelItem(0))->update();
    return;
  }
  Group *g=ObjectFactory::getInstance()->createGroup(e, groups, -1);
  if(g) {
    if(renameGroup) {
      QString s = e->ValueStr().c_str();
      s.remove(MBSIMNS);
      s += QString::number(groups->childCount());
      QString text = name.c_str();
      do {
        QMessageBox msgBox;
        msgBox.setText(QString("The name ") + text + " does already exist in group.");
        msgBox.exec();
        text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

      } while(getGroup(text.toStdString(),false));
      g->setName(text);
    }
    g->initializeUsingXML(e);
    ((Element*)treeWidget()->topLevelItem(0))->update();
    return;
  }
}

void Group::setActionPasteDisabled(bool flag) {
  actionPaste->setDisabled(flag);
  for(int i=0; i<groups->childCount(); i++)
    getGroup(i)->setActionPasteDisabled(flag);
}
