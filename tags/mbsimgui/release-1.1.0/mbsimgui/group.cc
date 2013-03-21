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
#include <QtGui/QInputDialog>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QVBoxLayout>
#include "objectfactory.h"
#include "rigidbody.h"
#include "constraint.h"
#include "spring_damper.h"
#include "joint.h"
#include "kinetic_excitation.h"
#include "contact.h"
#include "signal_.h"
#include "frame.h"
#include "contour.h"
#include "property_widget.h"
#include "basic_widgets.h"
#include "string_widgets.h"
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
  observers = new Container;
  observers->setText(0, "observers");
  observers->setForeground(0,brush);
  observers->setBackground(0,brush2);
  addChild(observers);

  //if(parentItem != treeWidget()->invisibleRootItem())
    //new Frame("I", frames, -1, false);
  //else
    new Frame("I", frames, -1, true);

  QAction *action;

  //properties->addTab("Frame positioning");
  //properties->addTab("Contour positioning");
  if(parentItem != treeWidget()->invisibleRootItem()) {
    properties->addTab("Kinematics");

    vector<PhysicalStringWidget*> input;
    input.push_back(new PhysicalStringWidget(new VecWidget(3),MBSIMNS"position",lengthUnits(),4));
    position = new ExtXMLWidget("Position",new ExtPhysicalVarWidget(input),true); 
    properties->addToTab("Kinematics", position);

    input.clear();
    input.push_back(new PhysicalStringWidget(new MatWidget(getEye<string>(3,3,"1","0")),MBSIMNS"orientation",noUnitUnits(),1));
    orientation = new ExtXMLWidget("Orientation",new ExtPhysicalVarWidget(input),true); 
    properties->addToTab("Kinematics", orientation);

    frameOfReference = new ExtXMLWidget("Frame of reference",new ParentFrameOfReferenceWidget(MBSIMNS"frameOfReference",((Group*)getParentElement())->getFrame(0),0),true);
    properties->addToTab("Kinematics", frameOfReference);
  }

  action=new QAction(Utils::QIconCached("newobject.svg"),"Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFrame()));
  contextMenu->addAction(action);

  QMenu *submenu = contextMenu->addMenu("Add contour");
  action=new QAction(Utils::QIconCached("newobject.svg"),"Point", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addPoint()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Line", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addLine()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Plane", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addPlane()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Sphere", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSphere()));
  submenu->addAction(action);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Add group", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addGroup()));
  contextMenu->addAction(action);

  //action=new QAction(Utils::QIconCached("newobject.svg"),"Add object", this);
  //connect(action,SIGNAL(triggered()),this,SLOT(addObject()));
  //contextMenu->addAction(action);
  submenu = contextMenu->addMenu("Add object");
//  action=new QAction(Utils::QIconCached("newobject.svg"),"Rigid bodies", this);
//  connect(action,SIGNAL(triggered()),this,SLOT(addRigidBodies()));
//  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Rigid body", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addRigidBody()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Kinematic constraint", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addKinematicConstraint()));
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
  action=new QAction(Utils::QIconCached("newobject.svg"),"Contact", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addContact()));
  submenu->addAction(action);
  QMenu *subsubmenu = submenu->addMenu("Sensor");
  action=new QAction(Utils::QIconCached("newobject.svg"),"AbsolutePositionSensor", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addAbsolutePositionSensor()));
  subsubmenu->addAction(action);

  submenu = contextMenu->addMenu("Add observer");
  action=new QAction(Utils::QIconCached("newobject.svg"),"AbsoluteKinematicsObserver", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addAbsoluteKinematicsObserver()));
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

int Group::getqSize() {
  int qSize = 0;
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      qSize += getGroup(i)->getqSize();
  }
  if(getContainerObject()) {
    for(int i=0; i<getContainerObject()->childCount(); i++)
      qSize += getObject(i)->getqSize();
  }
  return qSize;
}

int Group::getuSize() {
  int uSize = 0;
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      uSize += getGroup(i)->getuSize();
  }
  if(getContainerObject()) {
    for(int i=0; i<getContainerObject()->childCount(); i++)
      uSize += getObject(i)->getuSize();
  }
  return uSize;
}

int Group::getxSize() {
  int xSize = 0;
  if(getContainerGroup()) {
    for(int i=0; i<getContainerGroup()->childCount(); i++)
      xSize += getGroup(i)->getxSize();
  }
  if(getContainerLink()) {
    for(int i=0; i<getContainerLink()->childCount(); i++)
      xSize += getLink(i)->getxSize();
  }
  return xSize;
}

void Group::addRigidBody() {
  new RigidBody(newName(objects,"RigidBody"), objects, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addRigidBodies() {
  for(int i=0; i<3000; i++) {
    cout <<"add Body" << i+1<< endl;
    //new RigidBody(newName(objects,"Body"), objects, -1);
    //new FixedRelativeFrame(newName(frames,"P"), frames, -1);
    //new Frame("P", 0, -1);
    //QTreeWidgetItem *item = new Element2();
    //QTreeWidgetItem *item = new QTreeWidgetItem;
    //addChild(item); //
    //item->setText(0,newName(this,"P"));
    //item->setText(0,"Name");
    cout <<"end" << endl;
  }
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addKinematicConstraint() {
  new KinematicConstraint(newName(objects,"KinematicConstraint"), objects, -1);
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

void Group::addContact() {
  new Contact(newName(links,"Contact"), links, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addAbsolutePositionSensor() {
  new AbsolutePositionSensor(newName(links,"AbsolutePositionSensor"), links, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addAbsoluteKinematicsObserver() {
  new AbsoluteKinematicsObserver(newName(observers,"AbsoluteKinematicsObserver"), observers, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addFrame() {
  new FixedRelativeFrame(newName(frames,"P"), frames, -1);
  ((Element*)treeWidget()->topLevelItem(0))->update();
}

void Group::addPoint() {
  QString text = newName(contours,"Point");
  if (!text.isEmpty()) {
    new Point(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void Group::addLine() {
  QString text = newName(contours,"Line");
  if (!text.isEmpty()) {
    new Line(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void Group::addPlane() {
  QString text = newName(contours,"Plane");
  if (!text.isEmpty()) {
    new Plane(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void Group::addSphere() {
  QString text = newName(contours,"Sphere");
  if (!text.isEmpty()) {
    new Sphere(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
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
    bool ret=doc.LoadFile(file.toAscii().data());
    assert(ret==true);
    TiXml_PostLoadFile(&doc);
    TiXmlElement *e=doc.FirstChildElement();
    TiXml_setLineNrFromProcessingInstruction(e);
    map<string,string> dummy;
    incorporateNamespace(doc.FirstChildElement(), dummy);
    bool renameObject = false, renameGroup = false;
    QString name = e->Attribute("name");
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
        QString text = name;
        do {
          QMessageBox msgBox;
          msgBox.setText(QString("The name ") + text + " does already exist in group.");
          msgBox.exec();
          text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

        } while(getObject(text,false));
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
        QString text = name;
        do {
          QMessageBox msgBox;
          msgBox.setText(QString("The name ") + text + " does already exist in group.");
          msgBox.exec();
          text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

        } while(getGroup(text,false));
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
    FixedRelativeFrame *f=new FixedRelativeFrame(ec->Attribute("name"), frames, -1);
    f->initializeUsingXML(ec);
    f->initializeUsingXML2(E);
    E=E->NextSiblingElement();
  }
  while(E && E->ValueStr()==MBSIMNS"FixedRelativeFrame") {
    FixedRelativeFrame *f=new FixedRelativeFrame(E->Attribute("name"), frames, -1);
    f->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // contours
  E=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
  Contour *c;
  while(E && E->ValueStr()==MBSIMNS"contour") {
    TiXmlElement *ec=E->FirstChildElement();
    c=ObjectFactory::getInstance()->createContour(ec, contours, -1);
    if(c) c->initializeUsingXML(ec);
    FixedRelativeFrame *f=new FixedRelativeFrame(QString("ContourFrame")+QString::number(contours->childCount()), frames, -1);
    f->initializeUsingXML(ec);
    f->initializeUsingXML2(E);
    c->setSavedFrameOfReference(QString("../Frame[")+f->getName()+"]");
    E=E->NextSiblingElement();
  }
  while(E) {
    c=ObjectFactory::getInstance()->createContour(E, contours, -1);
    c->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // groups
  E=element->FirstChildElement(MBSIMNS"groups")->FirstChildElement();
  Group *g;
  while(E) {
    g=ObjectFactory::getInstance()->createGroup(E, groups, -1);
    if(g) g->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // objects
  E=element->FirstChildElement(MBSIMNS"objects")->FirstChildElement();
  Object *o;
  while(E) {
    o=ObjectFactory::getInstance()->createObject(E, objects, -1);
    if(o) o->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // extraDynamics
  if(element->FirstChildElement(MBSIMNS"extraDynamics")) {
    E=element->FirstChildElement(MBSIMNS"extraDynamics")->FirstChildElement();
    //ExtraDynamic *ed;
    while(E) {
      //        ed=ObjectFactory::getInstance()->createExtraDynamic(E);
      //        addExtraDynamic(ed);
      //        ed->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
  }

  // links
  E=element->FirstChildElement(MBSIMNS"links")->FirstChildElement();
  Link *l;
  while(E) {
    l=ObjectFactory::getInstance()->createLink(E, links, -1);
    if(l) l->initializeUsingXML(E);
    E=E->NextSiblingElement();
  }

  // observers
  if(element->FirstChildElement(MBSIMNS"observers")) {
    E=element->FirstChildElement(MBSIMNS"observers")->FirstChildElement();
    Observer *obsrv;
    while(E) {
      obsrv=ObjectFactory::getInstance()->createObserver(E, observers, -1);
      if(obsrv) obsrv->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
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
  for(int i=1; i<frames->childCount(); i++)
    getFrame(i)->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"contours" );
  for(int i=0; i<contours->childCount(); i++)
    getContour(i)->writeXMLFile(ele1);
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

  ele1 = new TiXmlElement( MBSIMNS"observers" );
  for(int i=0; i<observers->childCount(); i++)
    getObserver(i)->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  Frame *I = getFrame(0);
  if(I->openMBVFrame()) {
    ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameI" );
    I->writeXMLFile2(ele1);
    ele0->LinkEndChild(ele1);
  }

  return ele0;
}

Element * Group::getByPathSearch(QString path) {
  if (path.mid(0, 1)=="/") { // absolut path
    if(getParentElement())
      return getParentElement()->getByPathSearch(path);
    else
      return getByPathSearch(path.mid(1));
  }
  else if (path.mid(0, 3)=="../") // relative path
    return getParentElement()->getByPathSearch(path.mid(3));
  else { // local path
    size_t pos0=path.indexOf("[");
    QString container=path.mid(0, pos0);
    size_t pos1=path.indexOf("]", pos0);
    QString searched_name=path.mid(pos0+1, pos1-pos0-1);
    if(path.length()>pos1+1) { // weiter absteigen
      QString rest=path.mid(pos1+2);
      if (container=="Object")
        return getObject(searched_name)->getByPathSearch(rest);
      else if (container=="Link")
        return getLink(searched_name)->getByPathSearch(rest);
      //else if (container=="ExtraDynamic")
      //return getExtraDynamic(searched_name)->getByPathSearch(rest);
      else if (container=="Observer")
        return getObserver(searched_name)->getByPathSearch(rest);
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
      else if (container=="Observer")
        return getObserver(searched_name);
      else if (container=="Group")
        return getGroup(searched_name);
      else if (container=="Frame")
        return getFrame(searched_name);
      else if (container=="Contour")
        return getContour(searched_name);
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
  //bool ret=doc.LoadFile(file.toAscii().data());
  //assert(ret==true);
  //TiXml_PostLoadFile(&doc);
  TiXmlElement *e=copiedElement->FirstChildElement();
  TiXml_setLineNrFromProcessingInstruction(e);
  map<string,string> dummy;
  incorporateNamespace(copiedElement->FirstChildElement(), dummy);
  bool renameObject = false, renameGroup = false;
  QString name = e->Attribute("name");
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
      QString text = name;
      do {
        QMessageBox msgBox;
        msgBox.setText(QString("The name ") + text + " does already exist in group.");
        msgBox.exec();
        text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

      } while(getObject(text,false));
      o->setName(text);
    }
    o->initializeUsingXML(e);
    o->initialize();
    ((Element*)treeWidget()->topLevelItem(0))->update();
    return;
  }
  Group *g=ObjectFactory::getInstance()->createGroup(e, groups, -1);
  if(g) {
    if(renameGroup) {
      QString s = e->ValueStr().c_str();
      s.remove(MBSIMNS);
      s += QString::number(groups->childCount());
      QString text = name;
      do {
        QMessageBox msgBox;
        msgBox.setText(QString("The name ") + text + " does already exist in group.");
        msgBox.exec();
        text = QInputDialog::getText(0, tr("Rename"), tr("Name:"), QLineEdit::Normal, s);

      } while(getGroup(text,false));
      g->setName(text);
    }
    g->initializeUsingXML(e);
    g->initialize();
    ((Element*)treeWidget()->topLevelItem(0))->update();
    return;
  }
}

void Group::setActionPasteDisabled(bool flag) {
  actionPaste->setDisabled(flag);
  for(int i=0; i<groups->childCount(); i++)
    getGroup(i)->setActionPasteDisabled(flag);
}
