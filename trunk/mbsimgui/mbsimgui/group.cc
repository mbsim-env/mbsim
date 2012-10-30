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

  if(parentItem != treeWidget()->invisibleRootItem()) {
    properties->addTab("Kinematics");
    position=new Vec3Editor(properties, Utils::QIconCached("lines.svg"), "Position");
    orientation=new Vec3Editor(properties, Utils::QIconCached("lines.svg"), "Orientation");
    frameOfReference=new FrameOfReferenceEditor(this,properties, Utils::QIconCached("lines.svg"), "Frame of reference", "Kinematics", ((Group*)getParentElement())->getFrame(0));
  }

  framePos = new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Frame Pos", "General", new FramePositionsWidget(this));

  properties->addTab("Parameterfile");
  parameterFile = new FileEditor(properties, Utils::QIconCached("lines.svg"), "Parameterfile", "Parameterfile");

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

//void Group::remove() {
//  Group* parent = static_cast<Group*>(static_cast<QTreeWidgetItem*>(this)->parent());
//
//  if(parent) {
//    parent->removeChild(this);
//  }
//  else
//    treeWidget()->takeTopLevelItem(0);
//}


  void Group::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    Element::initializeUsingXML(element);
    e=element->FirstChildElement();

    // search first element known by Group
    while(e && e->ValueStr()!=MBSIMNS"frameOfReference" &&
        e->ValueStr()!=MBSIMNS"position" &&
        e->ValueStr()!=MBSIMNS"orientation" &&
        e->ValueStr()!=MBSIMNS"frames")
      e=e->NextSiblingElement();

    if(e && e->ValueStr()==MBSIMNS"frameOfReference") {
      QString ref=e->Attribute("ref");
      if(frameOfReference)
        frameOfReference->setFrame(getByPath<Frame>(ref)); // must be a Frame of the parent, so it allready exists (no need to resolve path later)
      e=e->NextSiblingElement();
    }

    if(e && e->ValueStr()==MBSIMNS"position") {
      if(position)
        position->setVec(getVec(e));
      e=e->NextSiblingElement();
    }

    if(e && e->ValueStr()==MBSIMNS"orientation") {
      vector<vector<double> > AIK = getSqrMat(e);
      if(orientation)
        orientation->setVec(AIK2Cardan(AIK));
      e=e->NextSiblingElement();
    }

    // frames
    TiXmlElement *E=e->FirstChildElement();
//    e=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
    while(E && E->ValueStr()==MBSIMNS"frame") {
      TiXmlElement *ec=E->FirstChildElement();
      Frame *f=new Frame(ec->Attribute("name"), frames, -1);
      f->initializeUsingXML(ec);
      E=E->NextSiblingElement();
    }

    framePos->initializeUsingXML(element->FirstChildElement(MBSIMNS"frames"));

//    TiXmlElement *E=e->FirstChildElement();
//    int i=0;
//    vector<QString> refFrame;
//    while(E && E->ValueStr()==MBSIMNS"frame") {
//      TiXmlElement *ec=E->FirstChildElement();
//      Frame *f = new Frame(ec->Attribute("name"), frames, -1);
//      f->initializeUsingXML(ec);
//      ec=ec->NextSiblingElement();
//      string refF="I";
//      if(ec->ValueStr()==MBSIMNS"frameOfReference") {
//        refF=ec->Attribute("ref");
//        refF=refF.substr(6, refF.length()-7); // reference frame is allways "Frame[X]"
//        ec=ec->NextSiblingElement();
//      }
//      vector<vector<double> > RrRF=getVec(ec);
//      ec=ec->NextSiblingElement();
//      vector<vector<double> > ARF=getSqrMat(ec);
//      f->getFramePosition()->setPosition(RrRF);
//      f->getFramePosition()->setOrientation(AIK2Cardan(ARF));
//      refFrame.push_back(refF.c_str());
//      E=E->NextSiblingElement();
//      i++;
//    }
////    framePositions->update();
//    for(unsigned int i=0; i<refFrame.size(); i++) 
//      getFrame(i+1)->getFramePosition()->setFrame(getFrame(refFrame[i].toStdString()));
    e=e->NextSiblingElement();

    // contours
    E=e->FirstChildElement();
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
    E=e->FirstChildElement();
    Group *g;
    while(E) {
      g=ObjectFactory::getInstance()->createGroup(E, groups, -1);
      g->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
    e=e->NextSiblingElement();

    // objects
    E=e->FirstChildElement();
    Object *o;
    while(E) {
      o=ObjectFactory::getInstance()->createObject(E, objects, -1);
      o->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }
    e=e->NextSiblingElement();

    // extraDynamics
    if (e->ValueStr()==MBSIMNS"extraDynamics") {
      E=e->FirstChildElement();
      //ExtraDynamic *ed;
      while(E) {
//        ed=ObjectFactory::getInstance()->createExtraDynamic(E);
//        addExtraDynamic(ed);
//        ed->initializeUsingXML(E);
        E=E->NextSiblingElement();
      }
      e=e->NextSiblingElement();
    }

    // links
    E=e->FirstChildElement();
    Link *l;
    while(E) {
      l=ObjectFactory::getInstance()->createLink(E, links, -1);
      l->initializeUsingXML(E);
      E=E->NextSiblingElement();
    }

    e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameI");
    if(e) {
      Frame *I = getFrame(0);
      I->setOpenMBVFrame(true);
      I->setSize(getDouble(e->FirstChildElement(MBSIMNS"size")));
      I->setOffset(getDouble(e->FirstChildElement(MBSIMNS"offset")));
    }

  }

  TiXmlElement* Group::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Element::writeXMLFile(parent);

    TiXmlElement *ele1;

    if(position) {
      ele1 = new TiXmlElement( MBSIMNS"frameOfReference" );
      ele1->SetAttribute("ref", frameOfReference->getFrame()->getXMLPath(this,true).toStdString());
      ele0->LinkEndChild(ele1);

      vector<vector<double> > AIK = orientation->getVec();
      addElementText(ele0,MBSIMNS"position",position->getVec());
      addElementText(ele0,MBSIMNS"orientation",Cardan2AIK(AIK));
    }

    ele1 = new TiXmlElement( MBSIMNS"frames" );
    framePos->writeXMLFile(ele1);
    ele0->LinkEndChild( ele1 );

//    ele1 = new TiXmlElement( MBSIMNS"frames" );
//    for(int i=1; i<frames->childCount(); i++) {
//      TiXmlElement* ele2 = new TiXmlElement( MBSIMNS"frame" );
//      ele1->LinkEndChild( ele2 );
//      getFrame(i)->writeXMLFile(ele2);
//      if(getFrame(i)->getFramePosition()->getFrame() != getFrame(0)) {
//        TiXmlElement *ele3 = new TiXmlElement( MBSIMNS"frameOfReference" );
//        QString str = QString("Frame[") + getFrame(i)->getFramePosition()->getFrame()->getName() + "]";
//        ele3->SetAttribute("ref", str.toStdString());
//        ele2->LinkEndChild(ele3);
//      }
//
//      vector<vector<double> > AIK = getFrame(i)->getFramePosition()->getOrientation();
//      addElementText(ele2,MBSIMNS"position",getFrame(i)->getFramePosition()->getPosition());
//      addElementText(ele2,MBSIMNS"orientation",Cardan2AIK(AIK));
//    }
//    ele0->LinkEndChild( ele1 );

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
      addElementText(ele1,MBSIMNS"size",I->getSize());
      addElementText(ele1,MBSIMNS"offset",I->getOffset());
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

//void Group::update() {
//  Element::update();
//  for(int i=0; i<groups->childCount(); i++)
//    getGroup(i)->update();
//  for(int i=0; i<frames->childCount(); i++)
//    getFrame(i)->update();
//  ///for(int i=0; i<contours->childCount(); i++)
//    ///getContour(i)->update();
//  for(int i=0; i<objects->childCount(); i++)
//    getObject(i)->update();
//  for(int i=0; i<links->childCount(); i++)
//    getLink(i)->update();
////  for(int i=0; i<extraDynamics->childCount(); i++)
////    getExtraDynamic(i)->update();
//}

//Frame* Group::getFrame(int i) {
//  return (Frame*)frames->child(i); 
//}
//
//Object* Group::getObject(int i) {
//  return (Object*)objects->child(i); 
//}
//
//Link* Group::getLink(int i) {
//  return (Link*)links->child(i); 
//}
//
//Group* Group::getGroup(int i) {
//  return (Group*)groups->child(i); 
//}
//
