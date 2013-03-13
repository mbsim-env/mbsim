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
#include "rigidbody.h"
#include "objectfactory.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "basic_properties.h"
#include "kinematics_properties.h"
#include "ombv_properties.h"
#include "string_widgets.h"
#include "kinematics_widgets.h"
#include "ombv_widgets.h"
#include "mainwindow.h"
#include <QMenu>

using namespace std;

extern MainWindow *mw;

RigidBody::RigidBody(const QString &str, QTreeWidgetItem *parentItem, int ind) : Body(str, parentItem, ind), constrained(false), RWidget(0), KWidget(0), massWidget(0), inertiaWidget(0), translationWidget(0), rotationWidget(0), ombvEditorWidget(0), weightArrowWidget(0), jointForceArrowWidget(0), jointMomentArrowWidget(0), isFrameOfBodyForRotationWidget(0), R(0,false), K(0,false), translation(0,false), rotation(0,false), ombvEditor(0,true), weightArrow(0,false), jointForceArrow(0,false), jointMomentArrow(0,false), isFrameOfBodyForRotation(0,false) {

  setText(1,getType());

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

  new Frame("C", frames, -1, true);

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFrame()));
  contextMenu->insertAction(actionSaveAs,action);

  QMenu *submenu = new QMenu("Add contour");
  contextMenu->insertMenu(actionSaveAs,submenu);
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

  contextMenu->insertSeparator(actionSaveAs);

  R.setProperty(new FrameOfReferenceProperty(((Group*)getParentElement())->getFrame(0),this,MBSIMNS"frameOfReference"));

  K.setProperty(new LocalFrameOfReferenceProperty(getFrame(0),this,MBSIMNS"frameForKinematics"));

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("1"),"kg",MBSIMNS"mass"));
  mass.setProperty(new ExtPhysicalVarProperty(input));

  input.clear();
  input.push_back(new PhysicalStringProperty(new MatProperty(getEye<string>(3,3,"0.01","0")),"kg*m^2",MBSIMNS"inertiaTensor"));
  inertia.setProperty(new ExtPhysicalVarProperty(input));

  translation.setProperty(new TranslationChoiceProperty(new LinearTranslationProperty,""));
  translation.setXMLName(MBSIMNS"translation");

  rotation.setProperty(new RotationChoiceProperty(new RotationAboutZAxisProperty,""));
  rotation.setXMLName(MBSIMNS"rotation");

  ombvEditor.setProperty(new OMBVBodySelectionProperty(this));

  weightArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  weightArrow.setXMLName(MBSIMNS"openMBVWeightArrow",false);
  ((OMBVArrowProperty*)weightArrow.getProperty())->setID(getID());

  jointForceArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  jointForceArrow.setXMLName(MBSIMNS"openMBVJointForceArrow",false);
  ((OMBVArrowProperty*)jointForceArrow.getProperty())->setID(getID());

  jointMomentArrow.setProperty(new OMBVArrowProperty("NOTSET"));
  jointMomentArrow.setXMLName(MBSIMNS"openMBVJointMomentArrow",false);
  ((OMBVArrowProperty*)jointMomentArrow.getProperty())->setID(getID());

  input.clear();
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"",MBSIMNS"isFrameOfBodyForRotation"));
  isFrameOfBodyForRotation.setProperty(new ExtPhysicalVarProperty(input)); 
}

RigidBody::~RigidBody() {
}

void RigidBody::initialize() {
  Body::initialize();
  R.initialize();
}

void RigidBody::initializeDialog() {
  Body::initializeDialog();

  dialog->addTab("Kinematics");
  dialog->addTab("Visualisation");
  dialog->addTab("Extra");

  RWidget = new ExtWidget("Frame of reference",new FrameOfReferenceWidget(this,0),true);
  dialog->addToTab("Kinematics",RWidget);

  KWidget = new ExtWidget("Frame for kinematics",new LocalFrameOfReferenceWidget(this,0),true);
  dialog->addToTab("Kinematics",KWidget);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),massUnits(),2));
  massWidget = new ExtWidget("Mass",new ExtPhysicalVarWidget(input));
  dialog->addToTab("General", massWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SymMatWidget(getEye<string>(3,3,"0.01","0")),inertiaUnits(),2));
  inertiaWidget = new ExtWidget("Inertia tensor",new ExtPhysicalVarWidget(input));
  dialog->addToTab("General", inertiaWidget);

  TranslationChoiceWidget *translationWidget_ = new TranslationChoiceWidget("");
  translationWidget = new ExtWidget("Translation",translationWidget_,true);
  dialog->addToTab("Kinematics", translationWidget);
  connect(translationWidget_,SIGNAL(translationChanged()),this,SLOT(resizeVariables()));
  connect(translationWidget,SIGNAL(resize()),this,SLOT(resizeVariables()));

  RotationChoiceWidget *rotationWidget_ = new RotationChoiceWidget("");
  rotationWidget = new ExtWidget("Rotation",rotationWidget_,true);
  dialog->addToTab("Kinematics", rotationWidget);
  connect(rotationWidget_,SIGNAL(rotationChanged()),this,SLOT(resizeVariables()));
  connect(rotationWidget,SIGNAL(resize()),this,SLOT(resizeVariables()));

  ombvEditorWidget = new ExtWidget("OpenMBV body",new OMBVBodySelectionWidget(this),true);
  dialog->addToTab("Visualisation", ombvEditorWidget);

  weightArrowWidget = new ExtWidget("OpenMBV weight arrow",new OMBVArrowWidget("NOTSET"),true);
  ((OMBVArrowWidget*)weightArrowWidget->getWidget())->setID(getID());
  dialog->addToTab("Visualisation",weightArrowWidget);

  jointForceArrowWidget = new ExtWidget("OpenMBV joint force arrow",new OMBVArrowWidget("NOTSET"),true);
  ((OMBVArrowWidget*)jointForceArrowWidget->getWidget())->setID(getID());
  dialog->addToTab("Visualisation",jointForceArrowWidget);

  jointMomentArrowWidget = new ExtWidget("OpenMBV joint moment arrow",new OMBVArrowWidget("NOTSET"),true);
  ((OMBVArrowWidget*)jointMomentArrowWidget->getWidget())->setID(getID());
  dialog->addToTab("Visualisation",jointMomentArrowWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"),QStringList(),1));
  isFrameOfBodyForRotationWidget = new ExtWidget("Use body frame for rotation",new ExtPhysicalVarWidget(input),true); 
  dialog->addToTab("Extra", isFrameOfBodyForRotationWidget);

}

void RigidBody::toWidget() {
  Body::toWidget();
  R.toWidget(RWidget);
  K.toWidget(KWidget);
  mass.toWidget(massWidget);
  inertia.toWidget(inertiaWidget);
  translation.toWidget(translationWidget);
  rotation.toWidget(rotationWidget);
  ombvEditor.toWidget(ombvEditorWidget);
  weightArrow.toWidget(weightArrowWidget);
  jointForceArrow.toWidget(jointForceArrowWidget);
  jointMomentArrow.toWidget(jointMomentArrowWidget);
  isFrameOfBodyForRotation.toWidget(isFrameOfBodyForRotationWidget);
}

void RigidBody::fromWidget() {
  Body::fromWidget();
  R.fromWidget(RWidget);
  K.fromWidget(KWidget);
  mass.fromWidget(massWidget);
  inertia.fromWidget(inertiaWidget);
  translation.fromWidget(translationWidget);
  rotation.fromWidget(rotationWidget);
  weightArrow.fromWidget(weightArrowWidget);
  jointForceArrow.fromWidget(jointForceArrowWidget);
  jointMomentArrow.fromWidget(jointMomentArrowWidget);
  ombvEditor.fromWidget(ombvEditorWidget);
  isFrameOfBodyForRotation.fromWidget(isFrameOfBodyForRotationWidget);
}

int RigidBody::getUnconstrainedSize() const {
  if(translationWidget)
    return (translationWidget->isActive()?((TranslationChoiceWidget*)translationWidget->getWidget())->getSize():0) + (rotationWidget->isActive()?((RotationChoiceWidget*)rotationWidget->getWidget())->getSize():0);
  return 0;
}

void RigidBody::addFrame() {
  QString text = newName(frames,"P");
  if (!text.isEmpty()) {
    new FixedRelativeFrame(text, frames, -1);
    ((Element*)treeWidget()->topLevelItem(0))->updateWidget();
  }
  mw->mbsimxml(1);
}

void RigidBody::addPoint() {
  QString text = newName(contours,"Point");
  if (!text.isEmpty()) {
    new Point(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->updateWidget();
  }
}

void RigidBody::addLine() {
  QString text = newName(contours,"Line");
  if (!text.isEmpty()) {
    new Line(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->updateWidget();
  }
}

void RigidBody::addPlane() {
  QString text = newName(contours,"Plane");
  if (!text.isEmpty()) {
    new Plane(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->updateWidget();
  }
}

void RigidBody::addSphere() {
  QString text = newName(contours,"Sphere");
  if (!text.isEmpty()) {
    new Sphere(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->updateWidget();
  }
}

void RigidBody::resizeGeneralizedPosition() {
  int size = getSize();
  if(q0 && q0->size() != size)
    q0->resize(size);
}

void RigidBody::resizeGeneralizedVelocity() {
  int size = getSize();
  if(u0 && u0->size() != size)
    u0->resize(size);
}

void RigidBody::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;

  // frames
  e=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
  while(e && e->ValueStr()==MBSIMNS"frame") {
    TiXmlElement *ec=e->FirstChildElement();
    FixedRelativeFrame *f=new FixedRelativeFrame(ec->Attribute("name"), frames, -1);
    f->initializeUsingXML(ec);
    f->initializeUsingXML2(e);
    e=e->NextSiblingElement();
  }
  while(e && e->ValueStr()==MBSIMNS"FixedRelativeFrame") {
    FixedRelativeFrame *f=new FixedRelativeFrame(e->Attribute("name"), frames, -1);
    f->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }

  // contours
  e=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
  Contour *c;
  while(e && e->ValueStr()==MBSIMNS"contour") {
    TiXmlElement *ec=e->FirstChildElement();
    c=ObjectFactory::getInstance()->createContour(ec, contours, -1);
    if(c) c->initializeUsingXML(ec);
    FixedRelativeFrame *f=new FixedRelativeFrame(QString("ContourFrame")+QString::number(contours->childCount()), frames, -1);
    f->initializeUsingXML(ec);
    f->initializeUsingXML2(e);
    c->setSavedFrameOfReference(QString("../Frame[")+f->getName()+"]");
    e=e->NextSiblingElement();
  }
  while(e) {
    c=ObjectFactory::getInstance()->createContour(e, contours, -1);
    c->initializeUsingXML(e);
    e=e->NextSiblingElement();
  }

  R.initializeUsingXML(element);
  K.initializeUsingXML(element);

  mass.initializeUsingXML(element);
  inertia.initializeUsingXML(element);

  translation.initializeUsingXML(element);
  rotation.initializeUsingXML(element);

  isFrameOfBodyForRotation.initializeUsingXML(element);

  ombvEditor.initializeUsingXML(element);

  e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameC");
  if(e)
    getFrame(0)->initializeUsingXML2(e);
  else
    getFrame(0)->setOpenMBVFrame(false);

  weightArrow.initializeUsingXML(element);

  jointForceArrow.initializeUsingXML(element);
  jointMomentArrow.initializeUsingXML(element);

  Body::initializeUsingXML(element);
}

TiXmlElement* RigidBody::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Body::writeXMLFile(parent);
  TiXmlElement *ele1;

  R.writeXMLFile(ele0);
  K.writeXMLFile(ele0);

  mass.writeXMLFile(ele0);
  inertia.writeXMLFile(ele0);

  translation.writeXMLFile(ele0);
  rotation.writeXMLFile(ele0);

  ele1 = new TiXmlElement( MBSIMNS"frames" );
  for(int i=1; i<frames->childCount(); i++)
    getFrame(i)->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"contours" );
  for(int i=0; i<contours->childCount(); i++)
    getContour(i)->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  isFrameOfBodyForRotation.writeXMLFile(ele0);

  ombvEditor.writeXMLFile(ele0);

  Frame *C = getFrame(0);
  if(C->openMBVFrame()) {
    ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameC" );
    C->writeXMLFile2(ele1);
    ele0->LinkEndChild(ele1);
  }

  weightArrow.writeXMLFile(ele0);

  jointForceArrow.writeXMLFile(ele0);
  jointMomentArrow.writeXMLFile(ele0);

  return ele0;
}
