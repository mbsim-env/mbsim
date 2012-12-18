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
#include "string_widgets.h"
#include "kinematics_widgets.h"
#include "ombv_widgets.h"
#include <QMenu>

using namespace std;

RigidBody::RigidBody(const QString &str, QTreeWidgetItem *parentItem, int ind) : Body(str, parentItem, ind), constrained(false) {
  setText(1,getType());
  properties->addTab("Kinematics");
  properties->addTab("Frame positioning");
  properties->addTab("Contour positioning");
  properties->addTab("Visualisation");

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

  frameForKinematics = new ExtXMLWidget("Frame for kinematics",new LocalFrameOfReferenceWidget(MBSIMNS"frameForKinematics",this,0));
  properties->addToTab("Kinematics", frameForKinematics);

  frameOfReference = new ExtXMLWidget("Frame of reference",new FrameOfReferenceWidget(MBSIMNS"frameOfReference",this,((Group*)getParentElement())->getFrame(0)));
  properties->addToTab("Kinematics", frameOfReference);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),MBSIMNS"mass",massUnits(),2));
  mass = new ExtXMLWidget("Mass",new ExtPhysicalVarWidget(input));
  properties->addToTab("General", mass);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SymMatWidget(getEye<string>(3,3,"0.01","0")),MBSIMNS"inertiaTensor",inertiaUnits(),2));
  inertia = new ExtXMLWidget("Inertia tensor",new ExtPhysicalVarWidget(input));
  properties->addToTab("General", inertia);

  framePos = new ExtXMLWidget("Position and orientation of frames",new FramePositionsWidget(this));
  properties->addToTab("Frame positioning", framePos);

  contourPos = new ExtXMLWidget("Position and orientation of contours",new ContourPositionsWidget(this));
  properties->addToTab("Contour positioning", contourPos);
  
  TranslationChoiceWidget *translation_ = new TranslationChoiceWidget("");
  translation = new ExtXMLWidget("Translation",translation_,true);
  translation->setXMLName(MBSIMNS"translation");
  properties->addToTab("Kinematics", translation);
  connect(translation_,SIGNAL(translationChanged()),this,SLOT(resizeVariables()));
  //connect(translation_,SIGNAL(translationChanged()),this,SLOT(resizeGeneralizedVelocity()));
  //connect(translation,SIGNAL(resize()),this,SLOT(resizeGeneralizedPosition()));
  connect(translation,SIGNAL(resize()),this,SLOT(resizeVariables()));

  RotationChoiceWidget *rotation_ = new RotationChoiceWidget("");
  rotation = new ExtXMLWidget("Rotation",rotation_,true);
  rotation->setXMLName(MBSIMNS"rotation");
  properties->addToTab("Kinematics", rotation);
  connect(rotation_,SIGNAL(rotationChanged()),this,SLOT(resizeVariables()));
//  connect(rotation_,SIGNAL(rotationChanged()),this,SLOT(resizeGeneralizedVelocity()));
//  connect(rotation,SIGNAL(resize()),this,SLOT(resizeGeneralizedPosition()));
  connect(rotation,SIGNAL(resize()),this,SLOT(resizeVariables()));
 
  ombvEditor = new ExtXMLWidget("OpenMBV body",new OMBVBodySelectionWidget(this),true);
  properties->addToTab("Visualisation", ombvEditor);

  weightArrow = new ExtXMLWidget("OpenMBV weight arrow",new OMBVArrowWidget("NOTSET"),true);
  weightArrow->setXMLName(MBSIMNS"openMBVWeightArrow",false);
  properties->addToTab("Visualisation",weightArrow);

  jointForceArrow = new ExtXMLWidget("OpenMBV joint force arrow",new OMBVArrowWidget("NOTSET"),true);
  jointForceArrow->setXMLName(MBSIMNS"openMBVJointForceArrow",false);
  properties->addToTab("Visualisation",jointForceArrow);

  jointMomentArrow = new ExtXMLWidget("OpenMBV joint moment arrow",new OMBVArrowWidget("NOTSET"),true);
  jointMomentArrow->setXMLName(MBSIMNS"openMBVJointMomentArrow",false);
  properties->addToTab("Visualisation",jointMomentArrow);

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFrame()));
  contextMenu->insertAction(actionSaveAs,action);

  QMenu *submenu = new QMenu("Add contour");
  contextMenu->insertMenu(actionSaveAs,submenu);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Add point", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addPoint()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Add line", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addLine()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Add plane", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addPlane()));
  submenu->addAction(action);
  action=new QAction(Utils::QIconCached("newobject.svg"),"Add sphere", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addSphere()));
  submenu->addAction(action);

  contextMenu->insertSeparator(actionSaveAs);

  properties->addStretch();
}

RigidBody::~RigidBody() {
}

int RigidBody::getUnconstrainedSize() const {
  return (translation->isActive()?((TranslationChoiceWidget*)translation->getWidget())->getSize():0) + (rotation->isActive()?((RotationChoiceWidget*)rotation->getWidget())->getSize():0);
}

void RigidBody::addFrame() {
  QString text = newName(frames,"P");
  if (!text.isEmpty()) {
    new Frame(text, frames, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void RigidBody::addPoint() {
  QString text = newName(contours,"Point");
  if (!text.isEmpty()) {
    new Point(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void RigidBody::addLine() {
  QString text = newName(contours,"Line");
  if (!text.isEmpty()) {
    new Line(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void RigidBody::addPlane() {
  QString text = newName(contours,"Plane");
  if (!text.isEmpty()) {
    new Plane(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void RigidBody::addSphere() {
  QString text = newName(contours,"Sphere");
  if (!text.isEmpty()) {
    new Sphere(text, contours, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void RigidBody::resizeGeneralizedPosition() {
  int size = getSize();
  if(q0->size() != size)
    q0->resize(size);
}

void RigidBody::resizeGeneralizedVelocity() {
  int size = getSize();
  if(u0->size() != size)
    u0->resize(size);
}

void RigidBody::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;

  // frames
  e=element->FirstChildElement(MBSIMNS"frames")->FirstChildElement();
  while(e && e->ValueStr()==MBSIMNS"frame") {
    TiXmlElement *ec=e->FirstChildElement();
    Frame *f=new Frame(ec->Attribute("name"), frames, -1);
    f->initializeUsingXML(ec);
    e=e->NextSiblingElement();
  }

  framePos->initializeUsingXML(element->FirstChildElement(MBSIMNS"frames"));

  // contours
  e=element->FirstChildElement(MBSIMNS"contours")->FirstChildElement();
  Contour *c;
  while(e && e->ValueStr()==MBSIMNS"contour") {
    TiXmlElement *ec=e->FirstChildElement();
    c=ObjectFactory::getInstance()->createContour(ec, contours, -1);
    c->initializeUsingXML(ec);
    e=e->NextSiblingElement();
  }

  contourPos->initializeUsingXML(element->FirstChildElement(MBSIMNS"contours"));

  frameOfReference->initializeUsingXML(element);

  frameForKinematics->initializeUsingXML(element);

  mass->initializeUsingXML(element);
  inertia->initializeUsingXML(element);

  translation->initializeUsingXML(element);
  rotation->initializeUsingXML(element);

  //    // BEGIN The following elements are rarly used. That is why they are optional
  //    e=element->FirstChildElement(MBSIMNS"jacobianOfTranslation");
  //    if(e) {
  //      Jacobian *jac=ObjectFactory::getInstance()->createJacobian(e->FirstChildElement());
  //      setJacobianOfTranslation(jac);
  //      jac->initializeUsingXML(e->FirstChildElement());
  //    }
  //    e=element->FirstChildElement(MBSIMNS"jacobianOfRotation");
  //    if(e) {
  //      Jacobian *jac=ObjectFactory::getInstance()->createJacobian(e->FirstChildElement());
  //      setJacobianOfRotation(jac);
  //      jac->initializeUsingXML(e->FirstChildElement());
  //    }
  //    e=element->FirstChildElement(MBSIMNS"derivativeOfJacobianOfTranslation");
  //    if(e) {
  //      Function3<Mat3V,Vec,Vec,double> *f=ObjectFactory::getInstance()->createFunction3_MVVS(e->FirstChildElement());
  //      setDerivativeOfJacobianOfTranslation(f);
  //      f->initializeUsingXML(e->FirstChildElement());
  //    }
  //    e=element->FirstChildElement(MBSIMNS"derivativeOfJacobianOfRotation");
  //    if(e) {
  //      Function3<Mat3V,Vec,Vec,double> *f=ObjectFactory::getInstance()->createFunction3_MVVS(e->FirstChildElement());
  //      setDerivativeOfJacobianOfRotation(f);
  //      f->initializeUsingXML(e->FirstChildElement());
  //    }
  //    e=element->FirstChildElement(MBSIMNS"guidingVelocityOfTranslation");
  //    if(e) {
  //      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
  //      setGuidingVelocityOfTranslation(f);
  //      f->initializeUsingXML(e->FirstChildElement());
  //    }
  //    e=element->FirstChildElement(MBSIMNS"guidingVelocityOfRotation");
  //    if(e) {
  //      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
  //      setGuidingVelocityOfRotation(f);
  //      f->initializeUsingXML(e->FirstChildElement());
  //    }
  //    e=element->FirstChildElement(MBSIMNS"derivativeOfGuidingVelocityOfTranslation");
  //    if(e) {
  //      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
  //      setDerivativeOfGuidingVelocityOfTranslation(f);
  //      f->initializeUsingXML(e->FirstChildElement());
  //    }
  //    e=element->FirstChildElement(MBSIMNS"derivativeOfGuidingVelocityOfRotation");
  //    if(e) {
  //      Function1<Vec3,double> *f=ObjectFactory::getInstance()->createFunction1_V3S(e->FirstChildElement());
  //      setDerivativeOfGuidingVelocityOfRotation(f);
  //      f->initializeUsingXML(e->FirstChildElement());
  //    }
  //    // END

  ombvEditor->initializeUsingXML(element);

  e=element->FirstChildElement(MBSIMNS"enableOpenMBVFrameC");
  if(e)
    getFrame(0)->initializeUsingXML2(e);

  weightArrow->initializeUsingXML(element);

  jointForceArrow->initializeUsingXML(element);
  jointMomentArrow->initializeUsingXML(element);

  Body::initializeUsingXML(element);
}

TiXmlElement* RigidBody::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Body::writeXMLFile(parent);

  frameOfReference->writeXMLFile(ele0);

  frameForKinematics->writeXMLFile(ele0);

  mass->writeXMLFile(ele0);
  inertia->writeXMLFile(ele0);

  translation->writeXMLFile(ele0);
  rotation->writeXMLFile(ele0);

  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frames" );
  framePos->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ele1 = new TiXmlElement( MBSIMNS"contours" );
  contourPos->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  ombvEditor->writeXMLFile(ele0);

  Frame *C = getFrame(0);
  if(C->openMBVFrame()) {
    ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameC" );
    C->writeXMLFile2(ele1);
    ele0->LinkEndChild(ele1);
  }

  weightArrow->writeXMLFile(ele0);

  jointForceArrow->writeXMLFile(ele0);
  jointMomentArrow->writeXMLFile(ele0);

  return ele0;
}
