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

#include "rigidbody.h"
#include <QtGui/QMenu>
#include <QtGui/QInputDialog>
#include <QtGui/QMessageBox>
#include "frame.h"
#include "utils.h"
#include "group.h"

using namespace std;

RigidBody::RigidBody(const QString &str, QTreeWidgetItem *parentItem, int ind) : Body(str, parentItem, ind), constrained(false) {
  setText(1,getType());
  properties->addTab("Kinematics");
  properties->addTab("Frame positioning");
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

  frameForKinematics=new FrameForKinematicsEditor(this,properties, Utils::QIconCached("lines.svg"), "Frame for kinematics", "Kinematics");
  frameOfReference=new FrameOfReferenceEditor(this,properties, Utils::QIconCached("lines.svg"), "Frame of reference", "Kinematics", ((Group*)getParentElement())->getFrame(0));
  vector<InputWidget*> input;
  input.push_back(new SScalarWidget("1"));
  ExtPhysicalVarWidget *d = new ExtPhysicalVarWidget(input,MBSIMNS"mass",massUnits(),2);
  mass=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Mass", "General", d);
  input.clear();
  input.push_back(new SSymMatWidget(getEye<string>(3,3,"0.01","0")));
  ExtPhysicalVarWidget *mat = new ExtPhysicalVarWidget(input,MBSIMNS"inertiaTensor",inertiaUnits(),2);
  inertia=new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Inertia tensor", "General", mat);

  framePos = new XMLEditor(properties, Utils::QIconCached("lines.svg"), "Position and orientation of frames", "Frame positioning", new FramePositionsWidget(this));
  
  translation=new TranslationEditor(properties, Utils::QIconCached("lines.svg"), "Translation");
  rotation=new RotationEditor(properties, Utils::QIconCached("lines.svg"), "Rotation");
 
  ombvEditor=new OMBVEditor(this, properties, Utils::QIconCached("lines.svg"), "OpenMBV");

  QAction *action=new QAction(Utils::QIconCached("newobject.svg"),"Add frame", this);
  connect(action,SIGNAL(triggered()),this,SLOT(addFrame()));
  contextMenu->insertAction(actionSaveAs,action);

  contextMenu->insertSeparator(actionSaveAs);

  action=new QAction(Utils::QIconCached("newobject.svg"),"Remove", this);
  connect(action,SIGNAL(triggered()),this,SLOT(remove()));
  contextMenu->addAction(action);

  properties->addStretch();
}

RigidBody::~RigidBody() {
  // delete scene graph
  // remove from map
}

void RigidBody::addFrame() {
  QString text = newName(frames,"P");
  if (!text.isEmpty()) {
    new Frame(text, frames, -1);
    ((Element*)treeWidget()->topLevelItem(0))->update();
  }
}

void RigidBody::resizeGeneralizedPosition() {
  int size = getSize();
  if(((SVecWidget*)initialGeneralizedPosition->getExtPhysicalWidget()->getInputWidget(0))->size() != size)
    ((SVecWidget*)initialGeneralizedPosition->getExtPhysicalWidget()->getInputWidget(0))->resize(size);
}

void RigidBody::resizeGeneralizedVelocity() {
  int size = getSize();
  if(((SVecWidget*)initialGeneralizedVelocity->getExtPhysicalWidget()->getInputWidget(0))->size() != size)
    ((SVecWidget*)initialGeneralizedVelocity->getExtPhysicalWidget()->getInputWidget(0))->resize(size);
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
    while(e && e->ValueStr()==MBSIMNS"contour") {
//      TiXmlElement *ec=e->FirstChildElement();
//      Contour *c=ObjectFactory::getInstance()->createContour(ec);
//      TiXmlElement *contourElement=ec; // save for later initialization
//      ec=ec->NextSiblingElement();
//      string refF="C";
//      if(ec->ValueStr()==MBSIMNS"frameOfReference") {
//        refF=ec->Attribute("ref");
//        refF=refF.substr(6, refF.length()-7); // reference frame is allways "Frame[X]"
//        ec=ec->NextSiblingElement();
//      }
//      Vec3 RrRC=getVec3(ec);
//      ec=ec->NextSiblingElement();
//      SqrMat3 ARC=getSqrMat3(ec);
//      addContour(c, RrRC, ARC, refF);
//      c->initializeUsingXML(contourElement);
      e=e->NextSiblingElement();
    }

    e=element->FirstChildElement(MBSIMNS"frameOfReference");
    saved_frameOfReference=e->Attribute("ref");
    //frameOfReference->update();

    e=element->FirstChildElement(MBSIMNS"frameForKinematics");
    //frameForKinematics->update();
    frameForKinematics->setFrame(getByPath<Frame>(e->Attribute("ref"))); // must be on of "Frame[X]" which allready exists

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
    if(e) {
      Frame *C = getFrame(0);
      C->setOpenMBVFrame(true);
      C->setSize(getDouble(e->FirstChildElement(MBSIMNS"size")));
      C->setOffset(getDouble(e->FirstChildElement(MBSIMNS"offset")));
    }
    Body::initializeUsingXML(element);
}

TiXmlElement* RigidBody::writeXMLFile(TiXmlNode *parent) {

  TiXmlElement *ele0 = Body::writeXMLFile(parent);

  TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frameOfReference" );
  // ele1->SetAttribute("ref", getFrameOfReference()->getXMLPath()); // absolute path
  ele1->SetAttribute("ref", frameOfReference->getFrame()->getXMLPath(this,true).toStdString()); // relative path
  ele0->LinkEndChild(ele1);

  ele1 = new TiXmlElement( MBSIMNS"frameForKinematics" );
  QString str = QString("Frame[") + frameForKinematics->getFrame()->getName() + "]";
  ele1->SetAttribute("ref", str.toStdString());
  ele0->LinkEndChild(ele1);
  
  mass->writeXMLFile(ele0);
  inertia->writeXMLFile(ele0);
  
  translation->writeXMLFile(ele0);
  rotation->writeXMLFile(ele0);

  ele1 = new TiXmlElement( MBSIMNS"frames" );
  framePos->writeXMLFile(ele1);
  ele0->LinkEndChild( ele1 );

  
      ele1 = new TiXmlElement( MBSIMNS"contours" );
 //     for(vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i) 
 //       (*i)->writeXMLFile(ele1);
      ele0->LinkEndChild( ele1 );
  
      ombvEditor->writeXMLFile(ele0);
  
      Frame *C = getFrame(0);
      if(C->openMBVFrame()) {
        ele1 = new TiXmlElement( MBSIMNS"enableOpenMBVFrameC" );
        addElementText(ele1,MBSIMNS"size",C->getSize());
        addElementText(ele1,MBSIMNS"offset",C->getOffset());
        ele0->LinkEndChild(ele1);
      }
  
  
  return ele0;
}

void RigidBody::initialize() {
  //if(saved_frameOfReference!="")
  frameOfReference->setFrame(getByPath<Frame>(saved_frameOfReference));
}


