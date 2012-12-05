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
#include "kinetics_widgets.h"
#include "function_widgets.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include "octaveutils.h"
#include <QtGui>

using namespace std;

TiXmlElement* GeneralizedForceLawWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  if(forceFunc) {
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"forceFunction" );
    forceFunc->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}

RegularizedBilateralConstraint::RegularizedBilateralConstraint() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized bilateral constraint"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  forceFunc = new LinearRegularizedBilateralConstraint;  
  layout->addWidget(forceFunc);
}

void RegularizedBilateralConstraint::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(forceFunc);
    delete forceFunc;
    forceFunc = new LinearRegularizedBilateralConstraint;  
    layout->addWidget(forceFunc);
  }
}

bool RegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"forceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedBilateralConstraint") {
    funcList->setCurrentIndex(0);
    forceFunc->initializeUsingXML(e->FirstChildElement());
  }
}

RegularizedUnilateralConstraint::RegularizedUnilateralConstraint() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized unilateral constraint"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  forceFunc = new LinearRegularizedUnilateralConstraint;  
  layout->addWidget(forceFunc);
}

void RegularizedUnilateralConstraint::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(forceFunc);
    delete forceFunc;
    forceFunc = new LinearRegularizedUnilateralConstraint;  
    layout->addWidget(forceFunc);
  }
}

bool RegularizedUnilateralConstraint::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"forceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedUnilateralConstraint") {
    funcList->setCurrentIndex(0);
    forceFunc->initializeUsingXML(e->FirstChildElement());
  }
}

TiXmlElement* GeneralizedImpactLawWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  return ele0;
}

UnilateralNewtonImpact::UnilateralNewtonImpact() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"restitutionCoefficient",noUnitUnits(),1));
  restitutionCoefficient = new ExtXMLWidget("Restitution coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(restitutionCoefficient);
}

bool UnilateralNewtonImpact::initializeUsingXML(TiXmlElement *element) {
  GeneralizedImpactLawWidget::initializeUsingXML(element);
  restitutionCoefficient->initializeUsingXML(element);
}

TiXmlElement* UnilateralNewtonImpact::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = GeneralizedImpactLawWidget::writeXMLFile(parent);
  restitutionCoefficient->writeXMLFile(ele);
  return ele;
}

TiXmlElement* FrictionForceLawWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  if(frictionForceFunc) {
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frictionForceFunction" );
    frictionForceFunc->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}

PlanarCoulombFriction::PlanarCoulombFriction() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"frictionCoefficient",noUnitUnits(),1));
  frictionCoefficient = new ExtXMLWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

bool PlanarCoulombFriction::initializeUsingXML(TiXmlElement *element) {
  FrictionForceLawWidget::initializeUsingXML(element);
  frictionCoefficient->initializeUsingXML(element);
}

TiXmlElement* PlanarCoulombFriction::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = FrictionForceLawWidget::writeXMLFile(parent);
  frictionCoefficient->writeXMLFile(ele);
  return ele;
}

RegularizedPlanarFriction::RegularizedPlanarFriction() {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  funcList = new QComboBox;
  funcList->addItem(tr("Linear regularized coulomb friction"));
  layout->addWidget(funcList);
  setLayout(layout);
  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
  frictionForceFunc = new LinearRegularizedCoulombFriction;  
  layout->addWidget(frictionForceFunc);
}

void RegularizedPlanarFriction::defineFunction(int index) {
  if(index==0) {
    layout->removeWidget(frictionForceFunc);
    delete frictionForceFunc;
    frictionForceFunc = new LinearRegularizedCoulombFriction;  
    layout->addWidget(frictionForceFunc);
  }
}

bool RegularizedPlanarFriction::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"frictionForceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedCoulombFriction") {
    funcList->setCurrentIndex(0);
    frictionForceFunc->initializeUsingXML(e->FirstChildElement());
  }
}

TiXmlElement* FrictionImpactLawWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  return ele0;
}

PlanarCoulombImpact::PlanarCoulombImpact() {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"frictionCoefficient",noUnitUnits(),1));
  frictionCoefficient = new ExtXMLWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
  layout->addWidget(frictionCoefficient);
}

bool PlanarCoulombImpact::initializeUsingXML(TiXmlElement *element) {
  FrictionImpactLawWidget::initializeUsingXML(element);
  frictionCoefficient->initializeUsingXML(element);
}

TiXmlElement* PlanarCoulombImpact::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = FrictionImpactLawWidget::writeXMLFile(parent);
  frictionCoefficient->writeXMLFile(ele);
  return ele;
}

GeneralizedForceLawChoiceWidget::GeneralizedForceLawChoiceWidget(const string &xmlName_) : generalizedForceLaw(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Bilateral constraint"));
  comboBox->addItem(tr("Regularized bilateral constraint"));
  comboBox->addItem(tr("Unilateral constraint"));
  comboBox->addItem(tr("Regularized unilateral constraint"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void GeneralizedForceLawChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(generalizedForceLaw);
  delete generalizedForceLaw;
  if(index==0) {
    generalizedForceLaw = new BilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  } 
  else if(index==1) {
    generalizedForceLaw = new RegularizedBilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  }
  else if(index==2) {
    generalizedForceLaw = new UnilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  }
  else if(index==3) {
    generalizedForceLaw = new RegularizedUnilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  }
}

bool GeneralizedForceLawChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"BilateralConstraint") {
        comboBox->setCurrentIndex(0);
        generalizedForceLaw->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"RegularizedBilateralConstraint") {
        comboBox->setCurrentIndex(1);
        generalizedForceLaw->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"UnilateralConstraint") {
        comboBox->setCurrentIndex(2);
        generalizedForceLaw->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"RegularizedUnilateralConstraint") {
        comboBox->setCurrentIndex(3);
        generalizedForceLaw->initializeUsingXML(ee);
      }
    }
    return true;
  }
  return false;
}

TiXmlElement* GeneralizedForceLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  if(generalizedForceLaw)
    generalizedForceLaw->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

GeneralizedImpactLawChoiceWidget::GeneralizedImpactLawChoiceWidget(const string &xmlName_) : generalizedImpactLaw(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Bilateral impact"));
  comboBox->addItem(tr("Unilateral Newton impact"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineImpactLaw(int)));
  defineImpactLaw(0);
}

void GeneralizedImpactLawChoiceWidget::defineImpactLaw(int index) {
  layout->removeWidget(generalizedImpactLaw);
  delete generalizedImpactLaw;
  if(index==0) {
    generalizedImpactLaw = new BilateralImpact;  
    layout->addWidget(generalizedImpactLaw);
  } 
  else if(index==1) {
    generalizedImpactLaw = new UnilateralNewtonImpact;  
    layout->addWidget(generalizedImpactLaw);
  } 
}

bool GeneralizedImpactLawChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"BilateralImpact")
        comboBox->setCurrentIndex(0);
      if(ee->ValueStr() == MBSIMNS"UnilateralNewtonImpact")
        comboBox->setCurrentIndex(1);
      generalizedImpactLaw->initializeUsingXML(ee);
      return true;
    }
  }
  return false;
}

TiXmlElement* GeneralizedImpactLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    if(generalizedImpactLaw)
      generalizedImpactLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    generalizedImpactLaw->writeXMLFile(parent);

  return 0;
}

FrictionForceLawChoiceWidget::FrictionForceLawChoiceWidget(const string &xmlName_) : frictionForceLaw(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Planar coulomb friction"));
  comboBox->addItem(tr("Regularized planar friction"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFrictionLaw(int)));
  defineFrictionLaw(0);
}

void FrictionForceLawChoiceWidget::defineFrictionLaw(int index) {
  layout->removeWidget(frictionForceLaw);
  delete frictionForceLaw;
  if(index==0) {
    frictionForceLaw = new PlanarCoulombFriction;  
    layout->addWidget(frictionForceLaw);
  } 
  if(index==1) {
    frictionForceLaw = new RegularizedPlanarFriction;  
    layout->addWidget(frictionForceLaw);
  } 
}

bool FrictionForceLawChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"PlanarCoulombFriction") {
        comboBox->setCurrentIndex(0);
        frictionForceLaw->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"RegularizedPlanarFriction") {
        comboBox->setCurrentIndex(1);
        frictionForceLaw->initializeUsingXML(ee);
      }
      return true;
    }
  }
  return false;
}

TiXmlElement* FrictionForceLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    if(frictionForceLaw)
      frictionForceLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    frictionForceLaw->writeXMLFile(parent);

  return 0;
}

FrictionImpactLawChoiceWidget::FrictionImpactLawChoiceWidget(const string &xmlName_) : frictionImpactLaw(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  comboBox->addItem(tr("Planar coloumb impact"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFrictionImpactLaw(int)));
  defineFrictionImpactLaw(0);
}

void FrictionImpactLawChoiceWidget::defineFrictionImpactLaw(int index) {
  layout->removeWidget(frictionImpactLaw);
  delete frictionImpactLaw;
  if(index==0) {
    frictionImpactLaw = new PlanarCoulombImpact;  
    layout->addWidget(frictionImpactLaw);
  } 
}

bool FrictionImpactLawChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"PlanarCoulombImpact")
        comboBox->setCurrentIndex(0);
      frictionImpactLaw->initializeUsingXML(ee);
      return true;
    }
  }
  return false;
}

TiXmlElement* FrictionImpactLawChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  if(xmlName!="") {
    TiXmlElement *ele0 = new TiXmlElement(xmlName);
    if(frictionImpactLaw)
      frictionImpactLaw->writeXMLFile(ele0);
    parent->LinkEndChild(ele0);
  }
  else
    frictionImpactLaw->writeXMLFile(parent);

  return 0;
}

GeneralizedForceChoiceWidget::GeneralizedForceChoiceWidget(const string &xmlName_, ExtXMLWidget* arrow_) : xmlName(xmlName_), arrow(arrow_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new MatColsVarWidget(3,1,1,3),MBSIMNS"direction",noUnitUnits(),1));
  mat_ = new ExtPhysicalVarWidget(input);  
  mat = new ExtXMLWidget("Direction vectors",mat_);
  layout->addWidget(mat);

  generalizedForceLaw_ = new GeneralizedForceLawChoiceWidget(MBSIMNS"generalizedForceLaw");
  generalizedForceLaw = new ExtXMLWidget("Generalized force law",generalizedForceLaw_);
  layout->addWidget(generalizedForceLaw);

  generalizedImpactLaw_ = new GeneralizedImpactLawChoiceWidget("");
  generalizedImpactLaw = new ExtXMLWidget("Generalized impact law",generalizedImpactLaw_,true);
  generalizedImpactLaw->setXMLName(MBSIMNS"generalizedImpactLaw");
  layout->addWidget(generalizedImpactLaw);
}

int GeneralizedForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(mat_->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

bool GeneralizedForceChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(xmlName);
  if(e) {
    mat->initializeUsingXML(e);
    generalizedForceLaw->initializeUsingXML(e);
    generalizedImpactLaw->initializeUsingXML(e);
    arrow->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* GeneralizedForceChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  //  if(getSize()) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  mat->writeXMLFile(ele0);
  generalizedForceLaw->writeXMLFile(ele0);
  generalizedImpactLaw->writeXMLFile(ele0);
  arrow->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  //  }

  return 0;
}

ForceChoiceWidget::ForceChoiceWidget(const string &xmlName_, ExtXMLWidget* arrow_) : xmlName(xmlName_), arrow(arrow_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  PhysicalStringWidget *mat = new PhysicalStringWidget(new MatColsVarWidget(3,1,1,3),MBSIMNS"directionVectors",noUnitUnits(),1);
  input.push_back(mat);
  widget = new ExtPhysicalVarWidget(input);  
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Direction vectors",widget);

  connect(widget,SIGNAL(inputDialogChanged(int)),this,SLOT(resizeVariables()));
  connect((MatColsVarWidget*)mat->getWidget(), SIGNAL(sizeChanged(int)), this, SLOT(resizeVariables()));
  layout->addWidget(extXMLWidget);

  forceLaw = new Function1ChoiceWidget(MBSIMNS"function");
  forceLaw->resize(1,1);
  extXMLWidget = new ExtXMLWidget("Function",forceLaw);

  layout->addWidget(extXMLWidget);
  connect(forceLaw,SIGNAL(resize()),this,SLOT(resizeVariables()));
}

void ForceChoiceWidget::resizeVariables() {
  forceLaw->resize(getSize(),1);
  //((Function1ChoiceWidget*)forceLaw->getWidget())->resize(getSize(),1);
}

int ForceChoiceWidget::getSize() const {
  string str = evalOctaveExpression(widget->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

bool ForceChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    widget->initializeUsingXML(e);
    forceLaw->initializeUsingXML(e);
    arrow->initializeUsingXML(e);
    return true;
  }
  return false;
}

TiXmlElement* ForceChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  //if(getSize()) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  widget->writeXMLFile(ele0);
  forceLaw->writeXMLFile(ele0);
  arrow->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);
  //}

  return 0;
}

ForceDirectionWidget::ForceDirectionWidget(const string &xmlName_, Element *element_) : element(element_), xmlName(xmlName_) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  forceDirWidget = new QWidget;
  QVBoxLayout *hlayout = new QVBoxLayout;
  hlayout->setMargin(0);
  forceDirWidget->setLayout(hlayout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3),MBSIMNS"direction",noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Direction vector",mat);
  hlayout->addWidget(extXMLWidget);
  refFrame = new FrameOfReferenceWidget(MBSIMNS"frameOfReference",element,0);
  extXMLWidget = new ExtXMLWidget("Frame of reference",refFrame);
  hlayout->addWidget(extXMLWidget);

  layout->addWidget(forceDirWidget);

  refFrame->update();
}

bool ForceDirectionWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    refFrame->initializeUsingXML(e);
    mat->initializeUsingXML(e);
  }
}

TiXmlElement* ForceDirectionWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  refFrame->writeXMLFile(ele0);
  mat->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

GeneralizedForceDirectionWidget::GeneralizedForceDirectionWidget(const string &xmlName) {

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new MatColsVarWidget(3,0,0,3),xmlName,noUnitUnits(),1));
  mat = new ExtPhysicalVarWidget(input);  
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Direction vectors",mat);
  layout->addWidget(extXMLWidget);
}

int GeneralizedForceDirectionWidget::getSize() const {
  string str = evalOctaveExpression(mat->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}

bool GeneralizedForceDirectionWidget::initializeUsingXML(TiXmlElement *element) {
  mat->initializeUsingXML(element);
}

TiXmlElement* GeneralizedForceDirectionWidget::writeXMLFile(TiXmlNode *parent) {
  if(getSize())
    mat->writeXMLFile(parent);
  return 0;
}

