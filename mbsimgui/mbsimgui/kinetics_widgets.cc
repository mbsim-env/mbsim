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

TiXmlElement* GeneralizedImpactLawWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  return ele0;
}

GeneralizedForceLawChoiceWidget::GeneralizedForceLawChoiceWidget(const string &xmlName_) : generalizedForceLaw(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Bilateral constraint"));
  comboBox->addItem(tr("Regularized bilateral constraint"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void GeneralizedForceLawChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(generalizedForceLaw);
  delete generalizedForceLaw;
//  if(index==0)
//    generalizedForceLaw = 0;
  if(index==0) {
    generalizedForceLaw = new BilateralConstraint;  
    layout->addWidget(generalizedForceLaw);
  } 
  else if(index==1) {
    generalizedForceLaw = new RegularizedBilateralConstraint;  
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
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Bilateral impact"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineImpactLaw(int)));
  defineImpactLaw(0);
}

void GeneralizedImpactLawChoiceWidget::defineImpactLaw(int index) {
  layout->removeWidget(generalizedImpactLaw);
  delete generalizedImpactLaw;
  //if(index==0)
    //generalizedImpactLaw = 0;
  if(index==0) {
    generalizedImpactLaw = new BilateralImpact;  
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

