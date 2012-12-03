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
#include "function_widgets.h"
#include "utils.h"
#include "octaveutils.h"
#include "string_widgets.h"
#include "extended_widgets.h"
#include <QtGui>

using namespace std;

TiXmlElement* Function1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  return ele0;
}

TiXmlElement* Function2::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  return ele0;
}

void DifferentiableFunction1::setDerivative(Function1 *diff,size_t degree) { 
  derivatives.resize(max(derivatives.size(),degree+1)); 
  derivatives[degree]=diff; 
}

TiXmlElement* DifferentiableFunction1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1::writeXMLFile(parent);
  addElementText(ele0,MBSIMNS"orderOfDerivative",order);
  return ele0;
}

bool DifferentiableFunction1::initializeUsingXML(TiXmlElement *element) {
  Function1::initializeUsingXML(element);
  TiXmlElement * e;
  e=element->FirstChildElement(MBSIMNS"orderOfDerivative");
  if (e)
    setOrderOfDerivative(atoi(e->GetText()));
}

ConstantFunction1::ConstantFunction1(const QString &ext) : Function1(ext) {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"value",QStringList(),0));
  c = new ExtPhysicalVarWidget(input),"VS";  
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Value",c);
  layout->addWidget(extXMLWidget);
}
int ConstantFunction1::getSize() const {
  string str = evalOctaveExpression(c->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}
void ConstantFunction1::resize(int m, int n) {
  if(((VecWidget*)c->getPhysicalStringWidget(0)->getWidget())->size() != m)
    ((VecWidget*)c->getPhysicalStringWidget(0)->getWidget())->resize(m);
}
bool ConstantFunction1::initializeUsingXML(TiXmlElement *element) {
  Function1::initializeUsingXML(element);
  c->initializeUsingXML(element);
}
TiXmlElement* ConstantFunction1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1::writeXMLFile(parent);
  c->writeXMLFile(ele0);
  return ele0;
} 

SinusFunction1::SinusFunction1() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"amplitude",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Amplitude",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"frequency",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Frequency",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"phase",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Phase",var[var.size()-1]);
  layout->addWidget(extXMLWidget);


  input.clear();
  input.push_back(new PhysicalStringWidget(new VecWidget(0,true),MBSIMNS"offset",QStringList(),0));
  var.push_back(new ExtPhysicalVarWidget(input));  
  extXMLWidget = new ExtXMLWidget("Offset",var[var.size()-1]);
  layout->addWidget(extXMLWidget);


}
int SinusFunction1::getSize() const {
  string str = evalOctaveExpression(var[0]->getCurrentPhysicalStringWidget()->getValue());
  vector<vector<string> > A = strToMat(str);
  return A.size()?A[0].size():0;
}
void SinusFunction1::resize(int m, int n) {
  for(unsigned int i=0; i<var.size(); i++)
    if(((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->size() != m)
      ((VecWidget*)var[i]->getPhysicalStringWidget(0)->getWidget())->resize(m);
}
bool SinusFunction1::initializeUsingXML(TiXmlElement *element) {
  DifferentiableFunction1::initializeUsingXML(element);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->initializeUsingXML(element);
}

TiXmlElement* SinusFunction1::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = DifferentiableFunction1::writeXMLFile(parent);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->writeXMLFile(ele0);
  return ele0;
}

LinearSpringDamperForce::LinearSpringDamperForce() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"stiffnessCoefficient",stiffnessUnits(),1));
  var.push_back(new ExtPhysicalVarWidget(input));
  ExtXMLWidget *extXMLWidget = new ExtXMLWidget("Stiffness coefficient",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"dampingCoefficient",dampingUnits(),0));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Damping coefficient",var[var.size()-1]);
  layout->addWidget(extXMLWidget);

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"unloadedLength",lengthUnits(),4));
  var.push_back(new ExtPhysicalVarWidget(input));
  extXMLWidget = new ExtXMLWidget("Unloaded length",var[var.size()-1]);
  layout->addWidget(extXMLWidget);
}
bool LinearSpringDamperForce::initializeUsingXML(TiXmlElement *element) {
  Function2::initializeUsingXML(element);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->initializeUsingXML(element);
}
TiXmlElement* LinearSpringDamperForce::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function2::writeXMLFile(parent);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->writeXMLFile(ele0);
  return ele0;
} 


LinearRegularizedBilateralConstraint::LinearRegularizedBilateralConstraint() {
  QVBoxLayout *layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"stiffnessCoefficient",stiffnessUnits(),1));
  var.push_back(new ExtXMLWidget("Stiffness coefficient",new ExtPhysicalVarWidget(input)));

  input.clear();
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),MBSIMNS"dampingCoefficient",dampingUnits(),0));
  var.push_back(new ExtXMLWidget("Damping coefficient",new ExtPhysicalVarWidget(input)));

  layout->addWidget(var[0]);
  layout->addWidget(var[1]);
}

bool LinearRegularizedBilateralConstraint::initializeUsingXML(TiXmlElement *element) {
  Function1::initializeUsingXML(element);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->initializeUsingXML(element);
}

TiXmlElement* LinearRegularizedBilateralConstraint::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = Function1::writeXMLFile(parent);
  for(unsigned int i=0; i<var.size(); i++)
    var[i]->writeXMLFile(ele0);
  return ele0;
}

Function1ChoiceWidget::Function1ChoiceWidget(const string &xmlName_) : function(0), xmlName(xmlName_) {

  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Constant function"));
  comboBox->addItem(tr("Sinus function"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function1ChoiceWidget::defineForceLaw(int index) {
  layout->removeWidget(function);
  delete function;
//  if(index==0) {
//    function = 0;
//  }
  if(index==0)
    function = new ConstantFunction1("VS");  
  else if(index==1)
    function = new SinusFunction1;

  if(function) {
    layout->addWidget(function);
    emit resize();
  }
}

bool Function1ChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"ConstantFunction1_VS") {
        comboBox->setCurrentIndex(0);
        function->initializeUsingXML(ee);
      }
      else if(ee->ValueStr() == MBSIMNS"SinusFunction1_VS") {
        comboBox->setCurrentIndex(1);
        function->initializeUsingXML(ee);
      }
    }
  }
}

TiXmlElement* Function1ChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  if(function)
    function->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

Function2ChoiceWidget::Function2ChoiceWidget(const string &xmlName_) : function(0), xmlName(xmlName_) {
  layout = new QVBoxLayout;
  layout->setMargin(0);
  setLayout(layout);

  comboBox = new QComboBox;
  //comboBox->addItem(tr("None"));
  comboBox->addItem(tr("Linear spring damper force"));
  layout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineForceLaw(int)));
  defineForceLaw(0);
}

void Function2ChoiceWidget::defineForceLaw(int index) {
  int cols = 0;
  layout->removeWidget(function);
  delete function;
//  if(index==0)
//    function = 0;
  if(index==0) {
    function = new LinearSpringDamperForce;  
    layout->addWidget(function);
  } 
  if(function) {
    //emit resize();
    //connect(function,SIGNAL(resize()),this,SIGNAL(resize()));
  }
}

bool Function2ChoiceWidget::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"LinearSpringDamperForce") {
        comboBox->setCurrentIndex(0);
        function->initializeUsingXML(ee);
      }
    }
  }
}

TiXmlElement* Function2ChoiceWidget::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  if(function)
    function->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}


