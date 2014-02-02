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
#include "kinetics_properties.h"
#include "function_properties.h"
#include "kinetics_widgets.h"
#include "variable_properties.h"
#include "function_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

GeneralizedForceLawProperty::GeneralizedForceLawProperty(const GeneralizedForceLawProperty &p) : forceFunc(static_cast<FunctionProperty*>(p.forceFunc->clone())) {
}
    
GeneralizedForceLawProperty::~GeneralizedForceLawProperty() {
  delete forceFunc;
}

GeneralizedForceLawProperty& GeneralizedForceLawProperty::operator=(const GeneralizedForceLawProperty &p) {
  delete forceFunc;
  forceFunc=static_cast<FunctionProperty*>(p.forceFunc->clone());
}

DOMElement* GeneralizedForceLawProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0=D(doc)->createElement(MBSIM%getType());
  if(forceFunc) {
    DOMElement *ele1 = D(doc)->createElement( MBSIM%"forceFunction" );
    forceFunc->writeXMLFile(ele1);
    ele0->insertBefore(ele1, NULL);
  }
  parent->insertBefore(ele0, NULL);
  return ele0;
}

void RegularizedBilateralConstraintProperty::defineFunction(int index_) {
  index = index_;
  if(index==0) {
    delete forceFunc;
    forceFunc = new LinearRegularizedBilateralConstraintProperty;  
  }
}

DOMElement* RegularizedBilateralConstraintProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
  DOMElement *e1 = e->getFirstElementChild();
  if(e1 && E(e1)->getTagName() == MBSIM%"LinearRegularizedBilateralConstraint") {
    defineFunction(0);
    forceFunc->initializeUsingXML(e->getFirstElementChild());
  }
  return e;
}

void RegularizedBilateralConstraintProperty::fromWidget(QWidget *widget) {
  defineFunction(static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->currentIndex());
  forceFunc->fromWidget(static_cast<RegularizedBilateralConstraintWidget*>(widget)->forceFunc);
}

void RegularizedBilateralConstraintProperty::toWidget(QWidget *widget) {
  static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->blockSignals(true);;
  static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->setCurrentIndex(index);
  static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->blockSignals(false);;
  static_cast<RegularizedBilateralConstraintWidget*>(widget)->defineFunction(index);
  forceFunc->toWidget(static_cast<RegularizedBilateralConstraintWidget*>(widget)->forceFunc);
}

void RegularizedUnilateralConstraintProperty::defineFunction(int index_) {
  index = index_;
  if(index==0) {
    delete forceFunc;
    forceFunc = new LinearRegularizedUnilateralConstraintProperty;  
  }
}

DOMElement* RegularizedUnilateralConstraintProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
  DOMElement *e1 = e->getFirstElementChild();
  if(e1 && E(e1)->getTagName() == MBSIM%"LinearRegularizedUnilateralConstraint") {
    defineFunction(0);
    forceFunc->initializeUsingXML(e->getFirstElementChild());
  }
  return e;
}

void RegularizedUnilateralConstraintProperty::fromWidget(QWidget *widget) {
  defineFunction(static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->currentIndex());
  forceFunc->fromWidget(static_cast<RegularizedUnilateralConstraintWidget*>(widget)->forceFunc);
}

void RegularizedUnilateralConstraintProperty::toWidget(QWidget *widget) {
  static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->blockSignals(true);;
  static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->setCurrentIndex(index);
  static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->blockSignals(false);;
  static_cast<RegularizedUnilateralConstraintWidget*>(widget)->defineFunction(index);;
  forceFunc->toWidget(static_cast<RegularizedUnilateralConstraintWidget*>(widget)->forceFunc);
}

DOMElement* GeneralizedImpactLawProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0=D(doc)->createElement(MBSIM%getType());
  parent->insertBefore(ele0, NULL);
  return ele0;
}

UnilateralNewtonImpactProperty::UnilateralNewtonImpactProperty() {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"restitutionCoefficient"));
  restitutionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

DOMElement* UnilateralNewtonImpactProperty::initializeUsingXML(DOMElement *element) {
  restitutionCoefficient.initializeUsingXML(element);
  return element;
}

DOMElement* UnilateralNewtonImpactProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele = GeneralizedImpactLawProperty::writeXMLFile(parent);
  restitutionCoefficient.writeXMLFile(ele);
  return ele;
}

void UnilateralNewtonImpactProperty::fromWidget(QWidget *widget) {
  restitutionCoefficient.fromWidget(static_cast<UnilateralNewtonImpactWidget*>(widget)->restitutionCoefficient);
}

void UnilateralNewtonImpactProperty::toWidget(QWidget *widget) {
  restitutionCoefficient.toWidget(static_cast<UnilateralNewtonImpactWidget*>(widget)->restitutionCoefficient);
}

FrictionForceLawProperty::FrictionForceLawProperty(const FrictionForceLawProperty &p) : frictionForceFunc(static_cast<FunctionProperty*>(p.frictionForceFunc->clone())) {
}

FrictionForceLawProperty::~FrictionForceLawProperty() {
  delete frictionForceFunc;
}

FrictionForceLawProperty& FrictionForceLawProperty::operator=(const FrictionForceLawProperty &p) {
  delete frictionForceFunc; 
  frictionForceFunc=static_cast<FunctionProperty*>(p.frictionForceFunc->clone());
}

DOMElement* FrictionForceLawProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0=D(doc)->createElement(MBSIM%getType());
  if(frictionForceFunc) {
    DOMElement *ele1 = D(doc)->createElement( MBSIM%"frictionForceFunction" );
    frictionForceFunc->writeXMLFile(ele1);
    ele0->insertBefore(ele1, NULL);
  }
  parent->insertBefore(ele0, NULL);
  return ele0;
}

PlanarCoulombFrictionProperty::PlanarCoulombFrictionProperty() {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

DOMElement* PlanarCoulombFrictionProperty::initializeUsingXML(DOMElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

DOMElement* PlanarCoulombFrictionProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele = FrictionForceLawProperty::writeXMLFile(parent);
  frictionCoefficient.writeXMLFile(ele);
  return ele;
}

void PlanarCoulombFrictionProperty::fromWidget(QWidget *widget) {
  frictionCoefficient.fromWidget(static_cast<PlanarCoulombFrictionWidget*>(widget)->frictionCoefficient);
}

void PlanarCoulombFrictionProperty::toWidget(QWidget *widget) {
  frictionCoefficient.toWidget(static_cast<PlanarCoulombFrictionWidget*>(widget)->frictionCoefficient);
}

SpatialCoulombFrictionProperty::SpatialCoulombFrictionProperty() {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

DOMElement* SpatialCoulombFrictionProperty::initializeUsingXML(DOMElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

DOMElement* SpatialCoulombFrictionProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele = FrictionForceLawProperty::writeXMLFile(parent);
  frictionCoefficient.writeXMLFile(ele);
  return ele;
}

void SpatialCoulombFrictionProperty::fromWidget(QWidget *widget) {
  frictionCoefficient.fromWidget(static_cast<SpatialCoulombFrictionWidget*>(widget)->frictionCoefficient);
}

void SpatialCoulombFrictionProperty::toWidget(QWidget *widget) {
  frictionCoefficient.toWidget(static_cast<SpatialCoulombFrictionWidget*>(widget)->frictionCoefficient);
}

void RegularizedPlanarFrictionProperty::defineFunction(int index_) {
  index = index_;
  delete frictionForceFunc;
  if(index==0)
    frictionForceFunc = new LinearRegularizedCoulombFrictionProperty;  
}

DOMElement* RegularizedPlanarFrictionProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  e=E(element)->getFirstElementChildNamed(MBSIM%"frictionForceFunction");
  DOMElement *e1 = e->getFirstElementChild();
  if(e1 && E(e1)->getTagName() == MBSIM%"LinearRegularizedCoulombFriction")
    index = 0;
  defineFunction(0);
  frictionForceFunc->initializeUsingXML(e->getFirstElementChild());
  return e;
}

void RegularizedPlanarFrictionProperty::fromWidget(QWidget *widget) {
  defineFunction(static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->currentIndex());
  frictionForceFunc->fromWidget(static_cast<RegularizedPlanarFrictionWidget*>(widget)->frictionForceFunc);
}

void RegularizedPlanarFrictionProperty::toWidget(QWidget *widget) {
  static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->blockSignals(true);;
  static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->setCurrentIndex(index);
  static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->blockSignals(false);;
  static_cast<RegularizedPlanarFrictionWidget*>(widget)->defineFunction(index);
  frictionForceFunc->toWidget(static_cast<RegularizedPlanarFrictionWidget*>(widget)->frictionForceFunc);
}

void RegularizedSpatialFrictionProperty::defineFunction(int index_) {
  index = index_;
  delete frictionForceFunc;
  if(index==0)
    frictionForceFunc = new LinearRegularizedCoulombFrictionProperty;  
}

DOMElement* RegularizedSpatialFrictionProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *e;
  e=E(element)->getFirstElementChildNamed(MBSIM%"frictionForceFunction");
  DOMElement *e1 = e->getFirstElementChild();
  if(e1 && E(e1)->getTagName() == MBSIM%"LinearRegularizedCoulombFriction")
    index = 0;
  defineFunction(0);
  frictionForceFunc->initializeUsingXML(e->getFirstElementChild());
  return e;
}

void RegularizedSpatialFrictionProperty::fromWidget(QWidget *widget) {
  defineFunction(static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->currentIndex());
  frictionForceFunc->fromWidget(static_cast<RegularizedSpatialFrictionWidget*>(widget)->frictionForceFunc);
}

void RegularizedSpatialFrictionProperty::toWidget(QWidget *widget) {
  static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->blockSignals(true);;
  static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->setCurrentIndex(index);
  static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->blockSignals(false);;
  static_cast<RegularizedSpatialFrictionWidget*>(widget)->defineFunction(index);
  frictionForceFunc->toWidget(static_cast<RegularizedSpatialFrictionWidget*>(widget)->frictionForceFunc);
}

DOMElement* FrictionImpactLawProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0=D(doc)->createElement(MBSIM%getType());
  parent->insertBefore(ele0, NULL);
  return ele0;
}

PlanarCoulombImpactProperty::PlanarCoulombImpactProperty() {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

DOMElement* PlanarCoulombImpactProperty::initializeUsingXML(DOMElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

DOMElement* PlanarCoulombImpactProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele = FrictionImpactLawProperty::writeXMLFile(parent);
  frictionCoefficient.writeXMLFile(ele);
  return ele;
}

void PlanarCoulombImpactProperty::fromWidget(QWidget *widget) {
  frictionCoefficient.fromWidget(static_cast<PlanarCoulombImpactWidget*>(widget)->frictionCoefficient);
}

void PlanarCoulombImpactProperty::toWidget(QWidget *widget) {
  frictionCoefficient.toWidget(static_cast<PlanarCoulombImpactWidget*>(widget)->frictionCoefficient);
}

SpatialCoulombImpactProperty::SpatialCoulombImpactProperty() {
  vector<PhysicalVariableProperty> input;
  input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

DOMElement* SpatialCoulombImpactProperty::initializeUsingXML(DOMElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

DOMElement* SpatialCoulombImpactProperty::writeXMLFile(DOMNode *parent) {
  DOMElement *ele = FrictionImpactLawProperty::writeXMLFile(parent);
  frictionCoefficient.writeXMLFile(ele);
  return ele;
}

void SpatialCoulombImpactProperty::fromWidget(QWidget *widget) {
  frictionCoefficient.fromWidget(static_cast<SpatialCoulombImpactWidget*>(widget)->frictionCoefficient);
}

void SpatialCoulombImpactProperty::toWidget(QWidget *widget) {
  frictionCoefficient.toWidget(static_cast<SpatialCoulombImpactWidget*>(widget)->frictionCoefficient);
}

void GeneralizedForceLawChoiceProperty::defineForceLaw(int index_) {
  index = index_;
  delete generalizedForceLaw;
  if(index==0)
    generalizedForceLaw = new BilateralConstraintProperty;  
  else if(index==1)
    generalizedForceLaw = new RegularizedBilateralConstraintProperty;  
  else if(index==2)
    generalizedForceLaw = new UnilateralConstraintProperty;  
  else if(index==3)
    generalizedForceLaw = new RegularizedUnilateralConstraintProperty;  
}

DOMElement* GeneralizedForceLawChoiceProperty::initializeUsingXML(DOMElement *element) {
  DOMElement  *e=E(element)->getFirstElementChildNamed(xmlName);
  if(e) {
    DOMElement* ee=e->getFirstElementChild();
    if(ee) {
      if(E(ee)->getTagName() == MBSIM%"BilateralConstraint")
        index = 0;
      else if(E(ee)->getTagName() == MBSIM%"RegularizedBilateralConstraint")
        index = 1;
      else if(E(ee)->getTagName() == MBSIM%"UnilateralConstraint")
        index = 2;
      else if(E(ee)->getTagName() == MBSIM%"RegularizedUnilateralConstraint")
        index = 3;
      defineForceLaw(index);
      generalizedForceLaw->initializeUsingXML(ee);
    }
  }
  return e;
}

DOMElement* GeneralizedForceLawChoiceProperty::writeXMLFile(DOMNode *parent) {
  DOMDocument *doc=parent->getOwnerDocument();
  DOMElement *ele0 = D(doc)->createElement(xmlName);
  if(generalizedForceLaw)
    generalizedForceLaw->writeXMLFile(ele0);
  parent->insertBefore(ele0, NULL);

  return 0;
}

void GeneralizedForceLawChoiceProperty::fromWidget(QWidget *widget) {
  defineForceLaw(static_cast<GeneralizedForceLawChoiceWidget*>(widget)->comboBox->currentIndex());
  generalizedForceLaw->fromWidget(static_cast<GeneralizedForceLawChoiceWidget*>(widget)->generalizedForceLaw);
}

void GeneralizedForceLawChoiceProperty::toWidget(QWidget *widget) {
  static_cast<GeneralizedForceLawChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<GeneralizedForceLawChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<GeneralizedForceLawChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<GeneralizedForceLawChoiceWidget*>(widget)->defineForceLaw(index);
  generalizedForceLaw->toWidget(static_cast<GeneralizedForceLawChoiceWidget*>(widget)->generalizedForceLaw);
}

void GeneralizedImpactLawChoiceProperty::defineImpactLaw(int index_) {
  index = index_;
  delete generalizedImpactLaw;
  if(index==0)
    generalizedImpactLaw = new BilateralImpactProperty;  
  else if(index==1)
    generalizedImpactLaw = new UnilateralNewtonImpactProperty;  
}

DOMElement* GeneralizedImpactLawChoiceProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *e=(xmlName==FQN())?element:E(element)->getFirstElementChildNamed(xmlName);
  if(e) {
    DOMElement* ee=e->getFirstElementChild();
    if(ee) {
      if(E(ee)->getTagName() == MBSIM%"BilateralImpact")
        index = 0;
      if(E(ee)->getTagName() == MBSIM%"UnilateralNewtonImpact")
        index = 1;
      defineImpactLaw(index);
      generalizedImpactLaw->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

DOMElement* GeneralizedImpactLawChoiceProperty::writeXMLFile(DOMNode *parent) {
  if(xmlName!=FQN()) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = D(doc)->createElement(xmlName);
    if(generalizedImpactLaw)
      generalizedImpactLaw->writeXMLFile(ele0);
    parent->insertBefore(ele0, NULL);
  }
  else
    generalizedImpactLaw->writeXMLFile(parent);

  return 0;
}

void GeneralizedImpactLawChoiceProperty::fromWidget(QWidget *widget) {
  defineImpactLaw(static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->comboBox->currentIndex());
  generalizedImpactLaw->fromWidget(static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->generalizedImpactLaw);
}

void GeneralizedImpactLawChoiceProperty::toWidget(QWidget *widget) {
  static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->comboBox->blockSignals(true);
  static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->comboBox->blockSignals(false);
  static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->defineImpactLaw(index);
  generalizedImpactLaw->toWidget(static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->generalizedImpactLaw);
}

void FrictionForceLawChoiceProperty::defineFrictionLaw(int index_) {
  index = index_;
  delete frictionForceLaw;
  if(index==0)
    frictionForceLaw = new PlanarCoulombFrictionProperty;  
  if(index==1)
    frictionForceLaw = new RegularizedPlanarFrictionProperty;  
  if(index==2)
    frictionForceLaw = new SpatialCoulombFrictionProperty;  
  if(index==3)
    frictionForceLaw = new RegularizedSpatialFrictionProperty;  
}

DOMElement* FrictionForceLawChoiceProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *e=(xmlName==FQN())?element:E(element)->getFirstElementChildNamed(xmlName);
  if(e) {
    DOMElement* ee=e->getFirstElementChild();
    if(ee) {
      if(E(ee)->getTagName() == MBSIM%"PlanarCoulombFriction")
        index = 0;
      else if(E(ee)->getTagName() == MBSIM%"RegularizedPlanarFriction")
        index = 1;
      else if(E(ee)->getTagName() == MBSIM%"SpatialCoulombFriction")
        index = 2;
      else if(E(ee)->getTagName() == MBSIM%"RegularizedSpatialFriction")
        index = 3;
      defineFrictionLaw(index);
      frictionForceLaw->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

DOMElement* FrictionForceLawChoiceProperty::writeXMLFile(DOMNode *parent) {
  if(xmlName!=FQN()) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = D(doc)->createElement(xmlName);
    if(frictionForceLaw)
      frictionForceLaw->writeXMLFile(ele0);
    parent->insertBefore(ele0, NULL);
  }
  else
    frictionForceLaw->writeXMLFile(parent);

  return 0;
}

void FrictionForceLawChoiceProperty::fromWidget(QWidget *widget) {
  defineFrictionLaw(static_cast<FrictionForceLawChoiceWidget*>(widget)->comboBox->currentIndex());
  frictionForceLaw->fromWidget(static_cast<FrictionForceLawChoiceWidget*>(widget)->frictionForceLaw);
}

void FrictionForceLawChoiceProperty::toWidget(QWidget *widget) {
  static_cast<FrictionForceLawChoiceWidget*>(widget)->comboBox->blockSignals(true);;
  static_cast<FrictionForceLawChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<FrictionForceLawChoiceWidget*>(widget)->comboBox->blockSignals(false);;
  static_cast<FrictionForceLawChoiceWidget*>(widget)->defineFrictionLaw(index);
  frictionForceLaw->toWidget(static_cast<FrictionForceLawChoiceWidget*>(widget)->frictionForceLaw);
}

void FrictionImpactLawChoiceProperty::defineFrictionImpactLaw(int index_) {
  index = index_;
  delete frictionImpactLaw;
  if(index==0)
    frictionImpactLaw = new PlanarCoulombImpactProperty;  
  else if(index==1)
    frictionImpactLaw = new SpatialCoulombImpactProperty;  
}

DOMElement* FrictionImpactLawChoiceProperty::initializeUsingXML(DOMElement *element) {
  DOMElement *e=(xmlName==FQN())?element:E(element)->getFirstElementChildNamed(xmlName);
  if(e) {
    DOMElement* ee=e->getFirstElementChild();
    if(ee) {
      if(E(ee)->getTagName() == MBSIM%"PlanarCoulombImpact")
        index = 0;
      else if(E(ee)->getTagName() == MBSIM%"SpatialCoulombImpact")
        index = 1;
      defineFrictionImpactLaw(index);
      frictionImpactLaw->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

DOMElement* FrictionImpactLawChoiceProperty::writeXMLFile(DOMNode *parent) {
  if(xmlName!=FQN()) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0 = D(doc)->createElement(xmlName);
    if(frictionImpactLaw)
      frictionImpactLaw->writeXMLFile(ele0);
    parent->insertBefore(ele0, NULL);
  }
  else
    frictionImpactLaw->writeXMLFile(parent);

  return 0;
}

void FrictionImpactLawChoiceProperty::fromWidget(QWidget *widget) {
  defineFrictionImpactLaw(static_cast<FrictionImpactLawChoiceWidget*>(widget)->comboBox->currentIndex());
  frictionImpactLaw->fromWidget(static_cast<FrictionImpactLawChoiceWidget*>(widget)->frictionImpactLaw);
}

void FrictionImpactLawChoiceProperty::toWidget(QWidget *widget) {
  static_cast<FrictionImpactLawChoiceWidget*>(widget)->comboBox->blockSignals(true);;
  static_cast<FrictionImpactLawChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  static_cast<FrictionImpactLawChoiceWidget*>(widget)->comboBox->blockSignals(false);;
  static_cast<FrictionImpactLawChoiceWidget*>(widget)->defineFrictionImpactLaw(index);
  frictionImpactLaw->toWidget(static_cast<FrictionImpactLawChoiceWidget*>(widget)->frictionImpactLaw);
}
