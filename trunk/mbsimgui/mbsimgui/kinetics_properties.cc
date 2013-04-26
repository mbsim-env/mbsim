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

TiXmlElement* GeneralizedForceLawProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  if(forceFunc) {
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"forceFunction" );
    forceFunc->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}

void RegularizedBilateralConstraintProperty::defineFunction(int index_) {
  index = index_;
  if(index==0) {
    delete forceFunc;
    forceFunc = new LinearRegularizedBilateralConstraintProperty;  
  }
}

TiXmlElement* RegularizedBilateralConstraintProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"forceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedBilateralConstraint") {
    defineFunction(0);
    forceFunc->initializeUsingXML(e->FirstChildElement());
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

TiXmlElement* RegularizedUnilateralConstraintProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"forceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedUnilateralConstraint") {
    defineFunction(0);
    forceFunc->initializeUsingXML(e->FirstChildElement());
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

TiXmlElement* GeneralizedImpactLawProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  parent->LinkEndChild(ele0);
  return ele0;
}

UnilateralNewtonImpactProperty::UnilateralNewtonImpactProperty() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"restitutionCoefficient"));
  restitutionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* UnilateralNewtonImpactProperty::initializeUsingXML(TiXmlElement *element) {
  restitutionCoefficient.initializeUsingXML(element);
  return element;
}

TiXmlElement* UnilateralNewtonImpactProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = GeneralizedImpactLawProperty::writeXMLFile(parent);
  restitutionCoefficient.writeXMLFile(ele);
  return ele;
}

void UnilateralNewtonImpactProperty::fromWidget(QWidget *widget) {
  restitutionCoefficient.fromWidget(static_cast<UnilateralNewtonImpactWidget*>(widget)->restitutionCoefficient);
}

void UnilateralNewtonImpactProperty::toWidget(QWidget *widget) {
  restitutionCoefficient.toWidget(static_cast<UnilateralNewtonImpactWidget*>(widget)->restitutionCoefficient);
}

TiXmlElement* FrictionForceLawProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  if(frictionForceFunc) {
    TiXmlElement *ele1 = new TiXmlElement( MBSIMNS"frictionForceFunction" );
    frictionForceFunc->writeXMLFile(ele1);
    ele0->LinkEndChild(ele1);
  }
  parent->LinkEndChild(ele0);
  return ele0;
}

PlanarCoulombFrictionProperty::PlanarCoulombFrictionProperty() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* PlanarCoulombFrictionProperty::initializeUsingXML(TiXmlElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

TiXmlElement* PlanarCoulombFrictionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = FrictionForceLawProperty::writeXMLFile(parent);
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
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* SpatialCoulombFrictionProperty::initializeUsingXML(TiXmlElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

TiXmlElement* SpatialCoulombFrictionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = FrictionForceLawProperty::writeXMLFile(parent);
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

TiXmlElement* RegularizedPlanarFrictionProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"frictionForceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedCoulombFriction")
    index = 0;
  defineFunction(0);
  frictionForceFunc->initializeUsingXML(e->FirstChildElement());
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

TiXmlElement* RegularizedSpatialFrictionProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"frictionForceFunction");
  TiXmlElement *e1 = e->FirstChildElement();
  if(e1 && e1->ValueStr() == MBSIMNS"LinearRegularizedCoulombFriction")
    index = 0;
  defineFunction(0);
  frictionForceFunc->initializeUsingXML(e->FirstChildElement());
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

TiXmlElement* FrictionImpactLawProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType());
  parent->LinkEndChild(ele0);
  return ele0;
}

PlanarCoulombImpactProperty::PlanarCoulombImpactProperty() {
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* PlanarCoulombImpactProperty::initializeUsingXML(TiXmlElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

TiXmlElement* PlanarCoulombImpactProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = FrictionImpactLawProperty::writeXMLFile(parent);
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
  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIMNS"frictionCoefficient"));
  frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* SpatialCoulombImpactProperty::initializeUsingXML(TiXmlElement *element) {
  frictionCoefficient.initializeUsingXML(element);
  return element;
}

TiXmlElement* SpatialCoulombImpactProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele = FrictionImpactLawProperty::writeXMLFile(parent);
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

TiXmlElement* GeneralizedForceLawChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement  *e=element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"BilateralConstraint")
        index = 0;
      else if(ee->ValueStr() == MBSIMNS"RegularizedBilateralConstraint")
        index = 1;
      else if(ee->ValueStr() == MBSIMNS"UnilateralConstraint")
        index = 2;
      else if(ee->ValueStr() == MBSIMNS"RegularizedUnilateralConstraint")
        index = 3;
      defineForceLaw(index);
      generalizedForceLaw->initializeUsingXML(ee);
    }
  }
  return e;
}

TiXmlElement* GeneralizedForceLawChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  if(generalizedForceLaw)
    generalizedForceLaw->writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

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

TiXmlElement* GeneralizedImpactLawChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"BilateralImpact")
        index = 0;
      if(ee->ValueStr() == MBSIMNS"UnilateralNewtonImpact")
        index = 1;
      defineImpactLaw(index);
      generalizedImpactLaw->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

TiXmlElement* GeneralizedImpactLawChoiceProperty::writeXMLFile(TiXmlNode *parent) {
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

TiXmlElement* FrictionForceLawChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"PlanarCoulombFriction")
        index = 0;
      else if(ee->ValueStr() == MBSIMNS"RegularizedPlanarFriction")
        index = 1;
      else if(ee->ValueStr() == MBSIMNS"SpatialCoulombFriction")
        index = 2;
      else if(ee->ValueStr() == MBSIMNS"RegularizedSpatialFriction")
        index = 3;
      defineFrictionLaw(index);
      frictionForceLaw->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

TiXmlElement* FrictionForceLawChoiceProperty::writeXMLFile(TiXmlNode *parent) {
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

TiXmlElement* FrictionImpactLawChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=(xmlName=="")?element:element->FirstChildElement(xmlName);
  if(e) {
    TiXmlElement* ee=e->FirstChildElement();
    if(ee) {
      if(ee->ValueStr() == MBSIMNS"PlanarCoulombImpact")
        index = 0;
      else if(ee->ValueStr() == MBSIMNS"SpatialCoulombImpact")
        index = 1;
      defineFrictionImpactLaw(index);
      frictionImpactLaw->initializeUsingXML(ee);
      return e;
    }
  }
  return 0;
}

TiXmlElement* FrictionImpactLawChoiceProperty::writeXMLFile(TiXmlNode *parent) {
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


GeneralizedForceChoiceProperty::GeneralizedForceChoiceProperty(ExtProperty &arrow_, const std::string &xmlName_) : arrow(arrow_), generalizedImpactLaw(0,false), xmlName(xmlName_) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMNS"direction"));
  mat.setProperty(new ExtPhysicalVarProperty(input));

  generalizedForceLaw.setProperty(new GeneralizedForceLawChoiceProperty(MBSIMNS"generalizedForceLaw"));

  generalizedImpactLaw.setProperty(new GeneralizedImpactLawChoiceProperty(MBSIMNS"generalizedImpactLaw"));
  generalizedImpactLaw.setXMLName(MBSIMNS"generalizedImpactLaw");
}

TiXmlElement* GeneralizedForceChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    mat.initializeUsingXML(e);
    generalizedForceLaw.initializeUsingXML(e);
    generalizedImpactLaw.initializeUsingXML(e);
    arrow.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* GeneralizedForceChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  mat.writeXMLFile(ele0);
  generalizedForceLaw.writeXMLFile(ele0);
  generalizedImpactLaw.writeXMLFile(ele0);
  arrow.writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

void GeneralizedForceChoiceProperty::fromWidget(QWidget *widget) {
  mat.fromWidget(static_cast<GeneralizedForceChoiceWidget*>(widget)->mat);
  generalizedForceLaw.fromWidget(static_cast<GeneralizedForceChoiceWidget*>(widget)->generalizedForceLaw);
  generalizedImpactLaw.fromWidget(static_cast<GeneralizedForceChoiceWidget*>(widget)->generalizedImpactLaw);
}

void GeneralizedForceChoiceProperty::toWidget(QWidget *widget) {
  mat.toWidget(static_cast<GeneralizedForceChoiceWidget*>(widget)->mat);
  generalizedForceLaw.toWidget(static_cast<GeneralizedForceChoiceWidget*>(widget)->generalizedForceLaw);
  generalizedImpactLaw.toWidget(static_cast<GeneralizedForceChoiceWidget*>(widget)->generalizedImpactLaw);
}

ForceChoiceProperty::ForceChoiceProperty(ExtProperty &arrow_, const std::string &xmlName_) : arrow(arrow_), xmlName(xmlName_) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new MatProperty(3,1),"-",MBSIMNS"directionVectors"));
  mat.setProperty(new ExtPhysicalVarProperty(input));

  forceLaw.setProperty(new Function1ChoiceProperty(MBSIMNS"function"));
}

TiXmlElement* ForceChoiceProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    mat.initializeUsingXML(e);
    forceLaw.initializeUsingXML(e);
    arrow.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* ForceChoiceProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  mat.writeXMLFile(ele0);
  forceLaw.writeXMLFile(ele0);
  arrow.writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

void ForceChoiceProperty::fromWidget(QWidget *widget) {
  mat.fromWidget(static_cast<ForceChoiceWidget*>(widget)->mat);
  forceLaw.fromWidget(static_cast<ForceChoiceWidget*>(widget)->forceLaw);
}

void ForceChoiceProperty::toWidget(QWidget *widget) {
  mat.toWidget(static_cast<ForceChoiceWidget*>(widget)->mat);
  forceLaw.toWidget(static_cast<ForceChoiceWidget*>(widget)->forceLaw);
}

ForceDirectionProperty::ForceDirectionProperty(Element *element_, const string &xmlName_) : element(element_), xmlName(xmlName_) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new VecProperty(3),"-",MBSIMNS"direction"));
  mat.setProperty(new ExtPhysicalVarProperty(input));
  refFrame.setProperty(new FrameOfReferenceProperty(0,element,MBSIMNS"frameOfReference"));
}
TiXmlElement* ForceDirectionProperty::initializeUsingXML(TiXmlElement *element) {
  TiXmlElement *e=element->FirstChildElement(xmlName);
  if(e) {
    refFrame.initializeUsingXML(e);
    mat.initializeUsingXML(e);
  }
  return e;
}

TiXmlElement* ForceDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0 = new TiXmlElement(xmlName);
  refFrame.writeXMLFile(ele0);
  mat.writeXMLFile(ele0);
  parent->LinkEndChild(ele0);

  return 0;
}

void ForceDirectionProperty::fromWidget(QWidget *widget) {
  mat.fromWidget(static_cast<ForceDirectionWidget*>(widget)->mat);
  refFrame.fromWidget(static_cast<ForceDirectionWidget*>(widget)->refFrame);
}

void ForceDirectionProperty::toWidget(QWidget *widget) {
  mat.toWidget(static_cast<ForceDirectionWidget*>(widget)->mat);
  refFrame.toWidget(static_cast<ForceDirectionWidget*>(widget)->refFrame);
}

GeneralizedForceDirectionProperty::GeneralizedForceDirectionProperty(const string &xmlName) {

  vector<PhysicalVariableProperty*> input;
  input.push_back(new PhysicalVariableProperty(new MatProperty(3,1),"-",xmlName));
  mat.setProperty(new ExtPhysicalVarProperty(input));
}

TiXmlElement* GeneralizedForceDirectionProperty::initializeUsingXML(TiXmlElement *element) {
  return mat.initializeUsingXML(element);
}

TiXmlElement* GeneralizedForceDirectionProperty::writeXMLFile(TiXmlNode *parent) {
  mat.writeXMLFile(parent);
  return 0;
}

void GeneralizedForceDirectionProperty::fromWidget(QWidget *widget) {
  mat.fromWidget(static_cast<GeneralizedForceDirectionWidget*>(widget)->mat);
}

void GeneralizedForceDirectionProperty::toWidget(QWidget *widget) {
  mat.toWidget(static_cast<GeneralizedForceDirectionWidget*>(widget)->mat);
}
