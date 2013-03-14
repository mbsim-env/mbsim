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
#include "string_properties.h"
#include "function_widgets.h"

using namespace std;

TiXmlElement* GeneralizedForceLawProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
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
  static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->setCurrentIndex(index);
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
  static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->setCurrentIndex(index);
  forceFunc->toWidget(static_cast<RegularizedUnilateralConstraintWidget*>(widget)->forceFunc);
}

TiXmlElement* GeneralizedImpactLawProperty::writeXMLFile(TiXmlNode *parent) {
  TiXmlElement *ele0=new TiXmlElement(MBSIMNS+getType().toStdString());
  parent->LinkEndChild(ele0);
  return ele0;
}

UnilateralNewtonImpactProperty::UnilateralNewtonImpactProperty() {
  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new ScalarProperty("0"),"-",""));
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

//PlanarCoulombFriction::PlanarCoulombFriction() {
//  QVBoxLayout *layout = new QVBoxLayout;
//  setLayout(layout);
//  vector<PhysicalStringWidget*> input;
//  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
//  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
//  layout->addWidget(frictionCoefficient);
//}
//
//SpatialCoulombFriction::SpatialCoulombFriction() {
//  QVBoxLayout *layout = new QVBoxLayout;
//  setLayout(layout);
//  vector<PhysicalStringWidget*> input;
//  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
//  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
//  layout->addWidget(frictionCoefficient);
//}
//
//RegularizedPlanarFriction::RegularizedPlanarFriction() {
//
//  layout = new QVBoxLayout;
//  layout->setMargin(0);
//  funcList = new QComboBox;
//  funcList->addItem(tr("Linear regularized coulomb friction"));
//  layout->addWidget(funcList);
//  setLayout(layout);
//  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
//  frictionForceFunc = new LinearRegularizedCoulombFriction;  
//  layout->addWidget(frictionForceFunc);
//}
//
//void RegularizedPlanarFriction::defineFunction(int index) {
//  if(index==0) {
//    layout->removeWidget(frictionForceFunc);
//    delete frictionForceFunc;
//    frictionForceFunc = new LinearRegularizedCoulombFriction;  
//    layout->addWidget(frictionForceFunc);
//  }
//}
//
//RegularizedSpatialFriction::RegularizedSpatialFriction() {
//
//  layout = new QVBoxLayout;
//  layout->setMargin(0);
//  funcList = new QComboBox;
//  funcList->addItem(tr("Linear regularized coulomb friction"));
//  layout->addWidget(funcList);
//  setLayout(layout);
//  connect(funcList, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFunction(int)));
//  frictionForceFunc = new LinearRegularizedCoulombFriction;  
//  layout->addWidget(frictionForceFunc);
//}
//
//void RegularizedSpatialFriction::defineFunction(int index) {
//  if(index==0) {
//    layout->removeWidget(frictionForceFunc);
//    delete frictionForceFunc;
//    frictionForceFunc = new LinearRegularizedCoulombFriction;  
//    layout->addWidget(frictionForceFunc);
//  }
//}
//
//PlanarCoulombImpact::PlanarCoulombImpact() {
//  QVBoxLayout *layout = new QVBoxLayout;
//  setLayout(layout);
//  vector<PhysicalStringWidget*> input;
//  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
//  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
//  layout->addWidget(frictionCoefficient);
//}
//
//SpatialCoulombImpact::SpatialCoulombImpact() {
//  QVBoxLayout *layout = new QVBoxLayout;
//  setLayout(layout);
//  vector<PhysicalStringWidget*> input;
//  input.push_back(new PhysicalStringWidget(new ScalarWidget("0"),noUnitUnits(),1));
//  frictionCoefficient = new ExtWidget("Friction coefficient",new ExtPhysicalVarWidget(input));
//  layout->addWidget(frictionCoefficient);
//}

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
  static_cast<GeneralizedForceLawChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
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
  static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->comboBox->setCurrentIndex(index);
  generalizedImpactLaw->toWidget(static_cast<GeneralizedImpactLawChoiceWidget*>(widget)->generalizedImpactLaw);
}


//FrictionForceLawChoiceWidget::FrictionForceLawChoiceWidget() : frictionForceLaw(0) {
//
//  layout = new QVBoxLayout;
//  layout->setMargin(0);
//  setLayout(layout);
//
//  comboBox = new QComboBox;
//  comboBox->addItem(tr("Planar coulomb friction"));
//  comboBox->addItem(tr("Regularized planar friction"));
//  comboBox->addItem(tr("Spatial coulomb friction"));
//  comboBox->addItem(tr("Regularized spatial friction"));
//  layout->addWidget(comboBox);
//  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFrictionLaw(int)));
//  defineFrictionLaw(0);
//}
//
//void FrictionForceLawChoiceWidget::defineFrictionLaw(int index) {
//  layout->removeWidget(frictionForceLaw);
//  delete frictionForceLaw;
//  if(index==0)
//    frictionForceLaw = new PlanarCoulombFriction;  
//  if(index==1)
//    frictionForceLaw = new RegularizedPlanarFriction;  
//  if(index==2)
//    frictionForceLaw = new SpatialCoulombFriction;  
//  if(index==3)
//    frictionForceLaw = new RegularizedSpatialFriction;  
//  layout->addWidget(frictionForceLaw);
//}
//
//FrictionImpactLawChoiceWidget::FrictionImpactLawChoiceWidget() : frictionImpactLaw(0) {
//
//  layout = new QVBoxLayout;
//  layout->setMargin(0);
//  setLayout(layout);
//
//  comboBox = new QComboBox;
//  comboBox->addItem(tr("Planar coloumb impact"));
//  comboBox->addItem(tr("Spatial coloumb impact"));
//  layout->addWidget(comboBox);
//  connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(defineFrictionImpactLaw(int)));
//  defineFrictionImpactLaw(0);
//}
//
//void FrictionImpactLawChoiceWidget::defineFrictionImpactLaw(int index) {
//  layout->removeWidget(frictionImpactLaw);
//  delete frictionImpactLaw;
//  if(index==0)
//    frictionImpactLaw = new PlanarCoulombImpact;  
//  else if(index==1)
//    frictionImpactLaw = new SpatialCoulombImpact;  
//  layout->addWidget(frictionImpactLaw);
//}

GeneralizedForceChoiceProperty::GeneralizedForceChoiceProperty(ExtProperty &arrow_, const std::string &xmlName_) : arrow(arrow_), generalizedImpactLaw(0,false), xmlName(xmlName_) {

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new MatProperty(3,1),"-",MBSIMNS"direction"));
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

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new MatProperty(3,1),"-",MBSIMNS"directionVectors"));
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


//
//ForceDirectionWidget::ForceDirectionWidget(Element *element_) : element(element_) {
//
//  QVBoxLayout *layout = new QVBoxLayout;
//  layout->setMargin(0);
//  setLayout(layout);
//
//  forceDirWidget = new QWidget;
//  QVBoxLayout *hlayout = new QVBoxLayout;
//  hlayout->setMargin(0);
//  forceDirWidget->setLayout(hlayout);
//
//  vector<PhysicalStringWidget*> input;
//  input.push_back(new PhysicalStringWidget(new VecWidget(3),noUnitUnits(),1));
//  mat = new ExtPhysicalVarWidget(input);
//  ExtWidget *extWidget = new ExtWidget("Direction vector",mat);
//  hlayout->addWidget(extWidget);
//  refFrame = new FrameOfReferenceWidget(element,0);
//  extWidget = new ExtWidget("Frame of reference",refFrame);
//  hlayout->addWidget(extWidget);
//
//  layout->addWidget(forceDirWidget);
//
//  refFrame->updateWidget();
//}

GeneralizedForceDirectionProperty::GeneralizedForceDirectionProperty(const string &xmlName) {

  vector<PhysicalStringProperty*> input;
  input.push_back(new PhysicalStringProperty(new MatProperty(3,1),"-",xmlName));
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
