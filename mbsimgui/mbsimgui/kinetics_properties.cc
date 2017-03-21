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
#include "function_property_factory.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  GeneralizedForceLaw::GeneralizedForceLaw(const GeneralizedForceLaw &p) : Element(p), forceFunc(static_cast<Function*>(p.forceFunc->clone())) {
  }

  GeneralizedForceLaw::~GeneralizedForceLaw() {
    delete forceFunc;
  }

  GeneralizedForceLaw& GeneralizedForceLaw::operator=(const GeneralizedForceLaw &p) {
    delete forceFunc;
    forceFunc=static_cast<Function*>(p.forceFunc->clone());
    return *this;
  }

  DOMElement* GeneralizedForceLaw::writeXMLFile(DOMNode *parent) {
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

  void RegularizedBilateralConstraint::defineFunction(int index_) {
    index = index_;
    if(index==0) {
      delete forceFunc;
      forceFunc = new LinearRegularizedBilateralConstraint("NoName");
    }
  }

  DOMElement* RegularizedBilateralConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    DOMElement *e1 = e->getFirstElementChild();
    if(e1 && E(e1)->getTagName() == MBSIM%"LinearRegularizedBilateralConstraint") {
      defineFunction(0);
      forceFunc->initializeUsingXML(e->getFirstElementChild());
    }
    return e;
  }

  void RegularizedBilateralConstraint::fromWidget(QWidget *widget) {
    defineFunction(static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->currentIndex());
    forceFunc->fromWidget(static_cast<RegularizedBilateralConstraintWidget*>(widget)->forceFunc);
  }

  void RegularizedBilateralConstraint::toWidget(QWidget *widget) {
    static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->blockSignals(true);;
    static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->setCurrentIndex(index);
    static_cast<RegularizedBilateralConstraintWidget*>(widget)->funcList->blockSignals(false);;
    static_cast<RegularizedBilateralConstraintWidget*>(widget)->defineFunction(index);
    forceFunc->toWidget(static_cast<RegularizedBilateralConstraintWidget*>(widget)->forceFunc);
  }

  void RegularizedUnilateralConstraint::defineFunction(int index_) {
    index = index_;
    if(index==0) {
      delete forceFunc;
      forceFunc = new LinearRegularizedUnilateralConstraint("NoName");
    }
  }

  DOMElement* RegularizedUnilateralConstraint::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"forceFunction");
    DOMElement *e1 = e->getFirstElementChild();
    if(e1 && E(e1)->getTagName() == MBSIM%"LinearRegularizedUnilateralConstraint") {
      defineFunction(0);
      forceFunc->initializeUsingXML(e->getFirstElementChild());
    }
    return e;
  }

  void RegularizedUnilateralConstraint::fromWidget(QWidget *widget) {
    defineFunction(static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->currentIndex());
    forceFunc->fromWidget(static_cast<RegularizedUnilateralConstraintWidget*>(widget)->forceFunc);
  }

  void RegularizedUnilateralConstraint::toWidget(QWidget *widget) {
    static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->blockSignals(true);;
    static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->setCurrentIndex(index);
    static_cast<RegularizedUnilateralConstraintWidget*>(widget)->funcList->blockSignals(false);;
    static_cast<RegularizedUnilateralConstraintWidget*>(widget)->defineFunction(index);;
    forceFunc->toWidget(static_cast<RegularizedUnilateralConstraintWidget*>(widget)->forceFunc);
  }

  DOMElement* GeneralizedImpactLaw::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(MBSIM%getType());
    parent->insertBefore(ele0, NULL);
    return ele0;
  }

  UnilateralNewtonImpact::UnilateralNewtonImpact(const string &name) : GeneralizedImpactLaw(name) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"restitutionCoefficient"));
    restitutionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* UnilateralNewtonImpact::initializeUsingXML(DOMElement *element) {
    restitutionCoefficient.initializeUsingXML(element);
    return element;
  }

  DOMElement* UnilateralNewtonImpact::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = GeneralizedImpactLaw::writeXMLFile(parent);
    restitutionCoefficient.writeXMLFile(ele);
    return ele;
  }

  void UnilateralNewtonImpact::fromWidget(QWidget *widget) {
    restitutionCoefficient.fromWidget(static_cast<UnilateralNewtonImpactWidget*>(widget)->restitutionCoefficient);
  }

  void UnilateralNewtonImpact::toWidget(QWidget *widget) {
    restitutionCoefficient.toWidget(static_cast<UnilateralNewtonImpactWidget*>(widget)->restitutionCoefficient);
  }

  FrictionForceLaw::FrictionForceLaw(const FrictionForceLaw &p) : Element(p), frictionForceFunc(static_cast<Function*>(p.frictionForceFunc->clone())) {
  }

  FrictionForceLaw::~FrictionForceLaw() {
    delete frictionForceFunc;
  }

  FrictionForceLaw& FrictionForceLaw::operator=(const FrictionForceLaw &p) {
    delete frictionForceFunc; 
    frictionForceFunc=static_cast<Function*>(p.frictionForceFunc->clone());
    return *this;
  }

  DOMElement* FrictionForceLaw::writeXMLFile(DOMNode *parent) {
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

  PlanarCoulombFriction::PlanarCoulombFriction(const std::string &name) : FrictionForceLaw(name) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
    frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* PlanarCoulombFriction::initializeUsingXML(DOMElement *element) {
    frictionCoefficient.initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarCoulombFriction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionForceLaw::writeXMLFile(parent);
    frictionCoefficient.writeXMLFile(ele);
    return ele;
  }

  void PlanarCoulombFriction::fromWidget(QWidget *widget) {
    frictionCoefficient.fromWidget(static_cast<PlanarCoulombFrictionWidget*>(widget)->frictionCoefficient);
  }

  void PlanarCoulombFriction::toWidget(QWidget *widget) {
    frictionCoefficient.toWidget(static_cast<PlanarCoulombFrictionWidget*>(widget)->frictionCoefficient);
  }

  SpatialCoulombFriction::SpatialCoulombFriction(const std::string &name) : FrictionForceLaw(name) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
    frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* SpatialCoulombFriction::initializeUsingXML(DOMElement *element) {
    frictionCoefficient.initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialCoulombFriction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionForceLaw::writeXMLFile(parent);
    frictionCoefficient.writeXMLFile(ele);
    return ele;
  }

  void SpatialCoulombFriction::fromWidget(QWidget *widget) {
    frictionCoefficient.fromWidget(static_cast<SpatialCoulombFrictionWidget*>(widget)->frictionCoefficient);
  }

  void SpatialCoulombFriction::toWidget(QWidget *widget) {
    frictionCoefficient.toWidget(static_cast<SpatialCoulombFrictionWidget*>(widget)->frictionCoefficient);
  }

  PlanarStribeckFriction::PlanarStribeckFriction(const std::string &name) : FrictionForceLaw(name) {
    frictionFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"frictionFunction",0));
  }

  DOMElement* PlanarStribeckFriction::initializeUsingXML(DOMElement *element) {
    frictionFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarStribeckFriction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionForceLaw::writeXMLFile(parent);
    frictionFunction.writeXMLFile(ele);
    return ele;
  }

  void PlanarStribeckFriction::fromWidget(QWidget *widget) {
    frictionFunction.fromWidget(static_cast<PlanarStribeckFrictionWidget*>(widget)->frictionFunction);
  }

  void PlanarStribeckFriction::toWidget(QWidget *widget) {
    frictionFunction.toWidget(static_cast<PlanarStribeckFrictionWidget*>(widget)->frictionFunction);
  }

  SpatialStribeckFriction::SpatialStribeckFriction(const std::string &name) : FrictionForceLaw(name) {
    frictionFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"frictionFunction",0));
  }

  DOMElement* SpatialStribeckFriction::initializeUsingXML(DOMElement *element) {
    frictionFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialStribeckFriction::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionForceLaw::writeXMLFile(parent);
    frictionFunction.writeXMLFile(ele);
    return ele;
  }

  void SpatialStribeckFriction::fromWidget(QWidget *widget) {
    frictionFunction.fromWidget(static_cast<SpatialStribeckFrictionWidget*>(widget)->frictionFunction);
  }

  void SpatialStribeckFriction::toWidget(QWidget *widget) {
    frictionFunction.toWidget(static_cast<SpatialStribeckFrictionWidget*>(widget)->frictionFunction);
  }

  void RegularizedPlanarFriction::defineFunction(int index_) {
    index = index_;
    delete frictionForceFunc;
    if(index==0)
      frictionForceFunc = new LinearRegularizedCoulombFriction("NoName");
    else if(index==1)
      frictionForceFunc = new LinearRegularizedStribeckFriction("NoName");
    else {
      vector<string> var;
      var.push_back("gd");
      var.push_back("laN");
      frictionForceFunc = new SymbolicFunction("NoName","VVS",var,1);
    }
  }

  DOMElement* RegularizedPlanarFriction::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"frictionForceFunction");
    DOMElement *e1 = e->getFirstElementChild();
    if(e1) {
      if(E(e1)->getTagName() == MBSIM%"LinearRegularizedCoulombFriction")
        index = 0;
      if(E(e1)->getTagName() == MBSIM%"LinearRegularizedStribeckFriction")
        index = 1;
      else if(E(e1)->getTagName() == MBSIM%"SymbolicFunction")
        index = 2;
    }
    defineFunction(index);
    frictionForceFunc->initializeUsingXML(e->getFirstElementChild());
    return e;
  }

  void RegularizedPlanarFriction::fromWidget(QWidget *widget) {
    defineFunction(static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->currentIndex());
    frictionForceFunc->fromWidget(static_cast<RegularizedPlanarFrictionWidget*>(widget)->frictionForceFunc);
  }

  void RegularizedPlanarFriction::toWidget(QWidget *widget) {
    static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->blockSignals(true);;
    static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->setCurrentIndex(index);
    static_cast<RegularizedPlanarFrictionWidget*>(widget)->funcList->blockSignals(false);;
    static_cast<RegularizedPlanarFrictionWidget*>(widget)->defineFunction(index);
    frictionForceFunc->toWidget(static_cast<RegularizedPlanarFrictionWidget*>(widget)->frictionForceFunc);
  }

  void RegularizedSpatialFriction::defineFunction(int index_) {
    index = index_;
    delete frictionForceFunc;
    if(index==0)
      frictionForceFunc = new LinearRegularizedCoulombFriction("NoName");
    else
      frictionForceFunc = new LinearRegularizedStribeckFriction("NoName");
  }

  DOMElement* RegularizedSpatialFriction::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"frictionForceFunction");
    DOMElement *e1 = e->getFirstElementChild();
    if(e1) {
    if(E(e1)->getTagName() == MBSIM%"LinearRegularizedCoulombFriction")
      index = 0;
    else if(E(e1)->getTagName() == MBSIM%"LinearRegularizedStribeckFriction")
      index = 1;
    }
    defineFunction(0);
    frictionForceFunc->initializeUsingXML(e->getFirstElementChild());
    return e;
  }

  void RegularizedSpatialFriction::fromWidget(QWidget *widget) {
    defineFunction(static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->currentIndex());
    frictionForceFunc->fromWidget(static_cast<RegularizedSpatialFrictionWidget*>(widget)->frictionForceFunc);
  }

  void RegularizedSpatialFriction::toWidget(QWidget *widget) {
    static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->blockSignals(true);;
    static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->setCurrentIndex(index);
    static_cast<RegularizedSpatialFrictionWidget*>(widget)->funcList->blockSignals(false);;
    static_cast<RegularizedSpatialFrictionWidget*>(widget)->defineFunction(index);
    frictionForceFunc->toWidget(static_cast<RegularizedSpatialFrictionWidget*>(widget)->frictionForceFunc);
  }

  DOMElement* FrictionImpactLaw::writeXMLFile(DOMNode *parent) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(MBSIM%getType());
    parent->insertBefore(ele0, NULL);
    return ele0;
  }

  PlanarCoulombImpact::PlanarCoulombImpact(const std::string &name) : FrictionImpactLaw(name) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
    frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* PlanarCoulombImpact::initializeUsingXML(DOMElement *element) {
    frictionCoefficient.initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarCoulombImpact::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionImpactLaw::writeXMLFile(parent);
    frictionCoefficient.writeXMLFile(ele);
    return ele;
  }

  void PlanarCoulombImpact::fromWidget(QWidget *widget) {
    frictionCoefficient.fromWidget(static_cast<PlanarCoulombImpactWidget*>(widget)->frictionCoefficient);
  }

  void PlanarCoulombImpact::toWidget(QWidget *widget) {
    frictionCoefficient.toWidget(static_cast<PlanarCoulombImpactWidget*>(widget)->frictionCoefficient);
  }

  SpatialCoulombImpact::SpatialCoulombImpact(const std::string &name) : FrictionImpactLaw(name) {
    vector<PhysicalVariableProperty> input;
    input.push_back(PhysicalVariableProperty(new ScalarProperty("0"),"-",MBSIM%"frictionCoefficient"));
    frictionCoefficient.setProperty(new ExtPhysicalVarProperty(input));
  }

  DOMElement* SpatialCoulombImpact::initializeUsingXML(DOMElement *element) {
    frictionCoefficient.initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialCoulombImpact::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionImpactLaw::writeXMLFile(parent);
    frictionCoefficient.writeXMLFile(ele);
    return ele;
  }

  void SpatialCoulombImpact::fromWidget(QWidget *widget) {
    frictionCoefficient.fromWidget(static_cast<SpatialCoulombImpactWidget*>(widget)->frictionCoefficient);
  }

  void SpatialCoulombImpact::toWidget(QWidget *widget) {
    frictionCoefficient.toWidget(static_cast<SpatialCoulombImpactWidget*>(widget)->frictionCoefficient);
  }

  PlanarStribeckImpact::PlanarStribeckImpact(const std::string &name) : FrictionImpactLaw(name) {
    frictionFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"frictionFunction",0));
  }

  DOMElement* PlanarStribeckImpact::initializeUsingXML(DOMElement *element) {
    frictionFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarStribeckImpact::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionImpactLaw::writeXMLFile(parent);
    frictionFunction.writeXMLFile(ele);
    return ele;
  }

  void PlanarStribeckImpact::fromWidget(QWidget *widget) {
    frictionFunction.fromWidget(static_cast<PlanarStribeckImpactWidget*>(widget)->frictionFunction);
  }

  void PlanarStribeckImpact::toWidget(QWidget *widget) {
    frictionFunction.toWidget(static_cast<PlanarStribeckImpactWidget*>(widget)->frictionFunction);
  }

  SpatialStribeckImpact::SpatialStribeckImpact(const std::string &name) : FrictionImpactLaw(name) {
    frictionFunction.setProperty(new ChoiceProperty2(new FunctionPropertyFactory2(this),MBSIM%"frictionFunction",0));
  }

  DOMElement* SpatialStribeckImpact::initializeUsingXML(DOMElement *element) {
    frictionFunction.initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialStribeckImpact::writeXMLFile(DOMNode *parent) {
    DOMElement *ele = FrictionImpactLaw::writeXMLFile(parent);
    frictionFunction.writeXMLFile(ele);
    return ele;
  }

  void SpatialStribeckImpact::fromWidget(QWidget *widget) {
    frictionFunction.fromWidget(static_cast<SpatialStribeckImpactWidget*>(widget)->frictionFunction);
  }

  void SpatialStribeckImpact::toWidget(QWidget *widget) {
    frictionFunction.toWidget(static_cast<SpatialStribeckImpactWidget*>(widget)->frictionFunction);
  }

  void GeneralizedForceLawChoiceProperty::defineForceLaw(int index_) {
    index = index_;
    delete generalizedForceLaw;
    if(index==0)
      generalizedForceLaw = new BilateralConstraint("NoName");
    else if(index==1)
      generalizedForceLaw = new RegularizedBilateralConstraint("NoName");
    else if(index==2)
      generalizedForceLaw = new UnilateralConstraint("NoName");
    else if(index==3)
      generalizedForceLaw = new RegularizedUnilateralConstraint("NoName");
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
      generalizedImpactLaw = new BilateralImpact("NoName");
    else if(index==1)
      generalizedImpactLaw = new UnilateralNewtonImpact("NoName");
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
      frictionForceLaw = new PlanarCoulombFriction("NoName");
    else if(index==1)
      frictionForceLaw = new PlanarStribeckFriction("NoName");
    else if(index==2)
      frictionForceLaw = new RegularizedPlanarFriction("NoName");
    else if(index==3)
      frictionForceLaw = new SpatialCoulombFriction("NoName");
    else if(index==4)
      frictionForceLaw = new SpatialStribeckFriction("NoName");
    else if(index==5)
      frictionForceLaw = new RegularizedSpatialFriction("NoName");
  }

  DOMElement* FrictionForceLawChoiceProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=(xmlName==FQN())?element:E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      DOMElement* ee=e->getFirstElementChild();
      if(ee) {
        if(E(ee)->getTagName() == MBSIM%"PlanarCoulombFriction")
          index = 0;
        else if(E(ee)->getTagName() == MBSIM%"PlanarStribeckFriction")
          index = 1;
        else if(E(ee)->getTagName() == MBSIM%"RegularizedPlanarFriction")
          index = 2;
        else if(E(ee)->getTagName() == MBSIM%"SpatialCoulombFriction")
          index = 3;
        else if(E(ee)->getTagName() == MBSIM%"SpatialStribeckFriction")
          index = 4;
        else if(E(ee)->getTagName() == MBSIM%"RegularizedSpatialFriction")
          index = 5;
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
      frictionImpactLaw = new PlanarCoulombImpact("NoName");
    else if(index==1)
      frictionImpactLaw = new PlanarStribeckImpact("NoName");
    else if(index==2)
      frictionImpactLaw = new SpatialCoulombImpact("NoName");
    else if(index==3)
      frictionImpactLaw = new SpatialStribeckImpact("NoName");
  }

  DOMElement* FrictionImpactLawChoiceProperty::initializeUsingXML(DOMElement *element) {
    DOMElement *e=(xmlName==FQN())?element:E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      DOMElement* ee=e->getFirstElementChild();
      if(ee) {
        if(E(ee)->getTagName() == MBSIM%"PlanarCoulombImpact")
          index = 0;
        else if(E(ee)->getTagName() == MBSIM%"PlanarStribeckImpact")
          index = 1;
        else if(E(ee)->getTagName() == MBSIM%"SpatialCoulombImpact")
          index = 2;
        else if(E(ee)->getTagName() == MBSIM%"SpatialStribeckImpact")
          index = 3;
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

}
