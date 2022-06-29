/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "kinetics_widgets.h"
#include "function_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "custom_widgets.h"
#include "function.h"
#include "function_widget_factory.h"
#include "unknown_widget.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  MBSIMGUI_REGOBJECTFACTORY(BilateralConstraintWidget);
  MBSIMGUI_REGOBJECTFACTORY(RegularizedBilateralConstraintWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnilateralConstraintWidget);
  MBSIMGUI_REGOBJECTFACTORY(RegularizedUnilateralConstraintWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnknownWidget<GeneralizedForceLawWidget>);

  MBSIMGUI_REGOBJECTFACTORY(PlanarCoulombFrictionWidget);
  MBSIMGUI_REGOBJECTFACTORY(PlanarStribeckFrictionWidget);
  MBSIMGUI_REGOBJECTFACTORY(RegularizedPlanarFrictionWidget);
  MBSIMGUI_REGOBJECTFACTORY(SpatialCoulombFrictionWidget);
  MBSIMGUI_REGOBJECTFACTORY(SpatialStribeckFrictionWidget);
  MBSIMGUI_REGOBJECTFACTORY(RegularizedSpatialFrictionWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnknownWidget<FrictionForceLawWidget>);

  MBSIMGUI_REGOBJECTFACTORY(BilateralImpactWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnilateralNewtonImpactWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnknownWidget<GeneralizedImpactLawWidget>);

  MBSIMGUI_REGOBJECTFACTORY(PlanarCoulombImpactWidget);
  MBSIMGUI_REGOBJECTFACTORY(PlanarStribeckImpactWidget);
  MBSIMGUI_REGOBJECTFACTORY(SpatialCoulombImpactWidget);
  MBSIMGUI_REGOBJECTFACTORY(SpatialStribeckImpactWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnknownWidget<FrictionImpactLawWidget>);

  MBSIMGUI_REGOBJECTFACTORY(MagicFormulaSharpWidget);
  MBSIMGUI_REGOBJECTFACTORY(UnknownWidget<TyreModelWidget>);

  DOMElement* GeneralizedForceLawWidget::initializeUsingXML(DOMElement *element) {
    if(forceFunc) forceFunc->initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedForceLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    if(forceFunc) forceFunc->writeXMLFile(ele0);
    return ele0;
  }

  RegularizedBilateralConstraintWidget::RegularizedBilateralConstraintWidget() {

    auto *layout = new QVBoxLayout;
    setLayout(layout);
    forceFunc = new ExtWidget("Force function",new ChoiceWidget(new RegularizedBilateralConstraintFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    layout->addWidget(forceFunc);
  }

  RegularizedUnilateralConstraintWidget::RegularizedUnilateralConstraintWidget() {

    auto *layout = new QVBoxLayout;
    setLayout(layout);
    forceFunc = new ExtWidget("Force function",new ChoiceWidget(new RegularizedUnilateralConstraintFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    layout->addWidget(forceFunc);
  }

  DOMElement* GeneralizedImpactLawWidget::initializeUsingXML(DOMElement *element) {
    return element;
  }

  DOMElement* GeneralizedImpactLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  UnilateralNewtonImpactWidget::UnilateralNewtonImpactWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    restitutionCoefficient = new ExtWidget("Restitution coefficient",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"restitutionCoefficient");
    layout->addWidget(restitutionCoefficient);
  }

  DOMElement* UnilateralNewtonImpactWidget::initializeUsingXML(DOMElement *element) {
    GeneralizedImpactLawWidget::initializeUsingXML(element);
    restitutionCoefficient->initializeUsingXML(element);
    return element;
  }

  DOMElement* UnilateralNewtonImpactWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = GeneralizedImpactLawWidget::writeXMLFile(parent,ref);
    restitutionCoefficient->writeXMLFile(ele0);
    return ele0;
  }

  DOMElement* FrictionForceLawWidget::initializeUsingXML(DOMElement *element) {
    if(frictionForceFunc) frictionForceFunc->initializeUsingXML(element);
    return element;
  }

  DOMElement* FrictionForceLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    if(frictionForceFunc) frictionForceFunc->writeXMLFile(ele0);
    return ele0;
  }

  PlanarCoulombFrictionWidget::PlanarCoulombFrictionWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
    layout->addWidget(frictionCoefficient);
  }

  DOMElement* PlanarCoulombFrictionWidget::initializeUsingXML(DOMElement *element) {
    FrictionForceLawWidget::initializeUsingXML(element);
    frictionCoefficient->initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarCoulombFrictionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionForceLawWidget::writeXMLFile(parent,ref);
    frictionCoefficient->writeXMLFile(ele0);
    return ele0;
  }

  SpatialCoulombFrictionWidget::SpatialCoulombFrictionWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
    layout->addWidget(frictionCoefficient);
  }

  DOMElement* SpatialCoulombFrictionWidget::initializeUsingXML(DOMElement *element) {
    FrictionForceLawWidget::initializeUsingXML(element);
    frictionCoefficient->initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialCoulombFrictionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionForceLawWidget::writeXMLFile(parent,ref);
    frictionCoefficient->writeXMLFile(ele0);
    return ele0;
  }

  PlanarStribeckFrictionWidget::PlanarStribeckFrictionWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *dummy = new Function; // Workaround for correct XML path. TODO: provide a consistent concept
    dummy->setParent(element);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(dummy,"v",1,FunctionWidget::scalar,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
    layout->addWidget(frictionFunction);
  }

  DOMElement* PlanarStribeckFrictionWidget::initializeUsingXML(DOMElement *element) {
    FrictionForceLawWidget::initializeUsingXML(element);
    frictionFunction->initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarStribeckFrictionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionForceLawWidget::writeXMLFile(parent,ref);
    frictionFunction->writeXMLFile(ele0);
    return ele0;
  }

  SpatialStribeckFrictionWidget::SpatialStribeckFrictionWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *dummy = new Function; // Workaround for correct XML path. TODO: provide a consistent concept
    dummy->setParent(element);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(dummy,"v",2,FunctionWidget::fixedVec,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
    layout->addWidget(frictionFunction);
  }

  DOMElement* SpatialStribeckFrictionWidget::initializeUsingXML(DOMElement *element) {
    FrictionForceLawWidget::initializeUsingXML(element);
    frictionFunction->initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialStribeckFrictionWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionForceLawWidget::writeXMLFile(parent,ref);
    frictionFunction->writeXMLFile(ele0);
    return ele0;
  }

  RegularizedPlanarFrictionWidget::RegularizedPlanarFrictionWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionForceFunc = new ExtWidget("Friction force function",new ChoiceWidget(new FrictionFunctionFactory(element,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionForceFunction");
    layout->addWidget(frictionForceFunc);
  }

  RegularizedSpatialFrictionWidget::RegularizedSpatialFrictionWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionForceFunc = new ExtWidget("Friction force function",new ChoiceWidget(new FrictionFunctionFactory(element,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionForceFunction");
    layout->addWidget(frictionForceFunc);
  }

  DOMElement* FrictionImpactLawWidget::initializeUsingXML(DOMElement *element) {
    return element;
  }

  DOMElement* FrictionImpactLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  PlanarCoulombImpactWidget::PlanarCoulombImpactWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
    layout->addWidget(frictionCoefficient);
  }

  DOMElement* PlanarCoulombImpactWidget::initializeUsingXML(DOMElement *element) {
    FrictionImpactLawWidget::initializeUsingXML(element);
    frictionCoefficient->initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarCoulombImpactWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionImpactLawWidget::writeXMLFile(parent,ref);
    frictionCoefficient->writeXMLFile(ele0);
    return ele0;
  }

  SpatialCoulombImpactWidget::SpatialCoulombImpactWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
    layout->addWidget(frictionCoefficient);
  }

  DOMElement* SpatialCoulombImpactWidget::initializeUsingXML(DOMElement *element) {
    FrictionImpactLawWidget::initializeUsingXML(element);
    frictionCoefficient->initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialCoulombImpactWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionImpactLawWidget::writeXMLFile(parent,ref);
    frictionCoefficient->writeXMLFile(ele0);
    return ele0;
  }

  PlanarStribeckImpactWidget::PlanarStribeckImpactWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *dummy = new Function; // Workaround for correct XML path. TODO: provide a consistent concept
    dummy->setParent(element);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(dummy,"v",1,FunctionWidget::scalar,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
    layout->addWidget(frictionFunction);
  }

  DOMElement* PlanarStribeckImpactWidget::initializeUsingXML(DOMElement *element) {
    FrictionImpactLawWidget::initializeUsingXML(element);
    frictionFunction->initializeUsingXML(element);
    return element;
  }

  DOMElement* PlanarStribeckImpactWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionImpactLawWidget::writeXMLFile(parent,ref);
    frictionFunction->writeXMLFile(ele0);
    return ele0;
  }

  SpatialStribeckImpactWidget::SpatialStribeckImpactWidget(Element *element, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    auto *dummy = new Function; // Workaround for correct XML path. TODO: provide a consistent concept
    dummy->setParent(element);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(dummy,"v",2,FunctionWidget::fixedVec,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
    layout->addWidget(frictionFunction);
  }

  DOMElement* SpatialStribeckImpactWidget::initializeUsingXML(DOMElement *element) {
    FrictionImpactLawWidget::initializeUsingXML(element);
    frictionFunction->initializeUsingXML(element);
    return element;
  }

  DOMElement* SpatialStribeckImpactWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = FrictionImpactLawWidget::writeXMLFile(parent,ref);
    frictionFunction->writeXMLFile(ele0);
    return ele0;
  }

  RegularizedBilateralConstraintFunctionFactory::RegularizedBilateralConstraintFunctionFactory() {
    name.emplace_back("Linear regularized bilateral constraint");
    xmlName.push_back(MBSIM%"LinearRegularizedBilateralConstraint");
  }

  Widget* RegularizedBilateralConstraintFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedBilateralConstraintWidget;
    return nullptr;
  }

  RegularizedUnilateralConstraintFunctionFactory::RegularizedUnilateralConstraintFunctionFactory() {
    name.emplace_back("Linear regularized unilateral constraint");
    xmlName.push_back(MBSIM%"LinearRegularizedUnilateralConstraint");
  }

  Widget* RegularizedUnilateralConstraintFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedUnilateralConstraintWidget;
    return nullptr;
  }

  FrictionFunctionFactory::FrictionFunctionFactory(Element *element_, QWidget *parent_) : element(element_), parent(parent_) {
    name.emplace_back("Linear regularized Coulomb friction");
    name.emplace_back("Linear regularized Stribeck friction");
    name.emplace_back("Symbolic function");
    xmlName.push_back(MBSIM%"LinearRegularizedCoulombFriction");
    xmlName.push_back(MBSIM%"LinearRegularizedStribeckFriction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
  }

  Widget* FrictionFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedCoulombFrictionWidget;
    if(i==1)
      return new LinearRegularizedStribeckFrictionWidget(element,parent);
    if(i==2) {
      QStringList var;
      var << "gd" << "laN";
      vector<FunctionWidget::VarType> argType(2);
      argType[0] = FunctionWidget::varVec;
      argType[1] = FunctionWidget::scalar;
      return new SymbolicFunctionWidget(QStringList("gd")<<"laN",vector<int>(2,1),argType,1,FunctionWidget::varVec);
    }
    return nullptr;
  }

  DOMElement* TyreModelWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

}
