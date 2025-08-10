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
#include "mainwindow.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "custom_widgets.h"
#include "function.h"
#include "function_widget_factory.h"
#include "unknown_widget.h"
#include "project.h"
#include <QTextStream>
#include <mbxmlutils/eval.h>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

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

  MBSIMGUI_REGOBJECTFACTORY(LinearTyreModelWidget);
  MBSIMGUI_REGOBJECTFACTORY(MagicFormulaSharpWidget);
  MBSIMGUI_REGOBJECTFACTORY(MagicFormula62Widget);
  MBSIMGUI_REGOBJECTFACTORY(UnknownWidget<TyreModelWidget>);

  GeneralizedForceLawWidget::GeneralizedForceLawWidget(Element *parentElement) {
    generalizedForceLaw = new Element;
    generalizedForceLaw->setParent(parentElement);
  }

  GeneralizedForceLawWidget::~GeneralizedForceLawWidget() {
    delete generalizedForceLaw;
  }

  DOMElement* GeneralizedForceLawWidget::initializeUsingXML(DOMElement *element) {
    if(forceFunc) forceFunc->initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedForceLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    if(forceFunc) forceFunc->writeXMLFile(ele0);
    return ele0;
  }

  RegularizedBilateralConstraintWidget::RegularizedBilateralConstraintWidget(Element *parentElement, QWidget *parent) : GeneralizedForceLawWidget(parentElement) {

    auto *layout = new QVBoxLayout;
    setLayout(layout);
    forceFunc = new ExtWidget("Force function",new ChoiceWidget(new RegularizedBilateralConstraintFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    layout->addWidget(forceFunc);
  }

  RegularizedUnilateralConstraintWidget::RegularizedUnilateralConstraintWidget(Element *parentElement, QWidget *parent) : GeneralizedForceLawWidget(parentElement) {

    auto *layout = new QVBoxLayout;
    setLayout(layout);
    forceFunc = new ExtWidget("Force function",new ChoiceWidget(new RegularizedUnilateralConstraintFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    layout->addWidget(forceFunc);
  }

  GeneralizedImpactLawWidget::GeneralizedImpactLawWidget(Element *parentElement) {
    generalizedImpactLaw = new Element;
    generalizedImpactLaw->setParent(parentElement);
  }

  GeneralizedImpactLawWidget::~GeneralizedImpactLawWidget() {
    delete generalizedImpactLaw;
  }

  DOMElement* GeneralizedImpactLawWidget::initializeUsingXML(DOMElement *element) {
    return element;
  }

  DOMElement* GeneralizedImpactLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  UnilateralNewtonImpactWidget::UnilateralNewtonImpactWidget(Element *parentElement, QWidget *parent) : GeneralizedImpactLawWidget(parentElement) {
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

  FrictionForceLawWidget::FrictionForceLawWidget(Element *parentElement) {
    frictionForceLaw = new Element;
    frictionForceLaw->setParent(parentElement);
  }

  FrictionForceLawWidget::~FrictionForceLawWidget() {
    delete frictionForceLaw;
  }

  DOMElement* FrictionForceLawWidget::initializeUsingXML(DOMElement *element) {
    if(frictionForceFunc) frictionForceFunc->initializeUsingXML(element);
    return element;
  }

  DOMElement* FrictionForceLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    if(frictionForceFunc) frictionForceFunc->writeXMLFile(ele0);
    return ele0;
  }

  PlanarCoulombFrictionWidget::PlanarCoulombFrictionWidget(Element *parentElement, QWidget *parent) : FrictionForceLawWidget(parentElement) {
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

  SpatialCoulombFrictionWidget::SpatialCoulombFrictionWidget(Element *parentElement, QWidget *parent) : FrictionForceLawWidget(parentElement) {
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

  PlanarStribeckFrictionWidget::PlanarStribeckFrictionWidget(Element *parentElement, QWidget *parent) : FrictionForceLawWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(frictionForceLaw,"v",1,FunctionWidget::scalar,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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

  SpatialStribeckFrictionWidget::SpatialStribeckFrictionWidget(Element *parentElement, QWidget *parent) : FrictionForceLawWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(frictionForceLaw,"v",2,FunctionWidget::fixedVec,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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

  RegularizedPlanarFrictionWidget::RegularizedPlanarFrictionWidget(Element *parentElement, QWidget *parent) : FrictionForceLawWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionForceFunc = new ExtWidget("Friction force function",new ChoiceWidget(new FrictionFunctionFactory(frictionForceLaw,1,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionForceFunction");
    layout->addWidget(frictionForceFunc);
  }

  RegularizedSpatialFrictionWidget::RegularizedSpatialFrictionWidget(Element *parentElement, QWidget *parent) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionForceFunc = new ExtWidget("Friction force function",new ChoiceWidget(new FrictionFunctionFactory(frictionForceLaw,2,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionForceFunction");
    layout->addWidget(frictionForceFunc);
  }

  FrictionImpactLawWidget::FrictionImpactLawWidget(Element *parentElement) {
    frictionImpactLaw = new Element;
    frictionImpactLaw->setParent(parentElement);
  }

  FrictionImpactLawWidget::~FrictionImpactLawWidget() {
    delete frictionImpactLaw;
  }

  DOMElement* FrictionImpactLawWidget::initializeUsingXML(DOMElement *element) {
    return element;
  }

  DOMElement* FrictionImpactLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  PlanarCoulombImpactWidget::PlanarCoulombImpactWidget(Element *parentElement, QWidget *parent) : FrictionImpactLawWidget(parentElement) {
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

  SpatialCoulombImpactWidget::SpatialCoulombImpactWidget(Element *parentElement, QWidget *parent) : FrictionImpactLawWidget(parentElement) {
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

  PlanarStribeckImpactWidget::PlanarStribeckImpactWidget(Element *parentElement, QWidget *parent) : FrictionImpactLawWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(frictionImpactLaw,"v",1,FunctionWidget::scalar,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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

  SpatialStribeckImpactWidget::SpatialStribeckImpactWidget(Element *parentElement, QWidget *parent) : FrictionImpactLawWidget(parentElement) {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget(new Function1ArgWidgetFactory(frictionImpactLaw,"v",2,FunctionWidget::fixedVec,1,FunctionWidget::scalar,parent),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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
    name.emplace_back("Symbolic function");
    name.emplace_back("Unknown function");
    xmlName.push_back(MBSIM%"LinearRegularizedBilateralConstraint");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"UnknownFunction");
  }

  Widget* RegularizedBilateralConstraintFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedBilateralConstraintWidget;
    if(i==1)
      return new SymbolicFunctionWidget({"g","gd"},{1,1},{FunctionWidget::scalar,FunctionWidget::scalar},1,FunctionWidget::scalar);
    if(i==2)
      return new UnknownWidget<FunctionWidget>;
    return nullptr;
  }

  RegularizedUnilateralConstraintFunctionFactory::RegularizedUnilateralConstraintFunctionFactory() {
    name.emplace_back("Linear regularized unilateral constraint");
    name.emplace_back("Symbolic function");
    name.emplace_back("Unknown function");
    xmlName.push_back(MBSIM%"LinearRegularizedUnilateralConstraint");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"UnknownFunction");
  }

  Widget* RegularizedUnilateralConstraintFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedUnilateralConstraintWidget;
    if(i==1)
      return new SymbolicFunctionWidget({"g","gd"},{1,1},{FunctionWidget::scalar,FunctionWidget::scalar},1,FunctionWidget::scalar);
    if(i==2)
      return new UnknownWidget<FunctionWidget>;
    return nullptr;
  }

  FrictionFunctionFactory::FrictionFunctionFactory(Element *element_, int nd_, QWidget *parent_) : element(element_), nd(nd_), parent(parent_) {
    name.emplace_back("Linear regularized Coulomb friction");
    name.emplace_back("Linear regularized Stribeck friction");
    name.emplace_back("Symbolic function");
    name.emplace_back("Unknown function");
    xmlName.push_back(MBSIM%"LinearRegularizedCoulombFriction");
    xmlName.push_back(MBSIM%"LinearRegularizedStribeckFriction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
    xmlName.push_back(MBSIM%"UnknownFunction");
  }

  Widget* FrictionFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedCoulombFrictionWidget;
    if(i==1)
      return new LinearRegularizedStribeckFrictionWidget(element,parent);
    if(i==2)
      return new SymbolicFunctionWidget({"gd","laN"},{nd,1},{FunctionWidget::varVec,FunctionWidget::scalar},nd,FunctionWidget::varVec);
    if(i==3)
      return new UnknownWidget<FunctionWidget>;
    return nullptr;
  }

  DOMElement* TyreModelWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getXMLType());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  LinearTyreModelWidget::LinearTyreModelWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    cz = new ExtWidget("cz",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"cz");
    layout->addWidget(cz);
    dz = new ExtWidget("dz",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dz");
    layout->addWidget(dz);
    cka = new ExtWidget("cka",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"cka");
    layout->addWidget(cka);
    cal = new ExtWidget("cal",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"cal");
    layout->addWidget(cal);
    cga = new ExtWidget("cga",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"cga");
    layout->addWidget(cga);
    cMzga = new ExtWidget("cMzga",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"cMzga");
    layout->addWidget(cMzga);
    t = new ExtWidget("t",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"t");
    layout->addWidget(t);
    sfFLo = new ExtWidget("Scale factor for longitudinal force",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLongitudinalForce");
    layout->addWidget(sfFLo);
    sfFLa = new ExtWidget("Scale factor for lateral force",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLateralForce");
    layout->addWidget(sfFLa);
    sfM = new ExtWidget("Scale factor for aligning moment",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForAligningMoment");
    layout->addWidget(sfM);
  }

  DOMElement* LinearTyreModelWidget::initializeUsingXML(DOMElement *element) {
    TyreModelWidget::initializeUsingXML(element);
    cz->initializeUsingXML(element);
    dz->initializeUsingXML(element);
    cka->initializeUsingXML(element);
    cal->initializeUsingXML(element);
    cga->initializeUsingXML(element);
    cMzga->initializeUsingXML(element);
    t->initializeUsingXML(element);
    sfFLo->initializeUsingXML(element);
    sfFLa->initializeUsingXML(element);
    sfM->initializeUsingXML(element);
    return element;
  }

  DOMElement* LinearTyreModelWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = TyreModelWidget::writeXMLFile(parent,ref);
    cz->writeXMLFile(ele0);
    dz->writeXMLFile(ele0);
    cka->writeXMLFile(ele0);
    cal->writeXMLFile(ele0);
    cga->writeXMLFile(ele0);
    cMzga->writeXMLFile(ele0);
    t->writeXMLFile(ele0);
    sfFLo->writeXMLFile(ele0);
    sfFLa->writeXMLFile(ele0);
    sfM->writeXMLFile(ele0);
    return ele0;
  }

  MagicFormulaSharpWidget::MagicFormulaSharpWidget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    cz = new ExtWidget("cz",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"cz");
    layout->addWidget(cz);
    dz = new ExtWidget("dz",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"dz");
    layout->addWidget(dz);
    Fz0 = new ExtWidget("Fz0",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,forceUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Fz0");
    layout->addWidget(Fz0);
    R0 = new ExtWidget("R0",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"R0");
    layout->addWidget(R0);
    pKy1 = new ExtWidget("pKy1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKy1");
    layout->addWidget(pKy1);
    pKy2 = new ExtWidget("pKy2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKy2");
    layout->addWidget(pKy2);
    pKy3 = new ExtWidget("pKy3",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKy3");
    layout->addWidget(pKy3);
    pKy4 = new ExtWidget("pKy4",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKy4");
    layout->addWidget(pKy4);
    pKy5 = new ExtWidget("pKy5",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKy5");
    layout->addWidget(pKy5);
    pKy6 = new ExtWidget("pKy6",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKy6");
    layout->addWidget(pKy6);
    pKy7 = new ExtWidget("pKy7",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKy7");
    layout->addWidget(pKy7);
    pDx1 = new ExtWidget("pDx1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pDx1");
    layout->addWidget(pDx1);
    pDx2 = new ExtWidget("pDx2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pDx2");
    layout->addWidget(pDx2);
    pEx1 = new ExtWidget("pEx1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pEx1");
    layout->addWidget(pEx1);
    pEx2 = new ExtWidget("pEx2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pEx2");
    layout->addWidget(pEx2);
    pEx3 = new ExtWidget("pEx3",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pEx3");
    layout->addWidget(pEx3);
    pEx4 = new ExtWidget("pEx4",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pEx4");
    layout->addWidget(pEx4);
    pKx1 = new ExtWidget("pKx1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKx1");
    layout->addWidget(pKx1);
    pKx2 = new ExtWidget("pKx2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKx2");
    layout->addWidget(pKx2);
    pKx3 = new ExtWidget("pKx3",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pKx3");
    layout->addWidget(pKx3);
    Cx = new ExtWidget("Cx",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Cx");
    layout->addWidget(Cx);
    Cy = new ExtWidget("Cy",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Cy");
    layout->addWidget(Cy);
    rBx1 = new ExtWidget("rBx1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"rBx1");
    layout->addWidget(rBx1);
    rBx2 = new ExtWidget("rBx2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"rBx2");
    layout->addWidget(rBx2);
    Cxal = new ExtWidget("Cxal",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Cxal");
    layout->addWidget(Cxal);
    pDy1 = new ExtWidget("pDy1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pDy1");
    layout->addWidget(pDy1);
    pDy2 = new ExtWidget("pDy2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pDy2");
    layout->addWidget(pDy2);
    pDy3 = new ExtWidget("pDy3",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pDy3");
    layout->addWidget(pDy3);
    pEy1 = new ExtWidget("pEy1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pEy1");
    layout->addWidget(pEy1);
    pEy2 = new ExtWidget("pEy2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pEy2");
    layout->addWidget(pEy2);
    pEy4 = new ExtWidget("pEy4",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"pEy4");
    layout->addWidget(pEy4);
    Cga = new ExtWidget("Cga",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Cga");
    layout->addWidget(Cga);
    Ega = new ExtWidget("Ega",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Ega");
    layout->addWidget(Ega);
    rBy1 = new ExtWidget("rBy1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"rBy1");
    layout->addWidget(rBy1);
    rBy2 = new ExtWidget("rBy2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"rBy2");
    layout->addWidget(rBy2);
    rBy3 = new ExtWidget("rBy3",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"rBy3");
    layout->addWidget(rBy3);
    Cyka = new ExtWidget("Cyka",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Cyka");
    layout->addWidget(Cyka);
    qHz3 = new ExtWidget("qHz3",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qHz3");
    layout->addWidget(qHz3);
    qHz4 = new ExtWidget("qHz4",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qHz4");
    layout->addWidget(qHz4);
    qBz1 = new ExtWidget("qBz1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qBz1");
    layout->addWidget(qBz1);
    qBz2 = new ExtWidget("qBz2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qBz2");
    layout->addWidget(qBz2);
    qBz5 = new ExtWidget("qBz5",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qBz5");
    layout->addWidget(qBz5);
    qBz6 = new ExtWidget("qBz6",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qBz6");
    layout->addWidget(qBz6);
    qBz9 = new ExtWidget("qBz9",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qBz9");
    layout->addWidget(qBz9);
    qBz10 = new ExtWidget("qBz10",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qBz10");
    layout->addWidget(qBz10);
    qDz1 = new ExtWidget("qDz1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz1");
    layout->addWidget(qDz1);
    qDz2 = new ExtWidget("qDz2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz2");
    layout->addWidget(qDz2);
    qDz3 = new ExtWidget("qDz3",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz3");
    layout->addWidget(qDz3);
    qDz4 = new ExtWidget("qDz4",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz4");
    layout->addWidget(qDz4);
    qDz8 = new ExtWidget("qDz8",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz8");
    layout->addWidget(qDz8);
    qDz9 = new ExtWidget("qDz9",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz9");
    layout->addWidget(qDz9);
    qDz10 = new ExtWidget("qDz10",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz10");
    layout->addWidget(qDz10);
    qDz11 = new ExtWidget("qDz11",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qDz11");
    layout->addWidget(qDz11);
    qEz1 = new ExtWidget("qEz1",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qEz1");
    layout->addWidget(qEz1);
    qEz2 = new ExtWidget("qEz2",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qEz2");
    layout->addWidget(qEz2);
    qEz5 = new ExtWidget("qEz5",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"qEz5");
    layout->addWidget(qEz5);
    Ct = new ExtWidget("Ct",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"Ct");
    layout->addWidget(Ct);
    c1Rel = new ExtWidget("c1Rel",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"c1Rel");
    layout->addWidget(c1Rel);
    c2Rel = new ExtWidget("c2Rel",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"c2Rel");
    layout->addWidget(c2Rel);
    c3Rel = new ExtWidget("c3Rel",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"c3Rel");
    layout->addWidget(c3Rel);
    sfKyga = new ExtWidget("Scale factor for camber stiffness",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForCamberStiffness");
    layout->addWidget(sfKyga);
    sfFLo = new ExtWidget("Scale factor for longitudinal force",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLongitudinalForce");
    layout->addWidget(sfFLo);
    sfFLa = new ExtWidget("Scale factor for lateral force",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLateralForce");
    layout->addWidget(sfFLa);
    sfM = new ExtWidget("Scale factor for aligning moment",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForAligningMoment");
    layout->addWidget(sfM);
  }

  DOMElement* MagicFormulaSharpWidget::initializeUsingXML(DOMElement *element) {
    TyreModelWidget::initializeUsingXML(element);
    cz->initializeUsingXML(element);
    dz->initializeUsingXML(element);
    Fz0->initializeUsingXML(element);
    R0->initializeUsingXML(element);
    pKy1->initializeUsingXML(element);
    pKy2->initializeUsingXML(element);
    pKy3->initializeUsingXML(element);
    pKy4->initializeUsingXML(element);
    pKy5->initializeUsingXML(element);
    pKy6->initializeUsingXML(element);
    pKy7->initializeUsingXML(element);
    pDx1->initializeUsingXML(element);
    pDx2->initializeUsingXML(element);
    pEx1->initializeUsingXML(element);
    pEx2->initializeUsingXML(element);
    pEx3->initializeUsingXML(element);
    pEx4->initializeUsingXML(element);
    pKx1->initializeUsingXML(element);
    pKx2->initializeUsingXML(element);
    pKx3->initializeUsingXML(element);
    Cx->initializeUsingXML(element);
    Cy->initializeUsingXML(element);
    rBx1->initializeUsingXML(element);
    rBx2->initializeUsingXML(element);
    Cxal->initializeUsingXML(element);
    pDy1->initializeUsingXML(element);
    pDy2->initializeUsingXML(element);
    pDy3->initializeUsingXML(element);
    pEy1->initializeUsingXML(element);
    pEy2->initializeUsingXML(element);
    pEy4->initializeUsingXML(element);
    Cga->initializeUsingXML(element);
    Ega->initializeUsingXML(element);
    rBy1->initializeUsingXML(element);
    rBy2->initializeUsingXML(element);
    rBy3->initializeUsingXML(element);
    Cyka->initializeUsingXML(element);
    qHz3->initializeUsingXML(element);
    qHz4->initializeUsingXML(element);
    qBz1->initializeUsingXML(element);
    qBz2->initializeUsingXML(element);
    qBz5->initializeUsingXML(element);
    qBz6->initializeUsingXML(element);
    qBz9->initializeUsingXML(element);
    qBz10->initializeUsingXML(element);
    qDz1->initializeUsingXML(element);
    qDz2->initializeUsingXML(element);
    qDz3->initializeUsingXML(element);
    qDz4->initializeUsingXML(element);
    qDz8->initializeUsingXML(element);
    qDz9->initializeUsingXML(element);
    qDz10->initializeUsingXML(element);
    qDz11->initializeUsingXML(element);
    qEz1->initializeUsingXML(element);
    qEz2->initializeUsingXML(element);
    qEz5->initializeUsingXML(element);
    Ct->initializeUsingXML(element);
    c1Rel->initializeUsingXML(element);
    c2Rel->initializeUsingXML(element);
    c3Rel->initializeUsingXML(element);
    sfKyga->initializeUsingXML(element);
    sfFLo->initializeUsingXML(element);
    sfFLa->initializeUsingXML(element);
    sfM->initializeUsingXML(element);
    return element;
  }

  DOMElement* MagicFormulaSharpWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = TyreModelWidget::writeXMLFile(parent,ref);
    cz->writeXMLFile(ele0);
    dz->writeXMLFile(ele0);
    Fz0->writeXMLFile(ele0);
    R0->writeXMLFile(ele0);
    pKy1->writeXMLFile(ele0);
    pKy2->writeXMLFile(ele0);
    pKy3->writeXMLFile(ele0);
    pKy4->writeXMLFile(ele0);
    pKy5->writeXMLFile(ele0);
    pKy6->writeXMLFile(ele0);
    pKy7->writeXMLFile(ele0);
    pDx1->writeXMLFile(ele0);
    pDx2->writeXMLFile(ele0);
    pEx1->writeXMLFile(ele0);
    pEx2->writeXMLFile(ele0);
    pEx3->writeXMLFile(ele0);
    pEx4->writeXMLFile(ele0);
    pKx1->writeXMLFile(ele0);
    pKx2->writeXMLFile(ele0);
    pKx3->writeXMLFile(ele0);
    Cx->writeXMLFile(ele0);
    Cy->writeXMLFile(ele0);
    rBx1->writeXMLFile(ele0);
    rBx2->writeXMLFile(ele0);
    Cxal->writeXMLFile(ele0);
    pDy1->writeXMLFile(ele0);
    pDy2->writeXMLFile(ele0);
    pDy3->writeXMLFile(ele0);
    pEy1->writeXMLFile(ele0);
    pEy2->writeXMLFile(ele0);
    pEy4->writeXMLFile(ele0);
    Cga->writeXMLFile(ele0);
    Ega->writeXMLFile(ele0);
    rBy1->writeXMLFile(ele0);
    rBy2->writeXMLFile(ele0);
    rBy3->writeXMLFile(ele0);
    Cyka->writeXMLFile(ele0);
    qHz3->writeXMLFile(ele0);
    qHz4->writeXMLFile(ele0);
    qBz1->writeXMLFile(ele0);
    qBz2->writeXMLFile(ele0);
    qBz5->writeXMLFile(ele0);
    qBz6->writeXMLFile(ele0);
    qBz9->writeXMLFile(ele0);
    qBz10->writeXMLFile(ele0);
    qDz1->writeXMLFile(ele0);
    qDz2->writeXMLFile(ele0);
    qDz3->writeXMLFile(ele0);
    qDz4->writeXMLFile(ele0);
    qDz8->writeXMLFile(ele0);
    qDz9->writeXMLFile(ele0);
    qDz10->writeXMLFile(ele0);
    qDz11->writeXMLFile(ele0);
    qEz1->writeXMLFile(ele0);
    qEz2->writeXMLFile(ele0);
    qEz5->writeXMLFile(ele0);
    Ct->writeXMLFile(ele0);
    c1Rel->writeXMLFile(ele0);
    c2Rel->writeXMLFile(ele0);
    c3Rel->writeXMLFile(ele0);
    sfKyga->writeXMLFile(ele0);
    sfFLo->writeXMLFile(ele0);
    sfFLa->writeXMLFile(ele0);
    sfM->writeXMLFile(ele0);
    return ele0;
  }

  MagicFormula62Widget::MagicFormula62Widget() {
    auto *layout = new QVBoxLayout;
    setLayout(layout);
    inputDataFile = new ExtWidget("Input data file name",new FileWidget("", "Open input data file", "Input data files (*.tir)", 0, true),false,false,MBSIM%"inputDataFileName");
    layout->addWidget(inputDataFile);
    vector<QString> list;
    list.emplace_back("\"left\"");
    list.emplace_back("\"right\"");
    tyreSide = new ExtWidget("Tyre side",new TextChoiceWidget(list,0,true),true,false,MBSIM%"tyreSide");
    layout->addWidget(tyreSide);
    mck = new ExtWidget("Motorcycle kinematics",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"motorcycleKinematics");
    layout->addWidget(mck);
    cpt = new ExtWidget("Contact point transformation",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"contactPointTransformation");
    layout->addWidget(cpt);
    ts = new ExtWidget("Turn slip",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"turnSlip");
    layout->addWidget(ts);
    p = new ExtWidget("Inflation pressure",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(4,pressureUnits()),vector<int>(4,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"inflationPressure");
    layout->addWidget(p);
    cz = new ExtWidget("Vertical stiffness",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,stiffnessUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"verticalStiffness");
    layout->addWidget(cz);
    dz = new ExtWidget("Vertical damping",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,dampingUnits()),vector<int>(2,0)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"verticalDamping");
    layout->addWidget(dz);
    six = new ExtWidget("Relaxation length for longitudinal slip",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"relaxationLengthForLongitudinalSlip");
    layout->addWidget(six);
    siy = new ExtWidget("Relaxation length for sideslip",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"relaxationLengthForSideslip");
    layout->addWidget(siy);
    rtw = new ExtWidget("Reference tread width",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"referenceTreadWidth");
    layout->addWidget(rtw);
    sfFx = new ExtWidget("Scale factor for longitudinal force",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLongitudinalForce");
    layout->addWidget(sfFx);
    sfFy = new ExtWidget("Scale factor for lateral force",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLateralForce");
    layout->addWidget(sfFy);
    sfMx = new ExtWidget("Scale factor for overturning moment",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForOverturningMoment");
    layout->addWidget(sfMx);
    sfMy = new ExtWidget("Scale factor for rolling resistance moment",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorRollingResistanceMoment");
    layout->addWidget(sfMy);
    sfMz = new ExtWidget("Scale factor for aligning moment",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForAligningMoment");
    layout->addWidget(sfMz);
    sfs = new ExtWidget("Scale factor for moment arm of longitudinal force",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForMomentArmOfLongitudinalForce");
    layout->addWidget(sfs);
    sfmux = new ExtWidget("Scale factor for longitudinal fricition coefficient",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLongitudinalFricitionCoefficient");
    layout->addWidget(sfmux);
    sfmuy = new ExtWidget("Scale factor for lateral fricition coefficient",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLateralFricitionCoefficient");
    layout->addWidget(sfmuy);
    sfkx = new ExtWidget("Scale factor for longitudinal slip stiffness",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLongitudinalSlipStiffness");
    layout->addWidget(sfkx);
    sfky = new ExtWidget("Scale factor for cornering stiffness",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForCorneringStiffness");
    layout->addWidget(sfky);
    sfkg = new ExtWidget("Scale factor for lateral force camber stiffness",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForLateralForceCamberStiffness");
    layout->addWidget(sfkg);
    sfkm = new ExtWidget("Scale factor for aligning moment camber stiffness",new ChoiceWidget(new ScalarWidgetFactory("",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleFactorForAligningMomentCamberStiffness");
    layout->addWidget(sfkm);
    connect(inputDataFile->getWidget<FileWidget>(),&FileWidget::valueChanged,this,&MagicFormula62Widget::inputFileChanged);
  }

  void MagicFormula62Widget::inputFileChanged(const QString &fileName) {
    auto inputFile = QString::fromStdString(mw->eval->cast<MBXMLUtils::CodeString>(mw->eval->stringToValue(fileName.toStdString(),mw->getProject()->getXMLElement())));
    inputFile = inputFile.mid(1,inputFile.size()-2);
    QFile data(inputFile);
    if(data.open(QFile::ReadOnly)) {
      QTextStream fstr(&data);
      QString str, value;
      do {
	auto line = fstr.readLine();
	if(line.contains("INFLPRES")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  p->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("VERTICAL_STIFFNESS")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  cz->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("VERTICAL_DAMPING")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  dz->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LMX")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfMx->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LMY")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfMy->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LS")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfs->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LMUX")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfmux->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LMUY")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfmuy->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LKX")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfkx->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LKYC")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfkg->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LKY")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfky->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
	else if(line.contains("LKZC")) {
	  QTextStream sstr(&line);
	  sstr >> str >> str >> value;
	  sfkm->getFirstWidget<VariableWidget>()->setDefaultValue(value);
	}
      } while(not fstr.atEnd());
    }
  }

  DOMElement* MagicFormula62Widget::initializeUsingXML(DOMElement *element) {
    TyreModelWidget::initializeUsingXML(element);
    inputDataFile->getWidget<FileWidget>()->blockSignals(true);
    inputDataFile->initializeUsingXML(element);
    inputDataFile->getWidget<FileWidget>()->blockSignals(false);
    tyreSide->initializeUsingXML(element);
    mck->initializeUsingXML(element);
    cpt->initializeUsingXML(element);
    ts->initializeUsingXML(element);
    p->initializeUsingXML(element);
    cz->initializeUsingXML(element);
    dz->initializeUsingXML(element);
    six->initializeUsingXML(element);
    siy->initializeUsingXML(element);
    rtw->initializeUsingXML(element);
    sfFx->initializeUsingXML(element);
    sfFy->initializeUsingXML(element);
    sfMx->initializeUsingXML(element);
    sfMy->initializeUsingXML(element);
    sfMz->initializeUsingXML(element);
    sfs->initializeUsingXML(element);
    sfmux->initializeUsingXML(element);
    sfmuy->initializeUsingXML(element);
    sfkx->initializeUsingXML(element);
    sfky->initializeUsingXML(element);
    sfkg->initializeUsingXML(element);
    sfkm->initializeUsingXML(element);
    inputFileChanged(inputDataFile->getWidget<FileWidget>()->getFile());
    return element;
  }

  DOMElement* MagicFormula62Widget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMElement *ele0 = TyreModelWidget::writeXMLFile(parent,ref);
    inputDataFile->writeXMLFile(ele0);
    tyreSide->writeXMLFile(ele0);
    mck->writeXMLFile(ele0);
    cpt->writeXMLFile(ele0);
    ts->writeXMLFile(ele0);
    p->writeXMLFile(ele0);
    cz->writeXMLFile(ele0);
    dz->writeXMLFile(ele0);
    six->writeXMLFile(ele0);
    siy->writeXMLFile(ele0);
    rtw->writeXMLFile(ele0);
    sfFx->writeXMLFile(ele0);
    sfFy->writeXMLFile(ele0);
    sfMx->writeXMLFile(ele0);
    sfMy->writeXMLFile(ele0);
    sfMz->writeXMLFile(ele0);
    sfs->writeXMLFile(ele0);
    sfmux->writeXMLFile(ele0);
    sfmuy->writeXMLFile(ele0);
    sfkx->writeXMLFile(ele0);
    sfky->writeXMLFile(ele0);
    sfkg->writeXMLFile(ele0);
    sfkm->writeXMLFile(ele0);
    return ele0;
  }

}
