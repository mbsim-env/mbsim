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
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "custom_widgets.h"
#include "function_widget_factory.h"
#include <QtGui>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  DOMElement* GeneralizedForceLawWidget::initializeUsingXML(DOMElement *element) {
    if(forceFunc) forceFunc->initializeUsingXML(element);
    return element;
  }

  DOMElement* GeneralizedForceLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getNameSpace()%getType().toStdString());
    parent->insertBefore(ele0, ref);
    if(forceFunc) forceFunc->writeXMLFile(ele0);
    return ele0;
  }

  RegularizedBilateralConstraintWidget::RegularizedBilateralConstraintWidget() {

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    forceFunc = new ExtWidget("Force function",new ChoiceWidget2(new RegularizedBilateralConstraintFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    layout->addWidget(forceFunc);
  }

  RegularizedUnilateralConstraintWidget::RegularizedUnilateralConstraintWidget() {

    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    forceFunc = new ExtWidget("Force function",new ChoiceWidget2(new RegularizedUnilateralConstraintFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    layout->addWidget(forceFunc);
  }

  DOMElement* GeneralizedImpactLawWidget::initializeUsingXML(DOMElement *element) {
    return element;
  }

  DOMElement* GeneralizedImpactLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getNameSpace()%getType().toStdString());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  UnilateralNewtonImpactWidget::UnilateralNewtonImpactWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    restitutionCoefficient = new ExtWidget("Restitution coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"restitutionCoefficient");
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
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getNameSpace()%getType().toStdString());
    parent->insertBefore(ele0, ref);
    if(frictionForceFunc) frictionForceFunc->writeXMLFile(ele0);
    return ele0;
  }

  PlanarCoulombFrictionWidget::PlanarCoulombFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
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
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
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

  PlanarStribeckFrictionWidget::PlanarStribeckFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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

  SpatialStribeckFrictionWidget::SpatialStribeckFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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

  RegularizedPlanarFrictionWidget::RegularizedPlanarFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionForceFunc = new ExtWidget("Friction force function",new ChoiceWidget2(new FrictionFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionForceFunction");
    layout->addWidget(frictionForceFunc);
  }

  RegularizedSpatialFrictionWidget::RegularizedSpatialFrictionWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionForceFunc = new ExtWidget("Friction force function",new ChoiceWidget2(new FrictionFunctionFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionForceFunction");
    layout->addWidget(frictionForceFunc);
  }

  DOMElement* FrictionImpactLawWidget::initializeUsingXML(DOMElement *element) {
    return element;
  }

  DOMElement* FrictionImpactLawWidget::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    DOMElement *ele0=D(doc)->createElement(getNameSpace()%getType().toStdString());
    parent->insertBefore(ele0, ref);
    return ele0;
  }

  PlanarCoulombImpactWidget::PlanarCoulombImpactWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
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
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionCoefficient = new ExtWidget("Friction coefficient",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"frictionCoefficient");
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

  PlanarStribeckImpactWidget::PlanarStribeckImpactWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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

  SpatialStribeckImpactWidget::SpatialStribeckImpactWidget() {
    QVBoxLayout *layout = new QVBoxLayout;
    setLayout(layout);
    frictionFunction = new ExtWidget("Friction function",new ChoiceWidget2(new FunctionWidgetFactory2(NULL),QBoxLayout::TopToBottom,0),false,false,MBSIM%"frictionFunction");
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

  GeneralizedForceLawWidgetFactory::GeneralizedForceLawWidgetFactory() {
    name.push_back("Bilateral constraint");
    name.push_back("Regularized bilateral constraint");
    name.push_back("Unilateral constraint");
    name.push_back("Regularized unilateral constraint");
    xmlName.push_back(MBSIM%"BilateralConstraint");
    xmlName.push_back(MBSIM%"RegularizedBilateralConstraint");
    xmlName.push_back(MBSIM%"UnilateralConstraint");
    xmlName.push_back(MBSIM%"RegularizedUnilateralConstraint");
  }

  QWidget* GeneralizedForceLawWidgetFactory::createWidget(int i) {
    if(i==0)
      return new BilateralConstraintWidget;
    if(i==1)
      return new RegularizedBilateralConstraintWidget;
    if(i==2)
      return new UnilateralConstraintWidget;
    if(i==3)
      return new RegularizedUnilateralConstraintWidget;
    return NULL;
  }

  FrictionForceLawWidgetFactory::FrictionForceLawWidgetFactory() {
    name.push_back("Planar Coulomb friction");
    name.push_back("Planar Stribeck friction");
    name.push_back("Regularized planar friction");
    name.push_back("Spatial Coulomb friction");
    name.push_back("Spatial Stribeck friction");
    name.push_back("Regularized spatial friction");
    xmlName.push_back(MBSIM%"PlanarCoulombFriction");
    xmlName.push_back(MBSIM%"PlanarStribeckFriction");
    xmlName.push_back(MBSIM%"RegularizedPlanarFriction");
    xmlName.push_back(MBSIM%"SpatialCoulombFriction");
    xmlName.push_back(MBSIM%"SpatialStribeckFriction");
    xmlName.push_back(MBSIM%"RegularizedSpatialFriction");
  }

  QWidget* FrictionForceLawWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PlanarCoulombFrictionWidget;
    if(i==1)
      return new PlanarStribeckFrictionWidget;
    if(i==2)
      return new RegularizedPlanarFrictionWidget;
    if(i==3)
      return new SpatialCoulombFrictionWidget;
    if(i==4)
      return new SpatialStribeckFrictionWidget;
    if(i==5)
      return new RegularizedSpatialFrictionWidget;
    return NULL;
  }

  GeneralizedImpactLawWidgetFactory::GeneralizedImpactLawWidgetFactory() {
    name.push_back("Bilateral impact");
    name.push_back("Unilateral Newton impact");
    xmlName.push_back(MBSIM%"BilateralImpact");
    xmlName.push_back(MBSIM%"UnilateralNewtonImpact");
  }

  QWidget* GeneralizedImpactLawWidgetFactory::createWidget(int i) {
    if(i==0)
      return new BilateralImpactWidget;
    if(i==1)
      return new UnilateralNewtonImpactWidget;
    return NULL;
  }

  FrictionImpactLawWidgetFactory::FrictionImpactLawWidgetFactory() {
    name.push_back("Planar Coulomb impact");
    name.push_back("Planar Stribeck impact");
    name.push_back("Spatial Coulomb impact");
    name.push_back("Spatial Stribeck impact");
    xmlName.push_back(MBSIM%"PlanarCoulombImpact");
    xmlName.push_back(MBSIM%"PlanarStribeckImpact");
    xmlName.push_back(MBSIM%"SpatialCoulombImpact");
    xmlName.push_back(MBSIM%"SpatialStribeckImpact");
  }

  QWidget* FrictionImpactLawWidgetFactory::createWidget(int i) {
    if(i==0)
      return new PlanarCoulombImpactWidget;
    if(i==1)
      return new PlanarStribeckImpactWidget;
    if(i==2)
      return new SpatialCoulombImpactWidget;
    if(i==3)
      return new SpatialStribeckImpactWidget;
    return NULL;
  }

  RegularizedBilateralConstraintFunctionFactory::RegularizedBilateralConstraintFunctionFactory() {
    name.push_back("Linear regularized bilateral constraint");
    xmlName.push_back(MBSIM%"LinearRegularizedBilateralConstraint");
  }

  QWidget* RegularizedBilateralConstraintFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedBilateralConstraintWidget;
    return NULL;
  }

  RegularizedUnilateralConstraintFunctionFactory::RegularizedUnilateralConstraintFunctionFactory() {
    name.push_back("Linear regularized unilateral constraint");
    xmlName.push_back(MBSIM%"LinearRegularizedUnilateralConstraint");
  }

  QWidget* RegularizedUnilateralConstraintFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedUnilateralConstraintWidget;
    return NULL;
  }

  FrictionFunctionFactory::FrictionFunctionFactory() {
    name.push_back("Linear regularized Coulomb friction");
    name.push_back("Linear regularized Stribeck friction");
    name.push_back("Symbolic function");
    xmlName.push_back(MBSIM%"LinearRegularizedCoulombFriction");
    xmlName.push_back(MBSIM%"LinearRegularizedStribeckFriction");
    xmlName.push_back(MBSIM%"SymbolicFunction");
  }

  QWidget* FrictionFunctionFactory::createWidget(int i) {
    if(i==0)
      return new LinearRegularizedCoulombFrictionWidget;
    if(i==1)
      return new LinearRegularizedStribeckFrictionWidget;
    if(i==2) {
      QStringList var;
      var << "gd" << "laN";
      return new SymbolicFunctionWidget(var,1,1);
    }
    return NULL;
  }

}
