/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2024 Martin Förg

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
#include "link_property_dialog.h"
#include "function_widget_factory.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "ombv_widgets.h"
#include "frame.h"
#include "contour.h"
#include "rigid_body.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  class ConnectRigidBodiesWidgetFactory : public WidgetFactory {
    public:
      ConnectRigidBodiesWidgetFactory(Element *element_, QWidget *parent_);
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      Element *element;
      std::vector<QString> name;
      QWidget *parent;
  };

  ConnectRigidBodiesWidgetFactory::ConnectRigidBodiesWidgetFactory(Element *element_, QWidget *parent_) : element(element_), parent(parent_) {
    name.emplace_back("1 rigid body");
    name.emplace_back("2 rigid bodies");
  }

  Widget* ConnectRigidBodiesWidgetFactory::createWidget(int i) {
    return new ConnectElementsWidget<RigidBody>(i+1,element,parent);
  }

  LinkPropertyDialog::LinkPropertyDialog(Element *link) : ElementPropertyDialog(link) {
  }

  MechanicalLinkPropertyDialog::MechanicalLinkPropertyDialog(Element *link) : LinkPropertyDialog(link) {
  }

  FrameLinkPropertyDialog::FrameLinkPropertyDialog(Element *link) : MechanicalLinkPropertyDialog(link) {
    addTab("Kinetics",1);
    addTab("Visualization",2);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Frame>(2,link,this),false,false,MBSIM%"connect");
    connections->getWidget<ConnectElementsWidget<Frame>>()->setDefaultElement("../Frame[I]");
    addToTab("Kinetics", connections);
  }

  DOMElement* FrameLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FrameLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  FixedFrameLinkPropertyDialog::FixedFrameLinkPropertyDialog(Element *link) : FrameLinkPropertyDialog(link) {
  }

  FloatingFrameLinkPropertyDialog::FloatingFrameLinkPropertyDialog(Element *link) : FrameLinkPropertyDialog(link) {
    vector<QString> list;
    list.emplace_back("\"firstFrame\"");
    list.emplace_back("\"secondFrame\"");
    refFrame = new ExtWidget("Frame of reference",new TextChoiceWidget(list,0,true),true,false,MBSIM%"frameOfReference");
    addToTab("Kinetics", refFrame);
  }

  DOMElement* FloatingFrameLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    refFrame->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FloatingFrameLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    refFrame->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodyLinkPropertyDialog::RigidBodyLinkPropertyDialog(Element *link) : MechanicalLinkPropertyDialog(link) {
    addTab("Visualization",2);

    support = new ExtWidget("Support frame",new ElementOfReferenceWidget<Frame>(link,nullptr,this),true,false,MBSIM%"supportFrame");
    addToTab("General",support);
  }

  DOMElement* RigidBodyLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    support->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodyLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    support->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DualRigidBodyLinkPropertyDialog::DualRigidBodyLinkPropertyDialog(Element *link) : RigidBodyLinkPropertyDialog(link) {
    addTab("Kinetics",1);

    connections = new ExtWidget("Connections",new ChoiceWidget(new ConnectRigidBodiesWidgetFactory(link,this),QBoxLayout::RightToLeft,5),false,false,MBSIM%"connect");
    addToTab("Kinetics",connections);
  }

  DOMElement* DualRigidBodyLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DualRigidBodyLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  KineticExcitationPropertyDialog::KineticExcitationPropertyDialog(Element *kineticExcitation) : FloatingFrameLinkPropertyDialog(kineticExcitation) {

    refFrame->getWidget<TextChoiceWidget>()->setCurrentIndex(1);

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics",forceDirection);

    forceFunction = new ExtWidget("Force function",new ChoiceWidget(new Function1ArgWidgetFactory(kineticExcitation,"t",1,FunctionWidget::scalar,1,FunctionWidget::fixedVec,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"forceFunction");
    addToTab("Kinetics",forceFunction);

    momentDirection = new ExtWidget("Moment direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics",momentDirection);

    momentFunction = new ExtWidget("Moment function",new ChoiceWidget(new Function1ArgWidgetFactory(kineticExcitation,"t",1,FunctionWidget::scalar,1,FunctionWidget::fixedVec,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"momentFunction");
    addToTab("Kinetics",momentFunction);

    arrow = new ExtWidget("Enable openMBV",new InteractionArrowMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization",arrow);

    connect(forceDirection->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&KineticExcitationPropertyDialog::updateWidget);
    connect(forceFunction->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&KineticExcitationPropertyDialog::updateWidget);
    connect(momentDirection->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&KineticExcitationPropertyDialog::updateWidget);
    connect(momentFunction->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&KineticExcitationPropertyDialog::updateWidget);
    connect(forceDirection,&ExtWidget::clicked,forceFunction,&ExtWidget::setActive);
    connect(forceFunction,&ExtWidget::clicked,forceDirection,&ExtWidget::setActive);
    connect(momentDirection,&ExtWidget::clicked,momentFunction,&ExtWidget::setActive);
    connect(momentFunction,&ExtWidget::clicked,momentDirection,&ExtWidget::setActive);
  }

  void KineticExcitationPropertyDialog::updateWidget() {
    if(forceDirection->isActive()) {
      int size = forceDirection->getFirstWidget<PhysicalVariableWidget>()->cols();
      forceFunction->resize_(size,1);
    }
    if(momentDirection->isActive()) {
      int size = momentDirection->getFirstWidget<PhysicalVariableWidget>()->cols();
      momentFunction->resize_(size,1);
    }
  }

  DOMElement* KineticExcitationPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    forceFunction->initializeUsingXML(item->getXMLElement());
    momentDirection->initializeUsingXML(item->getXMLElement());
    momentFunction->initializeUsingXML(item->getXMLElement());
    arrow->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* KineticExcitationPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    forceFunction->writeXMLFile(item->getXMLElement(),ref);
    momentDirection->writeXMLFile(item->getXMLElement(),ref);
    momentFunction->writeXMLFile(item->getXMLElement(),ref);
    arrow->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  SpringDamperPropertyDialog::SpringDamperPropertyDialog(Element *springDamper) : FixedFrameLinkPropertyDialog(springDamper) {

    forceFunction = new ExtWidget("Force function",new ChoiceWidget(new SpringDamperWidgetFactory(springDamper,false,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    addToTab("Kinetics", forceFunction);

    unloadedLength = new ExtWidget("Unloaded length",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"unloadedLength");
    addToTab("General",unloadedLength);

    coilSpring = new ExtWidget("Enable openMBV",new CoilSpringMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", coilSpring);
  }

  DOMElement* SpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FixedFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceFunction->initializeUsingXML(item->getXMLElement());
    unloadedLength->initializeUsingXML(item->getXMLElement());
    coilSpring->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FixedFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceFunction->writeXMLFile(item->getXMLElement(),ref);
    unloadedLength->writeXMLFile(item->getXMLElement(),ref);
    coilSpring->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DirectionalSpringDamperPropertyDialog::DirectionalSpringDamperPropertyDialog(Element *springDamper) : FloatingFrameLinkPropertyDialog(springDamper) {

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget(new VecWidgetFactory(3),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics",forceDirection);

    forceFunction = new ExtWidget("Force function",new ChoiceWidget(new SpringDamperWidgetFactory(springDamper,false,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    addToTab("Kinetics", forceFunction);

    unloadedLength = new ExtWidget("Unloaded length",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"unloadedLength");
    addToTab("General",unloadedLength);

    coilSpring = new ExtWidget("Enable openMBV",new CoilSpringMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", coilSpring);
  }

  DOMElement* DirectionalSpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    forceFunction->initializeUsingXML(item->getXMLElement());
    unloadedLength->initializeUsingXML(item->getXMLElement());
    coilSpring->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DirectionalSpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    forceFunction->writeXMLFile(item->getXMLElement(),ref);
    unloadedLength->writeXMLFile(item->getXMLElement(),ref);
    coilSpring->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  IsotropicRotationalSpringDamperPropertyDialog::IsotropicRotationalSpringDamperPropertyDialog(Element *springDamper) : FixedFrameLinkPropertyDialog(springDamper) {

    elasticMomentFunction = new ExtWidget("Elastic moment function",new ChoiceWidget(new Function1ArgWidgetFactory(springDamper,"phi",1,FunctionWidget::varVec,1,FunctionWidget::fixedVec,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"elasticMomentFunction");
    addToTab("Kinetics", elasticMomentFunction);

    dissipativeMomentFunction = new ExtWidget("Dissipative moment function",new ChoiceWidget(new Function1ArgWidgetFactory(springDamper,"phid",1,FunctionWidget::varVec,1,FunctionWidget::fixedVec,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"dissipativeMomentFunction");
    addToTab("Kinetics", dissipativeMomentFunction);
  }

  DOMElement* IsotropicRotationalSpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FixedFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    elasticMomentFunction->initializeUsingXML(item->getXMLElement());
    dissipativeMomentFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* IsotropicRotationalSpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FixedFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    elasticMomentFunction->writeXMLFile(item->getXMLElement(),ref);
    dissipativeMomentFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  JointPropertyDialog::JointPropertyDialog(Element *joint) : FloatingFrameLinkPropertyDialog(joint) {

    addTab("Extra");

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics",forceDirection);

    forceLaw = new ExtWidget("Force law",new ChoiceWidget(new WidgetFactoryFor<GeneralizedForceLawWidget>,QBoxLayout::TopToBottom,0),true,false,MBSIM%"forceLaw");
    addToTab("Kinetics",forceLaw);

    momentDirection = new ExtWidget("Moment direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics",momentDirection);

    momentLaw = new ExtWidget("Moment law",new ChoiceWidget(new WidgetFactoryFor<GeneralizedForceLawWidget>,QBoxLayout::TopToBottom,0),true,false,MBSIM%"momentLaw");
    addToTab("Kinetics",momentLaw);

    integrate = new ExtWidget("Integrate generalized relative velocity of rotation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"integrateGeneralizedRelativeVelocityOfRotation");
    addToTab("Extra",integrate);

    vector<QString> list;
    list.emplace_back("\"smallAngles\"");
    list.emplace_back("\"cardan\"");
    angleMode = new ExtWidget("Angle mode",new TextChoiceWidget(list,0,true),true,false,MBSIM%"angleMode");
    addToTab("Extra", angleMode);

    disableAngleWarning = new ExtWidget("Disable angle warning",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"disableAngleWarning");
    addToTab("Extra", disableAngleWarning);

    connect(forceDirection,&ExtWidget::clicked,forceLaw,&ExtWidget::setActive);
    connect(forceLaw,&ExtWidget::clicked,forceDirection,&ExtWidget::setActive);
    connect(momentDirection,&ExtWidget::clicked,momentLaw,&ExtWidget::setActive);
    connect(momentLaw,&ExtWidget::clicked,momentDirection,&ExtWidget::setActive);
  }

  DOMElement* JointPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    forceLaw->initializeUsingXML(item->getXMLElement());
    momentDirection->initializeUsingXML(item->getXMLElement());
    momentLaw->initializeUsingXML(item->getXMLElement());
    integrate->initializeUsingXML(item->getXMLElement());
    angleMode->initializeUsingXML(item->getXMLElement());
    disableAngleWarning->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* JointPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    forceLaw->writeXMLFile(item->getXMLElement(),ref);
    momentDirection->writeXMLFile(item->getXMLElement(),ref);
    momentLaw->writeXMLFile(item->getXMLElement(),ref);
    integrate->writeXMLFile(item->getXMLElement(),ref);
    angleMode->writeXMLFile(item->getXMLElement(),ref);
    disableAngleWarning->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ElasticJointPropertyDialog::ElasticJointPropertyDialog(Element *joint) : FloatingFrameLinkPropertyDialog(joint) {

    addTab("Extra");

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics", forceDirection);

    momentDirection = new ExtWidget("Moment direction",new ChoiceWidget(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics", momentDirection);

    function = new ExtWidget("Generalized force function",new ChoiceWidget(new SpringDamperWidgetFactory(joint,false,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);

    integrate = new ExtWidget("Integrate generalized relative velocity of rotation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"integrateGeneralizedRelativeVelocityOfRotation");
    addToTab("Extra", integrate);

    connect(forceDirection->getWidget<ChoiceWidget>(),&ExtWidget::widgetChanged,this,&ElasticJointPropertyDialog::updateWidget);
    connect(momentDirection->getWidget<ChoiceWidget>(),&ExtWidget::widgetChanged,this,&ElasticJointPropertyDialog::updateWidget);
    connect(function->getWidget<ChoiceWidget>(),&ExtWidget::widgetChanged,this,&ElasticJointPropertyDialog::updateWidget);
    connect(forceDirection,&ExtWidget::clicked,this,&ElasticJointPropertyDialog::updateFunctionCheckState);
    connect(momentDirection,&ExtWidget::clicked,this,&ElasticJointPropertyDialog::updateFunctionCheckState);
    connect(function,&ExtWidget::clicked,this,&ElasticJointPropertyDialog::updateDirectionsCheckState);
  }

  void ElasticJointPropertyDialog::updateWidget() {
    int size = 0;
    if(forceDirection->isActive())
      size += forceDirection->getFirstWidget<PhysicalVariableWidget>()->cols();
    if(momentDirection->isActive())
      size += momentDirection->getFirstWidget<PhysicalVariableWidget>()->cols();
    function->resize_(size,1);
  }

  void ElasticJointPropertyDialog::updateFunctionCheckState() {
    function->setActive(forceDirection->isActive() or momentDirection->isActive());
  }

  void ElasticJointPropertyDialog::updateDirectionsCheckState() {
    if(function->isActive())
      forceDirection->setActive(true);
    else {
      forceDirection->setActive(false);
      momentDirection->setActive(false);
    }
  }

  DOMElement* ElasticJointPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    momentDirection->initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    integrate->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ElasticJointPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    momentDirection->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    integrate->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedSpringDamperPropertyDialog::GeneralizedSpringDamperPropertyDialog(Element *springDamper) : DualRigidBodyLinkPropertyDialog(springDamper) {

    function = new ExtWidget("Generalized force function",new ChoiceWidget(new SpringDamperWidgetFactory(springDamper,false,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);

    unloadedLength = new ExtWidget("Generalized Unloaded length",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"generalizedUnloadedLength");
    addToTab("General",unloadedLength);
  }

  DOMElement* GeneralizedSpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    DualRigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    unloadedLength->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedSpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DualRigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    unloadedLength->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedFrictionPropertyDialog::GeneralizedFrictionPropertyDialog(Element *friction) : DualRigidBodyLinkPropertyDialog(friction) {

    frictionForceLaw = new ExtWidget("Generalized friction force law",new ChoiceWidget(new WidgetFactoryFor<FrictionForceLawWidget>(friction,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedFrictionForceLaw");
    addToTab("Kinetics", frictionForceLaw);

    frictionImpactLaw = new ExtWidget("Generalized friction impact law",new ChoiceWidget(new WidgetFactoryFor<FrictionImpactLawWidget>(friction,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"generalizedFrictionImpactLaw");
    addToTab("Kinetics", frictionImpactLaw);

    normalForceFunction = new ExtWidget("Generalized normal force function",new ChoiceWidget(new Function1ArgWidgetFactory(friction,"t",1,FunctionWidget::scalar,1,FunctionWidget::fixedVec,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedNormalForceFunction");
    addToTab("Kinetics",normalForceFunction);
  }

  DOMElement* GeneralizedFrictionPropertyDialog::initializeUsingXML(DOMElement *parent) {
    DualRigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    frictionForceLaw->initializeUsingXML(item->getXMLElement());
    frictionImpactLaw->initializeUsingXML(item->getXMLElement());
    normalForceFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedFrictionPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DualRigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frictionForceLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    normalForceFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedClutchPropertyDialog::GeneralizedClutchPropertyDialog(Element *friction) : DualRigidBodyLinkPropertyDialog(friction) {

    frictionForceLaw = new ExtWidget("Generalized friction force law",new ChoiceWidget(new WidgetFactoryFor<FrictionForceLawWidget>(friction,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedFrictionForceLaw");
    addToTab("Kinetics", frictionForceLaw);

    frictionImpactLaw = new ExtWidget("Generalized friction impact law",new ChoiceWidget(new WidgetFactoryFor<FrictionImpactLawWidget>(friction,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"generalizedFrictionImpactLaw");
    addToTab("Kinetics", frictionImpactLaw);

    engagementFunction = new ExtWidget("Engagement function",new ChoiceWidget(new Function1ArgWidgetFactory(friction,"t",1,FunctionWidget::scalar,1,FunctionWidget::scalar,this),QBoxLayout::TopToBottom,0),true,true,MBSIM%"engagementFunction");
    addToTab("Kinetics",engagementFunction);

    normalForceFunction = new ExtWidget("Generalized normal force function",new ChoiceWidget(new Function1ArgWidgetFactory(friction,"t",1,FunctionWidget::scalar,1,FunctionWidget::scalar,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedNormalForceFunction");
    addToTab("Kinetics",normalForceFunction);
  }

  DOMElement* GeneralizedClutchPropertyDialog::initializeUsingXML(DOMElement *parent) {
    DualRigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    frictionForceLaw->initializeUsingXML(item->getXMLElement());
    frictionImpactLaw->initializeUsingXML(item->getXMLElement());
    engagementFunction->initializeUsingXML(item->getXMLElement());
    normalForceFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedClutchPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DualRigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frictionForceLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    engagementFunction->writeXMLFile(item->getXMLElement(),ref);
    normalForceFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedGearPropertyDialog::GeneralizedGearPropertyDialog(Element *link) : RigidBodyLinkPropertyDialog(link) {
    addTab("Kinetics",1);
    addTab("Visualization",2);

    gearOutput = new ExtWidget("Gear output",new ElementOfReferenceWidget<RigidBody>(link,nullptr,this),false,false,MBSIM%"gearOutput");
    addToTab("General",gearOutput);

    gearInput = new ExtWidget("Gear inputs",new ElementsOfReferenceWidget<RigidBody>(MBSIM%"gearInput",link,1,100,true,this),false,false,"",true);
    addToTab("General",gearInput);

    function = new ExtWidget("Generalized force law",new ChoiceWidget(new WidgetFactoryFor<GeneralizedForceLawWidget>,QBoxLayout::TopToBottom,0),true,false,MBSIM%"generalizedForceLaw");
    addToTab("Kinetics",function);
  }

  DOMElement* GeneralizedGearPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    gearOutput->initializeUsingXML(item->getXMLElement());
    gearInput->initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedGearPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    gearOutput->writeXMLFile(item->getXMLElement(),ref);
    gearInput->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedElasticConnectionPropertyDialog::GeneralizedElasticConnectionPropertyDialog(Element *connection) : DualRigidBodyLinkPropertyDialog(connection) {

    function = new ExtWidget("Generalized force function",new ChoiceWidget(new SpringDamperWidgetFactory(connection,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);

    connect(function,&ExtWidget::widgetChanged,this,&GeneralizedElasticConnectionPropertyDialog::updateWidget);
    connect(connections->getWidget<ChoiceWidget>(),&ExtWidget::widgetChanged,this,&GeneralizedElasticConnectionPropertyDialog::updateWidget);
  }

  void GeneralizedElasticConnectionPropertyDialog::updateWidget() {
  }

  DOMElement* GeneralizedElasticConnectionPropertyDialog::initializeUsingXML(DOMElement *parent) {
    DualRigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedElasticConnectionPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DualRigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedElasticStructurePropertyDialog::GeneralizedElasticStructurePropertyDialog(Element *link) : RigidBodyLinkPropertyDialog(link) {
    addTab("Kinetics",1);

    rigidBody = new ExtWidget("Rigid bodies",new ElementsOfReferenceWidget<RigidBody>(MBSIM%"rigidBody",link,1,100,true,this),false,false,"",true);
    addToTab("General",rigidBody);

    function = new ExtWidget("Generalized force function",new ChoiceWidget(new SpringDamperWidgetFactory(link,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);
  }

  DOMElement* GeneralizedElasticStructurePropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    rigidBody->initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedElasticStructurePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    rigidBody->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ContactPropertyDialog::ContactPropertyDialog(Element *contact) : LinkPropertyDialog(contact) {

    addTab("Kinetics",1);
    addTab("Extra",3);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Contour>(2,contact,this),false,false,MBSIM%"connect");
    addToTab("Kinetics", connections);

    contactForceLaw = new ExtWidget("Normal force law",new ChoiceWidget(new WidgetFactoryFor<GeneralizedForceLawWidget>,QBoxLayout::TopToBottom,0),false,false,MBSIM%"normalForceLaw");
    addToTab("Kinetics", contactForceLaw);

    contactImpactLaw = new ExtWidget("Normal impact law",new ChoiceWidget(new WidgetFactoryFor<GeneralizedImpactLawWidget>,QBoxLayout::TopToBottom,0),true,false,MBSIM%"normalImpactLaw");
    addToTab("Kinetics", contactImpactLaw);

    frictionForceLaw = new ExtWidget("Tangential force law",new ChoiceWidget(new WidgetFactoryFor<FrictionForceLawWidget>(contact,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"tangentialForceLaw");
    addToTab("Kinetics", frictionForceLaw);

    frictionImpactLaw = new ExtWidget("Tangential impact law",new ChoiceWidget(new WidgetFactoryFor<FrictionImpactLawWidget>(contact,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"tangentialImpactLaw");
    addToTab("Kinetics", frictionImpactLaw);

    globalSearch = new ExtWidget("Global search",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"globalSearch");
    addToTab("Extra", globalSearch);

    initialGlobalSearch = new ExtWidget("Initial global search",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialGlobalSearch");
    addToTab("Extra", initialGlobalSearch);

    initialGuess = new ExtWidget("Initial guess",new ChoiceWidget(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialGuess");
    addToTab("Extra", initialGuess);

    tolerance = new ExtWidget("Tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"tolerance");
    addToTab("Extra", tolerance);

    maxNumContacts = new ExtWidget("Maximum number of contacts",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumNumberOfContacts");
    addToTab("Extra", maxNumContacts);
  }

  DOMElement* ContactPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    contactForceLaw->initializeUsingXML(item->getXMLElement());
    contactImpactLaw->initializeUsingXML(item->getXMLElement());
    frictionForceLaw->initializeUsingXML(item->getXMLElement());
    frictionImpactLaw->initializeUsingXML(item->getXMLElement());
    globalSearch->initializeUsingXML(item->getXMLElement());
    initialGlobalSearch->initializeUsingXML(item->getXMLElement());
    initialGuess->initializeUsingXML(item->getXMLElement());
    tolerance->initializeUsingXML(item->getXMLElement());
    maxNumContacts->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ContactPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    contactForceLaw->writeXMLFile(item->getXMLElement(),ref);
    contactImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionForceLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    globalSearch->writeXMLFile(item->getXMLElement(),ref);
    initialGlobalSearch->writeXMLFile(item->getXMLElement(),ref);
    initialGuess->writeXMLFile(item->getXMLElement(),ref);
    tolerance->writeXMLFile(item->getXMLElement(),ref);
    maxNumContacts->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DiskContactPropertyDialog::DiskContactPropertyDialog(Element *contact) : LinkPropertyDialog(contact) {

    addTab("Kinetics",1);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Contour>(2,contact,this),false,false,MBSIM%"connect");
    addToTab("Kinetics", connections);

    contactForceLaw = new ExtWidget("Normal force law",new ChoiceWidget(new WidgetFactoryFor<GeneralizedForceLawWidget>,QBoxLayout::TopToBottom,0),false,false,MBSIM%"normalForceLaw");
    addToTab("Kinetics", contactForceLaw);

    contactImpactLaw = new ExtWidget("Normal impact law",new ChoiceWidget(new WidgetFactoryFor<GeneralizedImpactLawWidget>,QBoxLayout::TopToBottom,0),true,false,MBSIM%"normalImpactLaw");
    addToTab("Kinetics", contactImpactLaw);

    frictionForceLaw = new ExtWidget("Tangential force law",new ChoiceWidget(new WidgetFactoryFor<FrictionForceLawWidget>(contact,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"tangentialForceLaw");
    addToTab("Kinetics", frictionForceLaw);

    frictionImpactLaw = new ExtWidget("Tangential impact law",new ChoiceWidget(new WidgetFactoryFor<FrictionImpactLawWidget>(contact,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"tangentialImpactLaw");
    addToTab("Kinetics", frictionImpactLaw);
  }

  DOMElement* DiskContactPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    contactForceLaw->initializeUsingXML(item->getXMLElement());
    contactImpactLaw->initializeUsingXML(item->getXMLElement());
    frictionForceLaw->initializeUsingXML(item->getXMLElement());
    frictionImpactLaw->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DiskContactPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    contactForceLaw->writeXMLFile(item->getXMLElement(),ref);
    contactImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionForceLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  TyreContactPropertyDialog::TyreContactPropertyDialog(Element *contact) : LinkPropertyDialog(contact) {

    addTab("Kinetics",1);
    addTab("Extra",3);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Contour>(2,contact,this),false,false,MBSIM%"connect");
    addToTab("Kinetics", connections);

    model = new ExtWidget("Tyre model",new ChoiceWidget(new WidgetFactoryFor<TyreModelWidget>,QBoxLayout::TopToBottom,0),false,false,MBSIM%"tyreModel");
    addToTab("Kinetics", model);

    initialGuess = new ExtWidget("Initial guess",new ChoiceWidget(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialGuess");
    addToTab("Extra", initialGuess);

    tolerance = new ExtWidget("Tolerance",new ChoiceWidget(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"tolerance");
    addToTab("Extra", tolerance);
  }

  DOMElement* TyreContactPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    model->initializeUsingXML(item->getXMLElement());
    initialGuess->initializeUsingXML(item->getXMLElement());
    tolerance->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TyreContactPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    model->writeXMLFile(item->getXMLElement(),ref);
    initialGuess->writeXMLFile(item->getXMLElement(),ref);
    tolerance->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedInitialConditionPropertyDialog::GeneralizedInitialConditionPropertyDialog(Element *initialCondition) : LinkPropertyDialog(initialCondition) {
    object = new ExtWidget("Object of reference",new ElementOfReferenceWidget<Object>(initialCondition,nullptr,this),false,false,MBSIM%"object");
    addToTab("General", object);

    indices = new ExtWidget("Indices",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"indices");
    addToTab("General", indices);

    values = new ExtWidget("Values",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"values");
    addToTab("General", values);

    connect(indices, &ExtWidget::widgetChanged, this, &GeneralizedInitialConditionPropertyDialog::updateWidget);
    connect(values, &ExtWidget::widgetChanged, this, &GeneralizedInitialConditionPropertyDialog::updateWidget);
  }

  void GeneralizedInitialConditionPropertyDialog::updateWidget() {
    if(indices->isActive()) {
      int size = indices->getFirstWidget<PhysicalVariableWidget>()->rows();
      values->resize_(size,1);
    }
  }

  DOMElement* GeneralizedInitialConditionPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    object->initializeUsingXML(item->getXMLElement());
    indices->initializeUsingXML(item->getXMLElement());
    values->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedInitialConditionPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    object->writeXMLFile(item->getXMLElement(),ref);
    indices->writeXMLFile(item->getXMLElement(),ref);
    values->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

}
