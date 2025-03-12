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
#include "object_property_dialog.h"
#include "function_widget_factory.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "ombv_widgets.h"
#include "extended_widgets.h"
#include "array_widgets.h"
#include "wizards.h"
#include "frame.h"
#include "project.h"
#include "mainwindow.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  ObjectPropertyDialog::ObjectPropertyDialog(Element *object) : ElementPropertyDialog(object) {
    addTab("Initial conditions",1);

    q0 = new ExtWidget("Generalized initial position",new ChoiceWidget(new VecWidgetFactory(0),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedInitialPosition");
    addToTab("Initial conditions", q0);

    u0 = new ExtWidget("Generalized initial velocity",new ChoiceWidget(new VecWidgetFactory(0),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedInitialVelocity");
    addToTab("Initial conditions", u0);

    connect(q0, &ExtWidget::widgetChanged, this, &ObjectPropertyDialog::updateWidget);
    connect(u0, &ExtWidget::widgetChanged, this, &ObjectPropertyDialog::updateWidget);
  }

  DOMElement* ObjectPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    q0->initializeUsingXML(item->getXMLElement());
    u0->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ObjectPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    q0->writeXMLFile(item->getXMLElement(),ref);
    u0->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  BodyPropertyDialog::BodyPropertyDialog(Element *body) : ObjectPropertyDialog(body) {
    addTab("Kinematics",1);

    R = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(body,body->getParent()->getFrame(0),this),true,false,MBSIM%"frameOfReference");
    R->getWidget<ElementOfReferenceWidget<Frame>>()->setDefaultElement("../Frame[I]");
    addToTab("Kinematics",R);
  }

  DOMElement* BodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObjectPropertyDialog::initializeUsingXML(item->getXMLElement());
    R->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* BodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObjectPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    R->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodyPropertyDialog::RigidBodyPropertyDialog(Element *body) : BodyPropertyDialog(body) {
    addTab("Visualization",3);

    K = new ExtWidget("Frame for kinematics",new LocalFrameOfReferenceWidget(body,nullptr),true,false,MBSIM%"frameForKinematics");
    addToTab("Kinematics",K);

    mass = new ExtWidget("Mass",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,massUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"mass");
    addToTab("General",mass);

    inertia = new ExtWidget("Inertia tensor",new ChoiceWidget(new SymMatWidgetFactory(getEye<QString>(3,3,"0.01","0"),vector<QStringList>(3,inertiaUnits()),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"inertiaTensor");
    addToTab("General",inertia);

    frameForInertiaTensor = new ExtWidget("Frame for inertia tensor",new LocalFrameOfReferenceWidget(body,nullptr),true,false,MBSIM%"frameForInertiaTensor");
    addToTab("General",frameForInertiaTensor);

    translation = new ExtWidget("Translation",new ChoiceWidget(new TranslationWidgetFactory(body,MBSIM,this),QBoxLayout::TopToBottom,3),true,false,"",true);
    addToTab("Kinematics", translation);
    connect(translation,&ExtWidget::widgetChanged,this,&RigidBodyPropertyDialog::updateWidget);

    rotation = new ExtWidget("Rotation",new ChoiceWidget(new RotationWidgetFactory(body,MBSIM,this),QBoxLayout::TopToBottom,3),true,false,"",true);
    addToTab("Kinematics", rotation);
    connect(rotation,&ExtWidget::widgetChanged,this,&RigidBodyPropertyDialog::updateWidget);

    translationDependentRotation = new ExtWidget("Translation dependent rotation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"translationDependentRotation");
    addToTab("Kinematics", translationDependentRotation);
    connect(translationDependentRotation,&ExtWidget::widgetChanged,this,&RigidBodyPropertyDialog::updateWidget);

    vector<QString> list;
    list.emplace_back("\"derivativeOfGeneralizedPositionOfRotation\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameOfReference\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameForKinematics\"");
    generalizedVelocityOfRotation = new ExtWidget("Generalized velocity of rotation",new TextChoiceWidget(list,0,true),true,false,MBSIM%"generalizedVelocityOfRotation");
    addToTab("Kinematics", generalizedVelocityOfRotation);

    ombv = new ExtWidget("OpenMBV body",new ChoiceWidget(new OMBVRigidBodyWidgetFactory,QBoxLayout::TopToBottom,0),true,true,MBSIM%"openMBVRigidBody");
    addToTab("Visualization", ombv);

    ombvFrameRef=new ExtWidget("OpenMBV frame of reference",new LocalFrameOfReferenceWidget(body),true,false,MBSIM%"openMBVFrameOfReference");
    addToTab("Visualization", ombvFrameRef);
  }

  DOMElement* RigidBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    BodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    K->initializeUsingXML(item->getXMLElement());
    mass->initializeUsingXML(item->getXMLElement());
    inertia->initializeUsingXML(item->getXMLElement());
    frameForInertiaTensor->initializeUsingXML(item->getXMLElement());
    translation->initializeUsingXML(item->getXMLElement());
    rotation->initializeUsingXML(item->getXMLElement());
    translationDependentRotation->initializeUsingXML(item->getXMLElement());
    generalizedVelocityOfRotation->initializeUsingXML(item->getXMLElement());
    ombv->initializeUsingXML(item->getXMLElement());
    ombvFrameRef->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    BodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    mass->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    inertia->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    frameForInertiaTensor->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translationDependentRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    generalizedVelocityOfRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    DOMElement *ele =getElement()->getXMLContours()->getNextElementSibling();
    ombv->writeXMLFile(item->getXMLElement(),ele);
    ombvFrameRef->writeXMLFile(item->getXMLElement(),ele);
    return nullptr;
  }

  int RigidBodyPropertyDialog::getqRelSize() const {
    int nqT=0, nqR=0;
    if(translation->isActive()) {
      if(translation->getWidget<ChoiceWidget>()->getIndex()!=2) {
        auto *trans = translation->getFirstWidget<FunctionWidget,true>();
        if(trans)
          nqT = trans->getArg1Size();
      }
    }
    if(rotation->isActive()) {
      if(rotation->getWidget<ChoiceWidget>()->getIndex()!=1) {
        auto *rot = rotation->getFirstWidget<FunctionWidget,true>();
        if(rot)
          nqR = rot->getArg1Size();
      }
    }
    if(translationDependentRotation->isActive() and translationDependentRotation->getWidget<ChoiceWidget>()->getIndex()==0 and translationDependentRotation->getFirstWidget<VariableWidget>()->getValue()==mw->getProject()->getVarTrue())
      return nqT;
    return nqT + nqR;
  }

  void RigidBodyPropertyDialog::resizeGeneralizedPosition() {
    q0->resize_(getqRelSize(),1);
  }

  void RigidBodyPropertyDialog::resizeGeneralizedVelocity() {
    u0->resize_(getuRelSize(),1);
  }

  GenericFlexibleFfrBodyPropertyDialog::GenericFlexibleFfrBodyPropertyDialog(Element *body) : BodyPropertyDialog(body) {

    translation = new ExtWidget("Translation",new ChoiceWidget(new TranslationWidgetFactory(body,MBSIMFLEX,this),QBoxLayout::TopToBottom,3),true,false,"",true);
    addToTab("Kinematics", translation);
    connect(translation,&ExtWidget::widgetChanged,this,&GenericFlexibleFfrBodyPropertyDialog::updateWidget);

    rotation = new ExtWidget("Rotation",new ChoiceWidget(new RotationWidgetFactory(body,MBSIMFLEX,this),QBoxLayout::TopToBottom,3),true,false,"",true);
    addToTab("Kinematics", rotation);
    connect(rotation,&ExtWidget::widgetChanged,this,&GenericFlexibleFfrBodyPropertyDialog::updateWidget);

    translationDependentRotation = new ExtWidget("Translation dependent rotation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"translationDependentRotation");
    addToTab("Kinematics", translationDependentRotation);

    vector<QString> list;
    list.emplace_back("\"derivativeOfGeneralizedPositionOfRotation\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameOfReference\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameForKinematics\"");
    generalizedVelocityOfRotation = new ExtWidget("Generalized velocity of rotation",new TextChoiceWidget(list,0,true),true,false,MBSIMFLEX%"generalizedVelocityOfRotation");
    addToTab("Kinematics", generalizedVelocityOfRotation);
  }

  int GenericFlexibleFfrBodyPropertyDialog::getqRelSize() const {
    int nqT=0, nqR=0;
    if(translation->isActive()) {
      if(translation->getWidget<ChoiceWidget>()->getIndex()!=2) {
        auto *trans = translation->getFirstWidget<FunctionWidget,true>();
        if(trans)
          nqT = trans->getArg1Size();
      }
    }
    if(rotation->isActive()) {
      if(rotation->getWidget<ChoiceWidget>()->getIndex()!=1) {
        auto *rot = rotation->getFirstWidget<FunctionWidget,true>();
        if(rot)
          nqR = rot->getArg1Size();
      }
    }
    return nqT + nqR + getqERelSize();
  }

  void GenericFlexibleFfrBodyPropertyDialog::resizeGeneralizedPosition() {
    q0->resize_(getqRelSize(),1);
  }

  void GenericFlexibleFfrBodyPropertyDialog::resizeGeneralizedVelocity() {
    u0->resize_(getuRelSize(),1);
  }

  DOMElement* GenericFlexibleFfrBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    BodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    translation->initializeUsingXML(item->getXMLElement());
    rotation->initializeUsingXML(item->getXMLElement());
    translationDependentRotation->initializeUsingXML(item->getXMLElement());
    generalizedVelocityOfRotation->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GenericFlexibleFfrBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    BodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    return nullptr;
  }

  FlexibleFfrBodyPropertyDialog::FlexibleFfrBodyPropertyDialog(Element *body) : GenericFlexibleFfrBodyPropertyDialog(body) {
    addTab("Nodal data", 4);
    addTab("Visualization",5);

    mass = new ExtWidget("Mass",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,massUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"mass");
    addToTab("General",mass);

    rdm = new ExtWidget("Position integral",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,QStringList()),vector<int>(3,0)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"positionIntegral");
    addToTab("General", rdm);

    rrdm = new ExtWidget("Position position integral",new ChoiceWidget(new SymMatWidgetFactory(getEye<QString>(3,3,"0","0"),vector<QStringList>(3,inertiaUnits()),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"positionPositionIntegral");
    addToTab("General",rrdm);

    Pdm = new ExtWidget("Shape function integral",new ChoiceWidget(new MatColsVarWidgetFactory(3,1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"shapeFunctionIntegral");
    addToTab("General",Pdm);

    rPdm = new ExtWidget("Position shape function integral array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(3,1),QBoxLayout::RightToLeft,4),"ele",3,true,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"positionShapeFunctionIntegralArray");
    addToTab("General",rPdm);

    PPdm = new ExtWidget("Shape function shape function integral array",new TwoDimensionalArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",3,3,true,true,true,true,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"shapeFunctionShapeFunctionIntegralArray");
    addToTab("General",PPdm);

    Ke = new ExtWidget("Stiffness matrix",new ChoiceWidget(new SymMatWidgetFactory(getMat<QString>(1,1,"0")),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"stiffnessMatrix");
    addToTab("General",Ke);

    De = new ExtWidget("Damping matrix",new ChoiceWidget(new SymMatWidgetFactory(getMat<QString>(1,1,"0")),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"dampingMatrix");
    addToTab("General",De);

    mDamping = new ExtWidget("Modal damping",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"modalDamping");
    addToTab("General", mDamping);

    beta = new ExtWidget("Proportional damping",new ChoiceWidget(new VecWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"proportionalDamping");
    addToTab("General", beta);

    Knl1 = new ExtWidget("Nonlinear stiffness matrix of first order array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",1,false,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrderArray");
    addToTab("General",Knl1);

    Knl2 = new ExtWidget("Nonlinear stiffness matrix of second order array",new TwoDimensionalArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",1,1,false,true,true,true,true,true,false,MBSIMFLEX),true,false,MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrderArray");
    addToTab("General",Knl2);

    ksigma0 = new ExtWidget("Initial stress integral",new ChoiceWidget(new VecWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"initialStressIntegral");
    addToTab("General", ksigma0);

    ksigma1 = new ExtWidget("Nonlinear initial stress integral",new ChoiceWidget(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"nonlinearInitialStressIntegral");
    addToTab("General", ksigma1);

    K0t = new ExtWidget("Geometric stiffness matrix due to acceleration array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",3,true,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"geometricStiffnessMatrixDueToAccelerationArray");
    addToTab("General",K0t);

    K0r = new ExtWidget("Geometric stiffness matrix due to angular acceleration array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",3,true,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAccelerationArray");
    addToTab("General",K0r);

    K0om = new ExtWidget("Geometric stiffness matrix due to angular velocity array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",3,true,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocityArray");
    addToTab("General",K0om);

    nodeNumbers = new ExtWidget("Node numbers",new ChoiceWidget(new VecWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"nodeNumbers");
    addToTab("Nodal data", nodeNumbers);

    r = new ExtWidget("Nodal relative position array",new ArrayWidget(new ChoiceWidgetFactory(new VecWidgetFactory(3),QBoxLayout::RightToLeft,4),"ele",1,false,false,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"nodalRelativePositionArray");
    addToTab("Nodal data", r);

    A = new ExtWidget("Nodal relative orientation array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(3,3),QBoxLayout::RightToLeft,4),"ele",1,false,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"nodalRelativeOrientationArray");
    addToTab("Nodal data", A);

    Phi = new ExtWidget("Nodal shape matrix of translation array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(3,1),QBoxLayout::RightToLeft,4),"ele",1,false,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"nodalShapeMatrixOfTranslationArray");
    addToTab("Nodal data", Phi);

    Psi = new ExtWidget("Nodal shape matrix of rotation array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(3,1),QBoxLayout::RightToLeft,4),"ele",1,false,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"nodalShapeMatrixOfRotationArray");
    addToTab("Nodal data", Psi);

    sigmahel = new ExtWidget("Nodal stress matrix array",new ArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(6,1),QBoxLayout::RightToLeft,4),"ele",1,false,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"nodalStressMatrixArray");
    addToTab("Nodal data", sigmahel);

    sigmahen = new ExtWidget("Nodal nonlinear stress matrix array",new TwoDimensionalArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(6,1),QBoxLayout::RightToLeft,4),"ele",1,1,false,true,true,true,true,true,false,MBSIMFLEX),true,false,MBSIMFLEX%"nodalNonlinearStressMatrixArray");
    addToTab("Nodal data", sigmahen);

    sigma0 = new ExtWidget("Nodal initial stress array",new ArrayWidget(new ChoiceWidgetFactory(new VecWidgetFactory(6),QBoxLayout::RightToLeft,4),"ele",1,false,true,true,true,MBSIMFLEX),true,false,MBSIMFLEX%"nodalInitialStressArray");
    addToTab("Nodal data", sigma0);

    K0F = new ExtWidget("Nodal geometric stiffness matrix due to force array",new TwoDimensionalArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",1,1,false,true,true,true,true,false,false,MBSIMFLEX),true,false,MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToForceArray");
    addToTab("Nodal data", K0F);

    K0M = new ExtWidget("Nodal geometric stiffness matrix due to moment array",new TwoDimensionalArrayWidget(new ChoiceWidgetFactory(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,4),"ele",1,1,false,true,true,true,true,false,false,MBSIMFLEX),true,false,MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToMomentArray");
    addToTab("Nodal data", K0M);

    ombv = new ExtWidget("OpenMBV body",new ChoiceWidget(new OMBVFlexibleBodyWidgetFactory,QBoxLayout::TopToBottom,0),true,true,MBSIMFLEX%"openMBVFlexibleBody");
    addToTab("Visualization", ombv);

    visuNodes = new ExtWidget("OpenMBV node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"openMBVNodeNumbers");
    addToTab("Visualization", visuNodes);

    vector<QString> list;
    list.emplace_back("\"none\"");
    list.emplace_back("\"xDisplacement\"");
    list.emplace_back("\"yDisplacement\"");
    list.emplace_back("\"zDisplacement\"");
    list.emplace_back("\"totalDisplacement\"");
    list.emplace_back("\"xxStress\"");
    list.emplace_back("\"yyStress\"");
    list.emplace_back("\"zzStress\"");
    list.emplace_back("\"xyStress\"");
    list.emplace_back("\"yzStress\"");
    list.emplace_back("\"zxStress\"");
    list.emplace_back("\"equivalentStress\"");
    ombvColorRepresentation = new ExtWidget("OpenMBV color representation",new TextChoiceWidget(list,0,true),true,false,MBSIMFLEX%"openMBVColorRepresentation");
    addToTab("Visualization", ombvColorRepresentation);

    plotNodes = new ExtWidget("Plot node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"plotNodeNumbers");
    addToTab("Visualization", plotNodes);

    connect(Pdm->getWidget<ChoiceWidget>(),&Widget::widgetChanged,this,&FlexibleFfrBodyPropertyDialog::updateWidget);
  }

  void FlexibleFfrBodyPropertyDialog::updateWidget() {
    GenericFlexibleFfrBodyPropertyDialog::updateWidget();
    int size = Pdm->getFirstWidget<VariableWidget>()->cols();
    rPdm->resize_(3,size);
    PPdm->resize_(size,size);
    Ke->resize_(size,size);
    De->resize_(size,size);
    if(Knl1->isActive())
      Knl1->getWidget<ArrayWidget>()->resize_(size,size,size);
    if(Knl2->isActive())
      Knl2->getWidget<TwoDimensionalArrayWidget>()->resize_(size,size,size,size);
    ksigma0->resize_(size,1);
    ksigma1->resize_(size,size);
    if(K0t->isActive())
      K0t->getWidget<ArrayWidget>()->resize_(3,size,size);
    if(K0r->isActive())
      K0r->getWidget<ArrayWidget>()->resize_(3,size,size);
    if(K0om->isActive())
      K0om->getWidget<ArrayWidget>()->resize_(3,size,size);
    if(r->isActive()) {
      int rsize = r->getWidget<ArrayWidget>()->getSize1();
      if(A->isActive())
        A->getWidget<ArrayWidget>()->resize_(rsize,3,3);
      if(Phi->isActive())
        Phi->getWidget<ArrayWidget>()->resize_(rsize,3,size);
      if(Psi->isActive())
        Psi->getWidget<ArrayWidget>()->resize_(rsize,3,size);
      if(sigmahel->isActive())
        sigmahel->getWidget<ArrayWidget>()->resize_(rsize,6,size);
      if(sigmahen->isActive())
        sigmahen->getWidget<TwoDimensionalArrayWidget>()->resize_(rsize,rsize,6,size);
      if(sigma0->isActive())
        sigma0->getWidget<ArrayWidget>()->resize_(rsize,6,1);
      if(K0F->isActive())
        K0F->getWidget<TwoDimensionalArrayWidget>()->resize_(rsize,size,size,size);
      if(K0M->isActive())
        K0M->getWidget<TwoDimensionalArrayWidget>()->resize_(rsize,size,size,size);
      if(nodeNumbers->isActive())
        nodeNumbers->resize_(rsize,1);
    }
  }

  int FlexibleFfrBodyPropertyDialog::getqERelSize() const {
    int nqE=0;
    if(Pdm->isActive())
      nqE = Pdm->getFirstWidget<VariableWidget>()->cols();
    return nqE;
  }

  DOMElement* FlexibleFfrBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GenericFlexibleFfrBodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    mass->initializeUsingXML(item->getXMLElement());
    rdm->initializeUsingXML(item->getXMLElement());
    rrdm->initializeUsingXML(item->getXMLElement());
    Pdm->initializeUsingXML(item->getXMLElement());
    rPdm->initializeUsingXML(item->getXMLElement());
    PPdm->initializeUsingXML(item->getXMLElement());
    Ke->initializeUsingXML(item->getXMLElement());
    De->initializeUsingXML(item->getXMLElement());
    mDamping->initializeUsingXML(item->getXMLElement());
    beta->initializeUsingXML(item->getXMLElement());
    Knl1->initializeUsingXML(item->getXMLElement());
    Knl2->initializeUsingXML(item->getXMLElement());
    ksigma0->initializeUsingXML(item->getXMLElement());
    ksigma1->initializeUsingXML(item->getXMLElement());
    K0t->initializeUsingXML(item->getXMLElement());
    K0r->initializeUsingXML(item->getXMLElement());
    K0om->initializeUsingXML(item->getXMLElement());
    nodeNumbers->initializeUsingXML(item->getXMLElement());
    r->initializeUsingXML(item->getXMLElement());
    A->initializeUsingXML(item->getXMLElement());
    Phi->initializeUsingXML(item->getXMLElement());
    Psi->initializeUsingXML(item->getXMLElement());
    sigmahel->initializeUsingXML(item->getXMLElement());
    sigmahen->initializeUsingXML(item->getXMLElement());
    sigma0->initializeUsingXML(item->getXMLElement());
    K0F->initializeUsingXML(item->getXMLElement());
    K0M->initializeUsingXML(item->getXMLElement());
    ombv->initializeUsingXML(item->getXMLElement());
    visuNodes->initializeUsingXML(item->getXMLElement());
    ombvColorRepresentation->initializeUsingXML(item->getXMLElement());
    plotNodes->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FlexibleFfrBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GenericFlexibleFfrBodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    mass->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rrdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Pdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rPdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    PPdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Ke->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    De->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    mDamping->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    beta->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Knl1->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Knl2->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    ksigma0->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    ksigma1->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0t->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0r->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0om->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    nodeNumbers->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    r->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    A->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Phi->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Psi->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    sigmahel->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    sigmahen->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    sigma0->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0F->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0M->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translationDependentRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    generalizedVelocityOfRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    DOMElement *ele =getElement()->getXMLContours()->getNextElementSibling();
    ombv->writeXMLFile(item->getXMLElement(),ele);
    visuNodes->writeXMLFile(item->getXMLElement(),ele);
    ombvColorRepresentation->writeXMLFile(item->getXMLElement(),ele);
    plotNodes->writeXMLFile(item->getXMLElement(),ele);
    return nullptr;
  }

  ExternalFlexibleFfrBodyPropertyDialog::ExternalFlexibleFfrBodyPropertyDialog(Element *body) : GenericFlexibleFfrBodyPropertyDialog(body) {
    addTab("Visualization",4);

    inputDataFile = new ExtWidget("Input data file name",new FileWidget("", "Open input data file", "Input data files (*.h5)", 0, true),false,false,MBSIMFLEX%"inputDataFileName");
    addToTab("General",inputDataFile);

    auto widget = new QWidget;
    auto hlayout = new QHBoxLayout;
    widget->setLayout(hlayout);
    auto label = new QLabel("Use flexible body tool to create input data:");
    hlayout->addWidget(label);
    auto button = new QPushButton(Utils::QIconCached(QString::fromStdString((MainWindow::getInstallPath()/"share"/"mbsimgui"/"icons"/"fbt.svg").string())),"Flexible body tool");
    connect(button,&QPushButton::clicked,this,[=](){ mw->flexibleBodyTool(); connect(mw->getFlexibleBodyTool(),&FlexibleBodyTool::finished,this,[=](int res) { if(res==1) inputDataFile->getWidget<FileWidget>()->setFile(mw->getFlexibleBodyTool()->getInputDataFile()); }); });

    hlayout->addWidget(button);
    addToTab("General",widget);

    ombv = new ExtWidget("Enable openMBV",new ExternalFlexibleFfrBodyMBSOMBVWidget,true,true,MBSIMFLEX%"enableOpenMBV");
    addToTab("Visualization",ombv);

    visuNodes = new ExtWidget("OpenMBV node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"openMBVNodeNumbers");
    addToTab("Visualization", visuNodes);

    plotNodes = new ExtWidget("Plot node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"plotNodeNumbers");
    addToTab("Visualization", plotNodes);
  }

  int ExternalFlexibleFfrBodyPropertyDialog::getqERelSize() const {
    int nqE=1;
    return nqE;
  }

  DOMElement* ExternalFlexibleFfrBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GenericFlexibleFfrBodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputDataFile->initializeUsingXML(item->getXMLElement());
    ombv->initializeUsingXML(item->getXMLElement());
    visuNodes->initializeUsingXML(item->getXMLElement());
    plotNodes->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ExternalFlexibleFfrBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GenericFlexibleFfrBodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    inputDataFile->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translationDependentRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    generalizedVelocityOfRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    DOMElement *ele =getElement()->getXMLContours()->getNextElementSibling();
    ombv->writeXMLFile(item->getXMLElement(),ele);
    visuNodes->writeXMLFile(item->getXMLElement(),ele);
    plotNodes->writeXMLFile(item->getXMLElement(),ele);
    return nullptr;
  }

}
