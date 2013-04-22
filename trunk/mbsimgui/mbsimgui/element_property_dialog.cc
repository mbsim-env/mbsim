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
#include "element_property_dialog.h"
#include "basic_widgets.h"
#include "string_widgets.h"
#include "kinematics_widgets.h"
#include "kinetics_widgets.h"
#include "function_widgets.h"
#include "ombv_widgets.h"
#include "integrator_widgets.h"
#include "extended_widgets.h"
#include "solver.h"
#include "frame.h"
#include "contour.h"
#include "rigidbody.h"
#include "constraint.h"
#include "embedded_elements.h"
#include "kinetic_excitation.h"
#include "spring_damper.h"
#include "joint.h"
#include "contact.h"
#include "observer.h"
#include "parameter.h"
#include "integrator.h"
#include <QPushButton>

using namespace std;

ElementPropertyDialog::ElementPropertyDialog(QWidget *parent, Qt::WindowFlags f) : PropertyDialog(parent,f) {
  addTab("General");
  name = new ExtWidget("Name",new TextWidget);
  name->setToolTip("Set the name of the element");
  addToTab("General", name);
}

void ElementPropertyDialog::toWidget(Element *element) {
  element->name.toWidget(name);
}

void ElementPropertyDialog::fromWidget(Element *element) {
  element->name.fromWidget(name);
}

FramePropertyDialog::FramePropertyDialog(Frame *frame, QWidget *parent, Qt::WindowFlags f) : ElementPropertyDialog(parent,f) {
  addTab("Visualisation");
  visu = new ExtWidget("OpenMBV frame",new OMBVFrameWidget("NOTSET"),true,true);
  visu->setToolTip("Set the visualisation parameters for the frame");
  addToTab("Visualisation", visu);
}

void FramePropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
  static_cast<Frame*>(element)->visu.toWidget(visu);
}

void FramePropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
  static_cast<Frame*>(element)->visu.fromWidget(visu);
}

FixedRelativeFramePropertyDialog::FixedRelativeFramePropertyDialog(FixedRelativeFrame *frame, QWidget *parent, Qt::WindowFlags f) : FramePropertyDialog(frame,parent,f) {
  addTab("Kinematics",1);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(3), lengthUnits(), 4));
  position = new ExtWidget("Relative position", new ExtPhysicalVarWidget(input),true);
  addToTab("Kinematics", position);

  input.clear();
  input.push_back(new PhysicalStringWidget(new MatWidget(getEye<string>(3,3,"1","0")),noUnitUnits(),1));
  orientation = new ExtWidget("Relative orientation",new ExtPhysicalVarWidget(input),true);
  addToTab("Kinematics", orientation);

  refFrame = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(frame,frame),true);
  addToTab("Kinematics", refFrame);
}

void FixedRelativeFramePropertyDialog::toWidget(Element *element) {
  FramePropertyDialog::toWidget(element);
  static_cast<FixedRelativeFrame*>(element)->position.toWidget(position);
  static_cast<FixedRelativeFrame*>(element)->orientation.toWidget(orientation);
  static_cast<FixedRelativeFrame*>(element)->refFrame.toWidget(refFrame);
}

void FixedRelativeFramePropertyDialog::fromWidget(Element *element) {
  FramePropertyDialog::fromWidget(element);
  static_cast<FixedRelativeFrame*>(element)->position.fromWidget(position);
  static_cast<FixedRelativeFrame*>(element)->orientation.fromWidget(orientation);
  static_cast<FixedRelativeFrame*>(element)->refFrame.fromWidget(refFrame);
}

PlanePropertyDialog::PlanePropertyDialog(Plane *plane, QWidget *parent, Qt::WindowFlags f) : ContourPropertyDialog(plane,parent,f) {
  addTab("Visualisation");
 
  visu = new ExtWidget("OpenMBV Plane",new OMBVPlaneWidget,true);
  addToTab("Visualisation", visu);
}

void PlanePropertyDialog::toWidget(Element *element) {
  ContourPropertyDialog::toWidget(element);
  static_cast<Plane*>(element)->visu.toWidget(visu);
}

void PlanePropertyDialog::fromWidget(Element *element) {
  ContourPropertyDialog::fromWidget(element);
  static_cast<Plane*>(element)->visu.fromWidget(visu);
}

SpherePropertyDialog::SpherePropertyDialog(Sphere *sphere, QWidget *parent, Qt::WindowFlags f) : ContourPropertyDialog(sphere,parent,f) {
  addTab("Visualisation");
 
  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"), lengthUnits(), 4));
  radius = new ExtWidget("Radius",new ExtPhysicalVarWidget(input));
  addToTab("General", radius);

  visu = new ExtWidget("OpenMBV Sphere",new OMBVEmptyWidget,true);
  addToTab("Visualisation", visu);
}

void SpherePropertyDialog::toWidget(Element *element) {
  ContourPropertyDialog::toWidget(element);
  static_cast<Sphere*>(element)->radius.toWidget(radius);
  static_cast<Sphere*>(element)->visu.toWidget(visu);
}

void SpherePropertyDialog::fromWidget(Element *element) {
  ContourPropertyDialog::fromWidget(element);
  static_cast<Sphere*>(element)->radius.fromWidget(radius);
  static_cast<Sphere*>(element)->visu.fromWidget(visu);
}

GroupPropertyDialog::GroupPropertyDialog(Group *group, QWidget *parent, Qt::WindowFlags f, bool kinematics) : ElementPropertyDialog(parent,f), position(0), orientation(0), frameOfReference(0) {
  if(kinematics) {
    addTab("Kinematics");

    vector<PhysicalStringWidget*> input;
    input.push_back(new PhysicalStringWidget(new VecWidget(3),lengthUnits(),4));
    position = new ExtWidget("Position",new ExtPhysicalVarWidget(input),true); 
    addToTab("Kinematics", position);

    input.clear();
    input.push_back(new PhysicalStringWidget(new MatWidget(getEye<string>(3,3,"1","0")),noUnitUnits(),1));
    orientation = new ExtWidget("Orientation",new ExtPhysicalVarWidget(input),true); 
    addToTab("Kinematics", orientation);

    frameOfReference = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(group,0),true);
    addToTab("Kinematics", frameOfReference);
  }
}

void GroupPropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
  if(position) {
    static_cast<Group*>(element)->position.toWidget(position);
    static_cast<Group*>(element)->orientation.toWidget(orientation);
    static_cast<Group*>(element)->frameOfReference.toWidget(frameOfReference);
  }
}

void GroupPropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
  if(position) {
    static_cast<Group*>(element)->position.fromWidget(position);
    static_cast<Group*>(element)->orientation.fromWidget(orientation);
    static_cast<Group*>(element)->frameOfReference.fromWidget(frameOfReference);
  }
}

SolverPropertyDialog::SolverPropertyDialog(Solver *solver, QWidget *parent, Qt::WindowFlags f) : GroupPropertyDialog(solver,parent,f,false) {
  addTab("Environment");
  addTab("Solver parameters");
  addTab("Extra");

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new VecWidget(vector<string>(3)),accelerationUnits(),0));
  environment = new ExtWidget("Acceleration of gravity",new ExtPhysicalVarWidget(input));
  addToTab("Environment", environment);

  solverParameters = new ExtWidget("Solver parameters",new SolverParametersWidget,true); 
  addToTab("Solver parameters",solverParameters);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("1"),QStringList(),1));
  inverseKinetics = new ExtWidget("Inverse kinetics",new ExtPhysicalVarWidget(input),true); 
  addToTab("Extra", inverseKinetics);
}

void SolverPropertyDialog::toWidget(Element *element) {
  GroupPropertyDialog::toWidget(element);
  static_cast<Solver*>(element)->environment.toWidget(environment);
  static_cast<Solver*>(element)->solverParameters.toWidget(solverParameters);
  static_cast<Solver*>(element)->inverseKinetics.toWidget(inverseKinetics);
}

void SolverPropertyDialog::fromWidget(Element *element) {
  GroupPropertyDialog::fromWidget(element);
  static_cast<Solver*>(element)->environment.fromWidget(environment);
  static_cast<Solver*>(element)->solverParameters.fromWidget(solverParameters);
  static_cast<Solver*>(element)->inverseKinetics.fromWidget(inverseKinetics);
}

ObjectPropertyDialog::ObjectPropertyDialog(Object *object, QWidget *parent, Qt::WindowFlags f) : ElementPropertyDialog(parent,f) {
  addTab("Initial conditions");
  vector<PhysicalStringWidget*> input;
  q0_ = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(q0_,QStringList(),1));
  ExtPhysicalVarWidget *var = new ExtPhysicalVarWidget(input);  
  q0 = new ExtWidget("Initial generalized position",var,true);
  addToTab("Initial conditions", q0);

  input.clear();
  u0_ = new VecWidget(0);
  input.push_back(new PhysicalStringWidget(u0_,QStringList(),1));
  var = new ExtPhysicalVarWidget(input);  
  u0 = new ExtWidget("Initial generalized velocity",var,true);
  addToTab("Initial conditions", u0);

  connect(buttonResize, SIGNAL(clicked(bool)), this, SLOT(resizeVariables()));
}

void ObjectPropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
  static_cast<Object*>(element)->q0.toWidget(q0);
  static_cast<Object*>(element)->u0.toWidget(u0);
}

void ObjectPropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
  static_cast<Object*>(element)->q0.fromWidget(q0);
  static_cast<Object*>(element)->u0.fromWidget(u0);
}

BodyPropertyDialog::BodyPropertyDialog(Body *body, QWidget *parent, Qt::WindowFlags f) : ObjectPropertyDialog(body,parent,f) {
  addTab("Kinematics");
  R = new ExtWidget("Frame of reference",new FrameOfReferenceWidget(body,0),true);
  addToTab("Kinematics",R);
}

void BodyPropertyDialog::toWidget(Element *element) {
  ObjectPropertyDialog::toWidget(element);
  static_cast<Body*>(element)->R.toWidget(R);
}

void BodyPropertyDialog::fromWidget(Element *element) {
  ObjectPropertyDialog::fromWidget(element);
  static_cast<Body*>(element)->R.fromWidget(R);
}

RigidBodyPropertyDialog::RigidBodyPropertyDialog(RigidBody *body_, QWidget *parent, Qt::WindowFlags f) : BodyPropertyDialog(body_,parent,f), body(body_) {
  addTab("Visualisation");
  addTab("Extra");

  K = new ExtWidget("Frame for kinematics",new LocalFrameOfReferenceWidget(body,0),true);
  addToTab("Kinematics",K);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("1"),massUnits(),2));
  mass = new ExtWidget("Mass",new ExtPhysicalVarWidget(input));
  addToTab("General", mass);

  input.clear();
  input.push_back(new PhysicalStringWidget(new SymMatWidget(getEye<string>(3,3,"0.01","0")),inertiaUnits(),2));
  inertia = new ExtWidget("Inertia tensor",new ExtPhysicalVarWidget(input));
  addToTab("General", inertia);

  TranslationChoiceWidget *translation_ = new TranslationChoiceWidget("");
  translation = new ExtWidget("Translation",translation_,true);
  addToTab("Kinematics", translation);
  connect(translation_,SIGNAL(translationChanged()),this,SLOT(resizeVariables()));
  connect(translation,SIGNAL(resize()),this,SLOT(resizeVariables()));

  RotationChoiceWidget *rotation_ = new RotationChoiceWidget("");
  rotation = new ExtWidget("Rotation",rotation_,true);
  addToTab("Kinematics", rotation);
  connect(rotation_,SIGNAL(rotationChanged()),this,SLOT(resizeVariables()));
  connect(rotation,SIGNAL(resize()),this,SLOT(resizeVariables()));

  ombvEditor = new ExtWidget("OpenMBV body",new OMBVBodySelectionWidget(body),true);
  addToTab("Visualisation", ombvEditor);

  weightArrow = new ExtWidget("OpenMBV weight arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",weightArrow);

  jointForceArrow = new ExtWidget("OpenMBV joint force arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",jointForceArrow);

  jointMomentArrow = new ExtWidget("OpenMBV joint moment arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",jointMomentArrow);

  input.clear();
  input.push_back(new PhysicalStringWidget(new BoolWidget("0"),QStringList(),1));
  isFrameOfBodyForRotation = new ExtWidget("Use body frame for rotation",new ExtPhysicalVarWidget(input),true); 
  addToTab("Extra", isFrameOfBodyForRotation);
}

void RigidBodyPropertyDialog::toWidget(Element *element) {
  BodyPropertyDialog::toWidget(element);
  static_cast<RigidBody*>(element)->K.toWidget(K);
  static_cast<RigidBody*>(element)->mass.toWidget(mass);
  static_cast<RigidBody*>(element)->inertia.toWidget(inertia);
  static_cast<RigidBody*>(element)->translation.toWidget(translation);
  static_cast<RigidBody*>(element)->rotation.toWidget(rotation);
  static_cast<RigidBody*>(element)->ombvEditor.toWidget(ombvEditor);
  static_cast<RigidBody*>(element)->weightArrow.toWidget(weightArrow);
  static_cast<RigidBody*>(element)->jointForceArrow.toWidget(jointForceArrow);
  static_cast<RigidBody*>(element)->jointMomentArrow.toWidget(jointMomentArrow);
  static_cast<RigidBody*>(element)->isFrameOfBodyForRotation.toWidget(isFrameOfBodyForRotation);
}

void RigidBodyPropertyDialog::fromWidget(Element *element) {
  BodyPropertyDialog::fromWidget(element);
  static_cast<RigidBody*>(element)->K.fromWidget(K);
  static_cast<RigidBody*>(element)->mass.fromWidget(mass);
  static_cast<RigidBody*>(element)->inertia.fromWidget(inertia);
  static_cast<RigidBody*>(element)->translation.fromWidget(translation);
  static_cast<RigidBody*>(element)->rotation.fromWidget(rotation);
  static_cast<RigidBody*>(element)->ombvEditor.fromWidget(ombvEditor);
  static_cast<RigidBody*>(element)->weightArrow.fromWidget(weightArrow);
  static_cast<RigidBody*>(element)->jointForceArrow.fromWidget(jointForceArrow);
  static_cast<RigidBody*>(element)->jointMomentArrow.fromWidget(jointMomentArrow);
  static_cast<RigidBody*>(element)->isFrameOfBodyForRotation.fromWidget(isFrameOfBodyForRotation);
}

int RigidBodyPropertyDialog::getSize() const {
  return (translation->isActive()?((TranslationChoiceWidget*)translation->getWidget())->getSize():0) + (rotation->isActive()?((RotationChoiceWidget*)rotation->getWidget())->getSize():0);
}

void RigidBodyPropertyDialog::resizeGeneralizedPosition() {
  int size =  body->isConstrained() ? 0 : getSize();
  if(q0_ && q0_->size() != size)
    q0_->resize(size);
}

void RigidBodyPropertyDialog::resizeGeneralizedVelocity() {
  int size = getSize();
  if(u0_ && u0_->size() != size)
    u0_->resize(size);
}

ConstraintPropertyDialog::ConstraintPropertyDialog(Constraint *constraint, QWidget *parent, Qt::WindowFlags f) : ObjectPropertyDialog(constraint,parent,f) {
}

GearConstraintPropertyDialog::GearConstraintPropertyDialog(GearConstraint *constraint, QWidget *parent, Qt::WindowFlags f) : ConstraintPropertyDialog(constraint,parent,f), refBody(0) {

  dependentBody = new ExtWidget("Dependent body",new RigidBodyOfReferenceWidget(constraint,0));
  connect((RigidBodyOfReferenceWidget*)dependentBody->getWidget(),SIGNAL(bodyChanged()),this,SLOT(updateReferenceBody()));
  addToTab("General", dependentBody);

  independentBodies = new ExtWidget("Independent bodies",new GearDependenciesWidget(constraint));
  //connect(dependentBodiesFirstSide_,SIGNAL(bodyChanged()),this,SLOT(resizeVariables()));
  addToTab("General", independentBodies);

  connect(buttonResize, SIGNAL(clicked(bool)), this, SLOT(resizeVariables()));
}

void GearConstraintPropertyDialog::updateReferenceBody() {
  if(refBody)
    refBody->setConstrained(false);
  refBody = ((RigidBodyOfReferenceWidget*)dependentBody->getWidget())->getBody();
  if(refBody)
    refBody->setConstrained(true);
}

void GearConstraintPropertyDialog::toWidget(Element *element) {
  ConstraintPropertyDialog::toWidget(element);
  static_cast<GearConstraint*>(element)->dependentBody.toWidget(dependentBody);
  static_cast<GearConstraint*>(element)->independentBodies.toWidget(independentBodies);
}

void GearConstraintPropertyDialog::fromWidget(Element *element) {
  ConstraintPropertyDialog::fromWidget(element);
  static_cast<GearConstraint*>(element)->dependentBody.fromWidget(dependentBody);
  static_cast<GearConstraint*>(element)->independentBodies.fromWidget(independentBodies);
}

KinematicConstraintPropertyDialog::KinematicConstraintPropertyDialog(KinematicConstraint *constraint, QWidget *parent, Qt::WindowFlags f) : ConstraintPropertyDialog(constraint,parent,f), refBody(0) {

  dependentBody = new ExtWidget("Dependent body",new RigidBodyOfReferenceWidget(constraint,0));
  connect((RigidBodyOfReferenceWidget*)dependentBody->getWidget(),SIGNAL(bodyChanged()),this,SLOT(updateReferenceBody()));
  addToTab("General", dependentBody);

  kinematicFunction = new ExtWidget("Kinematic function",new Function1ChoiceWidget,true);
  addToTab("General", kinematicFunction);
  connect((Function1ChoiceWidget*)kinematicFunction->getWidget(),SIGNAL(resize()),this,SLOT(resizeVariables()));

  firstDerivativeOfKinematicFunction = new ExtWidget("First derivative of kinematic function",new Function1ChoiceWidget(MBSIMNS"firstDerivativeOfKinematicFunction"),true);
  addToTab("General", firstDerivativeOfKinematicFunction);
  connect((Function1ChoiceWidget*)firstDerivativeOfKinematicFunction->getWidget(),SIGNAL(resize()),this,SLOT(resizeVariables()));

  secondDerivativeOfKinematicFunction = new ExtWidget("Second derivative of kinematic function",new Function1ChoiceWidget(MBSIMNS"secondDerivativeOfKinematicFunction"),true);
  addToTab("General", secondDerivativeOfKinematicFunction);
  connect((Function1ChoiceWidget*)secondDerivativeOfKinematicFunction->getWidget(),SIGNAL(resize()),this,SLOT(resizeVariables()));

  connect(buttonResize, SIGNAL(clicked(bool)), this, SLOT(resizeVariables()));
}

void KinematicConstraintPropertyDialog::resizeVariables() {
  int size = refBody?refBody->getqRelSize():0;
  ((Function1ChoiceWidget*)kinematicFunction->getWidget())->resize(size,1);
  ((Function1ChoiceWidget*)firstDerivativeOfKinematicFunction->getWidget())->resize(size,1);
  ((Function1ChoiceWidget*)secondDerivativeOfKinematicFunction->getWidget())->resize(size,1);
}

void KinematicConstraintPropertyDialog::updateReferenceBody() {
  if(refBody)
    refBody->setConstrained(false);
  refBody = ((RigidBodyOfReferenceWidget*)dependentBody->getWidget())->getBody();
  if(refBody) {
    refBody->setConstrained(true);
    resizeVariables();
  }
}

void KinematicConstraintPropertyDialog::toWidget(Element *element) {
  ConstraintPropertyDialog::toWidget(element);
  static_cast<KinematicConstraint*>(element)->dependentBody.toWidget(dependentBody);
  static_cast<KinematicConstraint*>(element)->kinematicFunction.toWidget(kinematicFunction);
  static_cast<KinematicConstraint*>(element)->firstDerivativeOfKinematicFunction.toWidget(firstDerivativeOfKinematicFunction);
  static_cast<KinematicConstraint*>(element)->secondDerivativeOfKinematicFunction.toWidget(secondDerivativeOfKinematicFunction);
}

void KinematicConstraintPropertyDialog::fromWidget(Element *element) {
  ConstraintPropertyDialog::fromWidget(element);
  static_cast<KinematicConstraint*>(element)->dependentBody.fromWidget(dependentBody);
  static_cast<KinematicConstraint*>(element)->kinematicFunction.fromWidget(kinematicFunction);
  static_cast<KinematicConstraint*>(element)->firstDerivativeOfKinematicFunction.fromWidget(firstDerivativeOfKinematicFunction);
  static_cast<KinematicConstraint*>(element)->secondDerivativeOfKinematicFunction.fromWidget(secondDerivativeOfKinematicFunction);
}

JointConstraintPropertyDialog::JointConstraintPropertyDialog(JointConstraint *constraint, QWidget *parent, Qt::WindowFlags f) : ConstraintPropertyDialog(constraint,parent,f) {

  addTab("Kinetics",1);

  independentBody = new ExtWidget("Independent body",new RigidBodyOfReferenceWidget(constraint,0));
  addToTab("General", independentBody);

  DependenciesWidget *dependentBodiesFirstSide_ = new DependenciesWidget(constraint);
  dependentBodiesFirstSide = new ExtWidget("Dependendent bodies first side",dependentBodiesFirstSide_);
  addToTab("General", dependentBodiesFirstSide);
  connect(dependentBodiesFirstSide_,SIGNAL(bodyChanged()),this,SLOT(resizeVariables()));

  DependenciesWidget *dependentBodiesSecondSide_ = new DependenciesWidget(constraint);
  dependentBodiesSecondSide = new ExtWidget("Dependendent bodies second side",dependentBodiesSecondSide_);
  addToTab("General", dependentBodiesSecondSide);
  connect(dependentBodiesSecondSide_,SIGNAL(bodyChanged()),this,SLOT(resizeVariables()));

  connections = new ExtWidget("Connections",new ConnectFramesWidget(2,constraint));
  addToTab("Kinetics", connections);

  force = new ExtWidget("Force",new GeneralizedForceDirectionWidget,true);
  addToTab("Kinetics", force);

  moment = new ExtWidget("Moment",new GeneralizedForceDirectionWidget,true);
  addToTab("Kinetics", moment);
 }

void JointConstraintPropertyDialog::resizeGeneralizedPosition() {
  int size = 0;
  for(int i=0; i<((DependenciesWidget*)dependentBodiesFirstSide->getWidget())->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesFirstSide->getWidget())->getBody(i))
    size += ((DependenciesWidget*)dependentBodiesFirstSide->getWidget())->getBody(i)->getqRelSize();
  for(int i=0; i<((DependenciesWidget*)dependentBodiesSecondSide->getWidget())->getSize(); i++)
    if(((DependenciesWidget*)dependentBodiesSecondSide->getWidget())->getBody(i))
      size += ((DependenciesWidget*)dependentBodiesSecondSide->getWidget())->getBody(i)->getqRelSize();
  if(q0_->size() != size)
    q0_->resize(size);
}

void JointConstraintPropertyDialog::toWidget(Element *element) {
  ConstraintPropertyDialog::toWidget(element);
  static_cast<JointConstraint*>(element)->independentBody.toWidget(independentBody);
  static_cast<JointConstraint*>(element)->dependentBodiesFirstSide.toWidget(dependentBodiesFirstSide);
  static_cast<JointConstraint*>(element)->dependentBodiesSecondSide.toWidget(dependentBodiesSecondSide);
  static_cast<JointConstraint*>(element)->connections.toWidget(connections);
  static_cast<JointConstraint*>(element)->force.toWidget(force);
  static_cast<JointConstraint*>(element)->moment.toWidget(moment);
}

void JointConstraintPropertyDialog::fromWidget(Element *element) {
  ConstraintPropertyDialog::fromWidget(element);
  static_cast<JointConstraint*>(element)->independentBody.fromWidget(independentBody);
  static_cast<JointConstraint*>(element)->dependentBodiesFirstSide.fromWidget(dependentBodiesFirstSide);
  static_cast<JointConstraint*>(element)->dependentBodiesSecondSide.fromWidget(dependentBodiesSecondSide);
  static_cast<JointConstraint*>(element)->connections.fromWidget(connections);
  static_cast<JointConstraint*>(element)->force.fromWidget(force);
  static_cast<JointConstraint*>(element)->moment.fromWidget(moment);
}

EmbeddedObjectPropertyDialog::EmbeddedObjectPropertyDialog(EmbeddedObject *object, QWidget *parent, Qt::WindowFlags f) : ObjectPropertyDialog(object,parent,f) {
  href = new ExtWidget("XML object file",new FileWidget("XML model files", "xml files (*.xml)"));
  connect(static_cast<FileWidget*>(href->getWidget()),SIGNAL(fileChanged(const QString&)),static_cast<TextWidget*>(name->getWidget()),SLOT(setText(const QString&)));
  addToTab("General",href);
  count = new ExtWidget("Count", new TextWidget, true);
  addToTab("General",count);
  counterName = new ExtWidget("Counter name", new TextWidget, true);
  addToTab("General",counterName);
  parameterList = new ExtWidget("XML parameter file", new FileWidget("XML model files", "xml files (*.mbsimparam.xml)"), true);
  addToTab("General",parameterList);
}

void EmbeddedObjectPropertyDialog::toWidget(Element *element) {
  ObjectPropertyDialog::toWidget(element);
  static_cast<EmbeddedObject*>(element)->href.toWidget(href);
  static_cast<EmbeddedObject*>(element)->count.toWidget(count);
  static_cast<EmbeddedObject*>(element)->counterName.toWidget(counterName);
  static_cast<EmbeddedObject*>(element)->parameterList.toWidget(parameterList);
}

void EmbeddedObjectPropertyDialog::fromWidget(Element *element) {
  ObjectPropertyDialog::fromWidget(element);
  static_cast<EmbeddedObject*>(element)->href.fromWidget(href);
  static_cast<EmbeddedObject*>(element)->count.fromWidget(count);
  static_cast<EmbeddedObject*>(element)->counterName.fromWidget(counterName);
  static_cast<EmbeddedObject*>(element)->parameterList.fromWidget(parameterList);
}

LinkPropertyDialog::LinkPropertyDialog(Link *link, QWidget *parent, Qt::WindowFlags f) : ElementPropertyDialog(parent,f) {
}

void LinkPropertyDialog::toWidget(Element *element) {
  ElementPropertyDialog::toWidget(element);
}

void LinkPropertyDialog::fromWidget(Element *element) {
  ElementPropertyDialog::fromWidget(element);
}

KineticExcitationPropertyDialog::KineticExcitationPropertyDialog(KineticExcitation *kineticExcitation, QWidget *parent, Qt::WindowFlags wf) : LinkPropertyDialog(kineticExcitation,parent,wf) {

  addTab("Kinetics");
  addTab("Visualisation");

  forceArrow = new ExtWidget("OpenMBV force arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",forceArrow);

  momentArrow = new ExtWidget("OpenMBV moment arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",momentArrow);

  vector<QWidget*> widget;
  vector<string> name;
  name.push_back("1 frame");
  name.push_back("2 frames");
  widget.push_back(new ConnectFramesWidget(1,kineticExcitation));
  widget.push_back(new ConnectFramesWidget(2,kineticExcitation));

  connections = new ExtWidget("Connections",new WidgetChoiceWidget(name,widget)); 
  addToTab("Kinetics",connections);

  ForceChoiceWidget *f = new ForceChoiceWidget;
  connect(buttonResize, SIGNAL(clicked(bool)), f, SLOT(resizeVariables()));
  force = new ExtWidget("Force",f,true);
  addToTab("Kinetics",force);

  ForceChoiceWidget *m = new ForceChoiceWidget;
  moment = new ExtWidget("Moment",m,true);
  addToTab("Kinetics",moment);

  FrameOfReferenceWidget* ref = new FrameOfReferenceWidget(kineticExcitation,0);
  frameOfReference = new ExtWidget("Frame of reference",ref,true);
  addToTab("Kinetics",frameOfReference);
}

void KineticExcitationPropertyDialog::toWidget(Element *element) {
  LinkPropertyDialog::toWidget(element);
  static_cast<KineticExcitation*>(element)->forceArrow.toWidget(forceArrow);
  static_cast<KineticExcitation*>(element)->momentArrow.toWidget(momentArrow);
  static_cast<KineticExcitation*>(element)->connections.toWidget(connections);
  static_cast<KineticExcitation*>(element)->force.toWidget(force);
  static_cast<KineticExcitation*>(element)->moment.toWidget(moment);
  static_cast<KineticExcitation*>(element)->frameOfReference.toWidget(frameOfReference);
}

void KineticExcitationPropertyDialog::fromWidget(Element *element) {
  LinkPropertyDialog::fromWidget(element);
  static_cast<KineticExcitation*>(element)->forceArrow.fromWidget(forceArrow);
  static_cast<KineticExcitation*>(element)->momentArrow.fromWidget(momentArrow);
  static_cast<KineticExcitation*>(element)->connections.fromWidget(connections);
  static_cast<KineticExcitation*>(element)->force.fromWidget(force);
  static_cast<KineticExcitation*>(element)->moment.fromWidget(moment);
  static_cast<KineticExcitation*>(element)->frameOfReference.fromWidget(frameOfReference);
}

SpringDamperPropertyDialog::SpringDamperPropertyDialog(SpringDamper *springDamper, QWidget *parent, Qt::WindowFlags f) : LinkPropertyDialog(springDamper,parent,f) {
  addTab("Kinetics");
  addTab("Visualisation");

  connections = new ExtWidget("Connections",new ConnectFramesWidget(2,springDamper));
  addToTab("Kinetics", connections);

  forceFunction = new ExtWidget("Force function",new Function2ChoiceWidget);
  addToTab("Kinetics", forceFunction);

  forceDirection = new ExtWidget("Force direction",new ForceDirectionWidget(springDamper),true);
  addToTab("Kinetics", forceDirection);

  coilSpring = new ExtWidget("Coil spring",new OMBVCoilSpringWidget("NOTSET"),true);
  addToTab("Visualisation", coilSpring);
}

void SpringDamperPropertyDialog::toWidget(Element *element) {
  LinkPropertyDialog::toWidget(element);
  static_cast<SpringDamper*>(element)->connections.toWidget(connections);
  static_cast<SpringDamper*>(element)->forceFunction.toWidget(forceFunction);
  static_cast<SpringDamper*>(element)->forceDirection.toWidget(forceDirection);
  static_cast<SpringDamper*>(element)->coilSpring.toWidget(coilSpring);
}

void SpringDamperPropertyDialog::fromWidget(Element *element) {
  LinkPropertyDialog::fromWidget(element);
  static_cast<SpringDamper*>(element)->connections.fromWidget(connections);
  static_cast<SpringDamper*>(element)->forceFunction.fromWidget(forceFunction);
  static_cast<SpringDamper*>(element)->forceDirection.fromWidget(forceDirection);
  static_cast<SpringDamper*>(element)->coilSpring.fromWidget(coilSpring);
}

JointPropertyDialog::JointPropertyDialog(Joint *joint, QWidget *parent, Qt::WindowFlags f) : LinkPropertyDialog(joint,parent,f) {
  addTab("Kinetics");
  addTab("Visualisation");

  forceArrow = new ExtWidget("OpenMBV force arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",forceArrow);

  momentArrow = new ExtWidget("OpenMBV moment arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",momentArrow);

  connections = new ExtWidget("Connections",new ConnectFramesWidget(2,joint));
  addToTab("Kinetics", connections);

  force = new ExtWidget("Force",new GeneralizedForceChoiceWidget,true);
  addToTab("Kinetics", force);

  moment = new ExtWidget("Moment",new GeneralizedForceChoiceWidget,true);
  addToTab("Kinetics", moment);
}

void JointPropertyDialog::toWidget(Element *element) {
  LinkPropertyDialog::toWidget(element);
  static_cast<Joint*>(element)->forceArrow.toWidget(forceArrow);
  static_cast<Joint*>(element)->momentArrow.toWidget(momentArrow);
  static_cast<Joint*>(element)->connections.toWidget(connections);
  static_cast<Joint*>(element)->force.toWidget(force);
  static_cast<Joint*>(element)->moment.toWidget(moment);
}

void JointPropertyDialog::fromWidget(Element *element) {
  LinkPropertyDialog::fromWidget(element);
  static_cast<Joint*>(element)->forceArrow.fromWidget(forceArrow);
  static_cast<Joint*>(element)->momentArrow.fromWidget(momentArrow);
  static_cast<Joint*>(element)->connections.fromWidget(connections);
  static_cast<Joint*>(element)->force.fromWidget(force);
  static_cast<Joint*>(element)->moment.fromWidget(moment);
}

ContactPropertyDialog::ContactPropertyDialog(Contact *contact, QWidget *parent, Qt::WindowFlags f) : LinkPropertyDialog(contact,parent,f) {

  addTab("Kinetics");
  addTab("Visualisation");

  connections = new ExtWidget("Connections",new ConnectContoursWidget(2,contact));
  addToTab("Kinetics", connections);

  contactForceLaw = new ExtWidget("Contact force law",new GeneralizedForceLawChoiceWidget);
  addToTab("Kinetics", contactForceLaw);

  contactImpactLaw = new ExtWidget("Contact impact law",new GeneralizedImpactLawChoiceWidget,true);
  addToTab("Kinetics", contactImpactLaw);

  frictionForceLaw = new ExtWidget("Friction force law",new FrictionForceLawChoiceWidget,true);
  addToTab("Kinetics", frictionForceLaw);

  frictionImpactLaw = new ExtWidget("Friction impact law",new FrictionImpactLawChoiceWidget,true);
  addToTab("Kinetics", frictionImpactLaw);

  vector<PhysicalStringWidget*> input;
  input.push_back(new PhysicalStringWidget(new ScalarWidget("0.1"),lengthUnits(),4));
  enableOpenMBVContactPoints = new ExtWidget("OpenMBV contact points",new ExtPhysicalVarWidget(input),true); 
  addToTab("Visualisation",enableOpenMBVContactPoints);

  normalForceArrow = new ExtWidget("OpenMBV normal force arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",normalForceArrow);

  frictionArrow = new ExtWidget("OpenMBV friction arrow",new OMBVArrowWidget("NOTSET"),true);
  addToTab("Visualisation",frictionArrow);
}

void ContactPropertyDialog::toWidget(Element *element) {
  LinkPropertyDialog::toWidget(element);
  static_cast<Contact*>(element)->contactForceLaw.toWidget(contactForceLaw);
  static_cast<Contact*>(element)->contactImpactLaw.toWidget(contactImpactLaw);
  static_cast<Contact*>(element)->frictionForceLaw.toWidget(frictionForceLaw);
  static_cast<Contact*>(element)->frictionImpactLaw.toWidget(frictionImpactLaw);
  static_cast<Contact*>(element)->connections.toWidget(connections);
  static_cast<Contact*>(element)->enableOpenMBVContactPoints.toWidget(enableOpenMBVContactPoints);
  static_cast<Contact*>(element)->normalForceArrow.toWidget(normalForceArrow);
  static_cast<Contact*>(element)->frictionArrow.toWidget(frictionArrow);
}

void ContactPropertyDialog::fromWidget(Element *element) {
  LinkPropertyDialog::fromWidget(element);
  static_cast<Contact*>(element)->contactForceLaw.fromWidget(contactForceLaw);
  static_cast<Contact*>(element)->contactImpactLaw.fromWidget(contactImpactLaw);
  static_cast<Contact*>(element)->frictionForceLaw.fromWidget(frictionForceLaw);
  static_cast<Contact*>(element)->frictionImpactLaw.fromWidget(frictionImpactLaw);
  static_cast<Contact*>(element)->connections.fromWidget(connections);
  static_cast<Contact*>(element)->enableOpenMBVContactPoints.fromWidget(enableOpenMBVContactPoints);
  static_cast<Contact*>(element)->normalForceArrow.fromWidget(normalForceArrow);
  static_cast<Contact*>(element)->frictionArrow.fromWidget(frictionArrow);
}

AbsoluteKinematicsObserverPropertyDialog::AbsoluteKinematicsObserverPropertyDialog(AbsoluteKinematicsObserver *observer, QWidget *parent, Qt::WindowFlags f) : ObserverPropertyDialog(observer,parent,f) {

  addTab("Visualisation");

  frame = new ExtWidget("Frame",new FrameOfReferenceWidget(observer,0));
  addToTab("General", frame);

  position = new ExtWidget("OpenMBV position arrow",new OMBVArrowWidget("NOTSET",true),true);
  addToTab("Visualisation",position);

  velocity = new ExtWidget("OpenMBV velocity arrow",new OMBVArrowWidget("NOTSET",true),true);
  addToTab("Visualisation",velocity);

  angularVelocity = new ExtWidget("OpenMBV angular velocity arrow",new OMBVArrowWidget("NOTSET",true),true);
  addToTab("Visualisation",angularVelocity);

  acceleration = new ExtWidget("OpenMBV acceleration arrow",new OMBVArrowWidget("NOTSET",true),true);
  addToTab("Visualisation",acceleration);

  angularAcceleration = new ExtWidget("OpenMBV angular acceleration arrow",new OMBVArrowWidget("NOTSET",true),true);
  addToTab("Visualisation",angularAcceleration);
}

void AbsoluteKinematicsObserverPropertyDialog::toWidget(Element *element) {
  ObserverPropertyDialog::toWidget(element);
  static_cast<AbsoluteKinematicsObserver*>(element)->frame.toWidget(frame);
  static_cast<AbsoluteKinematicsObserver*>(element)->position.toWidget(position);
  static_cast<AbsoluteKinematicsObserver*>(element)->velocity.toWidget(velocity);
  static_cast<AbsoluteKinematicsObserver*>(element)->angularVelocity.toWidget(angularVelocity);
  static_cast<AbsoluteKinematicsObserver*>(element)->acceleration.toWidget(acceleration);
  static_cast<AbsoluteKinematicsObserver*>(element)->angularAcceleration.toWidget(angularAcceleration);
}

void AbsoluteKinematicsObserverPropertyDialog::fromWidget(Element *element) {
  ObserverPropertyDialog::fromWidget(element);
  static_cast<AbsoluteKinematicsObserver*>(element)->frame.fromWidget(frame);
  static_cast<AbsoluteKinematicsObserver*>(element)->position.fromWidget(position);
  static_cast<AbsoluteKinematicsObserver*>(element)->velocity.fromWidget(velocity);
  static_cast<AbsoluteKinematicsObserver*>(element)->angularVelocity.fromWidget(angularVelocity);
  static_cast<AbsoluteKinematicsObserver*>(element)->acceleration.fromWidget(acceleration);
  static_cast<AbsoluteKinematicsObserver*>(element)->angularAcceleration.fromWidget(angularAcceleration);
}
