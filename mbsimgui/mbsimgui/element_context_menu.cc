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
#include "element_context_menu.h"
#include "mainwindow.h"
#include "rigidbody.h"
#include "constraint.h"
#include "linear_transfer_system.h"
#include "kinetic_excitation.h"
#include "spring_damper.h"
#include "joint.h"
#include "contact.h"
#include "actuator.h"
#include "sensor.h"
#include "observer.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
#include "friction.h"
#include "gear.h"
#include <QFileDialog>

namespace MBSimGUI {

  extern MainWindow *mw;

  EmbeddingContextMenu::EmbeddingContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add scalar parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addScalarParameter()));
    addAction(action);
    action = new QAction("Add vector parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addVectorParameter()));
    addAction(action);
    action = new QAction("Add matrix parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addMatrixParameter()));
    addAction(action);
    action = new QAction("Add string parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addStringParameter()));
    addAction(action);
    action = new QAction("Add search path parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addSearchPathParameter()));
    addAction(action);
  }

  void EmbeddingContextMenu::addScalarParameter() {
    mw->addParameter(new ScalarParameter("a"));
  }

  void EmbeddingContextMenu::addVectorParameter() {
    mw->addParameter(new VectorParameter("a"));
  }

  void EmbeddingContextMenu::addMatrixParameter() {
    mw->addParameter(new MatrixParameter("a"));
  }

  void EmbeddingContextMenu::addStringParameter() {
    mw->addParameter(new StringParameter("a"));
  }

  void EmbeddingContextMenu::addSearchPathParameter() {
    mw->addParameter(new SearchPathParameter);
  }

  ElementContextMenu::ElementContextMenu(Element *element_, QWidget *parent, bool removable) : QMenu(parent), element(element_) {
    if(removable) {
      QAction *action=new QAction("Save as", this);
      connect(action,SIGNAL(triggered()),mw,SLOT(saveElementAs()));
      addAction(action);
      addSeparator();
      action=new QAction("Remove", this);
      connect(action,SIGNAL(triggered()),mw,SLOT(removeElement()));
      addAction(action);
      addSeparator();
    }
  }

  FrameContextMenu::FrameContextMenu(Element *frame, QWidget * parent, bool removable) : ElementContextMenu(frame,parent,removable) {
  }

  FixedRelativeFrameContextMenu::FixedRelativeFrameContextMenu(Element *frame, QWidget * parent) : FrameContextMenu(frame,parent,true) {
  }

  GroupContextMenu::GroupContextMenu(Element *element, QWidget *parent, bool removable) : ElementContextMenu(element,parent,removable) {
    QAction *action;
    action = new QAction("Add frame", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
    addAction(action);
    QMenu *menu = new ContourContextContextMenu(element, "Add contour");
    addMenu(menu);
    action = new QAction("Add group", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGroup()));
    //menu = new GroupContextContextMenu(element, "Add group");
    //addMenu(menu);
    addAction(action);
    menu = new ObjectContextContextMenu(element, "Add object");
    addMenu(menu);
    menu = new LinkContextContextMenu(element, "Add link");
    addMenu(menu);
    menu = new ObserverContextContextMenu(element, "Add observer");
    addMenu(menu);
    action = new QAction("Add model", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addModel()));
    addAction(action);
  } 

  void GroupContextMenu::addFixedRelativeFrame() {
    mw->addFrame(new FixedRelativeFrame("P",element));
  }

  void GroupContextMenu::addModel() {
    QString file=QFileDialog::getOpenFileName(0, "XML model files", ".", "XML files (*.xml)");
    if(file!="") {
      Frame *frame = Frame::readXMLFile(file.toStdString(),element);
      if(frame) return mw->addFrame(frame);
      Contour *contour = Contour::readXMLFile(file.toStdString(),element);
      if(contour) return mw->addContour(contour);
      Group *group = Group::readXMLFile(file.toStdString(),element);
      if(group) return mw->addGroup(group);
      Object *object = Object::readXMLFile(file.toStdString(),element);
      if(object) return mw->addObject(object);
      Link *link = Link::readXMLFile(file.toStdString(),element);
      if(link) return mw->addLink(link);
      Observer *observer = Observer::readXMLFile(file.toStdString(),element);
      if(observer) return mw->addObserver(observer);
    }
  }

  void GroupContextMenu::addGroup() {
    mw->addGroup(new Group("Group",element));
  }

  void GroupContextMenu::addObject() {
    ObjectContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  void GroupContextMenu::addLink() {
    LinkContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  void GroupContextMenu::addObserver() {
    ObserverContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  DynamicSystemSolverContextMenu::DynamicSystemSolverContextMenu(Element *solver, QWidget * parent) : GroupContextMenu(solver,parent,false) {
  }

  ObjectContextMenu::ObjectContextMenu(Element *element, QWidget *parent) : ElementContextMenu(element,parent,true) {
  } 

  BodyContextMenu::BodyContextMenu(Element *element, QWidget *parent) : ObjectContextMenu(element,parent) {
    QAction *action;
    action = new QAction("Add frame", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
    addAction(action);
    QMenu *menu = new ContourContextContextMenu(element, "Add contour");
    addMenu(menu);
  } 

  void BodyContextMenu::addFixedRelativeFrame() {
    mw->addFrame(new FixedRelativeFrame("P",element));
  }

  FrameContextContextMenu::FrameContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add frame", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
    addAction(action);
  }

  void FrameContextContextMenu::addFixedRelativeFrame() {
    mw->addFrame(new FixedRelativeFrame("P",element));
  }

  ContourContextContextMenu::ContourContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add point", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addPoint()));
    addAction(action);
    action = new QAction("Add line", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addLine()));
    addAction(action);
    action = new QAction("Add plane", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addPlane()));
    addAction(action);
    action = new QAction("Add sphere", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addSphere()));
    addAction(action);
    action = new QAction("Add solid circle", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addCircleSolid()));
    addAction(action);
    action = new QAction("Add cuboid", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addCuboid()));
    addAction(action);
  }

  void ContourContextContextMenu::addPoint() {
    mw->addContour(new Point("Point",element));
  }

  void ContourContextContextMenu::addLine() {
    mw->addContour(new Line("Line",element));
  }

  void ContourContextContextMenu::addPlane() {
    mw->addContour(new Plane("Plane",element));
  }

  void ContourContextContextMenu::addSphere() {
    mw->addContour(new Sphere("Sphere",element));
  }

  void ContourContextContextMenu::addCircleSolid() {
    mw->addContour(new CircleSolid("CircleSolid",element));
  }

  void ContourContextContextMenu::addCuboid() {
    mw->addContour(new Cuboid("Cuboid",element));
  }

  GroupContextContextMenu::GroupContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add group", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGroup()));
    addAction(action);
  }

  void GroupContextContextMenu::addGroup() {
    mw->addGroup(new Group("Group",element));
  }

  ObjectContextContextMenu::ObjectContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QMenu *menu = new BodyContextContextMenu(element, "Add body");
    addMenu(menu);
    menu = new ConstraintContextContextMenu(element, "Add constraint");
    addMenu(menu);
  }

  BodyContextContextMenu::BodyContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add rigid body", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addRigidBody()));
    addAction(action);
  }

  void BodyContextContextMenu::addRigidBody() {
    mw->addObject(new RigidBody("RigidBody",element));
  }

  ConstraintContextContextMenu::ConstraintContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add generalized position constraint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedPositionConstraint()));
    addAction(action);
    action = new QAction("Add generalized velocity constraint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedVelocityConstraint()));
    addAction(action);
    action = new QAction("Add generalized acceleration constraint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedAccelerationConstraint()));
    addAction(action);
    action = new QAction("Add gear constraint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGearConstraint()));
    addAction(action);
    action = new QAction("Add joint constraint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addJointConstraint()));
    addAction(action);
  }

  void ConstraintContextContextMenu::addGearConstraint() {
    mw->addObject(new GearConstraint("GearConstraint",element));
  }

  void ConstraintContextContextMenu::addGeneralizedPositionConstraint() {
    mw->addObject(new GeneralizedPositionConstraint("GeneralizedPositionConstraint",element));
  }

  void ConstraintContextContextMenu::addGeneralizedVelocityConstraint() {
    mw->addObject(new GeneralizedVelocityConstraint("GeneralizedVelocityConstraint",element));
  }

  void ConstraintContextContextMenu::addGeneralizedAccelerationConstraint() {
    mw->addObject(new GeneralizedAccelerationConstraint("GeneralizedAccelerationConstraint",element));
  }

  void ConstraintContextContextMenu::addJointConstraint() {
    mw->addObject(new JointConstraint("JointConstraint",element));
  }

  LinkContextContextMenu::LinkContextContextMenu(Element *element_, const QString &title,  QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add kinetic excitation", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addKineticExcitation()));
    addAction(action);
    action = new QAction("Add spring damper", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addSpringDamper()));
    addAction(action);
    action = new QAction("Add directional spring damper", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addDirectionalSpringDamper()));
    addAction(action);
    action = new QAction("Add generalized spring damper", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedSpringDamper()));
    addAction(action);
    action = new QAction("Add joint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addJoint()));
    addAction(action);
    action = new QAction("Add contact", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addContact()));
    addAction(action);
    action = new QAction("Add actuator", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addActuator()));
    addAction(action);
    QMenu *menu = new SignalContextContextMenu(element, "Add signal");
    addMenu(menu);
    action = new QAction("Add linear transfer system", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addLinearTransferSystem()));
    addAction(action);
    action = new QAction("Add generalized friction", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedFriction()));
    addAction(action);
    action = new QAction("Add gear", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGear()));
    addAction(action);
  }

  void LinkContextContextMenu::addKineticExcitation() {
    mw->addLink(new KineticExcitation("KineticExcitation",element));
  }

  void LinkContextContextMenu::addSpringDamper() {
    mw->addLink(new SpringDamper("SpringDamper",element));
  }

  void LinkContextContextMenu::addDirectionalSpringDamper() {
    mw->addLink(new DirectionalSpringDamper("DirectionalSpringDamper",element));
  }

  void LinkContextContextMenu::addGeneralizedSpringDamper() {
    mw->addLink(new GeneralizedSpringDamper("GeneralizedSpringDamper",element));
  }

  void LinkContextContextMenu::addJoint() {
    mw->addLink(new Joint("Joint",element));
  }

  void LinkContextContextMenu::addContact() {
    mw->addLink(new Contact("Contact",element));
  }

  void LinkContextContextMenu::addActuator() {
    mw->addLink(new Actuator("Actuator",element));
  }

  void LinkContextContextMenu::addSignal() {
    SignalContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  void LinkContextContextMenu::addLinearTransferSystem() {
    mw->addLink(new LinearTransferSystem("LTS",element));
  }

  void LinkContextContextMenu::addGeneralizedFriction() {
    mw->addLink(new GeneralizedFriction("GeneralizedFriction",element));
  }

  void LinkContextContextMenu::addGear() {
    mw->addLink(new Gear("Gear",element));
  }

  ObserverContextContextMenu::ObserverContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QMenu *menu = new CoordinatesObserverContextContextMenu(element, "Add coordinates observer");
    addMenu(menu);
    menu = new KinematicsObserverContextContextMenu(element, "Add kinematics observer");
    addMenu(menu);
  }

  void ObserverContextContextMenu::addCoordinatesObserver() {
    CoordinatesObserverContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  void ObserverContextContextMenu::addKinematicsObserver() {
    KinematicsObserverContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  CoordinatesObserverContextContextMenu::CoordinatesObserverContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add cartesian coordinates observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addCartesianCoordinatesObserver()));
    addAction(action);
    action = new QAction("Add cylinder coordinates observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addCylinderCoordinatesObserver()));
    addAction(action);
    action = new QAction("Add natural coordinates observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addNaturalCoordinatesObserver()));
    addAction(action);
  }

  void CoordinatesObserverContextContextMenu::addCartesianCoordinatesObserver() {
    mw->addObserver(new CartesianCoordinatesObserver("CartesianCoordinatesObserver",element));
  }

  void CoordinatesObserverContextContextMenu::addCylinderCoordinatesObserver() {
    mw->addObserver(new CylinderCoordinatesObserver("CylinderCoordinatesObserver",element));
  }

  void CoordinatesObserverContextContextMenu::addNaturalCoordinatesObserver() {
    mw->addObserver(new NaturalCoordinatesObserver("NaturalCoordinatesObserver",element));
  }

  KinematicsObserverContextContextMenu::KinematicsObserverContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add absolute kinematics observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addAbsoluteKinematicsObserver()));
    addAction(action);
    action = new QAction("Add relative kinematics observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addRelativeKinematicsObserver()));
    addAction(action);
  }

  void KinematicsObserverContextContextMenu::addAbsoluteKinematicsObserver() {
    mw->addObserver(new AbsoluteKinematicsObserver("AbsoluteKinematicsObserver",element));
  }

  void KinematicsObserverContextContextMenu::addRelativeKinematicsObserver() {
    mw->addObserver(new RelativeKinematicsObserver("RelativeKinematicsObserver",element));
  }

  SignalContextContextMenu::SignalContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QMenu *menu = new SensorContextContextMenu(element,"Add sensor");
    addMenu(menu);
    QAction *action = new QAction("Add signal addition", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addSignalAddition()));
    addAction(action);
    action = new QAction("Add PID controller", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addPIDController()));
    addAction(action);
    action = new QAction("Add unary signal operation", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addUnarySignalOperation()));
    addAction(action);
    action = new QAction("Add binary signal operation", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addBinarySignalOperation()));
    addAction(action);
    action = new QAction("Add extern signal source", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addExternSignalSource()));
    addAction(action);
    action = new QAction("Add extern signal sink", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addExternSignalSink()));
    addAction(action);
  }

  void SignalContextContextMenu::addSensor() {
    SensorContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  void SignalContextContextMenu::addSignalAddition() {
    mw->addLink(new SignalAddition("SignalAddition",element));
  }

  void SignalContextContextMenu::addPIDController() {
    mw->addLink(new PIDController("PIDController",element));
  }

  void SignalContextContextMenu::addUnarySignalOperation() {
    mw->addLink(new UnarySignalOperation("UnarySignalOperation",element));
  }

  void SignalContextContextMenu::addBinarySignalOperation() {
    mw->addLink(new BinarySignalOperation("BinarySignalOperation",element));
  }

  void SignalContextContextMenu::addExternSignalSource() {
    mw->addLink(new ExternSignalSource("ExternSignalSource",element));
  }

  void SignalContextContextMenu::addExternSignalSink() {
    mw->addLink(new ExternSignalSink("ExternSignalSink",element));
  }

  SensorContextContextMenu::SensorContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add generalized position sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedPositionSensor()));
    addAction(action);
    action = new QAction("Add generalized velocity sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedVelocitySensor()));
    addAction(action);
    action = new QAction("Add absolute position sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addAbsolutePositionSensor()));
    addAction(action);
    action = new QAction("Add absolute velocity sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addAbsoluteVelocitySensor()));
    addAction(action);
    action = new QAction("Add absolute angular position sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addAbsoluteAngularPositionSensor()));
    addAction(action);
    action = new QAction("Add absolute angular velocity sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addAbsoluteAngularVelocitySensor()));
    addAction(action);
    action = new QAction("Add function sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFunctionSensor()));
    addAction(action);
    action = new QAction("Add signal processing system sensor", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addSignalProcessingSystemSensor()));
    addAction(action);
  }

  void SensorContextContextMenu::addGeneralizedPositionSensor() {
    mw->addLink(new GeneralizedPositionSensor("GeneralizedPositionSensor",element));
  }

  void SensorContextContextMenu::addGeneralizedVelocitySensor() {
    mw->addLink(new GeneralizedVelocitySensor("GeneralizedVelocitySensor",element));
  }

  void SensorContextContextMenu::addAbsolutePositionSensor() {
    mw->addLink(new AbsolutePositionSensor("AbsolutePositionSensor",element));
  }

  void SensorContextContextMenu::addAbsoluteVelocitySensor() {
    mw->addLink(new AbsoluteVelocitySensor("AbsoluteVelocitySensor",element));
  }

  void SensorContextContextMenu::addAbsoluteAngularPositionSensor() {
    mw->addLink(new AbsoluteAngularPositionSensor("AbsoluteAngularPositionSensor",element));
  }

  void SensorContextContextMenu::addAbsoluteAngularVelocitySensor() {
    mw->addLink(new AbsoluteAngularVelocitySensor("AbsoluteAngularVelocitySensor",element));
  }

  void SensorContextContextMenu::addFunctionSensor() {
    mw->addLink(new FunctionSensor("FunctionSensor",element));
  }

  void SensorContextContextMenu::addSignalProcessingSystemSensor() {
    mw->addLink(new SignalProcessingSystemSensor("SPS",element));
  }

}
