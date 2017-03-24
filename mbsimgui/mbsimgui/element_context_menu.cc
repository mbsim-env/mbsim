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
#include "rigid_body.h"
//#include "flexible_body_ffr.h"
#include "constraint.h"
//#include "linear_transfer_system.h"
//#include "kinetic_excitation.h"
#include "spring_damper.h"
//#include "joint.h"
//#include "contact.h"
//#include "sensor.h"
#include "observer.h"
#include "frame.h"
#include "contour.h"
#include "group.h"
//#include "friction.h"
//#include "gear.h"
//#include "connection.h"
#include "parameter.h"
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
    action = new QAction("Add import parameter", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addImportParameter()));
    addAction(action);
  }

  void EmbeddingContextMenu::addScalarParameter() {
    mw->addParameter(new ScalarParameter("a"), element);
  }

  void EmbeddingContextMenu::addVectorParameter() {
    mw->addParameter(new VectorParameter("a"), element);
  }

  void EmbeddingContextMenu::addMatrixParameter() {
    mw->addParameter(new MatrixParameter("a"), element);
  }

  void EmbeddingContextMenu::addStringParameter() {
    mw->addParameter(new StringParameter("a"), element);
  }

  void EmbeddingContextMenu::addImportParameter() {
    mw->addParameter(new ImportParameter, element);
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

  NodeFrameContextMenu::NodeFrameContextMenu(Element *frame, QWidget * parent) : FrameContextMenu(frame,parent,true) {
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
    menu = new ConstraintContextContextMenu(element, "Add constraint");
    addMenu(menu);
    menu = new ObserverContextContextMenu(element, "Add observer");
    addMenu(menu);
    action = new QAction("Add model", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addModel()));
    addAction(action);
  } 

  void GroupContextMenu::addFixedRelativeFrame() {
    mw->addFrame(new FixedRelativeFrame("P"), element);
  }

  void GroupContextMenu::addModel() {
    QString file=QFileDialog::getOpenFileName(0, "XML model files", ".", "XML files (*.xml)");
    if(file!="") {
      Frame *frame = Frame::readXMLFile(file);
      if(frame) return mw->addFrame(frame);
      Contour *contour = Contour::readXMLFile(file);
      if(contour) return mw->addContour(contour);
      Group *group = Group::readXMLFile(file);
      if(group) return mw->addGroup(group);
      Object *object = Object::readXMLFile(file);
      if(object) return mw->addObject(object);
      Link *link = Link::readXMLFile(file);
      if(link) return mw->addLink(link);
      Constraint *constraint = Constraint::readXMLFile(file);
      if(constraint) return mw->addConstraint(constraint);
      Observer *observer = Observer::readXMLFile(file);
      if(observer) return mw->addObserver(observer);
    }
  }

  void GroupContextMenu::addGroup() {
    mw->addGroup(new Group("Group"));
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

  RigidBodyContextMenu::RigidBodyContextMenu(Element *element, QWidget *parent) : ObjectContextMenu(element,parent) {
    QAction *action;
    action = new QAction("Add frame", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
    addAction(action);
    QMenu *menu = new ContourContextContextMenu(element, "Add contour");
    addMenu(menu);
  } 

  void RigidBodyContextMenu::addFixedRelativeFrame() {
    mw->addFrame(new FixedRelativeFrame("P"));
  }

  FlexibleBodyFFRContextMenu::FlexibleBodyFFRContextMenu(Element *element, QWidget *parent) : ObjectContextMenu(element,parent) {
    QAction *action;
    action = new QAction("Add frame", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addNodeFrame()));
    addAction(action);
    QMenu *menu = new ContourContextContextMenu(element, "Add contour");
    addMenu(menu);
  }

  void FlexibleBodyFFRContextMenu::addNodeFrame() {
    mw->addFrame(new NodeFrame("P"));
  }

  FixedRelativeFrameContextContextMenu::FixedRelativeFrameContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add frame", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFixedRelativeFrame()));
    addAction(action);
  }

  void FixedRelativeFrameContextContextMenu::addFixedRelativeFrame() {
    mw->addFrame(new FixedRelativeFrame("P"));
  }

  NodeFrameContextContextMenu::NodeFrameContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add node frame", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addNodeFrame()));
    addAction(action);
  }

  void NodeFrameContextContextMenu::addNodeFrame() {
    mw->addFrame(new NodeFrame("P"));
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
    action = new QAction("Add circle", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addCircle()));
    addAction(action);
    action = new QAction("Add cuboid", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addCuboid()));
    addAction(action);
    action = new QAction("Add line segment", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addLineSegment()));
    addAction(action);
    action = new QAction("Add planar contour", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addPlanarContour()));
    addAction(action);
    action = new QAction("Add spatial contour", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addSpatialContour()));
    addAction(action);
  }

  void ContourContextContextMenu::addPoint() {
    mw->addContour(new Point("Point"));
  }

  void ContourContextContextMenu::addLine() {
    mw->addContour(new Line("Line"));
  }

  void ContourContextContextMenu::addPlane() {
    mw->addContour(new Plane("Plane"));
  }

  void ContourContextContextMenu::addSphere() {
    mw->addContour(new Sphere("Sphere"));
  }

  void ContourContextContextMenu::addCircle() {
    mw->addContour(new Circle("Circle"));
  }

  void ContourContextContextMenu::addCuboid() {
    mw->addContour(new Cuboid("Cuboid"));
  }

  void ContourContextContextMenu::addLineSegment() {
    mw->addContour(new LineSegment("LineSegment"));
  }

  void ContourContextContextMenu::addPlanarContour() {
    mw->addContour(new PlanarContour("PlanarContour"));
  }

  void ContourContextContextMenu::addSpatialContour() {
    mw->addContour(new SpatialContour("SpatialContour"));
  }

  GroupContextContextMenu::GroupContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add group", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGroup()));
    addAction(action);
  }

  void GroupContextContextMenu::addGroup() {
    mw->addGroup(new Group("Group"));
  }

  ObjectContextContextMenu::ObjectContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QMenu *menu = new BodyContextContextMenu(element, "Add body");
    addMenu(menu);
  }

  BodyContextContextMenu::BodyContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add rigid body", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addRigidBody()));
    addAction(action);
    action = new QAction("Add flexible body ffr", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFlexibleBodyFFR()));
    addAction(action);
  }

  void BodyContextContextMenu::addRigidBody() {
    mw->addObject(new RigidBody("RigidBody"), element);
  }

  void BodyContextContextMenu::addFlexibleBodyFFR() {
//    mw->addObject(new FlexibleBodyFFR("FlexibleBodyFFR"));
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
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedGearConstraint()));
    addAction(action);
    action = new QAction("Add joint constraint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addJointConstraint()));
    addAction(action);
    action = new QAction("Add generalized connection constraint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedConnectionConstraint()));
    addAction(action);
  }

  void ConstraintContextContextMenu::addGeneralizedGearConstraint() {
    mw->addConstraint(new GeneralizedGearConstraint("GeneralizedGearConstraint"));
  }

  void ConstraintContextContextMenu::addGeneralizedPositionConstraint() {
    mw->addConstraint(new GeneralizedPositionConstraint("GeneralizedPositionConstraint"));
  }

  void ConstraintContextContextMenu::addGeneralizedVelocityConstraint() {
    mw->addConstraint(new GeneralizedVelocityConstraint("GeneralizedVelocityConstraint"));
  }

  void ConstraintContextContextMenu::addGeneralizedAccelerationConstraint() {
    mw->addConstraint(new GeneralizedAccelerationConstraint("GeneralizedAccelerationConstraint"));
  }

  void ConstraintContextContextMenu::addJointConstraint() {
    mw->addConstraint(new JointConstraint("JointConstraint"));
  }

  void ConstraintContextContextMenu::addGeneralizedConnectionConstraint() {
    mw->addConstraint(new GeneralizedConnectionConstraint("GeneralizedConnectionConstraint"));
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
    action = new QAction("Add elastic joint", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addElasticJoint()));
    addAction(action);
    action = new QAction("Add contact", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addContact()));
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
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedGear()));
    addAction(action);
    action = new QAction("Add generalized elastic connection", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addGeneralizedElasticConnection()));
    addAction(action);
  }

  void LinkContextContextMenu::addKineticExcitation() {
//    mw->addLink(new KineticExcitation("KineticExcitation"));
  }

  void LinkContextContextMenu::addSpringDamper() {
    mw->addLink(new SpringDamper("SpringDamper"));
  }

  void LinkContextContextMenu::addDirectionalSpringDamper() {
//    mw->addLink(new DirectionalSpringDamper("DirectionalSpringDamper"));
  }

  void LinkContextContextMenu::addGeneralizedSpringDamper() {
//    mw->addLink(new GeneralizedSpringDamper("GeneralizedSpringDamper"));
  }

  void LinkContextContextMenu::addJoint() {
//    mw->addLink(new Joint("Joint"));
  }

  void LinkContextContextMenu::addElasticJoint() {
//    mw->addLink(new ElasticJoint("ElasticJoint"));
  }

  void LinkContextContextMenu::addContact() {
//    mw->addLink(new Contact("Contact"));
  }

  void LinkContextContextMenu::addSignal() {
    SignalContextContextMenu menu(element);
    menu.exec(QCursor::pos());
  }

  void LinkContextContextMenu::addLinearTransferSystem() {
//    mw->addLink(new LinearTransferSystem("LTS"));
  }

  void LinkContextContextMenu::addGeneralizedFriction() {
//    mw->addLink(new GeneralizedFriction("GeneralizedFriction"));
  }

  void LinkContextContextMenu::addGeneralizedGear() {
//    mw->addLink(new GeneralizedGear("GeneralizedGear"));
  }

  void LinkContextContextMenu::addGeneralizedElasticConnection() {
//    mw->addLink(new GeneralizedElasticConnection("GeneralizedElasticConnection"));
  }

  ObserverContextContextMenu::ObserverContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QAction *action = new QAction("Add mechanical link observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addMechanicalLinkObserver()));
    addAction(action);
    action = new QAction("Add mechanical constraint observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addMechanicalConstraintObserver()));
    addAction(action);
    action = new QAction("Add contact observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addContactObserver()));
    addAction(action);
    action = new QAction("Add frame observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addFrameObserver()));
    addAction(action);
    action = new QAction("Add rigid body observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addRigidBodyObserver()));
    addAction(action);
    action = new QAction("Add kinematic coordinates observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addKinematicCoordinatesObserver()));
    addAction(action);
    action = new QAction("Add relative kinematics observer", this);
    connect(action,SIGNAL(triggered()),this,SLOT(addRelativeKinematicsObserver()));
    addAction(action);
  }

  void ObserverContextContextMenu::addMechanicalLinkObserver() {
    mw->addObserver(new MechanicalLinkObserver("MechanicalLinkObserver"));
  }

  void ObserverContextContextMenu::addMechanicalConstraintObserver() {
    mw->addObserver(new MechanicalConstraintObserver("MechanicalConstraintObserver"));
  }

  void ObserverContextContextMenu::addContactObserver() {
    mw->addObserver(new ContactObserver("ContactObserver"));
  }

  void ObserverContextContextMenu::addFrameObserver() {
    mw->addObserver(new FrameObserver("FrameObserver"));
  }

  void ObserverContextContextMenu::addRigidBodyObserver() {
    mw->addObserver(new RigidBodyObserver("RigidBodyObserver"));
  }

  void ObserverContextContextMenu::addKinematicCoordinatesObserver() {
    mw->addObserver(new KinematicCoordinatesObserver("KinematicCoordinatesObserver"));
  }

  void ObserverContextContextMenu::addRelativeKinematicsObserver() {
    mw->addObserver(new RelativeKinematicsObserver("RelativeKinematicsObserver"));
  }

  SignalContextContextMenu::SignalContextContextMenu(Element *element_, const QString &title, QWidget *parent) : QMenu(title,parent), element(element_) {
    QMenu *menu = new SensorContextContextMenu(element,"Add sensor");
    addMenu(menu);
    QAction *action = new QAction("Add PID controller", this);
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

  void SignalContextContextMenu::addPIDController() {
//    mw->addLink(new PIDController("PIDController"));
  }

  void SignalContextContextMenu::addUnarySignalOperation() {
//    mw->addLink(new UnarySignalOperation("UnarySignalOperation"));
  }

  void SignalContextContextMenu::addBinarySignalOperation() {
//    mw->addLink(new BinarySignalOperation("BinarySignalOperation"));
  }

  void SignalContextContextMenu::addExternSignalSource() {
//    mw->addLink(new ExternSignalSource("ExternSignalSource"));
  }

  void SignalContextContextMenu::addExternSignalSink() {
//    mw->addLink(new ExternSignalSink("ExternSignalSink"));
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
//    mw->addLink(new GeneralizedPositionSensor("GeneralizedPositionSensor"));
  }

  void SensorContextContextMenu::addGeneralizedVelocitySensor() {
//    mw->addLink(new GeneralizedVelocitySensor("GeneralizedVelocitySensor"));
  }

  void SensorContextContextMenu::addAbsolutePositionSensor() {
//    mw->addLink(new AbsolutePositionSensor("AbsolutePositionSensor"));
  }

  void SensorContextContextMenu::addAbsoluteVelocitySensor() {
//    mw->addLink(new AbsoluteVelocitySensor("AbsoluteVelocitySensor"));
  }

  void SensorContextContextMenu::addAbsoluteAngularPositionSensor() {
//    mw->addLink(new AbsoluteAngularPositionSensor("AbsoluteAngularPositionSensor"));
  }

  void SensorContextContextMenu::addAbsoluteAngularVelocitySensor() {
//    mw->addLink(new AbsoluteAngularVelocitySensor("AbsoluteAngularVelocitySensor"));
  }

  void SensorContextContextMenu::addFunctionSensor() {
//    mw->addLink(new FunctionSensor("FunctionSensor"));
  }

  void SensorContextContextMenu::addSignalProcessingSystemSensor() {
//    mw->addLink(new SignalProcessingSystemSensor("SPS"));
  }

}
