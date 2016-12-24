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

#ifndef _ELEMENT_PROPERTY_DIALOG_H_
#define _ELEMENT_PROPERTY_DIALOG_H_

#include "property_dialog.h"

namespace MBSimGUI {

  class Element;
  class Frame;
  class FixedRelativeFrame;
  class NodeFrame;
  class FixedNodalFrame;
  class Contour;
  class RigidContour;
  class Point;
  class Line;
  class Plane;
  class Sphere;
  class Circle;
  class Cuboid;
  class LineSegment;
  class PlanarContour;
  class SpatialContour;
  class DynamicSystemSolver;
  class Group;
  class Object;
  class Body;
  class RigidBody;
  class FlexibleBodyFFR;
  class Constraint;
  class GeneralizedConstraint;
  class GeneralizedGearConstraint;
  class GeneralizedDualConstraint;
  class GeneralizedPositionConstraint;
  class GeneralizedVelocityConstraint;
  class GeneralizedAccelerationConstraint;
  class JointConstraint;
  class GeneralizedConnectionConstraint;
  class SignalProcessingSystem;
  class LinearTransferSystem;
  class Link;
  class FrameLink;
  class FixedFrameLink;
  class FloatingFrameLink;
  class RigidBodyLink;
  class DualRigidBodyLink;
  class KineticExcitation;
  class SpringDamper;
  class DirectionalSpringDamper;
  class GeneralizedSpringDamper;
  class GeneralizedLinearElasticConnection;
  class Joint;
  class ElasticJoint;
  class Contact;
  class Observer;
  class CoordinatesObserver;
  class CartesianCoordinatesObserver;
  class CylinderCoordinatesObserver;
  class NaturalCoordinatesObserver;
  class KinematicsObserver;
  class AbsoluteKinematicsObserver;
  class RelativeKinematicsObserver;
  class NaturalCoordinatesObserver;
  class Signal;
  class Sensor;
  class GeneralizedCoordinateSensor;
  class GeneralizedPositionSensor;
  class GeneralizedVelocitySensor;
  class AbsoluteCoordinateSensor;
  class AbsolutePositionSensor;
  class AbsoluteVelocitySensor;
  class AbsoluteAngularPositionSensor;
  class AbsoluteAngularVelocitySensor;
  class FunctionSensor;
  class SignalProcessingSystemSensor;
  class PIDController;
  class UnarySignalOperation;
  class BinarySignalOperation;
  class TextWidget;
  class VecWidget;
  class ExtWidget;
  class ExternSignalSource;
  class ExternSignalSink;

  class ElementPropertyDialog : public PropertyDialog {

    public:
      ElementPropertyDialog(Element *element, QWidget * parent = 0, Qt::WindowFlags f = 0);
      virtual void toWidget(Element *element);
      virtual void fromWidget(Element *element);
      void toWidget() {toWidget(element);}
      void fromWidget() {fromWidget(element);}
      Element* getElement() {return element;}
    protected:
      void showXMLHelp();
      Element *element;
      ExtWidget *name, *plotFeature;
  };

  class FramePropertyDialog : public ElementPropertyDialog {

    public:
      FramePropertyDialog(Frame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *visu;
  };

  class FixedRelativeFramePropertyDialog : public FramePropertyDialog {

    public:
      FixedRelativeFramePropertyDialog(FixedRelativeFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *refFrame, *position, *orientation;
  };

  class NodeFramePropertyDialog : public FramePropertyDialog {
    Q_OBJECT

    public:
      NodeFramePropertyDialog(NodeFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *nodeNumber;
  };

  class FixedNodalFramePropertyDialog : public FramePropertyDialog {
    Q_OBJECT

    public:
      FixedNodalFramePropertyDialog(FixedNodalFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *position, *orientation, *Phi, *Psi, *sigmahel, *sigmahen, *sigma0, *K0F, *K0M;
    protected slots:
      void resizeVariables();
  };

  class ContourPropertyDialog : public ElementPropertyDialog {

    public:
      ContourPropertyDialog(Contour *contour, QWidget * parent = 0, Qt::WindowFlags f = 0); 
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *thickness;
  };

  class RigidContourPropertyDialog : public ContourPropertyDialog {

    public:
      RigidContourPropertyDialog(RigidContour *contour, QWidget * parent = 0, Qt::WindowFlags f = 0); 
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *refFrame;
  };

  class PointPropertyDialog : public RigidContourPropertyDialog {

    public:
      PointPropertyDialog(Point *point, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *visu;
  };

  class LinePropertyDialog : public RigidContourPropertyDialog {

    public:
      LinePropertyDialog(Line *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *visu;
  };

  class PlanePropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanePropertyDialog(Plane *plane, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *radius, *visu;
  };

  class SpherePropertyDialog : public RigidContourPropertyDialog {

    public:
      SpherePropertyDialog(Sphere *sphere, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *radius, *visu;
  };

  class CirclePropertyDialog : public RigidContourPropertyDialog {

    public:
      CirclePropertyDialog(Circle *circle, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *radius, *solid, *visu;
  };

  class CuboidPropertyDialog : public RigidContourPropertyDialog {

    public:
      CuboidPropertyDialog(Cuboid *circle, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *length, *visu;
  };

  class LineSegmentPropertyDialog : public RigidContourPropertyDialog {

    public:
      LineSegmentPropertyDialog(LineSegment *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *length, *visu;
  };

  class PlanarContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanarContourPropertyDialog(PlanarContour *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *nodes, *contourFunction, *open, *visu;
  };

  class SpatialContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      SpatialContourPropertyDialog(SpatialContour *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *etaNodes, *xiNodes, *contourFunction, *open, *visu;
  };

  class GroupPropertyDialog : public ElementPropertyDialog {

    public:
      GroupPropertyDialog(Group *group, QWidget * parent = 0, Qt::WindowFlags f = 0, bool kinematics=true);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *position, *orientation, *frameOfReference; 
  };

  class DynamicSystemSolverPropertyDialog : public GroupPropertyDialog {
    protected:
      ExtWidget *environment, *solverParameters, *inverseKinetics, *initialProjection, *useConstraintSolverForPlot;

    public:
      DynamicSystemSolverPropertyDialog(DynamicSystemSolver *solver, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
  };

  class ObjectPropertyDialog : public ElementPropertyDialog {

    public:
      ObjectPropertyDialog(Object *object, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
  };

  class BodyPropertyDialog : public ObjectPropertyDialog {
    Q_OBJECT;

    public:
    BodyPropertyDialog(Body *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
    void toWidget(Element *element);
    void fromWidget(Element *element);
    virtual void resizeGeneralizedPosition() {}
    virtual void resizeGeneralizedVelocity() {}
    protected:
    ExtWidget *q0, *u0, *R;
    VecWidget *q0_, *u0_;
    public slots:
      void resizeVariables() {resizeGeneralizedPosition();resizeGeneralizedVelocity();}
  };

  class RigidBodyPropertyDialog : public BodyPropertyDialog {

    public:
      RigidBodyPropertyDialog(RigidBody *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
      void resizeGeneralizedPosition();
      void resizeGeneralizedVelocity();
      int getqRelSize() const; 
      int getuRelSize() const; 
    protected:
      ExtWidget *K, *mass, *inertia, *frameForInertiaTensor, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *bodyFixedRepresentationOfAngularVelocity, *ombvEditor, *weightArrow, *jointForceArrow, *jointMomentArrow;
      RigidBody *body;
  };

  class FlexibleBodyFFRPropertyDialog : public BodyPropertyDialog {
    Q_OBJECT

    public:
      FlexibleBodyFFRPropertyDialog(FlexibleBodyFFR *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
      void resizeGeneralizedPosition();
      void resizeGeneralizedVelocity();
      int getqRelSize() const;
      int getuRelSize() const;
    protected:
      ExtWidget *mass, *pdm, *ppdm, *Pdm, *rPdm, *PPdm, *Ke, *De, *beta, *Knl1, *Knl2, *ksigma0, *ksigma1, *K0t, *K0r, *K0om, *r, *A, *Phi, *Psi, *sigmahel, *sigmahen, *sigma0, *K0F, *K0M, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *ombvEditor, *jointForceArrow, *jointMomentArrow;
      FlexibleBodyFFR *body;
    protected slots:
      void resizeVariables();
  };

  class ConstraintPropertyDialog : public ElementPropertyDialog {

    public:
      ConstraintPropertyDialog(Constraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class GeneralizedConstraintPropertyDialog : public ConstraintPropertyDialog {

    public:
      GeneralizedConstraintPropertyDialog(GeneralizedConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *support, *forceArrow, *momentArrow;
  };

  class GeneralizedGearConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedGearConstraintPropertyDialog(GeneralizedGearConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *dependentBody, *independentBodies;
  };

  class GeneralizedDualConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedDualConstraintPropertyDialog(GeneralizedDualConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *dependentBody, *independentBody;
  };

  class GeneralizedPositionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedPositionConstraintPropertyDialog(GeneralizedPositionConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *constraintFunction;
    protected slots:
      void resizeVariables();
  };

  class GeneralizedVelocityConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedVelocityConstraintPropertyDialog(GeneralizedVelocityConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *constraintFunction, *x0;
      VecWidget *x0_;
    protected slots:
      void resizeVariables();
  };

  class GeneralizedAccelerationConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedAccelerationConstraintPropertyDialog(GeneralizedAccelerationConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *constraintFunction, *x0;
      VecWidget *x0_;
    protected slots:
      void resizeVariables();
  };

  class JointConstraintPropertyDialog : public ConstraintPropertyDialog {
    Q_OBJECT

    public:
      JointConstraintPropertyDialog(JointConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *independentBody, *dependentBodiesFirstSide, *dependentBodiesSecondSide, *refFrameID, *force, *moment, *connections, *jointForceArrow, *jointMomentArrow, *q0;
      VecWidget *q0_;
    protected slots:
      void resizeVariables();
  };

  class GeneralizedConnectionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedConnectionConstraintPropertyDialog(GeneralizedConnectionConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class LinkPropertyDialog : public ElementPropertyDialog {

    public:
      LinkPropertyDialog(Link *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
  };

  class FrameLinkPropertyDialog : public LinkPropertyDialog {

    public:
      FrameLinkPropertyDialog(FrameLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *connections, *forceArrow, *momentArrow;
  };

  class FixedFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FixedFrameLinkPropertyDialog(FixedFrameLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class FloatingFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FloatingFrameLinkPropertyDialog(FloatingFrameLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *refFrameID;
  };

  class RigidBodyLinkPropertyDialog : public LinkPropertyDialog {

    public:
      RigidBodyLinkPropertyDialog(RigidBodyLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *support, *forceArrow, *momentArrow;
  };

  class DualRigidBodyLinkPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      DualRigidBodyLinkPropertyDialog(DualRigidBodyLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *connections;
  };

  class SignalProcessingSystemPropertyDialog : public LinkPropertyDialog {

    public:
      SignalProcessingSystemPropertyDialog(SignalProcessingSystem *sps, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *signalRef;
  };

  class KineticExcitationPropertyDialog : public FloatingFrameLinkPropertyDialog {
    Q_OBJECT

    public:
      KineticExcitationPropertyDialog(KineticExcitation *kineticExcitation, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *forceDirection, *forceFunction, *momentDirection, *momentFunction;
    protected slots:
      void resizeVariables();
  };

  class SpringDamperPropertyDialog : public FrameLinkPropertyDialog {

    public:
      SpringDamperPropertyDialog(SpringDamper *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *forceFunction, *unloadedLength, *coilSpring;
  };

  class DirectionalSpringDamperPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      DirectionalSpringDamperPropertyDialog(DirectionalSpringDamper *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *forceDirection, *unloadedLength, *forceFunction, *coilSpring;
  };

  class GeneralizedSpringDamperPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedSpringDamperPropertyDialog(DualRigidBodyLink *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *function, *unloadedLength;
  };

  class GeneralizedFrictionPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedFrictionPropertyDialog(DualRigidBodyLink *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *function, *normalForce;
  };

  class GeneralizedGearPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      GeneralizedGearPropertyDialog(RigidBodyLink *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *function, *gearOutput, *gearInput;
  };

  class GeneralizedElasticConnectionPropertyDialog : public DualRigidBodyLinkPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedElasticConnectionPropertyDialog(DualRigidBodyLink *connection, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *function;
    protected slots:
      void resizeVariables();
  };

  class JointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      JointPropertyDialog(Joint *joint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *forceDirection, *forceLaw, *momentDirection, *momentLaw;
  };

  class ElasticJointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      ElasticJointPropertyDialog(ElasticJoint *joint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *forceDirection, *momentDirection, *function;
  };

  class ContactPropertyDialog : public LinkPropertyDialog {

    public:
      ContactPropertyDialog(Contact *contact, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *contactForceLaw, *contactImpactLaw, *frictionForceLaw, *frictionImpactLaw, *connections, *searchAllContactPoints, *initialGuess, *enableOpenMBVContactPoints, *normalForceArrow, *frictionArrow;
  };

  class ObserverPropertyDialog : public ElementPropertyDialog {

    public:
      ObserverPropertyDialog(Observer *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class CoordinatesObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      CoordinatesObserverPropertyDialog(CoordinatesObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *frame, *position, *velocity, *acceleration, *ombvFrame;
  };

  class CartesianCoordinatesObserverPropertyDialog : public CoordinatesObserverPropertyDialog {

    public:
      CartesianCoordinatesObserverPropertyDialog(CartesianCoordinatesObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class CylinderCoordinatesObserverPropertyDialog : public CoordinatesObserverPropertyDialog {

    public:
      CylinderCoordinatesObserverPropertyDialog(CylinderCoordinatesObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class NaturalCoordinatesObserverPropertyDialog : public CoordinatesObserverPropertyDialog {

    public:
      NaturalCoordinatesObserverPropertyDialog(NaturalCoordinatesObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class KinematicsObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      KinematicsObserverPropertyDialog(KinematicsObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *frame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
  };

  class AbsoluteKinematicsObserverPropertyDialog : public KinematicsObserverPropertyDialog {

    public:
      AbsoluteKinematicsObserverPropertyDialog(AbsoluteKinematicsObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class RelativeKinematicsObserverPropertyDialog : public KinematicsObserverPropertyDialog {

    public:
      RelativeKinematicsObserverPropertyDialog(RelativeKinematicsObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *refFrame;
  };

  class SignalPropertyDialog: public LinkPropertyDialog {

    public:
      SignalPropertyDialog(Signal *signal, QWidget * parent = 0, Qt::WindowFlags f = 0); 
  };

  class SensorPropertyDialog : public SignalPropertyDialog {

    public:
      SensorPropertyDialog(Sensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0); 
  };

  class GeneralizedCoordinateSensorPropertyDialog : public SensorPropertyDialog {

    public:
      GeneralizedCoordinateSensorPropertyDialog(GeneralizedCoordinateSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *object, *index;
  };

  class GeneralizedPositionSensorPropertyDialog : public GeneralizedCoordinateSensorPropertyDialog {

    public:
      GeneralizedPositionSensorPropertyDialog(GeneralizedPositionSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class GeneralizedVelocitySensorPropertyDialog : public GeneralizedCoordinateSensorPropertyDialog {

    public:
      GeneralizedVelocitySensorPropertyDialog(GeneralizedVelocitySensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class AbsoluteCoordinateSensorPropertyDialog : public SensorPropertyDialog {

    public:
      AbsoluteCoordinateSensorPropertyDialog(AbsoluteCoordinateSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *frame, *direction;
  };

  class AbsolutePositionSensorPropertyDialog : public AbsoluteCoordinateSensorPropertyDialog {

    public:
      AbsolutePositionSensorPropertyDialog(AbsolutePositionSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class AbsoluteVelocitySensorPropertyDialog : public AbsoluteCoordinateSensorPropertyDialog {

    public:
      AbsoluteVelocitySensorPropertyDialog(AbsoluteVelocitySensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class AbsoluteAngularPositionSensorPropertyDialog : public AbsoluteCoordinateSensorPropertyDialog {

    public:
      AbsoluteAngularPositionSensorPropertyDialog(AbsoluteAngularPositionSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class AbsoluteAngularVelocitySensorPropertyDialog : public AbsoluteCoordinateSensorPropertyDialog {

    public:
      AbsoluteAngularVelocitySensorPropertyDialog(AbsoluteAngularVelocitySensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class FunctionSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FunctionSensorPropertyDialog(FunctionSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *function;
  };

  class SignalProcessingSystemSensorPropertyDialog : public SensorPropertyDialog {

    public:
      SignalProcessingSystemSensorPropertyDialog(SignalProcessingSystemSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *spsRef;
  };

  class PIDControllerPropertyDialog : public SignalPropertyDialog {

    public:
      PIDControllerPropertyDialog(PIDController *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *sRef, *sdRef, *P, *I, *D;
  };

  class UnarySignalOperationPropertyDialog : public SignalPropertyDialog {

    public:
      UnarySignalOperationPropertyDialog(UnarySignalOperation *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *sRef, *f;
  };

  class BinarySignalOperationPropertyDialog : public SignalPropertyDialog {

    public:
      BinarySignalOperationPropertyDialog(BinarySignalOperation *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *s1Ref, *s2Ref, *f;
  };

  class ExternSignalSourcePropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSourcePropertyDialog(ExternSignalSource *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *sourceSize;
  };

  class ExternSignalSinkPropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSinkPropertyDialog(ExternSignalSink *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      void toWidget(Element *element);
      void fromWidget(Element *element);
    protected:
      ExtWidget *inputSignal;
  };

}

#endif
