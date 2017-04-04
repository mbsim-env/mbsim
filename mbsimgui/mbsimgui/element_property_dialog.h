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
  class InternalFrame;
  class FixedRelativeFrame;
  class NodeFrame;
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
  class MechanicalConstraint;
  class GeneralizedConstraint;
  class GeneralizedGearConstraint;
  class GeneralizedDualConstraint;
  class GeneralizedPositionConstraint;
  class GeneralizedVelocityConstraint;
  class GeneralizedAccelerationConstraint;
  class JointConstraint;
  class GeneralizedConnectionConstraint;
  class LinearTransferSystem;
  class Link;
  class MechanicalLink;
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
  class KinematicCoordinatesObserver;
  class RelativeKinematicsObserver;
  class NaturalCoordinatesObserver;
  class MechanicalLinkObserver;
  class MechanicalConstraintObserver;
  class ContactObserver;
  class FrameObserver;
  class RigidBodyObserver;
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
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
      virtual void toWidget(Element *element);
      virtual void fromWidget(Element *element);
      void toWidget() {toWidget(element);}
      void fromWidget() {fromWidget(element);}
      Element* getElement() {return element;}
      void setName(const QString &str);
      void setReadOnly(bool readOnly);
    protected:
      void showXMLHelp();
      Element *element;
      ExtWidget *name, *plotFeature;
  };

  class FramePropertyDialog : public ElementPropertyDialog {

    public:
      FramePropertyDialog(Frame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *visu;
  };

  class InternalFramePropertyDialog : public ElementPropertyDialog {

    public:
      InternalFramePropertyDialog(InternalFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *visu;
  };

  class FixedRelativeFramePropertyDialog : public FramePropertyDialog {

    public:
      FixedRelativeFramePropertyDialog(FixedRelativeFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *refFrame, *position, *orientation;
  };

  class NodeFramePropertyDialog : public FramePropertyDialog {

    public:
      NodeFramePropertyDialog(NodeFrame *frame, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *nodeNumber;
  };

  class ContourPropertyDialog : public ElementPropertyDialog {

    public:
      ContourPropertyDialog(Contour *contour, QWidget * parent = 0, Qt::WindowFlags f = 0); 
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *thickness;
  };

  class RigidContourPropertyDialog : public ContourPropertyDialog {

    public:
      RigidContourPropertyDialog(RigidContour *contour, QWidget * parent = 0, Qt::WindowFlags f = 0); 
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *refFrame;
  };

  class PointPropertyDialog : public RigidContourPropertyDialog {

    public:
      PointPropertyDialog(Point *point, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *visu;
  };

  class LinePropertyDialog : public RigidContourPropertyDialog {

    public:
      LinePropertyDialog(Line *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *visu;
  };

  class PlanePropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanePropertyDialog(Plane *plane, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *radius, *visu;
  };

  class SpherePropertyDialog : public RigidContourPropertyDialog {

    public:
      SpherePropertyDialog(Sphere *sphere, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *radius, *visu;
  };

  class CirclePropertyDialog : public RigidContourPropertyDialog {

    public:
      CirclePropertyDialog(Circle *circle, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *radius, *solid, *visu;
  };

  class CuboidPropertyDialog : public RigidContourPropertyDialog {

    public:
      CuboidPropertyDialog(Cuboid *circle, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *length, *visu;
  };

  class LineSegmentPropertyDialog : public RigidContourPropertyDialog {

    public:
      LineSegmentPropertyDialog(LineSegment *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *length, *visu;
  };

  class PlanarContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanarContourPropertyDialog(PlanarContour *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *nodes, *contourFunction, *open, *visu;
  };

  class SpatialContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      SpatialContourPropertyDialog(SpatialContour *line, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *etaNodes, *xiNodes, *contourFunction, *open, *visu;
  };

  class GroupPropertyDialog : public ElementPropertyDialog {

    public:
      GroupPropertyDialog(Group *group, QWidget * parent = 0, Qt::WindowFlags f = 0, bool kinematics=true);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *position, *orientation, *frameOfReference; 
  };

  class DynamicSystemSolverPropertyDialog : public GroupPropertyDialog {
    protected:
      ExtWidget *environment, *constraintSolver, *impactSolver, *numberOfMaximalIterations, *projection, *g, *gd, *gdd, *la, *La, *inverseKinetics, *initialProjection, *useConstraintSolverForPlot;

    public:
      DynamicSystemSolverPropertyDialog(DynamicSystemSolver *solver, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

  class ObjectPropertyDialog : public ElementPropertyDialog {
    Q_OBJECT

    public:
      ObjectPropertyDialog(Object *object, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
      virtual void resizeGeneralizedPosition() {}
      virtual void resizeGeneralizedVelocity() {}
    protected:
      ExtWidget *q0, *u0, *R;
      VecWidget *q0_, *u0_;
    public slots:
      void resizeVariables() {resizeGeneralizedPosition();resizeGeneralizedVelocity();}
  };

  class BodyPropertyDialog : public ObjectPropertyDialog {

    public:
      BodyPropertyDialog(Body *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
  };

  class RigidBodyPropertyDialog : public BodyPropertyDialog {

    public:
      RigidBodyPropertyDialog(RigidBody *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
      void resizeGeneralizedPosition();
      void resizeGeneralizedVelocity();
      int getqRelSize() const; 
      int getuRelSize() const; 
    protected:
      ExtWidget *K, *mass, *inertia, *frameForInertiaTensor, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *bodyFixedRepresentationOfAngularVelocity, *ombv, *ombvFrameRef, *weightArrow, *jointForceArrow, *jointMomentArrow;
      RigidBody *body;
  };

  class FlexibleBodyFFRPropertyDialog : public BodyPropertyDialog {
    Q_OBJECT

    public:
      FlexibleBodyFFRPropertyDialog(FlexibleBodyFFR *body, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
      void resizeGeneralizedPosition();
      void resizeGeneralizedVelocity();
      int getqRelSize() const;
      int getuRelSize() const;
    protected:
      ExtWidget *mass, *pdm, *ppdm, *Pdm, *rPdm, *PPdm, *Ke, *De, *beta, *Knl1, *Knl2, *ksigma0, *ksigma1, *K0t, *K0r, *K0om, *r, *A, *Phi, *Psi, *sigmahel, *sigmahen, *sigma0, *K0F, *K0M, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *ombvEditor;
      FlexibleBodyFFR *body;
    protected slots:
      void resizeVariables();
  };

  class ConstraintPropertyDialog : public ElementPropertyDialog {

    public:
      ConstraintPropertyDialog(Constraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class MechanicalConstraintPropertyDialog : public ConstraintPropertyDialog {

    public:
      MechanicalConstraintPropertyDialog(MechanicalConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class GeneralizedConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {

    public:
      GeneralizedConstraintPropertyDialog(GeneralizedConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *support;
  };

  class GeneralizedGearConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedGearConstraintPropertyDialog(GeneralizedGearConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *dependentBody, *independentBodies;
  };

  class GeneralizedDualConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedDualConstraintPropertyDialog(GeneralizedDualConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *dependentBody, *independentBody;
  };

  class GeneralizedPositionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedPositionConstraintPropertyDialog(GeneralizedPositionConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *constraintFunction;
    protected slots:
      void resizeVariables();
  };

  class GeneralizedVelocityConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedVelocityConstraintPropertyDialog(GeneralizedVelocityConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *constraintFunction, *x0;
    protected slots:
      void resizeVariables();
  };

  class GeneralizedAccelerationConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedAccelerationConstraintPropertyDialog(GeneralizedAccelerationConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *constraintFunction, *x0;
    protected slots:
      void resizeVariables();
  };

  class JointConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {
    Q_OBJECT

    public:
      JointConstraintPropertyDialog(JointConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *independentBody, *dependentBodiesFirstSide, *dependentBodiesSecondSide, *refFrameID, *force, *moment, *connections, *q0;
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
    protected:
  };

  class MechanicalLinkPropertyDialog : public LinkPropertyDialog {

    public:
      MechanicalLinkPropertyDialog(MechanicalLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class FrameLinkPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      FrameLinkPropertyDialog(FrameLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *connections;
  };

  class FixedFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FixedFrameLinkPropertyDialog(FixedFrameLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class FloatingFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FloatingFrameLinkPropertyDialog(FloatingFrameLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *refFrameID;
  };

  class RigidBodyLinkPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      RigidBodyLinkPropertyDialog(RigidBodyLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *support;
  };

  class DualRigidBodyLinkPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      DualRigidBodyLinkPropertyDialog(DualRigidBodyLink *link, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *connections;
  };

  class KineticExcitationPropertyDialog : public FloatingFrameLinkPropertyDialog {
    Q_OBJECT

    public:
      KineticExcitationPropertyDialog(KineticExcitation *kineticExcitation, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *forceDirection, *forceFunction, *momentDirection, *momentFunction, *arrow;
    protected slots:
      void resizeVariables();
  };

  class SpringDamperPropertyDialog : public FixedFrameLinkPropertyDialog {

    public:
      SpringDamperPropertyDialog(SpringDamper *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *forceFunction, *unloadedLength, *coilSpring;
  };

  class DirectionalSpringDamperPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      DirectionalSpringDamperPropertyDialog(DirectionalSpringDamper *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *forceDirection, *unloadedLength, *forceFunction, *coilSpring;
  };

  class JointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      JointPropertyDialog(Joint *joint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *forceDirection, *forceLaw, *momentDirection, *momentLaw;
  };

  class ElasticJointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      ElasticJointPropertyDialog(ElasticJoint *joint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *forceDirection, *momentDirection, *function;
  };

  class GeneralizedSpringDamperPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedSpringDamperPropertyDialog(DualRigidBodyLink *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function, *unloadedLength;
  };

  class GeneralizedFrictionPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedFrictionPropertyDialog(DualRigidBodyLink *springDamper, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function, *normalForce;
  };

  class GeneralizedGearPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      GeneralizedGearPropertyDialog(RigidBodyLink *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function, *gearOutput, *gearInput;
  };

  class GeneralizedElasticConnectionPropertyDialog : public DualRigidBodyLinkPropertyDialog {
    Q_OBJECT

    public:
      GeneralizedElasticConnectionPropertyDialog(DualRigidBodyLink *connection, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function;
    protected slots:
      void resizeVariables();
  };

  class ContactPropertyDialog : public LinkPropertyDialog {

    public:
      ContactPropertyDialog(Contact *contact, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *contactForceLaw, *contactImpactLaw, *frictionForceLaw, *frictionImpactLaw, *connections, *searchAllContactPoints, *initialGuess;
  };

  class ObserverPropertyDialog : public ElementPropertyDialog {

    public:
      ObserverPropertyDialog(Observer *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class KinematicCoordinatesObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      KinematicCoordinatesObserverPropertyDialog(KinematicCoordinatesObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *frame, *frameOfReference, *position, *velocity, *acceleration;
  };

  class RelativeKinematicsObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RelativeKinematicsObserverPropertyDialog(RelativeKinematicsObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *frame, *refFrame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
  };

  class MechanicalLinkObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      MechanicalLinkObserverPropertyDialog(MechanicalLinkObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *link, *forceArrow, *momentArrow;
  };

  class MechanicalConstraintObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      MechanicalConstraintObserverPropertyDialog(MechanicalConstraintObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *constraint, *forceArrow, *momentArrow;
  };

  class ContactObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      ContactObserverPropertyDialog(ContactObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *link, *forceArrow, *momentArrow, *contactPoints, *normalForceArrow, *frictionArrow;
  };

  class FrameObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      FrameObserverPropertyDialog(FrameObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *frame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
  };

  class RigidBodyObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RigidBodyObserverPropertyDialog(RigidBodyObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *body, *weight, *jointForce, *jointMoment, *axisOfRotation;
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
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
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
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
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
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function;
  };

  class PIDControllerPropertyDialog : public SignalPropertyDialog {

    public:
      PIDControllerPropertyDialog(PIDController *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *sRef, *sdRef, *P, *I, *D;
  };

  class UnarySignalOperationPropertyDialog : public SignalPropertyDialog {

    public:
      UnarySignalOperationPropertyDialog(UnarySignalOperation *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *sRef, *f;
  };

  class BinarySignalOperationPropertyDialog : public SignalPropertyDialog {

    public:
      BinarySignalOperationPropertyDialog(BinarySignalOperation *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *s1Ref, *s2Ref, *f;
  };

  class ExternSignalSourcePropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSourcePropertyDialog(ExternSignalSource *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *sourceSize;
  };

  class ExternSignalSinkPropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSinkPropertyDialog(ExternSignalSink *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *inputSignal;
  };

}

#endif
