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
  class RigidBodyGroupObserver;
  class Signal;
  class Sensor;
  class ObjectSensor;
  class GeneralizedPositionSensor;
  class GeneralizedVelocitySensor;
  class FrameSensor;
  class PositionSensor;
  class OrientationSensor;
  class VelocitySensor;
  class AngularVelocitySensor;
  class FunctionSensor;
  class Multiplexer;
  class Demultiplexer;
  class LinearTransferSystem;
  class SignalOperation;
  class SignalOperation;
  class ExternSignalSource;
  class ExternSignalSink;
  class ExtWidget;

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
      ExtWidget *frameOfReference;
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

    public:
      ObjectPropertyDialog(Object *object, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
      virtual void resizeGeneralizedPosition() {}
      virtual void resizeGeneralizedVelocity() {}
    protected:
      ExtWidget *q0, *u0, *R;
      void updateWidget() {resizeGeneralizedPosition();resizeGeneralizedVelocity();}
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
      void updateWidget();
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

    public:
      GeneralizedPositionConstraintPropertyDialog(GeneralizedPositionConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *constraintFunction;
      void updateWidget();
  };

  class GeneralizedVelocityConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedVelocityConstraintPropertyDialog(GeneralizedVelocityConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *constraintFunction, *x0;
      void updateWidget();
  };

  class GeneralizedAccelerationConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedAccelerationConstraintPropertyDialog(GeneralizedAccelerationConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *constraintFunction, *x0;
      void updateWidget();
  };

  class JointConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {

    public:
      JointConstraintPropertyDialog(JointConstraint *constraint, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *independentBody, *dependentBodiesFirstSide, *dependentBodiesSecondSide, *refFrameID, *force, *moment, *connections, *q0;
      void updateWidget();
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

    public:
      KineticExcitationPropertyDialog(KineticExcitation *kineticExcitation, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *forceDirection, *forceFunction, *momentDirection, *momentFunction, *arrow;
      void updateWidget();
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

    public:
      GeneralizedElasticConnectionPropertyDialog(DualRigidBodyLink *connection, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function;
      void updateWidget();
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

  class RigidBodyGroupObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RigidBodyGroupObserverPropertyDialog(RigidBodyGroupObserver *observer, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *bodies, *frameOfReference, *position, *velocity, *acceleration, *weight, *momentum, *angularMomentum, *derivativeOfMomentum, *derivativeOfAngularMomentum;
  };

  class SignalPropertyDialog: public LinkPropertyDialog {

    public:
      SignalPropertyDialog(Signal *signal, QWidget * parent = 0, Qt::WindowFlags f = 0); 
  };

  class SensorPropertyDialog : public SignalPropertyDialog {

    public:
      SensorPropertyDialog(Sensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0); 
  };

  class ObjectSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ObjectSensorPropertyDialog(ObjectSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *object;
  };

  class GeneralizedPositionSensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      GeneralizedPositionSensorPropertyDialog(GeneralizedPositionSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class GeneralizedVelocitySensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      GeneralizedVelocitySensorPropertyDialog(GeneralizedVelocitySensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class FrameSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FrameSensorPropertyDialog(FrameSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *frame;
  };

  class PositionSensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      PositionSensorPropertyDialog(PositionSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class OrientationSensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      OrientationSensorPropertyDialog(OrientationSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class VelocitySensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      VelocitySensorPropertyDialog(VelocitySensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class AngularVelocitySensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      AngularVelocitySensorPropertyDialog(AngularVelocitySensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
  };

  class FunctionSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FunctionSensorPropertyDialog(FunctionSensor *sensor, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *function;
  };

  class MultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      MultiplexerPropertyDialog(Multiplexer *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *inputSignal;
  };

  class DemultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      DemultiplexerPropertyDialog(Demultiplexer *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *inputSignal, *indices;
  };

  class LinearTransferSystemPropertyDialog : public SignalPropertyDialog {

    public:
      LinearTransferSystemPropertyDialog(LinearTransferSystem *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *inputSignal, *A, *B, *C, *D;
      void updateWidget();
  };

  class SignalOperationPropertyDialog : public SignalPropertyDialog {
    Q_OBJECT

    public:
      SignalOperationPropertyDialog(SignalOperation *signal, QWidget * parent = 0, Qt::WindowFlags f = 0);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=NULL);
    protected:
      ExtWidget *inputSignal, *function;
      void updateWidget();
    protected slots:
      void updateFunctionFactory(bool defineWidget=true);
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
