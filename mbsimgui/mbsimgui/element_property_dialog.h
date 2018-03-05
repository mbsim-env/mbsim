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
#include "embeditemdata.h"

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
  class IsotropicRotationalSpringDamper;
  class GeneralizedSpringDamper;
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
  class RigidBodySystemObserver;
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

  class ElementPropertyDialog : public EmbedItemPropertyDialog {

    public:
      ElementPropertyDialog(Element *element, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      Element* getElement() const;
      void setName(const QString &str);
      void setReadOnly(bool readOnly);
    protected:
      void showXMLHelp() override;
      ExtWidget *name, *plotFeature;
  };

  class FramePropertyDialog : public ElementPropertyDialog {

    public:
      FramePropertyDialog(Frame *frame, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class InternalFramePropertyDialog : public ElementPropertyDialog {

    public:
      InternalFramePropertyDialog(InternalFrame *frame, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class FixedRelativeFramePropertyDialog : public FramePropertyDialog {

    public:
      FixedRelativeFramePropertyDialog(FixedRelativeFrame *frame, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *refFrame, *position, *orientation;
  };

  class NodeFramePropertyDialog : public FramePropertyDialog {

    public:
      NodeFramePropertyDialog(NodeFrame *frame, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodeNumber;
  };

  class ContourPropertyDialog : public ElementPropertyDialog {

    public:
      ContourPropertyDialog(Contour *contour, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr); 
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *thickness;
  };

  class RigidContourPropertyDialog : public ContourPropertyDialog {

    public:
      RigidContourPropertyDialog(RigidContour *contour, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr); 
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *refFrame;
  };

  class PointPropertyDialog : public RigidContourPropertyDialog {

    public:
      PointPropertyDialog(Point *point, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class LinePropertyDialog : public RigidContourPropertyDialog {

    public:
      LinePropertyDialog(Line *line, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class PlanePropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanePropertyDialog(Plane *plane, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *visu;
  };

  class SpherePropertyDialog : public RigidContourPropertyDialog {

    public:
      SpherePropertyDialog(Sphere *sphere, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *visu;
  };

  class CirclePropertyDialog : public RigidContourPropertyDialog {

    public:
      CirclePropertyDialog(Circle *circle, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *solid, *visu;
  };

  class CuboidPropertyDialog : public RigidContourPropertyDialog {

    public:
      CuboidPropertyDialog(Cuboid *circle, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length, *visu;
  };

  class LineSegmentPropertyDialog : public RigidContourPropertyDialog {

    public:
      LineSegmentPropertyDialog(LineSegment *line, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length, *visu;
  };

  class PlanarContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanarContourPropertyDialog(PlanarContour *contour, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodes, *contourFunction, *open, *visu;
  };

  class SpatialContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      SpatialContourPropertyDialog(SpatialContour *contour, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *etaNodes, *xiNodes, *contourFunction, *open, *visu;
  };

  class GroupPropertyDialog : public ElementPropertyDialog {

    public:
      GroupPropertyDialog(Group *group, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr, bool kinematics=true);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frameOfReference;
  };

  class DynamicSystemSolverPropertyDialog : public GroupPropertyDialog {
    protected:
      ExtWidget *environment, *constraintSolver, *impactSolver, *maxIter, *numericalJacobian, *linearAlgebra, *projection, *g, *gd, *gdd, *la, *La, *inverseKinetics, *initialProjection, *useConstraintSolverForPlot;

    public:
      DynamicSystemSolverPropertyDialog(DynamicSystemSolver *solver, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class ObjectPropertyDialog : public ElementPropertyDialog {

    public:
      ObjectPropertyDialog(Object *object, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      virtual void resizeGeneralizedPosition() {}
      virtual void resizeGeneralizedVelocity() {}
    protected:
      ExtWidget *q0, *u0, *R;
      void updateWidget() override {resizeGeneralizedPosition();resizeGeneralizedVelocity();}
  };

  class BodyPropertyDialog : public ObjectPropertyDialog {

    public:
      BodyPropertyDialog(Body *body, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
  };

  class RigidBodyPropertyDialog : public BodyPropertyDialog {

    public:
      RigidBodyPropertyDialog(RigidBody *body_, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      void resizeGeneralizedPosition() override;
      void resizeGeneralizedVelocity() override;
      int getqRelSize() const; 
      int getuRelSize() const; 
    protected:
      ExtWidget *K, *mass, *inertia, *frameForInertiaTensor, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *bodyFixedRepresentationOfAngularVelocity, *ombv, *ombvFrameRef, *weightArrow, *jointForceArrow, *jointMomentArrow;
      RigidBody *body;
  };

  class FlexibleBodyFFRPropertyDialog : public BodyPropertyDialog {

    public:
      FlexibleBodyFFRPropertyDialog(FlexibleBodyFFR *body_, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      void resizeGeneralizedPosition() override;
      void resizeGeneralizedVelocity() override;
      int getqRelSize() const;
      int getuRelSize() const;
    protected:
      ExtWidget *mass, *pdm, *ppdm, *Pdm, *rPdm, *PPdm, *Ke, *De, *beta, *Knl1, *Knl2, *ksigma0, *ksigma1, *K0t, *K0r, *K0om, *r, *A, *Phi, *Psi, *sigmahel, *sigmahen, *sigma0, *K0F, *K0M, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *ombvEditor;
      FlexibleBodyFFR *body;
      void updateWidget() override;
  };

  class ConstraintPropertyDialog : public ElementPropertyDialog {

    public:
      ConstraintPropertyDialog(Constraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class MechanicalConstraintPropertyDialog : public ConstraintPropertyDialog {

    public:
      MechanicalConstraintPropertyDialog(MechanicalConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class GeneralizedConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {

    public:
      GeneralizedConstraintPropertyDialog(GeneralizedConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *support;
  };

  class GeneralizedGearConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedGearConstraintPropertyDialog(GeneralizedGearConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dependentBody, *independentBodies;
  };

  class GeneralizedDualConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedDualConstraintPropertyDialog(GeneralizedDualConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dependentBody, *independentBody;
  };

  class GeneralizedPositionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedPositionConstraintPropertyDialog(GeneralizedPositionConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraintFunction;
      void updateWidget() override;
  };

  class GeneralizedVelocityConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedVelocityConstraintPropertyDialog(GeneralizedVelocityConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraintFunction, *x0;
      void updateWidget() override;
  };

  class GeneralizedAccelerationConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedAccelerationConstraintPropertyDialog(GeneralizedAccelerationConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraintFunction, *x0;
      void updateWidget() override;
  };

  class JointConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {

    public:
      JointConstraintPropertyDialog(JointConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *independentBody, *dependentBodiesFirstSide, *dependentBodiesSecondSide, *refFrameID, *force, *moment, *connections, *q0;
      void updateWidget() override;
  };

  class GeneralizedConnectionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedConnectionConstraintPropertyDialog(GeneralizedConnectionConstraint *constraint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class LinkPropertyDialog : public ElementPropertyDialog {

    public:
      LinkPropertyDialog(Link *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
    protected:
  };

  class MechanicalLinkPropertyDialog : public LinkPropertyDialog {

    public:
      MechanicalLinkPropertyDialog(MechanicalLink *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class FrameLinkPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      FrameLinkPropertyDialog(FrameLink *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections;
  };

  class FixedFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FixedFrameLinkPropertyDialog(FixedFrameLink *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class FloatingFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FloatingFrameLinkPropertyDialog(FloatingFrameLink *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *refFrameID;
  };

  class RigidBodyLinkPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      RigidBodyLinkPropertyDialog(RigidBodyLink *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *support;
  };

  class DualRigidBodyLinkPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      DualRigidBodyLinkPropertyDialog(DualRigidBodyLink *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections;
  };

  class KineticExcitationPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      KineticExcitationPropertyDialog(KineticExcitation *kineticExcitation, QWidget * parent = nullptr, const Qt::WindowFlags& wf = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *forceFunction, *momentDirection, *momentFunction, *arrow;
      void updateWidget() override;
  };

  class SpringDamperPropertyDialog : public FixedFrameLinkPropertyDialog {

    public:
      SpringDamperPropertyDialog(SpringDamper *springDamper, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceFunction, *unloadedLength, *coilSpring;
  };

  class DirectionalSpringDamperPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      DirectionalSpringDamperPropertyDialog(DirectionalSpringDamper *springDamper, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *unloadedLength, *forceFunction, *coilSpring;
  };

  class IsotropicRotationalSpringDamperPropertyDialog : public FixedFrameLinkPropertyDialog {

    public:
      IsotropicRotationalSpringDamperPropertyDialog(IsotropicRotationalSpringDamper *springDamper, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *elasticMomentFunction, *dissipativeMomentFunction;
  };

  class JointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      JointPropertyDialog(Joint *joint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *forceLaw, *momentDirection, *momentLaw, *integrate;
  };

  class ElasticJointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      ElasticJointPropertyDialog(ElasticJoint *joint, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *momentDirection, *function, *integrate;
      void updateWidget() override;
  };

  class GeneralizedSpringDamperPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedSpringDamperPropertyDialog(DualRigidBodyLink *springDamper, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *unloadedLength;
  };

  class GeneralizedFrictionPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedFrictionPropertyDialog(DualRigidBodyLink *friction, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *normalForce;
  };

  class GeneralizedGearPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      GeneralizedGearPropertyDialog(RigidBodyLink *link, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *gearOutput, *gearInput;
  };

  class GeneralizedElasticConnectionPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedElasticConnectionPropertyDialog(DualRigidBodyLink *connection, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function;
      void updateWidget() override;
  };

  class GeneralizedElasticStructurePropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      GeneralizedElasticStructurePropertyDialog(RigidBodyLink *connection, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *rigidBody;
  };

  class ContactPropertyDialog : public LinkPropertyDialog {

    public:
      ContactPropertyDialog(Contact *contact, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *contactForceLaw, *contactImpactLaw, *frictionForceLaw, *frictionImpactLaw, *connections, *searchAllContactPoints, *initialGuess;
  };

  class ObserverPropertyDialog : public ElementPropertyDialog {

    public:
      ObserverPropertyDialog(Observer *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class KinematicCoordinatesObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      KinematicCoordinatesObserverPropertyDialog(KinematicCoordinatesObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame, *frameOfReference, *position, *velocity, *acceleration;
  };

  class RelativeKinematicsObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RelativeKinematicsObserverPropertyDialog(RelativeKinematicsObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame, *refFrame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
  };

  class MechanicalLinkObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      MechanicalLinkObserverPropertyDialog(MechanicalLinkObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *link, *forceArrow, *momentArrow;
  };

  class MechanicalConstraintObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      MechanicalConstraintObserverPropertyDialog(MechanicalConstraintObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraint, *forceArrow, *momentArrow;
  };

  class ContactObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      ContactObserverPropertyDialog(ContactObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *link, *forceArrow, *momentArrow, *contactPoints, *normalForceArrow, *frictionArrow;
  };

  class FrameObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      FrameObserverPropertyDialog(FrameObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
  };

  class RigidBodyObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RigidBodyObserverPropertyDialog(RigidBodyObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *body, *frameOfReference, *weight, *jointForce, *jointMoment, *axisOfRotation, *momentum, *angularMomentum, *derivativeOfMomentum, *derivativeOfAngularMomentum;
  };

  class RigidBodySystemObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RigidBodySystemObserverPropertyDialog(RigidBodySystemObserver *observer, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *bodies, *frameOfReference, *position, *velocity, *acceleration, *weight, *momentum, *angularMomentum, *derivativeOfMomentum, *derivativeOfAngularMomentum;
  };

  class SignalPropertyDialog: public LinkPropertyDialog {

    public:
      SignalPropertyDialog(Signal *signal, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr); 
  };

  class SensorPropertyDialog : public SignalPropertyDialog {

    public:
      SensorPropertyDialog(Sensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr); 
  };

  class ObjectSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ObjectSensorPropertyDialog(ObjectSensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *object;
  };

  class GeneralizedPositionSensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      GeneralizedPositionSensorPropertyDialog(GeneralizedPositionSensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class GeneralizedVelocitySensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      GeneralizedVelocitySensorPropertyDialog(GeneralizedVelocitySensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class FrameSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FrameSensorPropertyDialog(FrameSensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame;
  };

  class PositionSensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      PositionSensorPropertyDialog(PositionSensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class OrientationSensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      OrientationSensorPropertyDialog(OrientationSensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class VelocitySensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      VelocitySensorPropertyDialog(VelocitySensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class AngularVelocitySensorPropertyDialog : public FrameSensorPropertyDialog {

    public:
      AngularVelocitySensorPropertyDialog(AngularVelocitySensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
  };

  class FunctionSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FunctionSensorPropertyDialog(FunctionSensor *sensor, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function;
  };

  class MultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      MultiplexerPropertyDialog(Multiplexer *signal, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal;
  };

  class DemultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      DemultiplexerPropertyDialog(Demultiplexer *signal, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *indices;
  };

  class LinearTransferSystemPropertyDialog : public SignalPropertyDialog {

    public:
      LinearTransferSystemPropertyDialog(LinearTransferSystem *signal, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *A, *B, *C, *D;
      void updateWidget() override;
  };

  class SignalOperationPropertyDialog : public SignalPropertyDialog {
    Q_OBJECT

    public:
      SignalOperationPropertyDialog(SignalOperation *signal, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *function;
      void updateWidget() override;
    protected slots:
      void updateFunctionFactory(bool defineWidget=true);
  };

  class ExternSignalSourcePropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSourcePropertyDialog(ExternSignalSource *signal, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *sourceSize;
  };

  class ExternSignalSinkPropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSinkPropertyDialog(ExternSignalSink *signal, QWidget * parent = nullptr, const Qt::WindowFlags& f = nullptr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal;
  };

}

#endif
