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
  class ExtWidget;
  class ImportDialog;

  class ElementPropertyDialog : public EmbedItemPropertyDialog {

    public:
      ElementPropertyDialog(Element *element);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      Element* getElement() const;
      void setName(const QString &str);
      void setReadOnly(bool readOnly);
    protected:
      void showXMLHelp() override;
      ExtWidget *name, *plotFeature;
  };

  class UnknownElementPropertyDialog : public ElementPropertyDialog {

    public:
      UnknownElementPropertyDialog(Element *element);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *editor;
  };

  class FramePropertyDialog : public ElementPropertyDialog {

    public:
      FramePropertyDialog(Element *frame);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class InternalFramePropertyDialog : public ElementPropertyDialog {

    public:
      InternalFramePropertyDialog(Element *frame);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class FixedRelativeFramePropertyDialog : public FramePropertyDialog {

    public:
      FixedRelativeFramePropertyDialog(Element *frame);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *refFrame, *position, *orientation;
  };

  class NodeFramePropertyDialog : public FramePropertyDialog {

    public:
      NodeFramePropertyDialog(Element *frame);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodeNumber;
  };

  class ContourPropertyDialog : public ElementPropertyDialog {

    public:
      ContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *thickness;
  };

  class RigidContourPropertyDialog : public ContourPropertyDialog {

    public:
      RigidContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *refFrame;
  };

  class PointPropertyDialog : public RigidContourPropertyDialog {

    public:
      PointPropertyDialog(Element *point);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class LinePropertyDialog : public RigidContourPropertyDialog {

    public:
      LinePropertyDialog(Element *line);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class PlanePropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanePropertyDialog(Element *plane);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *visu;
  };

  class SpherePropertyDialog : public RigidContourPropertyDialog {

    public:
      SpherePropertyDialog(Element *sphere);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *visu;
  };

  class CirclePropertyDialog : public RigidContourPropertyDialog {

    public:
      CirclePropertyDialog(Element *circle);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *solid, *visu;
  };

  class CuboidPropertyDialog : public RigidContourPropertyDialog {

    public:
      CuboidPropertyDialog(Element *circle);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length, *visu;
  };

  class LineSegmentPropertyDialog : public RigidContourPropertyDialog {

    public:
      LineSegmentPropertyDialog(Element *line);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length, *visu;
  };

  class PlanarContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanarContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodes, *contourFunction, *open, *visu;
  };

  class PlanarNurbsContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanarNurbsContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *interpolation, *controlPoints, *numberOfControlPoints, *knotVector, *degree, *open, *visu;
  };

  class SpatialContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      SpatialContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *etaNodes, *xiNodes, *contourFunction, *openEta, *openXi, *visu;
  };

  class SpatialNurbsContourPropertyDialog : public RigidContourPropertyDialog {

    public:
      SpatialNurbsContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *interpolation, *controlPoints, *numberOfEtaControlPoints, *numberOfXiControlPoints, *etaKnotVector, *xiKnotVector, *etaDegree, *xiDegree, *openEta, *openXi, *visu;
  };

  class DiskPropertyDialog : public RigidContourPropertyDialog {

    public:
      DiskPropertyDialog(Element *disk);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *outerRadius, *innerRadius, *width, *visu;
  };

  class GearWheelPropertyDialog : public RigidContourPropertyDialog {

    public:
      GearWheelPropertyDialog(Element *gearWheel);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *numberOfTeeth, *width, *helixAngle, *pitchAngle, *module, *pressureAngle, *backlash, *solid, *visu;
  };

  class FlexiblePlanarNurbsContourPropertyDialog : public ContourPropertyDialog {

    public:
      FlexiblePlanarNurbsContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *interpolation, *indices, *knotVector, *degree, *open, *visu;
  };

  class FlexibleSpatialNurbsContourPropertyDialog : public ContourPropertyDialog {

    public:
      FlexibleSpatialNurbsContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *interpolation, *indices, *etaKnotVector, *xiKnotVector, *etaDegree, *xiDegree, *openEta, *openXi, *visu;
  };

  class FCLContourPropertyDialog : public RigidContourPropertyDialog {
    public:
      FCLContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *computeLocalAABB;
  };

  class FCLBoxPropertyDialog : public FCLContourPropertyDialog {

    public:
      FCLBoxPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length, *visu;
  };

  class FCLSpherePropertyDialog : public FCLContourPropertyDialog {

    public:
      FCLSpherePropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *visu;
  };

  class FCLPlanePropertyDialog : public FCLContourPropertyDialog {

    public:
      FCLPlanePropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *normal, *offset, *visu;
  };

  class FCLMeshPropertyDialog : public FCLContourPropertyDialog {

    public:
      FCLMeshPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *vertices, *triangles, *collisionStructure, *visu;
  };

  class GroupPropertyDialog : public ElementPropertyDialog {

    public:
      GroupPropertyDialog(Element *group, bool kinematics=true);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frameOfReference;
  };

  class DynamicSystemSolverPropertyDialog : public GroupPropertyDialog {
    protected:
      ExtWidget *environment, *constraintSolver, *impactSolver, *maxIter, *numericalJacobian, *projection, *gTol, *gdTol, *gddTol, *laTol, *LaTol, *gCorr, *gdCorr, *inverseKinetics, *initialProjection, *determineEquilibriumState, *useConstraintSolverForSmoothMotion, *useConstraintSolverForPlot;

    public:
      DynamicSystemSolverPropertyDialog(Element *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

  class ObjectPropertyDialog : public ElementPropertyDialog {

    public:
      ObjectPropertyDialog(Element *object);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      virtual void resizeGeneralizedPosition() { }
      virtual void resizeGeneralizedVelocity() { }
    protected:
      ExtWidget *q0, *u0, *R;
      void updateWidget() override { resizeGeneralizedPosition(); resizeGeneralizedVelocity(); }
  };

  class BodyPropertyDialog : public ObjectPropertyDialog {

    public:
      BodyPropertyDialog(Element *body);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
  };

  class RigidBodyPropertyDialog : public BodyPropertyDialog {

    public:
      RigidBodyPropertyDialog(Element *body);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      void resizeGeneralizedPosition() override;
      void resizeGeneralizedVelocity() override;
      int getqRelSize() const; 
      int getuRelSize() const; 
    protected:
      ExtWidget *K, *mass, *inertia, *frameForInertiaTensor, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *bodyFixedRepresentationOfAngularVelocity, *ombv, *ombvFrameRef, *weightArrow, *jointForceArrow, *jointMomentArrow;
  };

  class FlexibleBodyFFRPropertyDialog : public BodyPropertyDialog {
    Q_OBJECT

    public:
      FlexibleBodyFFRPropertyDialog(Element *body_);
      ~FlexibleBodyFFRPropertyDialog();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      void resizeGeneralizedPosition() override;
      void resizeGeneralizedVelocity() override;
      int getqRelSize() const;
      int getuRelSize() const;
    protected:
      ExtWidget *mass, *rdm, *rrdm, *Pdm, *rPdm, *PPdm, *Ke, *De, *beta, *Knl1, *Knl2, *ksigma0, *ksigma1, *K0t, *K0r, *K0om, *r, *A, *Phi, *Psi, *sigmahel, *sigmahen, *sigma0, *K0F, *K0M, *translation, *rotation, *translationDependentRotation, *coordinateTransformationForRotation, *ombv, *ombvNodes, *ombvColorRepresentation;
      ImportDialog *dialog{0};
      void updateWidget() override;
    protected slots:
      void import();
  };

  class ConstraintPropertyDialog : public ElementPropertyDialog {

    public:
      ConstraintPropertyDialog(Element *constraint);
  };

  class MechanicalConstraintPropertyDialog : public ConstraintPropertyDialog {

    public:
      MechanicalConstraintPropertyDialog(Element *constraint);
  };

  class GeneralizedConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {

    public:
      GeneralizedConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *support;
  };

  class GeneralizedGearConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedGearConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dependentBody, *independentBodies;
  };

  class GeneralizedDualConstraintPropertyDialog : public GeneralizedConstraintPropertyDialog {

    public:
      GeneralizedDualConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dependentBody, *independentBody;
  };

  class GeneralizedPositionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedPositionConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraintFunction;
      void updateWidget() override;
  };

  class GeneralizedVelocityConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedVelocityConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraintFunction, *x0;
      void updateWidget() override;
  };

  class GeneralizedAccelerationConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedAccelerationConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraintFunction, *x0;
      void updateWidget() override;
  };

  class JointConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {

    public:
      JointConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *independentBody, *dependentBodiesFirstSide, *dependentBodiesSecondSide, *refFrameID, *force, *moment, *connections, *q0;
      void updateWidget() override;
  };

  class GeneralizedConnectionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedConnectionConstraintPropertyDialog(Element *constraint);
  };

  class LinkPropertyDialog : public ElementPropertyDialog {

    public:
      LinkPropertyDialog(Element *link);
    protected:
  };

  class MechanicalLinkPropertyDialog : public LinkPropertyDialog {

    public:
      MechanicalLinkPropertyDialog(Element *link);
  };

  class FrameLinkPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      FrameLinkPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections;
  };

  class FixedFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FixedFrameLinkPropertyDialog(Element *link);
  };

  class FloatingFrameLinkPropertyDialog : public FrameLinkPropertyDialog {

    public:
      FloatingFrameLinkPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *refFrameID;
  };

  class RigidBodyLinkPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      RigidBodyLinkPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *support;
  };

  class DualRigidBodyLinkPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      DualRigidBodyLinkPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections;
  };

  class KineticExcitationPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      KineticExcitationPropertyDialog(Element *kineticExcitationr);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *forceFunction, *momentDirection, *momentFunction, *arrow;
      void updateWidget() override;
  };

  class SpringDamperPropertyDialog : public FixedFrameLinkPropertyDialog {

    public:
      SpringDamperPropertyDialog(Element *springDamper);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceFunction, *unloadedLength, *coilSpring;
  };

  class DirectionalSpringDamperPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      DirectionalSpringDamperPropertyDialog(Element *springDamper);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *unloadedLength, *forceFunction, *coilSpring;
  };

  class IsotropicRotationalSpringDamperPropertyDialog : public FixedFrameLinkPropertyDialog {

    public:
      IsotropicRotationalSpringDamperPropertyDialog(Element *springDamper);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *elasticMomentFunction, *dissipativeMomentFunction;
  };

  class JointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      JointPropertyDialog(Element *joint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *forceLaw, *momentDirection, *momentLaw, *integrate;
  };

  class ElasticJointPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      ElasticJointPropertyDialog(Element *joint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *forceDirection, *momentDirection, *function, *integrate;
      void updateWidget() override;
  };

  class GeneralizedSpringDamperPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedSpringDamperPropertyDialog(Element *springDamper);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *unloadedLength;
  };

  class GeneralizedFrictionPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedFrictionPropertyDialog(Element *friction);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionForceLaw, *frictionImpactLaw, *normalForceFunction;
  };

  class GeneralizedGearPropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      GeneralizedGearPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *gearOutput, *gearInput;
  };

  class GeneralizedElasticConnectionPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedElasticConnectionPropertyDialog(Element *connection);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function;
      void updateWidget() override;
  };

  class GeneralizedElasticStructurePropertyDialog : public RigidBodyLinkPropertyDialog {

    public:
      GeneralizedElasticStructurePropertyDialog(Element *connection);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function, *rigidBody;
  };

  class ContactPropertyDialog : public LinkPropertyDialog {

    public:
      ContactPropertyDialog(Element *contact);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *contactForceLaw, *contactImpactLaw, *frictionForceLaw, *frictionImpactLaw, *connections, *searchAllContactPoints, *initialGuess, *tolerance, *maxNumContacts;
  };

  class DiskContactPropertyDialog : public LinkPropertyDialog {

    public:
      DiskContactPropertyDialog(Element *contact);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *contactForceLaw, *contactImpactLaw, *frictionForceLaw, *frictionImpactLaw, *connections;
  };

  class ObserverPropertyDialog : public ElementPropertyDialog {

    public:
      ObserverPropertyDialog(Element *observer);
  };

  class KinematicCoordinatesObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      KinematicCoordinatesObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame, *frameOfReference, *position, *velocity, *acceleration;
  };

  class RelativeKinematicsObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RelativeKinematicsObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame, *refFrame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
  };

  class MechanicalLinkObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      MechanicalLinkObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *link, *forceArrow, *momentArrow;
  };

  class MechanicalConstraintObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      MechanicalConstraintObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraint, *forceArrow, *momentArrow;
  };

  class ContactObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      ContactObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *link, *forceArrow, *momentArrow, *contactPoints, *normalForceArrow, *frictionArrow;
  };

  class FrameObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      FrameObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame, *position, *velocity, *angularVelocity, *acceleration, *angularAcceleration;
  };

  class RigidBodyObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RigidBodyObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *body, *frameOfReference, *weight, *jointForce, *jointMoment, *axisOfRotation, *momentum, *angularMomentum, *derivativeOfMomentum, *derivativeOfAngularMomentum;
  };

  class RigidBodySystemObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      RigidBodySystemObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *bodies, *frameOfReference, *position, *velocity, *acceleration, *weight, *momentum, *angularMomentum, *derivativeOfMomentum, *derivativeOfAngularMomentum;
  };

  class SignalPropertyDialog: public LinkPropertyDialog {

    public:
      SignalPropertyDialog(Element *signal);
  };

  class SensorPropertyDialog : public SignalPropertyDialog {

    public:
      SensorPropertyDialog(Element *sensor);
  };

  class ObjectSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ObjectSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *object;
  };

  class LinkSensorPropertyDialog : public SensorPropertyDialog {

    public:
      LinkSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *link;
  };

  class FrameSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FrameSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frame;
  };

  class FunctionSensorPropertyDialog : public SensorPropertyDialog {

    public:
      FunctionSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *function;
  };

  class ContactSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ContactSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *contact;
  };

  class MultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      MultiplexerPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal;
  };

  class DemultiplexerPropertyDialog : public SignalPropertyDialog {

    public:
      DemultiplexerPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *indices;
  };

  class LinearTransferSystemPropertyDialog : public SignalPropertyDialog {

    public:
      LinearTransferSystemPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal, *A, *B, *C, *D;
      void updateWidget() override;
  };

  class SignalOperationPropertyDialog : public SignalPropertyDialog {
    Q_OBJECT

    public:
      SignalOperationPropertyDialog(Element *signal);
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
      ExternSignalSourcePropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *sourceSize;
  };

  class ExternSignalSinkPropertyDialog : public SignalPropertyDialog {

    public:
      ExternSignalSinkPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *inputSignal;
  };

}

#endif
