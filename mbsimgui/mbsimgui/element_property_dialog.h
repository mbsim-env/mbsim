/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

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

#ifndef _ELEMENT_PROPERTY_DIALOG_H_
#define _ELEMENT_PROPERTY_DIALOG_H_

#include "property_dialog.h"

namespace MBSimGUI {

  class ExtWidget;
  class Element;

  class ElementPropertyDialog : public EmbedItemPropertyDialog {

    public:
      ElementPropertyDialog(Element *element);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      Element* getElement() const;
    protected:
      ExtWidget *name, *plotFeature;
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

  class InterfaceNodeFramePropertyDialog : public FramePropertyDialog {

    public:
      InterfaceNodeFramePropertyDialog(Element *frame, bool approx=false);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodeNumbers, *weightingFactors, *approx;
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

  class CylindricalGearPropertyDialog : public RigidContourPropertyDialog {

    public:
      CylindricalGearPropertyDialog(Element *gear);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *numberOfTeeth, *width, *helixAngle, *module, *pressureAngle, *backlash, *externalToothed, *outsideRadius, *visu;
  };

  class RackPropertyDialog : public RigidContourPropertyDialog {

    public:
      RackPropertyDialog(Element *rack);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *numberOfTeeth, *height, *width, *helixAngle, *module, *pressureAngle, *backlash, *visu;
  };

  class BevelGearPropertyDialog : public RigidContourPropertyDialog {

    public:
      BevelGearPropertyDialog(Element *gear);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *numberOfTeeth, *width, *helixAngle, *pitchAngle, *module, *pressureAngle, *backlash, *visu;
  };

  class PlanarGearPropertyDialog : public RigidContourPropertyDialog {

    public:
      PlanarGearPropertyDialog(Element *gear);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *numberOfTeeth, *height, *width, *helixAngle, *module, *pressureAngle, *backlash, *visu;
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

  class FclContourPropertyDialog : public RigidContourPropertyDialog {
    public:
      FclContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *computeLocalAABB;
  };

  class FclBoxPropertyDialog : public FclContourPropertyDialog {

    public:
      FclBoxPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length, *visu;
  };

  class FclSpherePropertyDialog : public FclContourPropertyDialog {

    public:
      FclSpherePropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *visu;
  };

  class FclPlanePropertyDialog : public FclContourPropertyDialog {

    public:
      FclPlanePropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *normal, *offset, *visu;
  };

  class FclMeshPropertyDialog : public FclContourPropertyDialog {

    public:
      FclMeshPropertyDialog(Element *contour);
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
      ExtWidget *environments, *constraintSolver, *impactSolver, *maxIter, *highIter, *numericalJacobian, *stopIfNoConvergence, *projection, *gTol, *gdTol, *gddTol, *laTol, *LaTol, *gCorr, *gdCorr, *inverseKinetics, *initialProjection, *determineEquilibriumState, *useConstraintSolverForSmoothMotion, *useConstraintSolverForPlot;

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
      int getuRelSize() const { return getqRelSize(); }
    protected:
      ExtWidget *K, *mass, *inertia, *frameForInertiaTensor, *translation, *rotation, *translationDependentRotation, *generalizedVelocityOfRotation, *ombv, *ombvFrameRef, *weightArrow, *jointForceArrow, *jointMomentArrow;
  };

  class GenericFlexibleFfrBodyPropertyDialog : public BodyPropertyDialog {

    public:
      GenericFlexibleFfrBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      void resizeGeneralizedPosition() override;
      void resizeGeneralizedVelocity() override;
      int getqRelSize() const;
      int getuRelSize() const { return getqRelSize(); }
      virtual int getqERelSize() const { return 0; }
    protected:
      ExtWidget *mRed, *minimumFrequency, *maximumFrequency, *modes, *mDamping, *translation, *rotation, *translationDependentRotation, *generalizedVelocityOfRotation;
  };

  class FlexibleFfrBodyPropertyDialog : public GenericFlexibleFfrBodyPropertyDialog {

    public:
      FlexibleFfrBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      int getqERelSize() const override;
    protected:
      ExtWidget *mass, *rdm, *rrdm, *Pdm, *rPdm, *PPdm, *Ke, *De, *beta, *Knl1, *Knl2, *ksigma0, *ksigma1, *K0t, *K0r, *K0om, *nodeNumbers, *r, *A, *Phi, *Psi, *sigmahel, *sigmahen, *sigma0, *K0F, *K0M, *ombv, *ombvColorRepresentation, *plotNodes;
      void updateWidget() override;
  };

  class CalculixBodyPropertyDialog : public GenericFlexibleFfrBodyPropertyDialog {

    public:
      CalculixBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      int getqERelSize() const override;
    protected:
      ExtWidget *resultFileName, *formalism, *beta, *ombv, *plotNodes;
  };

  class FlexibleFfrBeamPropertyDialog : public GenericFlexibleFfrBodyPropertyDialog {

    public:
      FlexibleFfrBeamPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      int getqERelSize() const override;
    protected:
      ExtWidget *n, *l, *A, *I, *E, *rho, *ten, *beny, *benz, *tor, *beta, *bc, *inodes, *nmodes, *fbnm, *ombv, *plotNodes;
  };

  class FiniteElementsFfrBodyPropertyDialog : public GenericFlexibleFfrBodyPropertyDialog {

    public:
      FiniteElementsFfrBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      int getqERelSize() const override;
    protected:
      ExtWidget *E, *rho, *nu, *nodes, *elements, *type, *beta, *bc, *inodes, *nmodes, *fbnm, *ombv, *plotNodes;
  };

  class ExternalFiniteElementsFfrBodyPropertyDialog : public GenericFlexibleFfrBodyPropertyDialog {

    public:
      ExternalFiniteElementsFfrBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      int getqERelSize() const override;
    protected:
      ExtWidget *net, *ner, *nodes, *mass, *stiff, *formalism, *beta, *bc, *inodes, *nmodes, *fbnm, *ombv, *plotNodes;
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
      ExtWidget *x0, *constraintFunction;
      void updateWidget() override;
  };

  class GeneralizedAccelerationConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedAccelerationConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *x0, *constraintFunction;
      void updateWidget() override;
  };

  class JointConstraintPropertyDialog : public MechanicalConstraintPropertyDialog {

    public:
      JointConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *independentBody, *dependentBodiesFirstSide, *dependentBodiesSecondSide, *refFrame, *force, *moment, *connections, *q0;
      void updateWidget() override;
  };

  class GeneralizedConnectionConstraintPropertyDialog : public GeneralizedDualConstraintPropertyDialog {

    public:
      GeneralizedConnectionConstraintPropertyDialog(Element *constraint);
  };

  class InverseKinematicsConstraintPropertyDialog : public ConstraintPropertyDialog {

    public:
      InverseKinematicsConstraintPropertyDialog(Element *constraint);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *kinematics, *frame, *translation, *rotation, *q0;
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
      ExtWidget *refFrame;
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
      void updateWidget() override;
      void updateFunctionCheckState();
      void updateDirectionsCheckState();
      ExtWidget *forceDirection, *momentDirection, *function, *integrate;
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

  class GeneralizedClutchPropertyDialog : public DualRigidBodyLinkPropertyDialog {

    public:
      GeneralizedClutchPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frictionForceLaw, *frictionImpactLaw, *normalForceFunction, *engagementFunction;
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
      ExtWidget *contactForceLaw, *contactImpactLaw, *frictionForceLaw, *frictionImpactLaw, *connections, *globalSearch, *initialGlobalSearch, *initialGuess, *tolerance, *maxNumContacts;
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

  class InverseKinematicsConstraintObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      InverseKinematicsConstraintObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraint, *ombv;
  };

  class SignalObserverPropertyDialog : public ObserverPropertyDialog {

    public:
      SignalObserverPropertyDialog(Element *observer);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *signal, *position, *ombv;
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

  class RigidBodyJointForceSensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      RigidBodyJointForceSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class RigidBodyJointMomentSensorPropertyDialog : public ObjectSensorPropertyDialog {

    public:
      RigidBodyJointMomentSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class LinkSensorPropertyDialog : public SensorPropertyDialog {

    public:
      LinkSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *link;
  };

  class MechanicalLinkForceSensorPropertyDialog : public LinkSensorPropertyDialog {

    public:
      MechanicalLinkForceSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class MechanicalLinkMomentSensorPropertyDialog : public LinkSensorPropertyDialog {

    public:
      MechanicalLinkMomentSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class ConstraintSensorPropertyDialog : public SensorPropertyDialog {

    public:
      ConstraintSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *constraint;
  };

  class MechanicalConstraintForceSensorPropertyDialog : public ConstraintSensorPropertyDialog {

    public:
      MechanicalConstraintForceSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
  };

  class MechanicalConstraintMomentSensorPropertyDialog : public ConstraintSensorPropertyDialog {

    public:
      MechanicalConstraintMomentSensorPropertyDialog(Element *sensor);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *number;
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
      ExtWidget *contact, *number;
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
      ExtWidget *x0, *inputSignal, *A, *B, *C, *D;
      void updateWidget() override;
  };

  class NonlinearTransferSystemPropertyDialog : public SignalPropertyDialog {

    public:
      NonlinearTransferSystemPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *x0, *inputSignal, *F, *H;
      void updateWidget() override;
  };

  class SignalOperationPropertyDialog : public SignalPropertyDialog {

    public:
      SignalOperationPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      void updateWidget() override;
      void updateFunctionFactory();
      ExtWidget *inputSignal, *function;
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

  class SwitchPropertyDialog : public SignalPropertyDialog {

    public:
      SwitchPropertyDialog(Element *signal);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dataSignal1, *dataSignal2, *controlSignal, *threshold, *rootFinding;
  };

  class UniversalGravitationPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      UniversalGravitationPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections, *gravitationalConstant, *enableOpenMBV;
  };

  class WeightPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      WeightPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections, *gravityFunction, *enableOpenMBV;
  };

  class BuoyancyPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      BuoyancyPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *displacedVolume, *densityFunction, *gravityFunction, *enableOpenMBV;
  };

  class DragPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      DragPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dragFunction, *enableOpenMBV;
  };

  class AerodynamicsPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      AerodynamicsPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *densityFunction, *coefficientFunction, *referenceSurface, *windSpeed, *enableOpenMBV;
  };

}

#endif
