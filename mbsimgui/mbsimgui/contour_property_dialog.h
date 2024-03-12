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

#ifndef _CONTOUR_PROPERTY_DIALOG_H_
#define _CONTOUR_PROPERTY_DIALOG_H_

#include "element_property_dialog.h"

namespace MBSimGUI {

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

  class CylinderPropertyDialog : public RigidContourPropertyDialog {

    public:
      CylinderPropertyDialog(Element *circle);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *height, *solid, *visu;
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

  class TyrePropertyDialog : public RigidContourPropertyDialog {

    public:
      TyrePropertyDialog(Element *tyre);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *r, *rRim, *w, *shape, *cp, *visu;
      void updateWidget() override;
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

  class NodesContourPropertyDialog : public ContourPropertyDialog {

    public:
      NodesContourPropertyDialog(Element *contour);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodeNumbers, *visu;
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

}

#endif
