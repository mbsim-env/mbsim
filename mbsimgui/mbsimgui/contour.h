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

#ifndef _CONTOUR__H_
#define _CONTOUR__H_

#include "element.h"

namespace MBSimGUI {

  class Contour : public Element {
    public:
      Contour();
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Contour"; }
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new ContourPropertyDialog(this); }
      QMenu* createContextMenu() override { return new ContourContextMenu(this); }
  };

  class UnknownContour : public Contour {
    public:
      QString getType() const override { return "Unknown contour"; }
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class RigidContour : public Contour {
    public:
      PropertyDialog* createPropertyDialog() override { return new RigidContourPropertyDialog(this); }
  };

  class Point : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Point"; }
      QString getType() const override { return "Point"; }
      PropertyDialog* createPropertyDialog() override { return new PointPropertyDialog(this); }
  };

  class Line : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Line"; }
      QString getType() const override { return "Line"; }
      PropertyDialog* createPropertyDialog() override { return new LinePropertyDialog(this); }
  };

  class Plane : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Plane"; }
      QString getType() const override { return "Plane"; }
      PropertyDialog* createPropertyDialog() override { return new PlanePropertyDialog(this); }
  };

  class Sphere : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Sphere"; }
      QString getType() const override { return "Sphere"; }
      PropertyDialog* createPropertyDialog() override { return new SpherePropertyDialog(this); }
  };

  class Circle : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Circle"; }
      QString getType() const override { return "Circle"; }
      PropertyDialog* createPropertyDialog() override { return new CirclePropertyDialog(this); }
  };

  class Cuboid : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Cuboid"; }
      QString getType() const override { return "Cuboid"; }
      PropertyDialog* createPropertyDialog() override { return new CuboidPropertyDialog(this); }
  };

  class LineSegment : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"LineSegment"; }
      QString getType() const override { return "Line segment"; }
      PropertyDialog* createPropertyDialog() override { return new LineSegmentPropertyDialog(this); }
  };

  class PlanarContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PlanarContour"; }
      QString getType() const override { return "Planar contour"; }
      PropertyDialog* createPropertyDialog() override { return new PlanarContourPropertyDialog(this); }
  };

  class PlanarNurbsContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PlanarNurbsContour"; }
      QString getType() const override { return "Planar nurbs contour"; }
      PropertyDialog* createPropertyDialog() override { return new PlanarNurbsContourPropertyDialog(this); }
  };

  class SpatialContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SpatialContour"; }
      QString getType() const override { return "Spatial contour"; }
      PropertyDialog* createPropertyDialog() override { return new SpatialContourPropertyDialog(this); }
  };

  class SpatialNurbsContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"SpatialNurbsContour"; }
      QString getType() const override { return "Spatial nurbs contour"; }
      PropertyDialog* createPropertyDialog() override { return new SpatialNurbsContourPropertyDialog(this); }
  };

  class Disk : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Disk"; }
      QString getType() const override { return "Disk"; }
      PropertyDialog* createPropertyDialog() override { return new DiskPropertyDialog(this); }
  };

  class CylindricalGear : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"CylindricalGear"; }
      QString getType() const override { return "Cylindrical gear"; }
      PropertyDialog* createPropertyDialog() override { return new CylindricalGearPropertyDialog(this); }
  };

  class Rack : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"Rack"; }
      QString getType() const override { return "Rack"; }
      PropertyDialog* createPropertyDialog() override { return new RackPropertyDialog(this); }
  };

  class BevelGear : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"BevelGear"; }
      QString getType() const override { return "Bevel gear"; }
      PropertyDialog* createPropertyDialog() override { return new BevelGearPropertyDialog(this); }
  };

  class PlanarGear : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIM%"PlanarGear"; }
      QString getType() const override { return "Planar gear"; }
      PropertyDialog* createPropertyDialog() override { return new PlanarGearPropertyDialog(this); }
  };

  class FlexiblePlanarNurbsContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFLEX%"FlexiblePlanarNurbsContour"; }
      QString getType() const override { return "Flexible planar nurbs contour"; }
      PropertyDialog* createPropertyDialog() override { return new FlexiblePlanarNurbsContourPropertyDialog(this); }
  };

  class FlexiblePlanarFfrNurbsContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFLEX%"FlexiblePlanarFfrNurbsContour"; }
      QString getType() const override { return "Flexible planar ffr nurbs contour"; }
      PropertyDialog* createPropertyDialog() override { return new FlexiblePlanarNurbsContourPropertyDialog(this); }
  };

  class FlexibleSpatialNurbsContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFLEX%"FlexibleSpatialNurbsContour"; }
      QString getType() const override { return "Flexible spatial nurbs contour"; }
      PropertyDialog* createPropertyDialog() override { return new FlexibleSpatialNurbsContourPropertyDialog(this); }
  };

  class FlexibleSpatialFfrNurbsContour : public RigidContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFLEX%"FlexibleSpatialFfrNurbsContour"; }
      QString getType() const override { return "Flexible spatial ffr nurbs contour"; }
      PropertyDialog* createPropertyDialog() override { return new FlexibleSpatialNurbsContourPropertyDialog(this); }
  };

  class FclContour : public RigidContour {
  };

  class FclBox : public FclContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFCL%"FclBox"; }
      QString getType() const override { return "Fcl box"; }
      PropertyDialog* createPropertyDialog() override { return new FclBoxPropertyDialog(this); }
  };

  class FclSphere : public FclContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFCL%"FclSphere"; }
      QString getType() const override { return "Fcl sphere"; }
      PropertyDialog* createPropertyDialog() override { return new FclSpherePropertyDialog(this); }
  };

  class FclPlane : public FclContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFCL%"FclPlane"; }
      QString getType() const override { return "Fcl plane"; }
      PropertyDialog* createPropertyDialog() override { return new FclPlanePropertyDialog(this); }
  };

  class FclMesh : public FclContour {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMFCL%"FclMesh"; }
      QString getType() const override { return "Fcl mesh"; }
      PropertyDialog* createPropertyDialog() override { return new FclMeshPropertyDialog(this); }
  };

}

#endif
