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
    MBSIMGUI_OBJECTFACTORY_CLASS(Contour, Element, MBSIM%"Contour", "Contour");
    public:
      Contour();
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new ContourPropertyDialog(this); }
      QMenu* createContextMenu() override { return new ContourContextMenu(this); }
  };

  class UnknownContour : public Contour {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownContour, Contour, MBSIM%"UnknownContour_dummy", "Unknown contour");
    public:
      UnknownContour();
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class RigidContour : public Contour {
    MBSIMGUI_OBJECTFACTORY_CLASS(RigidContour, Contour, MBSIM%"RigidContour", "Rigid contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new RigidContourPropertyDialog(this); }
  };

  class Point : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Point, RigidContour, MBSIM%"Point", "Point");
    public:
      PropertyDialog* createPropertyDialog() override { return new PointPropertyDialog(this); }
  };

  class Line : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Line, RigidContour, MBSIM%"Line", "Line");
    public:
      PropertyDialog* createPropertyDialog() override { return new LinePropertyDialog(this); }
  };

  class Plane : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Plane, RigidContour, MBSIM%"Plane", "Plane");
    public:
      PropertyDialog* createPropertyDialog() override { return new PlanePropertyDialog(this); }
  };

  class Sphere : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Sphere, RigidContour, MBSIM%"Sphere", "Sphere");
    public:
      PropertyDialog* createPropertyDialog() override { return new SpherePropertyDialog(this); }
  };

  class Circle : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Circle, RigidContour, MBSIM%"Circle", "Circle");
    public:
      PropertyDialog* createPropertyDialog() override { return new CirclePropertyDialog(this); }
  };

  class Cylinder : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Circle, RigidContour, MBSIM%"Cylinder", "Cylinder");
    public:
      PropertyDialog* createPropertyDialog() override { return new CylinderPropertyDialog(this); }
  };

  class Cuboid : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Cuboid, RigidContour, MBSIM%"Cuboid", "Cuboid");
    public:
      PropertyDialog* createPropertyDialog() override { return new CuboidPropertyDialog(this); }
  };

  class LineSegment : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(LineSegment, RigidContour, MBSIM%"LineSegment", "Line segment");
    public:
      PropertyDialog* createPropertyDialog() override { return new LineSegmentPropertyDialog(this); }
  };

  class PlanarContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(PlanarContour, RigidContour, MBSIM%"PlanarContour", "Planar contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new PlanarContourPropertyDialog(this); }
  };

  class PlanarNurbsContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(PlanarNurbsContour, RigidContour, MBSIM%"PlanarNurbsContour", "Planar nurbs contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new PlanarNurbsContourPropertyDialog(this); }
  };

  class SpatialContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(SpatialContour, RigidContour, MBSIM%"SpatialContour", "Spatial contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new SpatialContourPropertyDialog(this); }
  };

  class SpatialNurbsContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(SpatialNurbsContour, RigidContour, MBSIM%"SpatialNurbsContour", "Spatial nurbs contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new SpatialNurbsContourPropertyDialog(this); }
  };

  class Disk : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Disk, RigidContour, MBSIM%"Disk", "Disk");
    public:
      PropertyDialog* createPropertyDialog() override { return new DiskPropertyDialog(this); }
  };

  class CylindricalGear : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(CylindricalGear, RigidContour, MBSIM%"CylindricalGear", "Cylindrical gear");
    public:
      PropertyDialog* createPropertyDialog() override { return new CylindricalGearPropertyDialog(this); }
  };

  class Rack : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Rack, RigidContour, MBSIM%"Rack", "Rack");
    public:
      PropertyDialog* createPropertyDialog() override { return new RackPropertyDialog(this); }
  };

  class BevelGear : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(BevelGear, RigidContour, MBSIM%"BevelGear", "Bevel gear");
    public:
      PropertyDialog* createPropertyDialog() override { return new BevelGearPropertyDialog(this); }
  };

  class PlanarGear : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(PlanarGear, RigidContour, MBSIM%"PlanarGear", "Planar gear");
    public:
      PropertyDialog* createPropertyDialog() override { return new PlanarGearPropertyDialog(this); }
  };

  class Tyre : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(Tyre, RigidContour, MBSIM%"Tyre", "Tyre");
    public:
      PropertyDialog* createPropertyDialog() override { return new TyrePropertyDialog(this); }
  };

  class FlexiblePlanarNurbsContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FlexiblePlanarNurbsContour, RigidContour, MBSIMFLEX%"FlexiblePlanarNurbsContour", "Flexible planar nurbs contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new FlexiblePlanarNurbsContourPropertyDialog(this); }
  };

  class FlexiblePlanarFfrNurbsContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FlexiblePlanarFfrNurbsContour, RigidContour, MBSIMFLEX%"FlexiblePlanarFfrNurbsContour", "Flexible planar ffr nurbs contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new FlexiblePlanarNurbsContourPropertyDialog(this); }
  };

  class FlexibleSpatialNurbsContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FlexibleSpatialNurbsContour, RigidContour, MBSIMFLEX%"FlexibleSpatialNurbsContour", "Flexible spatial nurbs contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new FlexibleSpatialNurbsContourPropertyDialog(this); }
  };

  class FlexibleSpatialFfrNurbsContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FlexibleSpatialFfrNurbsContour, RigidContour, MBSIMFLEX%"FlexibleSpatialFfrNurbsContour", "Flexible spatial ffr nurbs contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new FlexibleSpatialNurbsContourPropertyDialog(this); }
  };

  class NodesContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(NodesContour, RigidContour, MBSIMFLEX%"NodesContour", "Nodes contour");
    public:
      PropertyDialog* createPropertyDialog() override { return new NodesContourPropertyDialog(this); }
  };

  class FclContour : public RigidContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FclContour, RigidContour, MBSIMFLEX%"FclContour", "Fcl contour");
  };

  class FclBox : public FclContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FclBox, FclContour, MBSIMFCL%"FclBox", "Fcl box");
    public:
      PropertyDialog* createPropertyDialog() override { return new FclBoxPropertyDialog(this); }
  };

  class FclSphere : public FclContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FclSphere, FclContour, MBSIMFCL%"FclSphere", "Fcl sphere");
    public:
      PropertyDialog* createPropertyDialog() override { return new FclSpherePropertyDialog(this); }
  };

  class FclPlane : public FclContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FclPlane, FclContour, MBSIMFCL%"FclPlane", "Fcl plane");
    public:
      PropertyDialog* createPropertyDialog() override { return new FclPlanePropertyDialog(this); }
  };

  class FclMesh : public FclContour {
    MBSIMGUI_OBJECTFACTORY_CLASS(FclMesh, FclContour, MBSIMFCL%"FclMesh", "Fcl mesh");
    public:
      PropertyDialog* createPropertyDialog() override { return new FclMeshPropertyDialog(this); }
  };

}

#endif
