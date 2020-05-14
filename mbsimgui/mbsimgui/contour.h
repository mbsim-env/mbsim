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

#ifndef _CONTOUR__H_
#define _CONTOUR__H_

#include "element.h"

namespace MBSimGUI {

  class Contour : public Element {
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override {return new ContourPropertyDialog(this);}
      QMenu* createContextMenu() override { return new ContourContextMenu(this); }
  };

  class UnknownContour : public Contour {
    public:
      QString getType() const override { return "UnknownContour"; }
      PropertyDialog* createPropertyDialog() override {return new UnknownElementPropertyDialog(this);}
  };

  class RigidContour : public Contour {
    public:
      PropertyDialog* createPropertyDialog() override {return new RigidContourPropertyDialog(this);}
  };

  class Point : public RigidContour {
    public:
    QString getType() const override { return "Point"; }
    PropertyDialog* createPropertyDialog() override {return new PointPropertyDialog(this);}
  };

  class Line : public RigidContour {
    public:
      QString getType() const override { return "Line"; }
      PropertyDialog* createPropertyDialog() override {return new LinePropertyDialog(this);}
  };

  class Plane : public RigidContour {
    public:
      QString getType() const override { return "Plane"; }
      PropertyDialog* createPropertyDialog() override {return new PlanePropertyDialog(this);}
  };

  class Sphere : public RigidContour {
    public:
      QString getType() const override { return "Sphere"; }
      PropertyDialog* createPropertyDialog() override {return new SpherePropertyDialog(this);}
  };

  class Circle : public RigidContour {
    public:
      QString getType() const override { return "Circle"; }
      PropertyDialog* createPropertyDialog() override {return new CirclePropertyDialog(this);}
  };

  class Cuboid : public RigidContour {
    public:
      QString getType() const override { return "Cuboid"; }
      PropertyDialog* createPropertyDialog() override {return new CuboidPropertyDialog(this);}
  };

  class LineSegment : public RigidContour {
    public:
      QString getType() const override { return "LineSegment"; }
      PropertyDialog* createPropertyDialog() override {return new LineSegmentPropertyDialog(this);}
  };

  class PlanarContour : public RigidContour {
    public:
      QString getType() const override { return "PlanarContour"; }
      PropertyDialog* createPropertyDialog() override {return new PlanarContourPropertyDialog(this);}
  };

  class PlanarNurbsContour : public RigidContour {
    public:
      QString getType() const override { return "PlanarNurbsContour"; }
      PropertyDialog* createPropertyDialog() override {return new PlanarNurbsContourPropertyDialog(this);}
  };

  class SpatialContour : public RigidContour {
    public:
      QString getType() const override { return "SpatialContour"; }
      PropertyDialog* createPropertyDialog() override {return new SpatialContourPropertyDialog(this);}
  };

  class SpatialNurbsContour : public RigidContour {
    public:
      QString getType() const override { return "SpatialNurbsContour"; }
      PropertyDialog* createPropertyDialog() override {return new SpatialNurbsContourPropertyDialog(this);}
  };

  class Disk : public RigidContour {
    public:
      QString getType() const override { return "Disk"; }
      PropertyDialog* createPropertyDialog() override {return new DiskPropertyDialog(this);}
  };

  class CylindricalGear : public RigidContour {
    public:
      QString getType() const override { return "CylindricalGear"; }
      PropertyDialog* createPropertyDialog() override {return new CylindricalGearPropertyDialog(this);}
  };

  class Rack : public RigidContour {
    public:
      QString getType() const override { return "Rack"; }
      PropertyDialog* createPropertyDialog() override {return new RackPropertyDialog(this);}
  };

  class BevelGear : public RigidContour {
    public:
      QString getType() const override { return "BevelGear"; }
      PropertyDialog* createPropertyDialog() override {return new BevelGearPropertyDialog(this);}
  };

  class PlanarGear : public RigidContour {
    public:
      QString getType() const override { return "PlanarGear"; }
      PropertyDialog* createPropertyDialog() override {return new PlanarGearPropertyDialog(this);}
  };

  class FlexiblePlanarNurbsContour : public RigidContour {
    public:
      QString getType() const override { return "FlexiblePlanarNurbsContour"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMFLEX; }
      PropertyDialog* createPropertyDialog() override {return new FlexiblePlanarNurbsContourPropertyDialog(this);}
  };

  class FlexiblePlanarFfrNurbsContour : public RigidContour {
    public:
      QString getType() const override { return "FlexiblePlanarFfrNurbsContour"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMFLEX; }
      PropertyDialog* createPropertyDialog() override {return new FlexiblePlanarNurbsContourPropertyDialog(this);}
  };

  class FlexibleSpatialNurbsContour : public RigidContour {
    public:
      QString getType() const override { return "FlexibleSpatialNurbsContour"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMFLEX; }
      PropertyDialog* createPropertyDialog() override {return new FlexibleSpatialNurbsContourPropertyDialog(this);}
  };

  class FlexibleSpatialFfrNurbsContour : public RigidContour {
    public:
      QString getType() const override { return "FlexibleSpatialFfrNurbsContour"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMFLEX; }
      PropertyDialog* createPropertyDialog() override {return new FlexibleSpatialNurbsContourPropertyDialog(this);}
  };

  class FclContour : public RigidContour {
    public:
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMFCL; }
  };

  class FclBox : public FclContour {
    public:
      QString getType() const override { return "FclBox"; }
      PropertyDialog* createPropertyDialog() override {return new FclBoxPropertyDialog(this);}
  };

  class FclSphere : public FclContour {
    public:
      QString getType() const override { return "FclSphere"; }
      PropertyDialog* createPropertyDialog() override {return new FclSpherePropertyDialog(this);}
  };

  class FclPlane : public FclContour {
    public:
      QString getType() const override { return "FclPlane"; }
      PropertyDialog* createPropertyDialog() override {return new FclPlanePropertyDialog(this);}
  };

  class FclMesh : public FclContour {
    public:
      QString getType() const override { return "FclMesh"; }
      PropertyDialog* createPropertyDialog() override {return new FclMeshPropertyDialog(this);}
  };

}

#endif
