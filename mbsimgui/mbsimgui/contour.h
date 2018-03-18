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
      ElementPropertyDialog* createPropertyDialog() override {return new ContourPropertyDialog(this);}
      QMenu* createContextMenu() override { return new ContourContextMenu(this); }
  };

  class RigidContour : public Contour {
    public:
      ElementPropertyDialog* createPropertyDialog() override {return new RigidContourPropertyDialog(this);}
  };

  class Point : public RigidContour {
    public:
    QString getType() const override { return "Point"; }
    ElementPropertyDialog* createPropertyDialog() override {return new PointPropertyDialog(this);}
  };

  class Line : public RigidContour {
    public:
      QString getType() const override { return "Line"; }
      ElementPropertyDialog* createPropertyDialog() override {return new LinePropertyDialog(this);}
  };

  class Plane : public RigidContour {
    public:
      QString getType() const override { return "Plane"; }
      ElementPropertyDialog* createPropertyDialog() override {return new PlanePropertyDialog(this);}
  };

  class Sphere : public RigidContour {
    public:
      QString getType() const override { return "Sphere"; }
      ElementPropertyDialog* createPropertyDialog() override {return new SpherePropertyDialog(this);}
  };

  class Circle : public RigidContour {
    public:
      QString getType() const override { return "Circle"; }
      ElementPropertyDialog* createPropertyDialog() override {return new CirclePropertyDialog(this);}
  };

  class Cuboid : public RigidContour {
    public:
      QString getType() const override { return "Cuboid"; }
      ElementPropertyDialog* createPropertyDialog() override {return new CuboidPropertyDialog(this);}
  };

  class LineSegment : public RigidContour {
    public:
      QString getType() const override { return "LineSegment"; }
      ElementPropertyDialog* createPropertyDialog() override {return new LineSegmentPropertyDialog(this);}
  };

  class PlanarContour : public RigidContour {
    public:
      QString getType() const override { return "PlanarContour"; }
      ElementPropertyDialog* createPropertyDialog() override {return new PlanarContourPropertyDialog(this);}
  };

  class SpatialContour : public RigidContour {
    public:
      QString getType() const override { return "SpatialContour"; }
      ElementPropertyDialog* createPropertyDialog() override {return new SpatialContourPropertyDialog(this);}
  };

  class FCLBox : public RigidContour {
    public:
      QString getType() const override { return "FCLBox"; }
      ElementPropertyDialog* createPropertyDialog() override {return new FCLBoxPropertyDialog(this);}
  };

  class FCLSphere : public RigidContour {
    public:
      QString getType() const override { return "FCLSphere"; }
      ElementPropertyDialog* createPropertyDialog() override {return new FCLSpherePropertyDialog(this);}
  };

  class FCLPlane : public RigidContour {
    public:
      QString getType() const override { return "FCLPlane"; }
      ElementPropertyDialog* createPropertyDialog() override {return new FCLPlanePropertyDialog(this);}
  };

  class FCLMesh : public RigidContour {
    public:
      QString getType() const override { return "FCLMesh"; }
      ElementPropertyDialog* createPropertyDialog() override {return new FCLMeshPropertyDialog(this);}
  };

}

#endif
