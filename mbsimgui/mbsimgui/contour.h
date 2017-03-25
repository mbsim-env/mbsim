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
      Contour(const QString &str="");
      ElementPropertyDialog* createPropertyDialog() {return new ContourPropertyDialog(this);}
  };

  class RigidContour : public Contour {
    public:
      RigidContour(const QString &str="");
//      void setSavedFrameOfReference(const QString &str);
      ElementPropertyDialog* createPropertyDialog() {return new RigidContourPropertyDialog(this);}
  };

  class Point : public RigidContour {
    public:
    Point(const QString &str="");
    QString getType() const { return "Point"; }
    ElementPropertyDialog* createPropertyDialog() {return new PointPropertyDialog(this);}
  };

  class Line : public RigidContour {
    public:
      Line(const QString &str="");
      QString getType() const { return "Line"; }
      ElementPropertyDialog* createPropertyDialog() {return new LinePropertyDialog(this);}
  };

  class Plane : public RigidContour {
    public:
      Plane(const QString &str="");
      QString getType() const { return "Plane"; }
      ElementPropertyDialog* createPropertyDialog() {return new PlanePropertyDialog(this);}
  };

  class Sphere : public RigidContour {
    public:
      Sphere(const QString &str="");
      QString getType() const { return "Sphere"; }
      ElementPropertyDialog* createPropertyDialog() {return new SpherePropertyDialog(this);}
  };

  class Circle : public RigidContour {
    public:
      Circle(const QString &str="");
      QString getType() const { return "Circle"; }
      ElementPropertyDialog* createPropertyDialog() {return new CirclePropertyDialog(this);}
  };

  class Cuboid : public RigidContour {
    public:
      Cuboid(const QString &str="");
      QString getType() const { return "Cuboid"; }
      ElementPropertyDialog* createPropertyDialog() {return new CuboidPropertyDialog(this);}
  };

  class LineSegment : public RigidContour {
    public:
      LineSegment(const QString &str="");
      QString getType() const { return "LineSegment"; }
      ElementPropertyDialog* createPropertyDialog() {return new LineSegmentPropertyDialog(this);}
  };

  class PlanarContour : public RigidContour {
    public:
      PlanarContour(const QString &str="");
      QString getType() const { return "PlanarContour"; }
      ElementPropertyDialog* createPropertyDialog() {return new PlanarContourPropertyDialog(this);}
  };

  class SpatialContour : public RigidContour {
    public:
      SpatialContour(const QString &str="");
      QString getType() const { return "SpatialContour"; }
      ElementPropertyDialog* createPropertyDialog() {return new SpatialContourPropertyDialog(this);}
  };

}

#endif
