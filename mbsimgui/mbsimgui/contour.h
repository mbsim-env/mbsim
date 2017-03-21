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
#include "extended_properties.h"

namespace MBSimGUI {

  class Contour : public Element {
    friend class ContourPropertyDialog;
    public:
    Contour(const std::string &str="");
    static Contour* readXMLFile(const std::string &filename);
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new ContourPropertyDialog(this);}
    protected:
    ExtProperty thickness;
  };

  class RigidContour : public Contour {
    friend class RigidContourPropertyDialog;
    public:
    RigidContour(const std::string &str="");
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void setSavedFrameOfReference(const std::string &str);
    virtual void initialize();
    ElementPropertyDialog* createPropertyDialog() {return new RigidContourPropertyDialog(this);}
    protected:
    ExtProperty refFrame;
  };

  class Point : public RigidContour {
    friend class PointPropertyDialog;
    public:
    Point(const std::string &str="");
    std::string getType() const { return "Point"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new PointPropertyDialog(this);}
    protected:
    ExtProperty visu;
  };

  class Line : public RigidContour {
    friend class LinePropertyDialog;
    public:
    Line(const std::string &str="");
    std::string getType() const { return "Line"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new LinePropertyDialog(this);}
    protected:
    ExtProperty visu;
  };

  class Plane : public RigidContour {
    friend class PlanePropertyDialog;
    public:
    Plane(const std::string &str="");
    std::string getType() const { return "Plane"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new PlanePropertyDialog(this);}
    protected:
    ExtProperty visu;
  };

  class Sphere : public RigidContour {
    friend class SpherePropertyDialog;
    public:
    Sphere(const std::string &str="");
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "Sphere"; }
    ElementPropertyDialog* createPropertyDialog() {return new SpherePropertyDialog(this);}
    protected:
    ExtProperty radius, visu;
  };

  class Circle : public RigidContour {
    friend class CirclePropertyDialog;
    public:
    Circle(const std::string &str="");
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "Circle"; }
    ElementPropertyDialog* createPropertyDialog() {return new CirclePropertyDialog(this);}
    protected:
    ExtProperty radius, solid, visu;
  };

  class Cuboid : public RigidContour {
    friend class CuboidPropertyDialog;
    public:
    Cuboid(const std::string &str="");
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "Cuboid"; }
    ElementPropertyDialog* createPropertyDialog() {return new CuboidPropertyDialog(this);}
    protected:
    ExtProperty length, visu;
  };

  class LineSegment : public RigidContour {
    friend class LineSegmentPropertyDialog;
    public:
    LineSegment(const std::string &str="");
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "LineSegment"; }
    ElementPropertyDialog* createPropertyDialog() {return new LineSegmentPropertyDialog(this);}
    protected:
    ExtProperty length, visu;
  };

  class PlanarContour : public RigidContour {
    friend class PlanarContourPropertyDialog;
    public:
    PlanarContour(const std::string &str="");
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "PlanarContour"; }
    ElementPropertyDialog* createPropertyDialog() {return new PlanarContourPropertyDialog(this);}
    protected:
    ExtProperty nodes, contourFunction, open, visu;
  };

  class SpatialContour : public RigidContour {
    friend class SpatialContourPropertyDialog;
    public:
    SpatialContour(const std::string &str="");
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "SpatialContour"; }
    ElementPropertyDialog* createPropertyDialog() {return new SpatialContourPropertyDialog(this);}
    protected:
    ExtProperty etaNodes, xiNodes, contourFunction, open, visu;
  };

}

#endif
