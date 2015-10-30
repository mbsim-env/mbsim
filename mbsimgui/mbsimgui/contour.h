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
    Contour(const std::string &str, Element *parent);
    ~Contour();
    static Contour* readXMLFile(const std::string &filename, Element *parent);
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    void setSavedFrameOfReference(const std::string &str);
    virtual void initialize();
    ElementPropertyDialog* createPropertyDialog() {return new ContourPropertyDialog(this);}
    protected:
    ExtProperty refFrame;
  };

  class Point : public Contour {
    friend class PointPropertyDialog;
    public:
    Point(const std::string &str, Element *parent);
    ~Point();
    std::string getType() const { return "Point"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new PointPropertyDialog(this);}
    protected:
    ExtProperty visu;
  };

  class Line : public Contour {
    friend class LinePropertyDialog;
    public:
    Line(const std::string &str, Element *parent);
    ~Line();
    std::string getType() const { return "Line"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new LinePropertyDialog(this);}
    protected:
    ExtProperty visu;
  };

  class Plane : public Contour {
    friend class PlanePropertyDialog;
    public:
    Plane(const std::string &str, Element *parent);
    ~Plane();
    std::string getType() const { return "Plane"; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new PlanePropertyDialog(this);}
    protected:
    ExtProperty visu;
  };

  class Sphere : public Contour {
    friend class SpherePropertyDialog;
    public:
    Sphere(const std::string &str, Element *parent);
    ~Sphere();
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "Sphere"; }
    ElementPropertyDialog* createPropertyDialog() {return new SpherePropertyDialog(this);}
    protected:
    ExtProperty radius, visu;
  };

  class CircleSolid : public Contour {
    friend class CircleSolidPropertyDialog;
    public:
    CircleSolid(const std::string &str, Element *parent);
    ~CircleSolid();
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "CircleSolid"; }
    ElementPropertyDialog* createPropertyDialog() {return new CircleSolidPropertyDialog(this);}
    protected:
    ExtProperty radius, visu;
  };

  class Cuboid : public Contour {
    friend class CuboidPropertyDialog;
    public:
    Cuboid(const std::string &str, Element *parent);
    ~Cuboid();
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    std::string getType() const { return "Cuboid"; }
    ElementPropertyDialog* createPropertyDialog() {return new CuboidPropertyDialog(this);}
    protected:
    ExtProperty length, visu;
  };

}

#endif
