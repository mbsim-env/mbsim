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

class Contour : public Element {
  public:
    Contour(const std::string &str, Element *parent);
    ~Contour();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual Element *getByPathSearch(std::string path);
    void setSavedFrameOfReference(const std::string &str);
    virtual void initialize();
  protected:
    ExtProperty refFrame;
};

class Point : public Contour {
  public:
    Point(const std::string &str, Element *parent);
    ~Point();
};

class Line : public Contour {
  public:
    Line(const std::string &str, Element *parent);
    ~Line();
};

class Plane : public Contour {
  public:
    Plane(const std::string &str, Element *parent);
    ~Plane();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    ExtProperty visu;
};

class Sphere : public Contour {
  public:
    Sphere(const std::string &str, Element *parent);
    ~Sphere();
    virtual void initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
  protected:
    ExtProperty radius, visu;
};


#endif
