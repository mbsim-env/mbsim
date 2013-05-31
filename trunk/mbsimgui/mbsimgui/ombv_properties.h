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

#ifndef _OMBV_PROPERTIES_H_
#define _OMBV_PROPERTIES_H_

#include "basic_properties.h"
#include "extended_properties.h"

//class ExtProperty;
class RigidBody;

class OMBVObjectProperty : public Property {

  public:
    OMBVObjectProperty(const std::string &name_="NOTSET", const std::string &ID_=0) : name(name_), ID(ID_) {}
    virtual std::string getType() const = 0;
    void setName(const std::string &name_) {name = name_;}
  protected:
    std::string name;
    std::string ID;
    void writeXMLFileID(MBXMLUtils::TiXmlNode *parent);
};

class OMBVFrameProperty : public OMBVObjectProperty {

  public:
    OMBVFrameProperty(const std::string &name="NOTSET", const std::string &xmlName="", const std::string &ID_=0);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    virtual std::string getType() const { return "Frame"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty size, offset;
    std::string xmlName;
};

class OMBVDynamicColoredObjectProperty : public OMBVObjectProperty {

  public:
    OMBVDynamicColoredObjectProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty minimalColorValue, maximalColorValue, staticColor;
};


class OMBVArrowProperty : public OMBVDynamicColoredObjectProperty {

  public:
    OMBVArrowProperty(const std::string &name="NOTSET", const std::string &ID_=0, bool fromPoint=false);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    virtual std::string getType() const { return "Arrow"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty diameter, headDiameter, headLength, type, referencePoint, scaleLength;
};

class OMBVCoilSpringProperty : public OMBVObjectProperty {

  public:
    OMBVCoilSpringProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
   virtual std::string getType() const { return "CoilSpring"; }
  protected:
    ExtProperty type, numberOfCoils, springRadius, crossSectionRadius, nominalLength, scaleFactor;
};

class OMBVBodyProperty : public OMBVObjectProperty {

  public:
    OMBVBodyProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    virtual std::string getType() const = 0;
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty trans, rot, color, scale;
};

class InvisibleBodyProperty : public OMBVBodyProperty {

  public:
    InvisibleBodyProperty(const std::string &name="NOTSET", const std::string &ID=0) : OMBVBodyProperty(name,ID) {}
    virtual std::string getType() const { return "InvisibleBody"; }
};

class CubeProperty : public OMBVBodyProperty {

  public:
    CubeProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "Cube"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty length;
};

class CuboidProperty : public OMBVBodyProperty {

  public:
    CuboidProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "Cuboid"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty length;
};

class SphereProperty : public OMBVBodyProperty {

  public:
    SphereProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "Sphere"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty radius;
};

class FrustumProperty : public OMBVBodyProperty {
  public:
    FrustumProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "Frustum"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty top, base, height, innerBase, innerTop;
};

class IvBodyProperty : public OMBVBodyProperty {
  public:
    IvBodyProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "IvBody"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty ivFileName, creaseEdges, boundaryEdges;
};

class CompoundRigidBodyProperty : public OMBVBodyProperty {

  public:
    CompoundRigidBodyProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "CompoundRigidBody"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    std::vector<ChoiceProperty*> body;
};

class OMBVBodySelectionProperty : public Property {
  public:

    OMBVBodySelectionProperty(RigidBody* body);

    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty ombv, ref;
};

class OMBVEmptyProperty : public OMBVObjectProperty {

  public:
    OMBVEmptyProperty(const std::string &xmlName_, const std::string &ID=0) : OMBVObjectProperty("Empty",ID), xmlName(xmlName_) {}

    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
    virtual std::string getType() const { return "Empty"; }

  protected:
    std::string xmlName;
};

class OMBVPlaneProperty : public OMBVObjectProperty {

  public:
    OMBVPlaneProperty(const std::string &xmlName, const std::string &ID_=0);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    virtual std::string getType() const { return "Plane"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty size, numberOfLines;
    std::string xmlName;
};

#endif
