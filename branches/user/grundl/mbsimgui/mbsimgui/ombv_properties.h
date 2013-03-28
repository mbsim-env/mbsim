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
    OMBVObjectProperty(const std::string &name_) : name(name_) {}
    virtual QString getType() const = 0;
    void setName(const std::string &name_) {name = name_;}
    void setID(const std::string &_ID) { ID=_ID; }
  protected:
    std::string name;
    std::string ID;
    void writeXMLFileID(TiXmlNode *parent);
};

class OMBVFrameProperty : public OMBVObjectProperty {

  public:
    OMBVFrameProperty(const std::string &name, const std::string &xmlName);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Frame"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty size, offset;
    std::string xmlName;
};

class OMBVDynamicColoredObjectProperty : public OMBVObjectProperty {

  public:
    OMBVDynamicColoredObjectProperty(const std::string &name);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty minimalColorValue, maximalColorValue, staticColor;
};


class OMBVArrowProperty : public OMBVDynamicColoredObjectProperty {

  public:
    OMBVArrowProperty(const std::string &name, bool fromPoint=false);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Arrow"; }
  protected:
    ExtProperty diameter, headDiameter, headLength, type, referencePoint, scaleLength;
};

class OMBVCoilSpringProperty : public OMBVObjectProperty {

  public:
    OMBVCoilSpringProperty(const std::string &name);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
   virtual QString getType() const { return "CoilSpring"; }
  protected:
    ExtProperty type, numberOfCoils, springRadius, crossSectionRadius, nominalLength, scaleFactor;
};

class OMBVBodyProperty : public OMBVObjectProperty {

  public:
    OMBVBodyProperty(const std::string &name);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const = 0;
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty trans, rot, color, scale;
};

class CubeProperty : public OMBVBodyProperty {

  public:
    CubeProperty(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Cube"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty length;
};

class CuboidProperty : public OMBVBodyProperty {

  public:
    CuboidProperty(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Cuboid"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty length;
};

class SphereProperty : public OMBVBodyProperty {

  public:
    SphereProperty(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Sphere"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty radius;
};

class FrustumProperty : public OMBVBodyProperty {
  public:
    FrustumProperty(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Frustum"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty top, base, height, innerBase, innerTop;
};

class IvBodyProperty : public OMBVBodyProperty {
  public:
    IvBodyProperty(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "IvBody"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty ivFileName, creaseEdges, boundaryEdges;
};
class OMBVBodyChoiceProperty;

class CompoundRigidBodyProperty : public OMBVBodyProperty {

  public:
    CompoundRigidBodyProperty(const std::string &name) : OMBVBodyProperty(name) {}
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "CompoundRigidBody"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    std::vector<OMBVBodyChoiceProperty*> body;
};

class OMBVBodyChoiceProperty : public Property {

  public:

    OMBVBodyChoiceProperty(const std::string &name, bool flag=true, const std::string &ID="");

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setName(const std::string &name) {ombv->setName(name);}
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    void ombvSelection(int index);

    OMBVBodyProperty *ombv;
    int index;
    std::string name;
    std::string ID;
};

class OMBVBodySelectionProperty : public Property {
  public:

    OMBVBodySelectionProperty(RigidBody* body);

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    OMBVBodyChoiceProperty ombv;
    LocalFrameOfReferenceProperty ref;
};

class OMBVEmptyProperty : public OMBVObjectProperty {

  public:
    OMBVEmptyProperty(const std::string &xmlName_) : OMBVObjectProperty("Empty"), xmlName(xmlName_) {}

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void fromWidget(QWidget *widget) {}
    void toWidget(QWidget *widget) {}
    virtual QString getType() const { return "Empty"; }

  protected:
    std::string xmlName;
};

class OMBVPlaneProperty : public OMBVObjectProperty {

  public:
    OMBVPlaneProperty(const std::string &xmlName);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Plane"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty size, numberOfLines;
    std::string xmlName;
};

#endif
