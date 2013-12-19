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
#include "ombv_widgets.h"

class RigidBody;

class OMBVBodyPropertyFactory: public PropertyFactory {
  public:
    OMBVBodyPropertyFactory(const std::string &ID);
    Property* createProperty(int i=0);
    std::string getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<std::string> name;
    std::string ID;
    int count;
};

class OMBVObjectProperty : public Property {

  public:
    OMBVObjectProperty(const std::string &name="", const std::string &ID_=0) : Property(name), ID(ID_) {}
    OMBVObjectProperty(const std::string &name, const std::string &value, const std::string &ID_) : Property(name,value), ID(ID_) {}
    virtual std::string getType() const = 0;
//    void setName(const std::string &name_) {name = name_;}
    const std::string& getID() const { return ID; }
  protected:
//    std::string name;
    std::string ID;
    void writeXMLFileID(MBXMLUtils::TiXmlNode *parent);
};

class OMBVFrameProperty : public OMBVObjectProperty {

  public:
    OMBVFrameProperty(const std::string &name="NOTSET", const std::string &xmlName="", const std::string &ID_=0);
    virtual Property* clone() const {return new OMBVFrameProperty(*this);}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    virtual std::string getType() const { return "Frame"; }
  protected:
    std::string xmlName;
};

class OMBVDynamicColoredObjectProperty : public OMBVObjectProperty {

  public:
    OMBVDynamicColoredObjectProperty(const std::string &name="NOTSET", const std::string &ID_=0, bool readXMLType=false);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
//  protected:
//    ExtProperty minimalColorValue, maximalColorValue, diffuseColor, transparency;
    bool readXMLType;
};


class OMBVArrowProperty : public OMBVDynamicColoredObjectProperty {

  public:
    OMBVArrowProperty(const std::string &name="NOTSET", const std::string &ID_=0, bool fromPoint=false);
    virtual Property* clone() const {return new OMBVArrowProperty(*this);}
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
    virtual Property* clone() const {return new OMBVCoilSpringProperty(*this);}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
   virtual std::string getType() const { return "CoilSpring"; }
  protected:
    ExtProperty type, numberOfCoils, springRadius, crossSectionRadius, nominalLength, scaleFactor;
};

class OMBVBodyProperty : public OMBVDynamicColoredObjectProperty {

  public:
    OMBVBodyProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    virtual std::string getType() const = 0;
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
//    Widget* createWidget() { return new OpenMBVRigidBodyChoiceWidget; }
//    QMenu* createContextMenu() { return new OpenMBVRigidBodyChoiceContextMenu(this); }
   QMenu* createContextMenu() { return new OpenMBVRigidBodyChoiceContextMenu(this); }};

class InvisibleBodyProperty : public OMBVBodyProperty {

  public:
    InvisibleBodyProperty(const std::string &name="NOTSET", const std::string &ID=0) : OMBVBodyProperty(name,ID) {}
    virtual Property* clone() const {return new InvisibleBodyProperty(*this);}
    virtual std::string getType() const { return "InvisibleBody"; }
};

class CubeProperty : public OMBVBodyProperty {

  public:
    CubeProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    virtual Property* clone() const {return new CubeProperty(*this);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "Cube"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
//  protected:
//    ExtProperty length;
};

class CuboidProperty : public OMBVBodyProperty {

  public:
    CuboidProperty(const std::string &name="NOTSET", const std::string &ID_=0);
    virtual Property* clone() const {return new CuboidProperty(*this);}
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
    virtual Property* clone() const {return new SphereProperty(*this);}
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
    virtual Property* clone() const {return new FrustumProperty(*this);}
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
    virtual Property* clone() const {return new IvBodyProperty(*this);}
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
    virtual Property* clone() const {return new CompoundRigidBodyProperty(*this);}
    MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);
    virtual std::string getType() const { return "CompoundRigidBody"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);
  protected:
    ExtProperty bodies;
};

class OMBVBodySelectionProperty : public Property {
  public:

    OMBVBodySelectionProperty(RigidBody* body);
    virtual Property* clone() const {return new OMBVBodySelectionProperty(*this);}

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
    virtual Property* clone() const {return new OMBVEmptyProperty(*this);}

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
    virtual Property* clone() const {return new OMBVPlaneProperty(*this);}
    virtual MBXMLUtils::TiXmlElement* initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element); 
    virtual std::string getType() const { return "Plane"; }
    void fromWidget(QWidget *widget);
    void toWidget(QWidget *widget);

  protected:
    ExtProperty size, numberOfLines;
    std::string xmlName;
};

class OpenMBVRigidBodyChoiceProperty : public Property {
  public:
    OpenMBVRigidBodyChoiceProperty(const std::string &name="") : Property(name) { }
//    Widget* createWidget() { return new OpenMBVRigidBodyChoiceWidget; }
    QMenu* createContextMenu() { return new OpenMBVRigidBodyChoiceContextMenu(this); }
};

#endif
