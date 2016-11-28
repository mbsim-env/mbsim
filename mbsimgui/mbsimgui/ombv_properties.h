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

namespace MBSimGUI {

  class Body;

  class OMBVRigidBodyPropertyFactory: public PropertyFactory {
    public:
      OMBVRigidBodyPropertyFactory(const std::string &ID);
      Property* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<MBXMLUtils::FQN> name;
      std::string ID;
      int count;
  };

  class OMBVObjectProperty : public Property {

    public:
      OMBVObjectProperty(const std::string &name_="NOTSET", const std::string &ID_=0) : name(name_), ID(ID_) {}
      virtual std::string getType() const = 0;
      void setName(const std::string &name_) {name = name_;}
    protected:
      std::string name;
      std::string ID;
      void writeXMLFileID(xercesc::DOMNode *parent);
  };

  class MBSOMBVProperty : public OMBVObjectProperty {

    public:
      MBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID);
      virtual PropertyInterface* clone() const {return new MBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      virtual xercesc::DOMElement* initXMLFile(xercesc::DOMNode *element); 
      virtual xercesc::DOMElement* writeProperties(xercesc::DOMElement *e); 
      virtual std::string getType() const { return "MBSOMBVProperty"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty diffuseColor, transparency;
      MBXMLUtils::FQN xmlName;
  };

  class PointMBSOMBVProperty : public MBSOMBVProperty {

    public:
      PointMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID);
      virtual PropertyInterface* clone() const {return new PointMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty size;
  };

  class LineMBSOMBVProperty : public MBSOMBVProperty {

    public:
      LineMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID);
      virtual PropertyInterface* clone() const {return new LineMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty length;
  };

  class PlaneMBSOMBVProperty : public MBSOMBVProperty {

    public:
      PlaneMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID);
      virtual PropertyInterface* clone() const {return new PlaneMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty length;
  };

  class PlanarContourMBSOMBVProperty : public MBSOMBVProperty {

    public:
      PlanarContourMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID);
      virtual PropertyInterface* clone() const {return new PlanarContourMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty nodes;
  };

  class SpatialContourMBSOMBVProperty : public MBSOMBVProperty {

    public:
      SpatialContourMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID);
      virtual PropertyInterface* clone() const {return new SpatialContourMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty etaNodes, xiNodes;
  };

  class ArrowMBSOMBVProperty : public MBSOMBVProperty {

    public:
      ArrowMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID, bool fromPoint=false);
      virtual PropertyInterface* clone() const {return new ArrowMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty scaleLength, scaleSize, referencePoint, minCol, maxCol;
  };

  class CoilSpringMBSOMBVProperty : public MBSOMBVProperty {

    public:
      CoilSpringMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID_);
      virtual PropertyInterface* clone() const {return new CoilSpringMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty type, numberOfCoils, springRadius, crossSectionRadius, nominalLength, minCol, maxCol;
  };

  class OMBVFrameProperty : public OMBVObjectProperty {

    public:
      OMBVFrameProperty(const std::string &name="NOTSET", const MBXMLUtils::FQN &xmlName="", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new OMBVFrameProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      virtual std::string getType() const { return "Frame"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty size, offset, transparency;
      MBXMLUtils::FQN xmlName;
  };

  class OMBVDynamicColoredObjectProperty : public OMBVObjectProperty {

    public:
      OMBVDynamicColoredObjectProperty(const std::string &name="NOTSET", const std::string &ID_=0, bool readXMLType=false);
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty minimalColorValue, maximalColorValue, diffuseColor, transparency;
      bool readXMLType;
  };

  class OMBVRigidBodyProperty : public OMBVDynamicColoredObjectProperty {

    public:
      OMBVRigidBodyProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element); 
      virtual std::string getType() const = 0;
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty trans, rot, scale;
  };

  class InvisibleBodyProperty : public OMBVRigidBodyProperty {

    public:
      InvisibleBodyProperty(const std::string &name="NOTSET", const std::string &ID=0) : OMBVRigidBodyProperty(name,ID) {}
      virtual PropertyInterface* clone() const {return new InvisibleBodyProperty(*this);}
      virtual std::string getType() const { return "InvisibleBody"; }
  };

  class CubeProperty : public OMBVRigidBodyProperty {

    public:
      CubeProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new CubeProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "Cube"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty length;
  };

  class CuboidProperty : public OMBVRigidBodyProperty {

    public:
      CuboidProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new CuboidProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "Cuboid"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty length;
  };

  class SphereProperty : public OMBVRigidBodyProperty {

    public:
      SphereProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new SphereProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "Sphere"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty radius;
  };

  class FrustumProperty : public OMBVRigidBodyProperty {
    public:
      FrustumProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new FrustumProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "Frustum"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty top, base, height, innerBase, innerTop;
  };

  class ExtrusionProperty : public OMBVRigidBodyProperty {
    public:
      ExtrusionProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new ExtrusionProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "Extrusion"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty windingRule, height, contour;
  };

  class IvBodyProperty : public OMBVRigidBodyProperty {
    public:
      IvBodyProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new IvBodyProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "IvBody"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty ivFileName, creaseEdges, boundaryEdges;
  };

  class CompoundRigidBodyProperty : public OMBVRigidBodyProperty {

    public:
      CompoundRigidBodyProperty(const std::string &name="NOTSET", const std::string &ID_=0);
      virtual PropertyInterface* clone() const {return new CompoundRigidBodyProperty(*this);}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      virtual std::string getType() const { return "CompoundRigidBody"; }
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty bodies;
  };

  class OMBVRigidBodySelectionProperty : public Property {
    public:

      OMBVRigidBodySelectionProperty(Body* body, const MBXMLUtils::NamespaceURI &uri=MBSIM);
      virtual PropertyInterface* clone() const {return new OMBVRigidBodySelectionProperty(*this);}

      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);

    protected:
      ExtProperty ombv, ref;
  };

  class FlexibleBodyFFRMBSOMBVProperty : public MBSOMBVProperty {

    public:
      FlexibleBodyFFRMBSOMBVProperty(const std::string &name, const MBXMLUtils::FQN &xmlName, const std::string &ID);
      virtual PropertyInterface* clone() const {return new FlexibleBodyFFRMBSOMBVProperty(*this);}
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      void fromWidget(QWidget *widget);
      void toWidget(QWidget *widget);
    protected:
      ExtProperty minCol, maxCol, nodes, indices;
  };

}

#endif
