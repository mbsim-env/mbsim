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

#ifndef _OMBV_WIDGETS_H_
#define _OMBV_WIDGETS_H_

#include <utility>

#include "widget.h"
#include "basic_widgets.h"

class QVBoxLayout;
class QListWidget;
class QComboBox;

namespace MBSimGUI {

  class ExtWidget;
  class Body;

  class OMBVRigidBodyWidgetFactory : public WidgetFactory {
    public:
      OMBVRigidBodyWidgetFactory();
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      int count{1};
  };

  class OMBVObjectWidget : public Widget {

    public:
      OMBVObjectWidget(const QString &name_="NOTSET", MBXMLUtils::FQN xmlName_="") : name(name_), xmlName(std::move(xmlName_)) {}
      void setName(const QString &name_) {name = name_;}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      QString name;
      MBXMLUtils::FQN xmlName;
  };

  class MBSOMBVWidget : public OMBVObjectWidget {

    public:
      MBSOMBVWidget(const QString &name, const MBXMLUtils::FQN &xmlName_="", const MBXMLUtils::NamespaceURI &url=MBSIM);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      virtual xercesc::DOMElement* initXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr);
      virtual xercesc::DOMElement* writeProperties(xercesc::DOMElement *e);
    protected:
      ExtWidget *diffuseColor, *transparency;
      MBXMLUtils::FQN xmlName;
  };

  class PointMBSOMBVWidget : public MBSOMBVWidget {

    public:
      PointMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *size;
  };

  class LineMBSOMBVWidget : public MBSOMBVWidget {

    public:
      LineMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length;
  };

  class PlaneMBSOMBVWidget : public MBSOMBVWidget {

    public:
      PlaneMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length;
  };

  class PlanarContourMBSOMBVWidget : public MBSOMBVWidget {

    public:
      PlanarContourMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodes, *filled;
  };

  class SpatialContourMBSOMBVWidget : public MBSOMBVWidget {

    public:
      SpatialContourMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *etaNodes, *xiNodes;
  };

  class ArrowMBSOMBVWidget : public MBSOMBVWidget {

    public:
      ArrowMBSOMBVWidget(const QString &name="NOTSET", bool fromPoint=false);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *scaleLength, *scaleSize, *referencePoint;
  };

  class CoilSpringMBSOMBVWidget : public MBSOMBVWidget {

    public:
      CoilSpringMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *type, *numberOfCoils, *springRadius, *crossSectionRadius, *nominalLength, *minCol, *maxCol;
  };

  class FrameMBSOMBVWidget : public MBSOMBVWidget {

    public:
      FrameMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *size, *offset;
  };

  class OMBVDynamicColoredObjectWidget : public OMBVObjectWidget {

    public:
      OMBVDynamicColoredObjectWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      QVBoxLayout *layout;
      ExtWidget *minimalColorValue, *maximalColorValue, *diffuseColor, *transparency;
  };

  class OMBVRigidBodyWidget : public OMBVDynamicColoredObjectWidget {

    public:
      OMBVRigidBodyWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *trans, *rot, *scale;
  };

  class InvisibleBodyWidget : public OMBVRigidBodyWidget {

    public:
      InvisibleBodyWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="") : OMBVRigidBodyWidget(name,xmlName) {}
  };

  class CubeWidget : public OMBVRigidBodyWidget {

    public:
      CubeWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length;
  };

  class CuboidWidget : public OMBVRigidBodyWidget {

    public:
      CuboidWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length;
  };

  class SphereWidget : public OMBVRigidBodyWidget {

    public:
      SphereWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius;
  };

  class FrustumWidget : public OMBVRigidBodyWidget {

    public:
      FrustumWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *top, *base, *height, *innerBase, *innerTop;
  };

  class ExtrusionWidget : public OMBVRigidBodyWidget {

    public:
      ExtrusionWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *windingRule, *height, *contour;
  };

  class IvBodyWidget : public OMBVRigidBodyWidget {

    public:
      IvBodyWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *ivFileName, *creaseEdges, *boundaryEdges;
  };

  class CompoundRigidBodyWidget : public OMBVRigidBodyWidget {

    public:
      CompoundRigidBodyWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *bodies;
  };

  class FlexibleBodyFFRMBSOMBVWidget : public MBSOMBVWidget {

    public:
      FlexibleBodyFFRMBSOMBVWidget(const QString &name="NOTSET");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *minCol, *maxCol, *nodes, *indices, *colorEntity;
  };

}

#endif
