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
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getDefaultIndex() const override { return 1; }
      int getFallbackIndex() const override { return 10; }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      int count{1};
  };

  class OMBVFlexibleBodyWidgetFactory : public WidgetFactory {
    public:
      OMBVFlexibleBodyWidgetFactory();
      Widget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getDefaultIndex() const override { return 2; }
      int getFallbackIndex() const override { return 3; }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      int count{1};
  };

  class MBSOMBVWidget : public Widget {

    public:
      MBSOMBVWidget() { }
  };

  class MBSOMBVColoreBodyWidget : public MBSOMBVWidget {

    public:
      MBSOMBVColoreBodyWidget(const std::vector<QString> &c=getBlueColor());
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *diffuseColor, *transparency, *pointSize, *lineWidth;
  };

  class MBSOMBVDynamicColoreBodyWidget : public MBSOMBVColoreBodyWidget {

    public:
      MBSOMBVDynamicColoreBodyWidget(const std::vector<QString> &c=getBlueColor(), const std::vector<QString> &cRL=std::vector<QString>(1,"\"none\""));
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *colorRepresentation, *minimalColorValue, *maximalColorValue;
  };

  class LineMBSOMBVWidget : public MBSOMBVColoreBodyWidget {

    public:
      LineMBSOMBVWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length;
  };

  class PlaneMBSOMBVWidget : public MBSOMBVColoreBodyWidget {

    public:
      PlaneMBSOMBVWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *length;
  };

  class PlanarContourMBSOMBVWidget : public MBSOMBVColoreBodyWidget {

    public:
      PlanarContourMBSOMBVWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *nodes, *filled;
  };

  class SpatialContourMBSOMBVWidget : public MBSOMBVColoreBodyWidget {

    public:
      SpatialContourMBSOMBVWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *etaNodes, *xiNodes;
  };

  class ArrowMBSOMBVWidget : public MBSOMBVDynamicColoreBodyWidget {

    public:
      ArrowMBSOMBVWidget(const std::vector<QString> &c=getRedColor(), const std::vector<QString> &cRL=getColorRepresentation(), int refPoint=1);
      static std::vector<QString> getColorRepresentation();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *scaleLength, *scaleSize, *type, *referencePoint;
  };

  class InteractionArrowMBSOMBVWidget : public ArrowMBSOMBVWidget {

    public:
      InteractionArrowMBSOMBVWidget(const std::vector<QString> &cRL=getColorRepresentation());
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *sideOfInteraction;
  };

  class FrictionArrowMBSOMBVWidget : public InteractionArrowMBSOMBVWidget {

    public:
      FrictionArrowMBSOMBVWidget(const std::vector<QString> &cRL=getColorRepresentation()) : InteractionArrowMBSOMBVWidget(cRL) { }
      static std::vector<QString> getColorRepresentation();
  };

  class CoilSpringMBSOMBVWidget : public MBSOMBVDynamicColoreBodyWidget {

    public:
      CoilSpringMBSOMBVWidget(const std::vector<QString> &cRL=getColorRepresentation());
      static std::vector<QString> getColorRepresentation();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *colorRepresentation, *type, *numberOfCoils, *springRadius, *crossSectionRadius, *nominalLength;
  };

  class FrameMBSOMBVWidget : public MBSOMBVColoreBodyWidget {

    public:
      FrameMBSOMBVWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *size, *offset;
  };

  class FlexibleBodyMBSOMBVWidget : public MBSOMBVDynamicColoreBodyWidget {

    public:
      FlexibleBodyMBSOMBVWidget(const std::vector<QString> &cRL=getColorRepresentation()) : MBSOMBVDynamicColoreBodyWidget(getBlueColor(),cRL) { }
      static std::vector<QString> getColorRepresentation();
  };

  class CalculixBodyMBSOMBVWidget : public FlexibleBodyMBSOMBVWidget {

    public:
      CalculixBodyMBSOMBVWidget(const std::vector<QString> &cRL=getColorRepresentation());
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *visu;
  };

  class OMBVObjectWidget : public Widget {

    public:
      OMBVObjectWidget(const QString &name_="NOTSET", MBXMLUtils::FQN xmlName_="") : name(name_), xmlName(std::move(xmlName_)) { }
      void setName(const QString &name_) {name = name_;}
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      QString name;
      MBXMLUtils::FQN xmlName;
  };

  class OMBVBodyWidget : public OMBVObjectWidget {

    public:
      OMBVBodyWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      QVBoxLayout *layout;
      ExtWidget *pointSize, *lineWidth;
  };

  class OMBVDynamicColoredObjectWidget : public OMBVBodyWidget {

    public:
      OMBVDynamicColoredObjectWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
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

  class OMBVFlexibleBodyWidget : public OMBVDynamicColoredObjectWidget {

    public:
      OMBVFlexibleBodyWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *numvp;
  };

  class DynamicPointSetWidget : public OMBVFlexibleBodyWidget {

    public:
      DynamicPointSetWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="") : OMBVFlexibleBodyWidget(name,xmlName) { }
  };

  class DynamicIndexedLineSetWidget : public OMBVFlexibleBodyWidget {

    public:
      DynamicIndexedLineSetWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      ExtWidget* getIndices() { return indices; }
    protected:
      ExtWidget *indices;
  };

  class DynamicIndexedFaceSetWidget : public OMBVFlexibleBodyWidget {

    public:
      DynamicIndexedFaceSetWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      ExtWidget* getIndices() { return indices; }
    protected:
      ExtWidget *indices;
  };

  class CylindricalGearWidget : public OMBVRigidBodyWidget {

    public:
      CylindricalGearWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *numberOfTeeth, *width, *helixAngle, *module, *pressureAngle, *backlash, *externalToothed;
  };

  class CylinderWidget : public OMBVRigidBodyWidget {

    public:
      CylinderWidget(const QString &name="NOTSET", const MBXMLUtils::FQN &xmlName="");
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *radius, *height;
  };

}

#endif
