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

#include "xml_widget.h"
#include "basic_widgets.h"

class ExtXMLWidget;
class QVBoxLayout;
class QStackedWidget;
class QListWidget;
class QComboBox;
class RigidBody;

class OMBVObjectWidget : public XMLWidget {

  public:
    OMBVObjectWidget(const std::string &name_) : name(name_) {}
    virtual QString getType() const = 0;
    void setName(const std::string &name_) {name = name_;}
    void setID(const std::string &_ID) { ID=_ID; }
  protected:
    std::string name;
    std::string ID;
    void writeXMLFileID(TiXmlNode *parent);
};

class OMBVFrameWidget : public OMBVObjectWidget {

  public:
    OMBVFrameWidget(const std::string &name, const std::string &xmlName);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Frame"; }
  protected:
    ExtXMLWidget *size, *offset;
    std::string xmlName;
};

class OMBVDynamicColoredObjectWidget : public OMBVObjectWidget {

  public:
    OMBVDynamicColoredObjectWidget(const std::string &name);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
  protected:
    QVBoxLayout *layout;
    ExtXMLWidget *minimalColorValue, *maximalColorValue, *staticColor;
};


class OMBVArrowWidget : public OMBVDynamicColoredObjectWidget {

  public:
    OMBVArrowWidget(const std::string &name, bool fromPoint=false);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Arrow"; }
  protected:
    ExtXMLWidget *diameter, *headDiameter, *headLength, *type, *referencePoint, *scaleLength;
};

class OMBVCoilSpringWidget : public OMBVObjectWidget {

  public:
    OMBVCoilSpringWidget(const std::string &name);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "CoilSpring"; }
  protected:
    ExtXMLWidget *type, *numberOfCoils, *springRadius, *crossSectionRadius, *nominalLength, *scaleFactor;
};

class OMBVBodyWidget : public OMBVObjectWidget {

  public:
    OMBVBodyWidget(const std::string &name);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const = 0;
  protected:
    QVBoxLayout *layout;
    ExtXMLWidget *trans, *rot, *color, *scale;
};

class CubeWidget : public OMBVBodyWidget {

  public:
    CubeWidget(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Cube"; }
  protected:
    ExtXMLWidget *length;
};

class CuboidWidget : public OMBVBodyWidget {

  public:
    CuboidWidget(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Cuboid"; }
  protected:
    ExtXMLWidget *length;
};

class SphereWidget : public OMBVBodyWidget {

  public:
    SphereWidget(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Sphere"; }
  protected:
    ExtXMLWidget *radius;
};

class FrustumWidget : public OMBVBodyWidget {
  public:
    FrustumWidget(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Frustum"; }
  protected:
    ExtXMLWidget *top, *base, *height, *innerBase, *innerTop;
};

class IvBodyWidget : public OMBVBodyWidget {
  public:
    IvBodyWidget(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "IvBody"; }
  protected:
    ExtXMLWidget *ivFileName, *creaseEdges, *boundaryEdges;
};
class OMBVBodyChoiceWidget;

class CompoundRigidBodyWidget : public OMBVBodyWidget {
  Q_OBJECT

  public:
    CompoundRigidBodyWidget(const std::string &name);
    TiXmlElement* initializeUsingXML(TiXmlElement *element);
    TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "CompoundRigidBody"; }
  protected:
    std::vector<OMBVBodyChoiceWidget*> body;
    //std::vector<OMBVBodyWidget*> body;
    QStackedWidget *stackedWidget; 
    QListWidget *bodyList; 
  protected slots:
    void changeCurrent(int idx);
    void openContextMenu(const QPoint &pos);
    void addBody();
    void removeBody();
};

class OMBVBodyChoiceWidget : public XMLWidget {
  Q_OBJECT

  public:

    OMBVBodyChoiceWidget(const std::string &name, bool flag=true, const std::string &ID="");

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    void setName(const std::string &name) {ombv->setName(name);}

  protected slots:
    void ombvSelection(int index);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    OMBVBodyWidget *ombv;
    std::string name;
    std::string ID;
};

class OMBVBodySelectionWidget : public XMLWidget {
  public:

    OMBVBodySelectionWidget(RigidBody* body);

    virtual void update() {ref->update();}
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

  protected:
    OMBVBodyChoiceWidget *ombv;
    LocalFrameOfReferenceWidget *ref;
};

class OMBVEmptyWidget : public OMBVObjectWidget {

  public:
    OMBVEmptyWidget(const std::string &xmlName);

    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element);
    virtual QString getType() const { return "Empty"; }

  protected:
    std::string xmlName;
};

class OMBVPlaneWidget : public OMBVObjectWidget {

  public:
    OMBVPlaneWidget(const std::string &xmlName);
    virtual TiXmlElement* initializeUsingXML(TiXmlElement *element);
    virtual TiXmlElement* writeXMLFile(TiXmlNode *element); 
    virtual QString getType() const { return "Plane"; }

  protected:
    ExtXMLWidget *size, *numberOfLines;
    std::string xmlName;
};

#endif
