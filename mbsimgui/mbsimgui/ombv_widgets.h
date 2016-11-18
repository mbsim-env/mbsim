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
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
  };

  class OMBVObjectWidget : public Widget {

    public:
      OMBVObjectWidget(const QString &name_="NOTSET") : name(name_) {}
      void setName(const QString &name_) {name = name_;}
    protected:
      QString name;
  };

  class MBSOMBVWidget : public OMBVObjectWidget {

    friend class MBSOMBVProperty;

    public:
    MBSOMBVWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *diffuseColor, *transparency;
  };

  class PointMBSOMBVWidget : public MBSOMBVWidget {

    friend class PointMBSOMBVProperty;

    public:
    PointMBSOMBVWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *size;
  };

  class LineMBSOMBVWidget : public MBSOMBVWidget {

    friend class LineMBSOMBVProperty;

    public:
    LineMBSOMBVWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *length;
  };

  class PlaneMBSOMBVWidget : public MBSOMBVWidget {

    friend class PlaneMBSOMBVProperty;

    public:
    PlaneMBSOMBVWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *length;
  };

  class PlanarContourMBSOMBVWidget : public MBSOMBVWidget {

    friend class PlanarContourMBSOMBVProperty;

    public:
    PlanarContourMBSOMBVWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *nodes;
  };

  class SpatialContourMBSOMBVWidget : public MBSOMBVWidget {

    friend class SpatialContourMBSOMBVProperty;

    public:
    SpatialContourMBSOMBVWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *etaNodes, *xiNodes;
  };

  class OMBVFrameWidget : public OMBVObjectWidget {

    friend class OMBVFrameProperty;

    public:
    OMBVFrameWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *size, *offset, *transparency;
  };

  class OMBVDynamicColoredObjectWidget : public OMBVObjectWidget {

    friend class OMBVDynamicColoredObjectProperty;

    public:
    OMBVDynamicColoredObjectWidget(const QString &name="NOTSET");
    protected:
    QVBoxLayout *layout;
    ExtWidget *minimalColorValue, *maximalColorValue, *diffuseColor, *transparency;
  };

  class OMBVArrowWidget : public OMBVObjectWidget {

    friend class OMBVArrowProperty;

    public:
    OMBVArrowWidget(const QString &name="NOTSET", bool fromPoint=false);
    protected:
    ExtWidget *scaleLength, *scaleSize, *referencePoint, *diffuseColor, *transparency;
  };

  class OMBVCoilSpringWidget : public OMBVObjectWidget {

    friend class OMBVCoilSpringProperty;

    public:
    OMBVCoilSpringWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *type, *numberOfCoils, *springRadius, *crossSectionRadius, *nominalLength, *diffuseColor, *transparency;
  };

  class OMBVRigidBodyWidget : public OMBVDynamicColoredObjectWidget {

    friend class OMBVRigidBodyProperty;

    public:
    OMBVRigidBodyWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *trans, *rot, *scale;
  };

  class InvisibleBodyWidget : public OMBVRigidBodyWidget {

    public:
      InvisibleBodyWidget(const QString &name="NOTSET") : OMBVRigidBodyWidget(name) {}
  };

  class CubeWidget : public OMBVRigidBodyWidget {

    friend class CubeProperty;

    public:
    CubeWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *length;
  };

  class CuboidWidget : public OMBVRigidBodyWidget {

    friend class CuboidProperty;

    public:
    CuboidWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *length;
  };

  class SphereWidget : public OMBVRigidBodyWidget {

    friend class SphereProperty;

    public:
    SphereWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *radius;
  };

  class FrustumWidget : public OMBVRigidBodyWidget {

    friend class FrustumProperty;

    public:
    FrustumWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *top, *base, *height, *innerBase, *innerTop;
  };

  class ExtrusionWidget : public OMBVRigidBodyWidget {

    friend class ExtrusionProperty;

    public:
    ExtrusionWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *windingRule, *height, *contour;
  };

  class IvBodyWidget : public OMBVRigidBodyWidget {

    friend class IvBodyProperty;

    public:
    IvBodyWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *ivFileName, *creaseEdges, *boundaryEdges;
  };
  class OMBVRigidBodyChoiceWidget;

  class CompoundRigidBodyWidget : public OMBVRigidBodyWidget {

    friend class CompoundRigidBodyProperty;

    public:
    CompoundRigidBodyWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *bodies; 
  };

  class OMBVRigidBodySelectionWidget : public Widget {

    friend class OMBVRigidBodySelectionProperty;

    public:

    OMBVRigidBodySelectionWidget(Body* body);

    virtual void updateWidget() {ref->updateWidget();}

    protected:
    ExtWidget *ombv, *ref;
  };

  class FlexibleBodyFFRMBSOMBVWidget : public MBSOMBVWidget {

    friend class FlexibleBodyFFRMBSOMBVProperty;

    public:
    FlexibleBodyFFRMBSOMBVWidget(const QString &name="NOTSET");
    protected:
    ExtWidget *minCol, *maxCol, *nodes, *indices;
  };

}

#endif
