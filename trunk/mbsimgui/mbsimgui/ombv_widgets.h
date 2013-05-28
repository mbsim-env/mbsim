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

class ExtWidget;
class QVBoxLayout;
class QStackedWidget;
class QListWidget;
class QComboBox;
class RigidBody;

class OMBVObjectWidget : public Widget {

  public:
    OMBVObjectWidget(const QString &name_) : name(name_) {}
    virtual QString getType() const = 0;
    void setName(const QString &name_) {name = name_;}
  protected:
    QString name;
};

class OMBVFrameWidget : public OMBVObjectWidget {

  friend class OMBVFrameProperty;

  public:
    OMBVFrameWidget(const QString &name);
    virtual QString getType() const { return "Frame"; }
  protected:
    ExtWidget *size, *offset;
};

class OMBVDynamicColoredObjectWidget : public OMBVObjectWidget {

  friend class OMBVDynamicColoredObjectProperty;

  public:
    OMBVDynamicColoredObjectWidget(const QString &name);
  protected:
    QVBoxLayout *layout;
    ExtWidget *minimalColorValue, *maximalColorValue, *staticColor;
};


class OMBVArrowWidget : public OMBVDynamicColoredObjectWidget {

  friend class OMBVArrowProperty;

  public:
    OMBVArrowWidget(const QString &name, bool fromPoint=false);
    virtual QString getType() const { return "Arrow"; }
  protected:
    ExtWidget *diameter, *headDiameter, *headLength, *type, *referencePoint, *scaleLength;
};

class OMBVCoilSpringWidget : public OMBVObjectWidget {

  friend class OMBVCoilSpringProperty;

  public:
    OMBVCoilSpringWidget(const QString &name);
    virtual QString getType() const { return "CoilSpring"; }
  protected:
    ExtWidget *type, *numberOfCoils, *springRadius, *crossSectionRadius, *nominalLength, *scaleFactor;
};

class OMBVBodyWidget : public OMBVObjectWidget {

  friend class OMBVBodyProperty;

  public:
    OMBVBodyWidget(const QString &name);
    virtual QString getType() const = 0;
  protected:
    QVBoxLayout *layout;
    ExtWidget *trans, *rot, *color, *scale;
};

class InvisibleBodyWidget : public OMBVBodyWidget {

  public:
    InvisibleBodyWidget(const QString &name) : OMBVBodyWidget(name) {}
    virtual QString getType() const { return "InvisibleBody"; }
};

class CubeWidget : public OMBVBodyWidget {

  friend class CubeProperty;

  public:
    CubeWidget(const QString &name);
    virtual QString getType() const { return "Cube"; }
  protected:
    ExtWidget *length;
};

class CuboidWidget : public OMBVBodyWidget {

  friend class CuboidProperty;

  public:
    CuboidWidget(const QString &name);
    virtual QString getType() const { return "Cuboid"; }
  protected:
    ExtWidget *length;
};

class SphereWidget : public OMBVBodyWidget {

  friend class SphereProperty;

  public:
    SphereWidget(const QString &name);
    virtual QString getType() const { return "Sphere"; }
  protected:
    ExtWidget *radius;
};

class FrustumWidget : public OMBVBodyWidget {

  friend class FrustumProperty;

  public:
    FrustumWidget(const QString &name);
    virtual QString getType() const { return "Frustum"; }
  protected:
    ExtWidget *top, *base, *height, *innerBase, *innerTop;
};

class IvBodyWidget : public OMBVBodyWidget {

  friend class IvBodyProperty;

  public:
    IvBodyWidget(const QString &name);
    virtual QString getType() const { return "IvBody"; }
  protected:
    ExtWidget *ivFileName, *creaseEdges, *boundaryEdges;
};
class OMBVBodyChoiceWidget;

class CompoundRigidBodyWidget : public OMBVBodyWidget {
  Q_OBJECT

  friend class CompoundRigidBodyProperty;

  public:
    CompoundRigidBodyWidget(const QString &name);
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

class OMBVBodyChoiceWidget : public Widget {
  Q_OBJECT

  friend class OMBVBodyChoiceProperty;

  public:

    OMBVBodyChoiceWidget(const QString &name);

    void setName(const QString &name) {ombv->setName(name);}

  protected slots:
    void ombvSelection(int index);

  protected:
    QComboBox *comboBox;
    QVBoxLayout *layout;
    OMBVBodyWidget *ombv;
    QString name;
};

class OMBVBodySelectionWidget : public Widget {

  friend class OMBVBodySelectionProperty;

  public:

    OMBVBodySelectionWidget(RigidBody* body);

    virtual void updateWidget() {ref->updateWidget();}

  protected:
    OMBVBodyChoiceWidget *ombv;
    LocalFrameOfReferenceWidget *ref;
};

class OMBVEmptyWidget : public OMBVObjectWidget {

  public:
    OMBVEmptyWidget();

    virtual QString getType() const { return "Empty"; }
};

class OMBVPlaneWidget : public OMBVObjectWidget {

  friend class OMBVPlaneProperty;

  public:
    OMBVPlaneWidget();
    virtual QString getType() const { return "Plane"; }

  protected:
    ExtWidget *size, *numberOfLines;
};

#endif
