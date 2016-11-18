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

#include <config.h>
#include "ombv_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "body.h"
#include "frame.h"
#include <QtGui>

using namespace std;

namespace MBSimGUI {

  OMBVRigidBodyWidgetFactory::OMBVRigidBodyWidgetFactory() {
    name.push_back("Cube");
    name.push_back("Cuboid");
    name.push_back("Frustum");
    name.push_back("Extrusion");
    name.push_back("Sphere");
    name.push_back("IvBody");
    name.push_back("CompoundRigidBody");
    name.push_back("InvisibleBody");
  }

  QWidget* OMBVRigidBodyWidgetFactory::createWidget(int i) {
    if(i==0)
      return new CubeWidget;
    if(i==1)
      return new CuboidWidget;
    if(i==2)
      return new FrustumWidget;
    if(i==3)
      return new ExtrusionWidget;
    if(i==4)
      return new SphereWidget;
    if(i==5)
      return new IvBodyWidget;
    if(i==6)
      return new CompoundRigidBodyWidget;
    if(i==7)
      return new InvisibleBodyWidget;
    return NULL;
  }

  //class OmbvBodyWidgetFactory : public WidgetFactory {
  //  public:
  //    OmbvBodyWidgetFactory() { }
  //    Widget* createWidget(int i);
  //};
  //
  //Widget* OmbvBodyWidgetFactory::createWidget(int i) {
  //
  //  vector<QWidget*> widget;
  //  vector<QString> name;
  //  widget.push_back(new CubeWidget); name.push_back("Cube");
  //  widget.push_back(new CuboidWidget); name.push_back("Cuboid");
  //  widget.push_back(new FrustumWidget); name.push_back("Frustum");
  //  widget.push_back(new SphereWidget); name.push_back("Sphere");
  //  widget.push_back(new IvBodyWidget); name.push_back("IvBody");
  //  widget.push_back(new InvisibleBodyWidget); name.push_back("InvisibleBody");
  //  return new ChoiceWidget(widget,name);
  //}

  MBSOMBVWidget::MBSOMBVWidget(const QString &name) : OMBVObjectWidget(name) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget,true);
    layout->addWidget(diffuseColor);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.3"), noUnitUnits(), 1));
    transparency = new ExtWidget("Transparency",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(transparency);
  }

  PointMBSOMBVWidget::PointMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.001"), lengthUnits(), 4));
    size = new ExtWidget("Size",new ExtPhysicalVarWidget(input),true);
    layout()->addWidget(size);
  }

  LineMBSOMBVWidget::LineMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    length = new ExtWidget("Size",new ExtPhysicalVarWidget(input),true);
    layout()->addWidget(length);
  }

  PlaneMBSOMBVWidget::PlaneMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new VecWidget(getScalars<QString>(2,"1")), lengthUnits(), 4));
    length = new ExtWidget("Size",new ExtPhysicalVarWidget(input),true);
    layout()->addWidget(length);
  }

  PlanarContourMBSOMBVWidget::PlanarContourMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new VecSizeVarWidget(2,1,100), noUnitUnits(), 4));
    nodes = new ExtWidget("Nodes",new ExtPhysicalVarWidget(input),true);
    layout()->addWidget(nodes);
  }

  SpatialContourMBSOMBVWidget::SpatialContourMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new VecSizeVarWidget(2,1,100), noUnitUnits(), 4));
    etaNodes = new ExtWidget("Eta nodes",new ExtPhysicalVarWidget(input),true);
    layout()->addWidget(etaNodes);
    input.clear();
    input.push_back(new PhysicalVariableWidget(new VecSizeVarWidget(2,1,100), noUnitUnits(), 4));
    xiNodes = new ExtWidget("Xi nodes",new ExtPhysicalVarWidget(input),true);
    layout()->addWidget(xiNodes);
  }

  OMBVFrameWidget::OMBVFrameWidget(const QString &name) : OMBVObjectWidget(name) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    size = new ExtWidget("Size",new ExtPhysicalVarWidget(input),true);
    size->setToolTip("Set the size of the frame");
    layout->addWidget(size);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
    offset = new ExtWidget("Offset",new ExtPhysicalVarWidget(input),true);
    offset->setToolTip("Set the offset of the frame");
    layout->addWidget(offset);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.3"), noUnitUnits(), 1));
    transparency = new ExtWidget("Transparency",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(transparency);
  }

  OMBVDynamicColoredObjectWidget::OMBVDynamicColoredObjectWidget(const QString &name) : OMBVObjectWidget(name) {
    layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), noUnitUnits(), 1));
    minimalColorValue = new ExtWidget("Minimal color value",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(minimalColorValue);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
    maximalColorValue = new ExtWidget("Maximal color value",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(maximalColorValue);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget,true);
    layout->addWidget(diffuseColor);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.3"), noUnitUnits(), 1));
    transparency = new ExtWidget("Transparency",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(transparency);
  }

  OMBVArrowWidget::OMBVArrowWidget(const QString &name, bool fromPoint) : OMBVObjectWidget(name) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
    scaleLength = new ExtWidget("Scale length",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(scaleLength);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
    scaleSize = new ExtWidget("Scale size",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(scaleSize);

    vector<QString> list;
    list.push_back("toPoint");
    list.push_back("fromPoint");
    list.push_back("midPoint");
    referencePoint = new ExtWidget("Reference point",new TextChoiceWidget(list,fromPoint?1:0),true);
    if(fromPoint)
      referencePoint->setChecked(true);
    layout->addWidget(referencePoint);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget,true);
    layout->addWidget(diffuseColor);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.3"), noUnitUnits(), 1));
    transparency = new ExtWidget("Transparency",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(transparency);
  }

  OMBVCoilSpringWidget::OMBVCoilSpringWidget(const QString &name) : OMBVObjectWidget(name) {
    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("3"), QStringList(), 1));
    numberOfCoils= new ExtWidget("Number of coils",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(numberOfCoils);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    springRadius= new ExtWidget("Spring radius",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(springRadius);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("-1"), lengthUnits(), 4));
    crossSectionRadius = new ExtWidget("Cross section radius",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(crossSectionRadius);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("-1"), lengthUnits(), 4));
    nominalLength= new ExtWidget("Nominal length",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(nominalLength);

    vector<QString> list;
    list.push_back("tube");
    list.push_back("scaledTube");
    list.push_back("polyline");
    type = new ExtWidget("Type",new TextChoiceWidget(list,0),true);
    layout->addWidget(type);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget,true);
    layout->addWidget(diffuseColor);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0.3"), noUnitUnits(), 1));
    transparency = new ExtWidget("Transparency",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(transparency);
  }

  OMBVRigidBodyWidget::OMBVRigidBodyWidget(const QString &name) : OMBVDynamicColoredObjectWidget(name) {

    transparency->setActive(true);

    trans = new ExtWidget("Initial translation",new ChoiceWidget2(new VecWidgetFactory(3),QBoxLayout::RightToLeft));
    layout->addWidget(trans);

    rot = new ExtWidget("Initial rotation",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,angleUnits())),QBoxLayout::RightToLeft));
    layout->addWidget(rot);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), noUnitUnits(), 1));
    scale = new ExtWidget("Scale factor",new ExtPhysicalVarWidget(input));
    layout->addWidget(scale);
  }

  CubeWidget::CubeWidget(const QString &name) : OMBVRigidBodyWidget(name) {

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    length = new ExtWidget("Length",new ExtPhysicalVarWidget(input));
    layout->addWidget(length);
  }

  CuboidWidget::CuboidWidget(const QString &name) : OMBVRigidBodyWidget(name) {

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new VecWidget(getScalars<QString>(3,"1"),true), lengthUnits(), 4));
    length = new ExtWidget("Length",new ExtPhysicalVarWidget(input));
    layout->addWidget(length);
  }

  SphereWidget::SphereWidget(const QString &name) : OMBVRigidBodyWidget(name) {

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    radius = new ExtWidget("Radius",new ExtPhysicalVarWidget(input));
    layout->addWidget(radius);
  }

  FrustumWidget::FrustumWidget(const QString &name) : OMBVRigidBodyWidget(name) {
    vector<QString> r(3);
    r[2] = "0.5";
    static_cast<VecWidget*>(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(trans->getWidget())->getWidget())->getWidget())->setVec(r);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    top = new ExtWidget("Top radius",new ExtPhysicalVarWidget(input));
    layout->addWidget(top);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    base = new ExtWidget("Base radius",new ExtPhysicalVarWidget(input));
    layout->addWidget(base);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("1"), lengthUnits(), 4));
    height = new ExtWidget("Height",new ExtPhysicalVarWidget(input));
    layout->addWidget(height);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), lengthUnits(), 4));
    innerTop = new ExtWidget("Inner top radius",new ExtPhysicalVarWidget(input));
    layout->addWidget(innerTop);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("0"), lengthUnits(), 4));
    innerBase = new ExtWidget("Inner base radius",new ExtPhysicalVarWidget(input));
    layout->addWidget(innerBase);
  }

  ExtrusionWidget::ExtrusionWidget(const QString &name) : OMBVRigidBodyWidget(name) {
    vector<QString> list;
    list.push_back("odd");
    list.push_back("nonzero");
    list.push_back("positive");
    list.push_back("negative");
    list.push_back("absGEqTwo");
    windingRule = new ExtWidget("Winding rule",new TextChoiceWidget(list,0));
    layout->addWidget(windingRule);

    height = new ExtWidget("Height",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft));
    layout->addWidget(height);

    contour = new ExtWidget("Contour",new ChoiceWidget2(new MatRowsVarWidgetFactory(3,3,vector<QStringList>(3,lengthUnits()),vector<int>(3,2)),QBoxLayout::RightToLeft));
    layout->addWidget(contour);
  }

  IvBodyWidget::IvBodyWidget(const QString &name) : OMBVRigidBodyWidget(name) {

    ivFileName = new ExtWidget("Iv file name",new FileWidget("XML model files", "iv files (*.iv *.wrl)"));
    layout->addWidget(ivFileName);

    vector<PhysicalVariableWidget*> input;
    input.push_back(new PhysicalVariableWidget(new ScalarWidget("-1"), angleUnits(), 0));
    creaseEdges = new ExtWidget("Crease edges",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(creaseEdges);

    input.clear();
    input.push_back(new PhysicalVariableWidget(new BoolWidget("0"), QStringList(), 4));
    boundaryEdges = new ExtWidget("Boundary edges",new ExtPhysicalVarWidget(input),true);
    layout->addWidget(boundaryEdges);
  }

  CompoundRigidBodyWidget::CompoundRigidBodyWidget(const QString &name) : OMBVRigidBodyWidget(name) {
    bodies = new ExtWidget("Bodies",new ListWidget(new ChoiceWidgetFactory(new OMBVRigidBodyWidgetFactory),"Body",1,1));
    layout->addWidget(bodies);
  }

  OMBVRigidBodySelectionWidget::OMBVRigidBodySelectionWidget(Body *body) {

    QVBoxLayout *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    ombv = new ExtWidget("Body",new ChoiceWidget2(new OMBVRigidBodyWidgetFactory),true);

    ref=new ExtWidget("Frame of reference",new LocalFrameOfReferenceWidget(body),true);
    layout->addWidget(ombv);
    layout->addWidget(ref);
  }

  FlexibleBodyFFRMBSOMBVWidget::FlexibleBodyFFRMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    minCol = new ExtWidget("Minimal color value",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft),true);
    layout()->addWidget(minCol);
    maxCol = new ExtWidget("Maximal color value",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft),true);
    layout()->addWidget(maxCol);
    nodes = new ExtWidget("Nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(1,vector<QStringList>(3)),QBoxLayout::RightToLeft),true);
    layout()->addWidget(nodes);
    indices = new ExtWidget("Indices",new ChoiceWidget2(new VecSizeVarWidgetFactory(1,vector<QStringList>(3)),QBoxLayout::RightToLeft),true);
    layout()->addWidget(indices);
  }


}
