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
#include "mainwindow.h"
#include <QtGui>
#include <utility>
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  OMBVRigidBodyWidgetFactory::OMBVRigidBodyWidgetFactory()  {
    name.emplace_back("Cube");
    name.emplace_back("Cuboid");
    name.emplace_back("Frustum");
    name.emplace_back("Extrusion");
    name.emplace_back("Sphere");
    name.emplace_back("IvBody");
    name.emplace_back("CompoundRigidBody");
    name.emplace_back("InvisibleBody");
    xmlName.push_back(OPENMBV%"Cube");
    xmlName.push_back(OPENMBV%"Cuboid");
    xmlName.push_back(OPENMBV%"Frustum");
    xmlName.push_back(OPENMBV%"Extrusion");
    xmlName.push_back(OPENMBV%"Sphere");
    xmlName.push_back(OPENMBV%"IvBody");
    xmlName.push_back(OPENMBV%"CompoundRigidBody");
    xmlName.push_back(OPENMBV%"InvisibleBody");
  }

  QWidget* OMBVRigidBodyWidgetFactory::createWidget(int i) {
    if(i==0)
      return new CubeWidget("Cube"+toQStr(count++),OPENMBV%"Cube");
    if(i==1)
      return new CuboidWidget("Cuboid"+toQStr(count++),OPENMBV%"Cuboid");
    if(i==2)
      return new FrustumWidget("Frustum"+toQStr(count++),OPENMBV%"Frustum");
    if(i==3)
      return new ExtrusionWidget("Extrusion"+toQStr(count++),OPENMBV%"Extrusion");
    if(i==4)
      return new SphereWidget("Sphere"+toQStr(count++),OPENMBV%"Sphere");
    if(i==5)
      return new IvBodyWidget("IvBody"+toQStr(count++),OPENMBV%"IvBody");
    if(i==6)
      return new CompoundRigidBodyWidget("CompoundRigidBody"+toQStr(count++),OPENMBV%"CompoundRigidBody");
    if(i==7)
      return new InvisibleBodyWidget("InvisibleBody"+toQStr(count++),OPENMBV%"InvisibleBody");
    return nullptr;
  }

  DOMElement* OMBVObjectWidget::initializeUsingXML(DOMElement *element) {
    return element;
  }

  DOMElement* OMBVObjectWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *newele;
    if(xmlName!=FQN()) {
      DOMDocument *doc = parent->getOwnerDocument();
      newele=D(doc)->createElement(xmlName);
      E(newele)->setAttribute("name",name.toStdString());
      parent->insertBefore(newele,ref);
    }
    else
      newele = (DOMElement*)parent;
    return newele;
  }

  MBSOMBVWidget::MBSOMBVWidget(const QString &name, FQN xmlName_) : OMBVObjectWidget(name), xmlName(std::move(xmlName_)) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget,true,false,MBSIM%"diffuseColor");
    layout->addWidget(diffuseColor);

    transparency = new ExtWidget("Transparency",new ChoiceWidget2(new ScalarWidgetFactory("0.3",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"transparency");
    layout->addWidget(transparency);
  }

  DOMElement* MBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=(xmlName==FQN())?element:E(element)->getFirstElementChildNamed(xmlName);
    if(e) {
      diffuseColor->initializeUsingXML(e);
      transparency->initializeUsingXML(e);
    }
    return e;
  }

  DOMElement* MBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=initXMLFile(parent,ref);
    writeProperties(e);
    return e;
  }

  DOMElement* MBSOMBVWidget::initXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *newele;
    if(xmlName!=FQN()) {
    DOMDocument *doc = parent->getOwnerDocument();
    newele=D(doc)->createElement(xmlName);
    parent->insertBefore(newele,ref);
    }
    else
      newele = (DOMElement*)parent;
    return newele;
  }

  DOMElement* MBSOMBVWidget::writeProperties(DOMElement *e) {
    diffuseColor->writeXMLFile(e);
    transparency->writeXMLFile(e);
    return e;
  }

  PointMBSOMBVWidget::PointMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    size = new ExtWidget("Size",new ChoiceWidget2(new ScalarWidgetFactory("0.001",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"size");
    layout()->addWidget(size);
  }

  DOMElement* PointMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    size->initializeUsingXML(e);
    return e;
  }

  DOMElement* PointMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    size->writeXMLFile(e);
    writeProperties(e);
    return nullptr;
  }

  LineMBSOMBVWidget::LineMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    length = new ExtWidget("Length",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"length");
    layout()->addWidget(length);
  }

  DOMElement* LineMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    length->initializeUsingXML(e);
    return e;
  }

  DOMElement* LineMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    length->writeXMLFile(e);
    writeProperties(e);
    return nullptr;
  }

  PlaneMBSOMBVWidget::PlaneMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    length = new ExtWidget("Length",new ChoiceWidget2(new VecWidgetFactory(getScalars<QString>(2,"1"),vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"length");
    layout()->addWidget(length);
  }

  DOMElement* PlaneMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    length->initializeUsingXML(e);
    return e;
  }

  DOMElement* PlaneMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    length->writeXMLFile(e);
    writeProperties(e);
    return nullptr;
  }

  PlanarContourMBSOMBVWidget::PlanarContourMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    nodes = new ExtWidget("Nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"nodes");
    layout()->addWidget(nodes);
  }

  DOMElement* PlanarContourMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    nodes->initializeUsingXML(e);
    return e;
  }

  DOMElement* PlanarContourMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    nodes->writeXMLFile(e);
    writeProperties(e);
    return nullptr;
  }

  SpatialContourMBSOMBVWidget::SpatialContourMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    etaNodes = new ExtWidget("Eta nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"etaNodes");
    layout()->addWidget(etaNodes);
    xiNodes = new ExtWidget("Xi nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"xiNodes");
    layout()->addWidget(xiNodes);
  }

  DOMElement* SpatialContourMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    etaNodes->initializeUsingXML(e);
    xiNodes->initializeUsingXML(e);
    return e;
  }

  DOMElement* SpatialContourMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    etaNodes->writeXMLFile(e);
    xiNodes->writeXMLFile(e);
    writeProperties(e);
    return nullptr;
  }

  ArrowMBSOMBVWidget::ArrowMBSOMBVWidget(const QString &name, bool fromPoint) : MBSOMBVWidget(name) {
    scaleLength = new ExtWidget("Scale length",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleLength");
    layout()->addWidget(scaleLength);

    scaleSize = new ExtWidget("Scale size",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleSize");
    layout()->addWidget(scaleSize);

    vector<QString> list;
    list.emplace_back("\"toPoint\"");
    list.emplace_back("\"fromPoint\"");
    list.emplace_back("\"midPoint\"");
    referencePoint = new ExtWidget("Reference point",new TextChoiceWidget(list,fromPoint?1:0,true),true,false,MBSIM%"referencePoint");
    if(fromPoint)
      referencePoint->setChecked(true);
    layout()->addWidget(referencePoint);
  }

  DOMElement* ArrowMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    scaleLength->initializeUsingXML(e);
    scaleSize->initializeUsingXML(e);
    referencePoint->initializeUsingXML(e);
    return element;
  }

  DOMElement* ArrowMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    scaleLength->writeXMLFile(e);
    scaleSize->writeXMLFile(e);
    referencePoint->writeXMLFile(e);
    return e;
  }

  CoilSpringMBSOMBVWidget::CoilSpringMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    numberOfCoils = new ExtWidget("Number of coils",new ChoiceWidget2(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numberOfCoils");
    layout()->addWidget(numberOfCoils);

    springRadius = new ExtWidget("Spring radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"springRadius");
    layout()->addWidget(springRadius);

    crossSectionRadius = new ExtWidget("Cross section radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"crossSectionRadius");
    layout()->addWidget(crossSectionRadius);

    nominalLength = new ExtWidget("Nominal length",new ChoiceWidget2(new ScalarWidgetFactory("-1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"nominalLength");
    layout()->addWidget(nominalLength);

    vector<QString> list;
    list.emplace_back("\"tube\"");
    list.emplace_back("\"scaledTube\"");
    list.emplace_back("\"polyline\"");
    type = new ExtWidget("Type",new TextChoiceWidget(list,0,true),true,false,MBSIM%"type");
    layout()->addWidget(type);

    minCol = new ExtWidget("Minimal color value",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"minimalColorValue");
    layout()->addWidget(minCol);
    maxCol = new ExtWidget("Maximal color value",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximalColorValue");
    layout()->addWidget(maxCol);
  }

  DOMElement* CoilSpringMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    numberOfCoils->initializeUsingXML(e);
    springRadius->initializeUsingXML(e);
    crossSectionRadius->initializeUsingXML(e);
    nominalLength->initializeUsingXML(e);
    type->initializeUsingXML(e);
    minCol->initializeUsingXML(e);
    maxCol->initializeUsingXML(e);
    return element;
  }

  DOMElement* CoilSpringMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    numberOfCoils->writeXMLFile(e);
    springRadius->writeXMLFile(e);
    crossSectionRadius->writeXMLFile(e);
    nominalLength->writeXMLFile(e);
    type->writeXMLFile(e);
//    writeProperties(e);
    minCol->writeXMLFile(e);
    maxCol->writeXMLFile(e);
    return e;
  }

  FrameMBSOMBVWidget::FrameMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name,"") {
    size = new ExtWidget("Size",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"size");
    size->setToolTip("Set the size of the frame");
    layout()->addWidget(size);

    offset = new ExtWidget("Offset",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"offset");
    offset->setToolTip("Set the offset of the frame");
    layout()->addWidget(offset);
  }

  DOMElement* FrameMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    size->initializeUsingXML(element);
    offset->initializeUsingXML(element);
    transparency->initializeUsingXML(element);
    return element;
  }

  DOMElement* FrameMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    size->writeXMLFile(parent);
    offset->writeXMLFile(parent);
    transparency->writeXMLFile(parent);
    return nullptr;
  }

  OMBVDynamicColoredObjectWidget::OMBVDynamicColoredObjectWidget(const QString &name, const FQN &xmlName) : OMBVObjectWidget(name,xmlName) {
    layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    minimalColorValue = new ExtWidget("Minimal color value",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"minimalColorValue");
    layout->addWidget(minimalColorValue);

    maximalColorValue = new ExtWidget("Maximal color value",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"maximalColorValue");
    layout->addWidget(maximalColorValue);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget,true,false,OPENMBV%"diffuseColor");
    layout->addWidget(diffuseColor);

    transparency = new ExtWidget("Transparency",new ChoiceWidget2(new ScalarWidgetFactory("0.3",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,true,OPENMBV%"transparency");
    layout->addWidget(transparency);
  }

  DOMElement* OMBVDynamicColoredObjectWidget::initializeUsingXML(DOMElement *element) {
    OMBVObjectWidget::initializeUsingXML(element);
    minimalColorValue->initializeUsingXML(element);
    maximalColorValue->initializeUsingXML(element);
    diffuseColor->initializeUsingXML(element);
    transparency->initializeUsingXML(element);
    return element;
  }

  DOMElement* OMBVDynamicColoredObjectWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVObjectWidget::writeXMLFile(parent);
    minimalColorValue->writeXMLFile(e);
    maximalColorValue->writeXMLFile(e);
    diffuseColor->writeXMLFile(e);
    transparency->writeXMLFile(e);
    return e;
  }

  OMBVRigidBodyWidget::OMBVRigidBodyWidget(const QString &name, const FQN &xmlName) : OMBVDynamicColoredObjectWidget(name,xmlName) {

    transparency->setActive(true);

    trans = new ExtWidget("Initial translation",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"initialTranslation");
    layout->addWidget(trans);

    rot = new ExtWidget("Initial rotation",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,angleUnits())),QBoxLayout::RightToLeft),false,false,OPENMBV%"initialRotation");
    layout->addWidget(rot);
//
    scale = new ExtWidget("Scale factor",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"scaleFactor");
    layout->addWidget(scale);
  }

  DOMElement* OMBVRigidBodyWidget::initializeUsingXML(DOMElement *element) {
    OMBVDynamicColoredObjectWidget::initializeUsingXML(element);
    trans->initializeUsingXML(element);
    rot->initializeUsingXML(element);
    scale->initializeUsingXML(element);
    return element;
  }

  DOMElement* OMBVRigidBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVDynamicColoredObjectWidget::writeXMLFile(parent);
    trans->writeXMLFile(e);
    rot->writeXMLFile(e);
    scale->writeXMLFile(e);
    return e;
  }

  CubeWidget::CubeWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {

    length = new ExtWidget("Length",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"length");
    layout->addWidget(length);
  }

  DOMElement* CubeWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    length->initializeUsingXML(element);
    return element;
  }

  DOMElement* CubeWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    length->writeXMLFile(e);
    return e;
  }

  CuboidWidget::CuboidWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name, xmlName) {

    length = new ExtWidget("Length",new ChoiceWidget2(new VecWidgetFactory(strToVec(QString("[1;1;1]")),vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"length");
    layout->addWidget(length);
  }

  DOMElement* CuboidWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    length->initializeUsingXML(element);
    return element;
  }

  DOMElement* CuboidWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    length->writeXMLFile(e);
    return e;
  }

  SphereWidget::SphereWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {

    radius = new ExtWidget("Radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"radius");
    layout->addWidget(radius);
  }

  DOMElement* SphereWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    radius->initializeUsingXML(element);
    return element;
  }

  DOMElement* SphereWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    radius->writeXMLFile(e);
    return e;
  }

  FrustumWidget::FrustumWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {
    vector<QString> r(3);
    r[2] = "0.5";
    static_cast<VecWidget*>(static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(trans->getWidget())->getWidget())->getWidget())->setVec(r);

    base = new ExtWidget("Base radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"baseRadius");
    layout->addWidget(base);

    top = new ExtWidget("Top radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"topRadius");
    layout->addWidget(top);

    height = new ExtWidget("Height",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"height");
    layout->addWidget(height);

    innerBase = new ExtWidget("Inner base radius",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"innerBaseRadius");
    layout->addWidget(innerBase);

    innerTop = new ExtWidget("Inner top radius",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"innerTopRadius");
    layout->addWidget(innerTop);
  }

  DOMElement* FrustumWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    base->initializeUsingXML(element);
    top->initializeUsingXML(element);
    height->initializeUsingXML(element);
    innerBase->initializeUsingXML(element);
    innerTop->initializeUsingXML(element);
    return element;
  }

  DOMElement* FrustumWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    base->writeXMLFile(e);
    top->writeXMLFile(e);
    height->writeXMLFile(e);
    innerBase->writeXMLFile(e);
    innerTop->writeXMLFile(e);
    return e;
  }

  ExtrusionWidget::ExtrusionWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {
    vector<QString> list;
    list.emplace_back("\"odd\"");
    list.emplace_back("\"nonzero\"");
    list.emplace_back("\"positive\"");
    list.emplace_back("\"negative\"");
    list.emplace_back("\"absGEqTwo\"");
    windingRule = new ExtWidget("Winding rule",new TextChoiceWidget(list,1,true));
    layout->addWidget(windingRule);

    height = new ExtWidget("Height",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft));
    layout->addWidget(height);

    contour = new ExtWidget("Contour",new ChoiceWidget2(new MatRowsVarWidgetFactory(3,3,vector<QStringList>(3,lengthUnits()),vector<int>(3,2)),QBoxLayout::RightToLeft));
    layout->addWidget(contour);
  }

  DOMElement* ExtrusionWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    windingRule->initializeUsingXML(element);
    height->initializeUsingXML(element);
    contour->initializeUsingXML(element);
    return element;
  }

  DOMElement* ExtrusionWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    windingRule->writeXMLFile(e);
    height->writeXMLFile(e);
    contour->writeXMLFile(e);
    return e;
  }

  IvBodyWidget::IvBodyWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {

    ivFileName = new ExtWidget("Iv file name",new FileWidget("", "XML model files", "iv files (*.iv *.wrl)", 0, true, false),false,false,OPENMBV%"ivFileName");
    layout->addWidget(ivFileName);

    creaseEdges = new ExtWidget("Crease edges",new ChoiceWidget2(new ScalarWidgetFactory("-1",vector<QStringList>(2,angleUnits())),QBoxLayout::RightToLeft),true,false,OPENMBV%"creaseEdges");
    layout->addWidget(creaseEdges);

    boundaryEdges = new ExtWidget("Boundary edges",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft),true,false,OPENMBV%"boundaryEdges");
    layout->addWidget(boundaryEdges);
  }

  DOMElement* IvBodyWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    ivFileName->initializeUsingXML(element);
    creaseEdges->initializeUsingXML(element);
    boundaryEdges->initializeUsingXML(element);
    return element;
  }

  DOMElement* IvBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    ivFileName->writeXMLFile(e);
    creaseEdges->writeXMLFile(e);
    boundaryEdges->writeXMLFile(e);
    return e;
  }

  CompoundRigidBodyWidget::CompoundRigidBodyWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {
    bodies = new ExtWidget("Bodies",new ListWidget(new ChoiceWidgetFactory(new OMBVRigidBodyWidgetFactory,1),"Body",0,1));
    layout->addWidget(bodies);
  }

  DOMElement* CompoundRigidBodyWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(OPENMBV%"scaleFactor");
    bodies->initializeUsingXML(e->getNextElementSibling());
    return element;
  }

  DOMElement* CompoundRigidBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    bodies->writeXMLFile(e);
    return e;
  }

  FlexibleBodyFFRMBSOMBVWidget::FlexibleBodyFFRMBSOMBVWidget(const QString &name) : MBSOMBVWidget(name) {
    nodes = new ExtWidget("Nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"nodes");
    layout()->addWidget(nodes);
    indices = new ExtWidget("Indices",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"indices");
    layout()->addWidget(indices);
    minCol = new ExtWidget("Minimal color value",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"minimalColorValue");
    layout()->addWidget(minCol);
    maxCol = new ExtWidget("Maximal color value",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"maximalColorValue");
    layout()->addWidget(maxCol);
  }

  DOMElement* FlexibleBodyFFRMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVWidget::initializeUsingXML(element);
    nodes->initializeUsingXML(e);
    indices->initializeUsingXML(e);
    minCol->initializeUsingXML(e);
    maxCol->initializeUsingXML(e);
    return e;
  }

  DOMElement* FlexibleBodyFFRMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVWidget::initXMLFile(parent);
    nodes->writeXMLFile(e);
    indices->writeXMLFile(e);
    minCol->writeXMLFile(e);
    maxCol->writeXMLFile(e);
    writeProperties(e);
    return nullptr;
  }

}
