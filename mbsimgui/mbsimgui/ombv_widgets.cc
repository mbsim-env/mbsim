/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#include <config.h>
#include "ombv_widgets.h"
#include "variable_widgets.h"
#include "extended_widgets.h"
#include "body.h"
#include "frame.h"
#include "unknown_widget.h"
#include "mainwindow.h"
#include <utility>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  OMBVRigidBodyWidgetFactory::OMBVRigidBodyWidgetFactory()  {
    name.emplace_back("Compound rigid body");
    name.emplace_back("Cube");
    name.emplace_back("Cuboid");
    name.emplace_back("Cylinder");
    name.emplace_back("Cylindrical gear");
    name.emplace_back("Extrusion");
    name.emplace_back("Frustum");
    name.emplace_back("Invisible body");
    name.emplace_back("Open Inventor body");
    name.emplace_back("Sphere");
    name.emplace_back("Unknown object");
    xmlName.push_back(OPENMBV%"CompoundRigidBody");
    xmlName.push_back(OPENMBV%"Cube");
    xmlName.push_back(OPENMBV%"Cuboid");
    xmlName.push_back(OPENMBV%"Cylinder");
    xmlName.push_back(OPENMBV%"CylindricalGear");
    xmlName.push_back(OPENMBV%"Extrusion");
    xmlName.push_back(OPENMBV%"Frustum");
    xmlName.push_back(OPENMBV%"InvisibleBody");
    xmlName.push_back(OPENMBV%"IvBody");
    xmlName.push_back(OPENMBV%"Sphere");
    xmlName.push_back(OPENMBV%"UnknownObject");
  }

  Widget* OMBVRigidBodyWidgetFactory::createWidget(int i) {
    if(i==0)
      return new CompoundRigidBodyWidget("CompoundRigidBody"+toQStr(count++),OPENMBV%"CompoundRigidBody");
    if(i==1)
      return new CubeWidget("Cube"+toQStr(count++),OPENMBV%"Cube");
    if(i==2)
      return new CuboidWidget("Cuboid"+toQStr(count++),OPENMBV%"Cuboid");
    if(i==3)
      return new CylinderWidget("Cylinder"+toQStr(count++),OPENMBV%"Cylinder");
    if(i==4)
      return new CylindricalGearWidget("CylindricalGear"+toQStr(count++),OPENMBV%"CylindricalGear");
    if(i==5)
      return new ExtrusionWidget("Extrusion"+toQStr(count++),OPENMBV%"Extrusion");
    if(i==6)
      return new FrustumWidget("Frustum"+toQStr(count++),OPENMBV%"Frustum");
    if(i==7)
      return new InvisibleBodyWidget("InvisibleBody"+toQStr(count++),OPENMBV%"InvisibleBody");
    if(i==8)
      return new IvBodyWidget("IvBody"+toQStr(count++),OPENMBV%"IvBody");
    if(i==9)
      return new SphereWidget("Sphere"+toQStr(count++),OPENMBV%"Sphere");
    if(i==10)
      return new UnknownWidget<OMBVRigidBodyWidget>;
    return nullptr;
  }

  OMBVFlexibleBodyWidgetFactory::OMBVFlexibleBodyWidgetFactory()  {
    name.emplace_back("Dynamic indexed face set");
    name.emplace_back("Dynamic indexed line set");
    name.emplace_back("Dynamic point set");
    name.emplace_back("Unknown object");
    xmlName.push_back(OPENMBV%"DynamicIndexedFaceSet");
    xmlName.push_back(OPENMBV%"DynamicIndexedLineSet");
    xmlName.push_back(OPENMBV%"DynamicPointSet");
    xmlName.push_back(OPENMBV%"UnknownObject");
  }

  Widget* OMBVFlexibleBodyWidgetFactory::createWidget(int i) {
    if(i==0)
      return new DynamicIndexedFaceSetWidget("DynamicIndexedFaceSet"+toQStr(count++),OPENMBV%"DynamicIndexedFaceSet");
    if(i==1)
      return new DynamicIndexedLineSetWidget("DynamicIndexedLineSet"+toQStr(count++),OPENMBV%"DynamicIndexedLineSet");
    if(i==2)
      return new DynamicPointSetWidget("DynamicPointSet"+toQStr(count++),OPENMBV%"DynamicPointSet");
    if(i==3)
      return new UnknownWidget<OMBVObjectWidget>;
    return nullptr;
  }

  OMBVIvScreenAnnotationWidgetFactory::OMBVIvScreenAnnotationWidgetFactory()  {
    name.emplace_back("Iv screen annotation");
    name.emplace_back("Unknown object");
    xmlName.push_back(OPENMBV%"IvScreenAnnotation");
    xmlName.push_back(OPENMBV%"UnknownObject");
  }

  Widget* OMBVIvScreenAnnotationWidgetFactory::createWidget(int i) {
    if(i==0)
      return new IvScreenAnnotationWidget("IvScreenAnnotation",OPENMBV%"IvScreenAnnotation");
    else
      return new UnknownWidget<OMBVObjectWidget>;
    return nullptr;
  }

  IvDataWidgetFactory::IvDataWidgetFactory()  {
    name.emplace_back("From filename");
    name.emplace_back("From string content");
    xmlName.push_back(OPENMBV%"ivFileName");
    xmlName.push_back(OPENMBV%"ivContent");
  }

  Widget* IvDataWidgetFactory::createWidget(int i) {
    if(i==0)
      return new ExtWidget("Open Inventor file name",new FileWidget("", "Open Open Inventor file", "Inventor files (*.iv);;VRML files (*.wrl *.vrml)", 0, true));
    if(i==1) {
      auto tew = new TextEditorWidget;
      tew->enableMonospaceFont();
      return new ExtWidget("Open Inventor string content",tew);
    }
    return nullptr;
  }

  MBSOMBVColoreBodyWidget::MBSOMBVColoreBodyWidget(const vector<QString> &c) {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget(c),true,false,MBSIM%"diffuseColor");
    layout->addWidget(diffuseColor);

    transparency = new ExtWidget("Transparency",new ChoiceWidget(new ScalarWidgetFactory("0.3",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"transparency");
    layout->addWidget(transparency);

    pointSize = new ExtWidget("Point size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pointSize");
    layout->addWidget(pointSize);

    lineWidth = new ExtWidget("Line width",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"lineWidth");
    layout->addWidget(lineWidth);
  }

  DOMElement* MBSOMBVColoreBodyWidget::initializeUsingXML(DOMElement *e) {
    diffuseColor->initializeUsingXML(e);
    transparency->initializeUsingXML(e);
    pointSize->initializeUsingXML(e);
    lineWidth->initializeUsingXML(e);
    return e;
  }

  MBSOMBVRigidBodyWidget::MBSOMBVRigidBodyWidget() : MBSOMBVColoreBodyWidget() {
    path = new ExtWidget("Draw path",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"path");
    layout()->addWidget(path);
  }

  DOMElement* MBSOMBVRigidBodyWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVColoreBodyWidget::initializeUsingXML(element);
    path->initializeUsingXML(e);
    return e;
  }

  DOMElement* MBSOMBVRigidBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVColoreBodyWidget::writeXMLFile(parent);
    path->writeXMLFile(e);
    return e;
  }

  DOMElement* MBSOMBVColoreBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    diffuseColor->writeXMLFile(parent);
    transparency->writeXMLFile(parent);
    pointSize->writeXMLFile(parent);
    lineWidth->writeXMLFile(parent);
    return static_cast<DOMElement*>(parent);
  }

  MBSOMBVDynamicColoreBodyWidget::MBSOMBVDynamicColoreBodyWidget(const vector<QString> &c, const vector<QString> &cRL) : MBSOMBVColoreBodyWidget(c) {

    colorRepresentation = new ExtWidget("Color representation",new TextChoiceWidget(cRL,0,true),true,false,MBSIM%"colorRepresentation");
    layout()->addWidget(colorRepresentation);

    minimalColorValue = new ExtWidget("Minimal color value",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"minimalColorValue");
    layout()->addWidget(minimalColorValue);

    maximalColorValue = new ExtWidget("Maximal color value",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximalColorValue");
    layout()->addWidget(maximalColorValue);
  }

  DOMElement* MBSOMBVDynamicColoreBodyWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVColoreBodyWidget::initializeUsingXML(element);
    colorRepresentation->initializeUsingXML(e);
    minimalColorValue->initializeUsingXML(e);
    maximalColorValue->initializeUsingXML(e);
    return e;
  }

  DOMElement* MBSOMBVDynamicColoreBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVColoreBodyWidget::writeXMLFile(parent);
    colorRepresentation->writeXMLFile(e);
    minimalColorValue->writeXMLFile(e);
    maximalColorValue->writeXMLFile(e);
    return e;
  }

  LineMBSOMBVWidget::LineMBSOMBVWidget() {
    length = new ExtWidget("Length",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"length");
    layout()->addWidget(length);
  }

  DOMElement* LineMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVColoreBodyWidget::initializeUsingXML(element);
    length->initializeUsingXML(e);
    return e;
  }

  DOMElement* LineMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVColoreBodyWidget::writeXMLFile(parent);
    length->writeXMLFile(e);
    return nullptr;
  }

  PlaneMBSOMBVWidget::PlaneMBSOMBVWidget() {
    length = new ExtWidget("Length",new ChoiceWidget(new VecWidgetFactory(getScalars<QString>(2,"1"),vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"length");
    layout()->addWidget(length);
  }

  DOMElement* PlaneMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVColoreBodyWidget::initializeUsingXML(element);
    length->initializeUsingXML(e);
    return e;
  }

  DOMElement* PlaneMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVColoreBodyWidget::writeXMLFile(parent);
    length->writeXMLFile(e);
    return nullptr;
  }

  PlanarContourMBSOMBVWidget::PlanarContourMBSOMBVWidget() {
    nodes = new ExtWidget("Nodes",new ChoiceWidget(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"nodes");
    layout()->addWidget(nodes);
    filled = new ExtWidget("Filled",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft),true,false,MBSIM%"filled");
    layout()->addWidget(filled);
  }

  DOMElement* PlanarContourMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVColoreBodyWidget::initializeUsingXML(element);
    nodes->initializeUsingXML(e);
    filled->initializeUsingXML(e);
    return e;
  }

  DOMElement* PlanarContourMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVColoreBodyWidget::writeXMLFile(parent);
    nodes->writeXMLFile(e);
    filled->writeXMLFile(e);
    return nullptr;
  }

  SpatialContourMBSOMBVWidget::SpatialContourMBSOMBVWidget() {
    etaNodes = new ExtWidget("Eta nodes",new ChoiceWidget(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"etaNodes");
    layout()->addWidget(etaNodes);
    xiNodes = new ExtWidget("Xi nodes",new ChoiceWidget(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"xiNodes");
    layout()->addWidget(xiNodes);
  }

  DOMElement* SpatialContourMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVColoreBodyWidget::initializeUsingXML(element);
    etaNodes->initializeUsingXML(e);
    xiNodes->initializeUsingXML(e);
    return e;
  }

  DOMElement* SpatialContourMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVColoreBodyWidget::writeXMLFile(parent);
    etaNodes->writeXMLFile(e);
    xiNodes->writeXMLFile(e);
    return nullptr;
  }

  ArrowMBSOMBVWidget::ArrowMBSOMBVWidget(const vector<QString> &c, const vector<QString> &cRL, int refPoint) : MBSOMBVDynamicColoreBodyWidget(c,cRL) {

    scaleLength = new ExtWidget("Scale length",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleLength");
    layout()->addWidget(scaleLength);

    scaleSize = new ExtWidget("Scale size",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"scaleSize");
    layout()->addWidget(scaleSize);

    vector<QString> list;
    list.emplace_back("\"line\"");
    list.emplace_back("\"fromHead\"");
    list.emplace_back("\"toHead\"");
    list.emplace_back("\"bothHead\"");
    list.emplace_back("\"fromDoubleHead\"");
    list.emplace_back("\"toDoubleHead\"");
    list.emplace_back("\"bothDoubleHead\"");
    type = new ExtWidget("Type",new TextChoiceWidget(list,2,true),true,false,MBSIM%"type");
    layout()->addWidget(type);

    list.clear();
    list.emplace_back("\"toPoint\"");
    list.emplace_back("\"fromPoint\"");
    list.emplace_back("\"midPoint\"");
    referencePoint = new ExtWidget("Reference point",new TextChoiceWidget(list,refPoint,true),true,false,MBSIM%"referencePoint");
    layout()->addWidget(referencePoint);
  }

  vector<QString> ArrowMBSOMBVWidget::getColorRepresentation() {
    vector<QString> cRL;
    cRL.emplace_back("\"none\"");
    cRL.emplace_back("\"absoluteValue\"");
    return cRL;
  }

  DOMElement* ArrowMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVDynamicColoreBodyWidget::initializeUsingXML(element);
    scaleLength->initializeUsingXML(e);
    scaleSize->initializeUsingXML(e);
    type->initializeUsingXML(e);
    referencePoint->initializeUsingXML(e);
    return element;
  }

  DOMElement* ArrowMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVDynamicColoreBodyWidget::writeXMLFile(parent);
    scaleLength->writeXMLFile(e);
    scaleSize->writeXMLFile(e);
    type->writeXMLFile(e);
    referencePoint->writeXMLFile(e);
    return e;
  }

  InteractionArrowMBSOMBVWidget::InteractionArrowMBSOMBVWidget(const vector<QString> &cRL) : ArrowMBSOMBVWidget(getRedColor(),cRL,0) {

    vector<QString> list;
    list.emplace_back("\"action\"");
    list.emplace_back("\"reaction\"");
    list.emplace_back("\"both\"");
    sideOfInteraction = new ExtWidget("Side of interaction",new TextChoiceWidget(list,0,true),true,false,MBSIM%"sideOfInteraction");
    layout()->addWidget(sideOfInteraction);
  }

  DOMElement* InteractionArrowMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=ArrowMBSOMBVWidget::initializeUsingXML(element);
    sideOfInteraction->initializeUsingXML(e);
    return element;
  }

  DOMElement* InteractionArrowMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=ArrowMBSOMBVWidget::writeXMLFile(parent);
    sideOfInteraction->writeXMLFile(e);
    return e;
  }

  vector<QString> FrictionArrowMBSOMBVWidget::getColorRepresentation() {
    vector<QString> cRL;
    cRL.emplace_back("\"none\"");
    cRL.emplace_back("\"absoluteValue\"");
    cRL.emplace_back("\"stickslip\"");
    return cRL;
  }

  CoilSpringMBSOMBVWidget::CoilSpringMBSOMBVWidget(const std::vector<QString> &cRL) : MBSOMBVDynamicColoreBodyWidget(getBlueColor(),cRL) {

    numberOfCoils = new ExtWidget("Number of coils",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numberOfCoils");
    layout()->addWidget(numberOfCoils);

    springRadius = new ExtWidget("Spring radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"springRadius");
    layout()->addWidget(springRadius);

    crossSectionRadius = new ExtWidget("Cross section radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"crossSectionRadius");
    layout()->addWidget(crossSectionRadius);

    nominalLength = new ExtWidget("Nominal length",new ChoiceWidget(new ScalarWidgetFactory("-1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"nominalLength");
    layout()->addWidget(nominalLength);

    vector<QString> list;
    list.emplace_back("\"tube\"");
    list.emplace_back("\"scaledTube\"");
    list.emplace_back("\"polyline\"");
    type = new ExtWidget("Type",new TextChoiceWidget(list,0,true),true,false,MBSIM%"type");
    layout()->addWidget(type);
  }

  vector<QString> CoilSpringMBSOMBVWidget::getColorRepresentation() {
    vector<QString> cRL;
    cRL.emplace_back("\"none\"");
    cRL.emplace_back("\"deflection\"");
    cRL.emplace_back("\"tensileForce\"");
    cRL.emplace_back("\"compressiveForce\"");
    cRL.emplace_back("\"absoluteForce\"");
    return cRL;
  }

  DOMElement* CoilSpringMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    DOMElement *e=MBSOMBVDynamicColoreBodyWidget::initializeUsingXML(element);
    numberOfCoils->initializeUsingXML(e);
    springRadius->initializeUsingXML(e);
    crossSectionRadius->initializeUsingXML(e);
    nominalLength->initializeUsingXML(e);
    type->initializeUsingXML(e);
    return element;
  }

  DOMElement* CoilSpringMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVDynamicColoreBodyWidget::writeXMLFile(parent);
    numberOfCoils->writeXMLFile(e);
    springRadius->writeXMLFile(e);
    crossSectionRadius->writeXMLFile(e);
    nominalLength->writeXMLFile(e);
    type->writeXMLFile(e);
    return e;
  }

  FrameMBSOMBVWidget::FrameMBSOMBVWidget() {
    size = new ExtWidget("Size",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"size");
    layout()->addWidget(size);

    offset = new ExtWidget("Offset",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"offset");
    layout()->addWidget(offset);
  }

  DOMElement* FrameMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    MBSOMBVRigidBodyWidget::initializeUsingXML(element);
    size->initializeUsingXML(element);
    offset->initializeUsingXML(element);
    return element;
  }

  DOMElement* FrameMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=MBSOMBVRigidBodyWidget::writeXMLFile(parent);
    size->writeXMLFile(e);
    offset->writeXMLFile(e);
    return nullptr;
  }

  vector<QString> FlexibleBodyMBSOMBVWidget::getColorRepresentation() {
    vector<QString> cRL;
    cRL.emplace_back("\"none\"");
    cRL.emplace_back("\"xDisplacement\"");
    cRL.emplace_back("\"yDisplacement\"");
    cRL.emplace_back("\"zDisplacement\"");
    cRL.emplace_back("\"totalDisplacement\"");
    cRL.emplace_back("\"xxStress\"");
    cRL.emplace_back("\"yyStress\"");
    cRL.emplace_back("\"zzStress\"");
    cRL.emplace_back("\"xyStress\"");
    cRL.emplace_back("\"yzStress\"");
    cRL.emplace_back("\"zxStress\"");
    cRL.emplace_back("\"equivalentStress\"");
    return cRL;
  }

  FlexibleFfrBodyMBSOMBVWidget::FlexibleFfrBodyMBSOMBVWidget(const vector<QString> &cRL) : FlexibleBodyMBSOMBVWidget(cRL) {

    vector<QString> list;
    list.emplace_back("\"points\"");
    visu = new ExtWidget("Visualization",new TextChoiceWidget(list,1,true),true,false,MBSIMFLEX%"visualization");
    layout()->addWidget(visu);
  }

  DOMElement* FlexibleFfrBodyMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    FlexibleBodyMBSOMBVWidget::initializeUsingXML(element);
    visu->initializeUsingXML(element);
    return element;
  }

  DOMElement* FlexibleFfrBodyMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=FlexibleBodyMBSOMBVWidget::writeXMLFile(parent);
    visu->writeXMLFile(e);
    return e;
  }

  ExternalFlexibleFfrBodyMBSOMBVWidget::ExternalFlexibleFfrBodyMBSOMBVWidget(const vector<QString> &cRL) : FlexibleBodyMBSOMBVWidget(cRL) {

    vector<QString> list;
    list.emplace_back("\"points\"");
    list.emplace_back("\"lines\"");
    list.emplace_back("\"faces\"");
    visu = new ExtWidget("Visualization",new TextChoiceWidget(list,0,true),true,false,MBSIMFLEX%"visualization");
    layout()->addWidget(visu);
  }

  DOMElement* ExternalFlexibleFfrBodyMBSOMBVWidget::initializeUsingXML(DOMElement *element) {
    FlexibleBodyMBSOMBVWidget::initializeUsingXML(element);
    visu->initializeUsingXML(element);
    return element;
  }

  DOMElement* ExternalFlexibleFfrBodyMBSOMBVWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=FlexibleBodyMBSOMBVWidget::writeXMLFile(parent);
    visu->writeXMLFile(e);
    return e;
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

  OMBVBodyWidget::OMBVBodyWidget(const QString &name, const FQN &xmlName) : OMBVObjectWidget(name,xmlName) {
    layout = new QVBoxLayout;
    layout->setMargin(0);
    setLayout(layout);

    pointSize = new ExtWidget("Point size",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"pointSize");
    layout->addWidget(pointSize);

    lineWidth = new ExtWidget("Line width",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,QStringList()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"lineWidth");
    layout->addWidget(lineWidth);
  }

  DOMElement* OMBVBodyWidget::initializeUsingXML(DOMElement *element) {
    OMBVObjectWidget::initializeUsingXML(element);
    if(E(element)->hasAttribute("pointSize")) {
      pointSize->setActive(true);
      pointSize->getFirstWidget<PhysicalVariableWidget>()->setValue(QString::fromStdString(E(element)->getAttribute("pointSize")));
    }
    if(E(element)->hasAttribute("lineWidth")) {
      lineWidth->setActive(true);
      lineWidth->getFirstWidget<PhysicalVariableWidget>()->setValue(QString::fromStdString(E(element)->getAttribute("lineWidth")));
    }
    return element;
  }

  DOMElement* OMBVBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVObjectWidget::writeXMLFile(parent);
    if(pointSize->isActive())
      E(e)->setAttribute("pointSize", pointSize->getFirstWidget<PhysicalVariableWidget>()->getValue().toStdString());
    if(lineWidth->isActive())
      E(e)->setAttribute("lineWidth", lineWidth->getFirstWidget<PhysicalVariableWidget>()->getValue().toStdString());
    return e;
  }

  OMBVDynamicColoredObjectWidget::OMBVDynamicColoredObjectWidget(const QString &name, const FQN &xmlName) : OMBVBodyWidget(name,xmlName) {

    minimalColorValue = new ExtWidget("Minimal color value",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"minimalColorValue");
    layout->addWidget(minimalColorValue);

    maximalColorValue = new ExtWidget("Maximal color value",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"maximalColorValue");
    layout->addWidget(maximalColorValue);

    diffuseColor = new ExtWidget("Diffuse color",new ColorWidget,true,false,OPENMBV%"diffuseColor");
    layout->addWidget(diffuseColor);

    transparency = new ExtWidget("Transparency",new ChoiceWidget(new ScalarWidgetFactory("0.3",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,true,OPENMBV%"transparency");
    layout->addWidget(transparency);
  }

  DOMElement* OMBVDynamicColoredObjectWidget::initializeUsingXML(DOMElement *element) {
    OMBVBodyWidget::initializeUsingXML(element);
    minimalColorValue->initializeUsingXML(element);
    maximalColorValue->initializeUsingXML(element);
    diffuseColor->initializeUsingXML(element);
    transparency->initializeUsingXML(element);
    return element;
  }

  DOMElement* OMBVDynamicColoredObjectWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVBodyWidget::writeXMLFile(parent);
    minimalColorValue->writeXMLFile(e);
    maximalColorValue->writeXMLFile(e);
    diffuseColor->writeXMLFile(e);
    transparency->writeXMLFile(e);
    return e;
  }

  OMBVRigidBodyWidget::OMBVRigidBodyWidget(const QString &name, const FQN &xmlName) : OMBVDynamicColoredObjectWidget(name,xmlName) {

    transparency->setActive(true);

    trans = new ExtWidget("Initial translation",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"initialTranslation");
    layout->addWidget(trans);

    rot = new ExtWidget("Initial rotation",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,angleUnits())),QBoxLayout::RightToLeft),true,false,OPENMBV%"initialRotation");
    layout->addWidget(rot);
//
    scale = new ExtWidget("Scale factor",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,noUnitUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"scaleFactor");
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

    length = new ExtWidget("Length",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"length");
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

    length = new ExtWidget("Length",new ChoiceWidget(new VecWidgetFactory(strToVec(QString("[1;1;1]")),vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"length");
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

    radius = new ExtWidget("Radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"radius");
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
    trans->getFirstWidget<VecWidget>()->setVec(r);

    base = new ExtWidget("Base radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"baseRadius");
    layout->addWidget(base);

    top = new ExtWidget("Top radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"topRadius");
    layout->addWidget(top);

    height = new ExtWidget("Height",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"height");
    layout->addWidget(height);

    innerBase = new ExtWidget("Inner base radius",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"innerBaseRadius");
    layout->addWidget(innerBase);

    innerTop = new ExtWidget("Inner top radius",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"innerTopRadius");
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
    windingRule = new ExtWidget("Winding rule",new TextChoiceWidget(list,1,true),false,false,OPENMBV%"windingRule");
    layout->addWidget(windingRule);

    height = new ExtWidget("Height",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"height");
    layout->addWidget(height);

    contour = new ExtWidget("Contour",new ChoiceWidget(new MatRowsVarWidgetFactory(3,3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4),true),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"contour");
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

    ivData = new ExtWidget("Open Inventor scene graph",new ChoiceWidget(new IvDataWidgetFactory,QBoxLayout::TopToBottom,3),false,true);
    layout->addWidget(ivData);
    connect(ivData,&ExtWidget::widgetChanged,this,&IvBodyWidget::updateWidget);

    creaseEdges = new ExtWidget("Crease edges",new ChoiceWidget(new ScalarWidgetFactory("-1",vector<QStringList>(2,angleUnits())),QBoxLayout::RightToLeft),true,false,OPENMBV%"creaseEdges");
    layout->addWidget(creaseEdges);

    boundaryEdges = new ExtWidget("Boundary edges",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft),true,false,OPENMBV%"boundaryEdges");
    layout->addWidget(boundaryEdges);

    removeNodesByName = new ExtWidget("Remove nodes by name",new TextListWidget("Name", OPENMBV%"removeNodesByName"),true,false,"");
    layout->addWidget(removeNodesByName);

    removeNodesByType = new ExtWidget("Remove nodes by type",new TextListWidget("Type", OPENMBV%"removeNodesByType"),true,false,"");
    layout->addWidget(removeNodesByType);
  }

  DOMElement* IvBodyWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    ivData->initializeUsingXML(element);
    creaseEdges->initializeUsingXML(element);
    boundaryEdges->initializeUsingXML(element);
    removeNodesByName->initializeUsingXML(element);
    removeNodesByType->initializeUsingXML(element);
    return element;
  }

  DOMElement* IvBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    ivData->writeXMLFile(e);
    creaseEdges->writeXMLFile(e);
    boundaryEdges->writeXMLFile(e);
    removeNodesByName->writeXMLFile(e);
    removeNodesByType->writeXMLFile(e);
    return e;
  }

  IvScreenAnnotationWidget::IvScreenAnnotationWidget(const QString &name, const FQN &xmlName) : OMBVBodyWidget(name,xmlName) {

    scale1To1At = new ExtWidget("Scale 1 To 1 at",new ChoiceWidget(new VecWidgetFactory(getScalars<QString>(2,"1"),vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"scale1To1At");
    layout->addWidget(scale1To1At);

    ivData = new ExtWidget("Open Inventor scene graph",new ChoiceWidget(new IvDataWidgetFactory,QBoxLayout::TopToBottom,3),false,true);
    layout->addWidget(ivData);
    connect(ivData,&ExtWidget::widgetChanged,this,&IvScreenAnnotationWidget::updateWidget);

    columnLabel = new ExtWidget("Column label",new TextListWidget("Type", OPENMBV%"columnLabel",true),true,false,"");
    layout->addWidget(columnLabel);
  }

  DOMElement* IvScreenAnnotationWidget::initializeUsingXML(DOMElement *element) {
    OMBVBodyWidget::initializeUsingXML(element);
    scale1To1At->initializeUsingXML(element);
    ivData->initializeUsingXML(element);
    columnLabel->initializeUsingXML(element);
    return element;
  }

  DOMElement* IvScreenAnnotationWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVBodyWidget::writeXMLFile(parent);
    scale1To1At->writeXMLFile(e);
    ivData->writeXMLFile(e);
    columnLabel->writeXMLFile(e);
    return e;
  }

  CompoundRigidBodyWidget::CompoundRigidBodyWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {
    bodies = new ExtWidget("Bodies",new ListWidget(new ChoiceWidgetFactory(new OMBVRigidBodyWidgetFactory,1),"Body",1,1,false,1));
    layout->addWidget(bodies);
  }

  DOMElement* CompoundRigidBodyWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);

    // search last element of OMBVRigidBodyWidget
    // TODO/MISSING: initializeUsingXML(...) should return the first unparsed DOMElement NOT just the input DOMElement "element".
    // Then we can avoid such ugly hacks to detect the element here which needs to know all base class definitions.
    DOMElement *e;
           e=E(element)->getFirstElementChildNamed(OPENMBV%"scaleFactor");
    if(!e) e=E(element)->getFirstElementChildNamed(OPENMBV%"initialRotation");
    if(!e) e=E(element)->getFirstElementChildNamed(OPENMBV%"initialTranslation");
    if(!e) e=E(element)->getFirstElementChildNamed(OPENMBV%"transparency");
    if(!e) e=E(element)->getFirstElementChildNamed(OPENMBV%"diffuseColor");
    if(!e) e=E(element)->getFirstElementChildNamed(OPENMBV%"maximalColorValue");
    if(!e) e=E(element)->getFirstElementChildNamed(OPENMBV%"minimalColorValue");
    if(e)
      e=e->getNextElementSibling();
    else
      e=element->getFirstElementChild();

    bodies->initializeUsingXML(e);
    return element;
  }

  DOMElement* CompoundRigidBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    bodies->writeXMLFile(e);
    return e;
  }

  OMBVFlexibleBodyWidget::OMBVFlexibleBodyWidget(const QString &name, const FQN &xmlName) : OMBVDynamicColoredObjectWidget(name,xmlName) {

    numvp = new ExtWidget("Number of vertex positions",new ChoiceWidget(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"numberOfVertexPositions");
    layout->addWidget(numvp);

  }

  DOMElement* OMBVFlexibleBodyWidget::initializeUsingXML(DOMElement *element) {
    OMBVDynamicColoredObjectWidget::initializeUsingXML(element);
    numvp->initializeUsingXML(element);
    return element;
  }

  DOMElement* OMBVFlexibleBodyWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVDynamicColoredObjectWidget::writeXMLFile(parent);
    numvp->writeXMLFile(e);
    return e;
  }

  DynamicIndexedLineSetWidget::DynamicIndexedLineSetWidget(const QString &name, const FQN &xmlName) : OMBVFlexibleBodyWidget(name,xmlName) {

    indices = new ExtWidget("Indices",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"indices");
    layout->addWidget(indices);
  }

  DOMElement* DynamicIndexedLineSetWidget::initializeUsingXML(DOMElement *element) {
    OMBVFlexibleBodyWidget::initializeUsingXML(element);
    indices->initializeUsingXML(element);
    return element;
  }

  DOMElement* DynamicIndexedLineSetWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVFlexibleBodyWidget::writeXMLFile(parent);
    indices->writeXMLFile(e);
    return e;
  }

  DynamicIndexedFaceSetWidget::DynamicIndexedFaceSetWidget(const QString &name, const FQN &xmlName) : OMBVFlexibleBodyWidget(name,xmlName) {

    indices = new ExtWidget("Indices",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"indices");
    layout->addWidget(indices);
  }

  DOMElement* DynamicIndexedFaceSetWidget::initializeUsingXML(DOMElement *element) {
    OMBVFlexibleBodyWidget::initializeUsingXML(element);
    indices->initializeUsingXML(element);
    return element;
  }

  DOMElement* DynamicIndexedFaceSetWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVFlexibleBodyWidget::writeXMLFile(parent);
    indices->writeXMLFile(e);
    return e;
  }

  CylindricalGearWidget::CylindricalGearWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {

    numberOfTeeth = new ExtWidget("Number of teeth",new ChoiceWidget(new ScalarWidgetFactory("15",vector<QStringList>(2,QStringList())),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"numberOfTeeth");
    layout->addWidget(numberOfTeeth);

    width = new ExtWidget("Width",new ChoiceWidget(new ScalarWidgetFactory("5e-2",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"width");
    layout->addWidget(width);

    helixAngle = new ExtWidget("Helix angle",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"helixAngle");
    layout->addWidget(helixAngle);

    module = new ExtWidget("Module",new ChoiceWidget(new ScalarWidgetFactory("16e-3",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"module");
    layout->addWidget(module);

    pressureAngle = new ExtWidget("Pressure angle",new ChoiceWidget(new ScalarWidgetFactory("20",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"pressureAngle");
    layout->addWidget(pressureAngle);

    backlash = new ExtWidget("Backlash",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,OPENMBV%"backlash");
    layout->addWidget(backlash);

    externalToothed = new ExtWidget("Solid",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft),true,false,OPENMBV%"externalToothed");
    layout->addWidget(externalToothed);
  }

  DOMElement* CylindricalGearWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    numberOfTeeth->initializeUsingXML(element);
    width->initializeUsingXML(element);
    helixAngle->initializeUsingXML(element);
    module->initializeUsingXML(element);
    pressureAngle->initializeUsingXML(element);
    backlash->initializeUsingXML(element);
    externalToothed->initializeUsingXML(element);
    return element;
  }

  DOMElement* CylindricalGearWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    numberOfTeeth->writeXMLFile(e);
    width->writeXMLFile(e);
    helixAngle->writeXMLFile(e);
    module->writeXMLFile(e);
    pressureAngle->writeXMLFile(e);
    backlash->writeXMLFile(e);
    externalToothed->writeXMLFile(e);
    return e;
  }

  CylinderWidget::CylinderWidget(const QString &name, const FQN &xmlName) : OMBVRigidBodyWidget(name,xmlName) {

    radius = new ExtWidget("Radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"radius");
    layout->addWidget(radius);

    height = new ExtWidget("Height",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,OPENMBV%"height");
    layout->addWidget(height);
  }

  DOMElement* CylinderWidget::initializeUsingXML(DOMElement *element) {
    OMBVRigidBodyWidget::initializeUsingXML(element);
    radius->initializeUsingXML(element);
    height->initializeUsingXML(element);
    return element;
  }

  DOMElement* CylinderWidget::writeXMLFile(DOMNode *parent, xercesc::DOMNode *ref) {
    DOMElement *e=OMBVRigidBodyWidget::writeXMLFile(parent);
    radius->writeXMLFile(e);
    height->writeXMLFile(e);
    return e;
  }

}
