/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2024 Martin Förg

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
#include "contour_property_dialog.h"
#include "variable_widgets.h"
#include "ombv_widgets.h"
#include "function_widget_factory.h"
#include "element.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ContourPropertyDialog::ContourPropertyDialog(Element *contour) : ElementPropertyDialog(contour) {
    thickness = new ExtWidget("Thickness",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"thickness");
    addToTab("General", thickness);
  }

  DOMElement* ContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    thickness->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    thickness->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  RigidContourPropertyDialog::RigidContourPropertyDialog(Element *contour) : ContourPropertyDialog(contour) {
    refFrame = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(contour,nullptr),true,false,MBSIM%"frameOfReference");
    addToTab("General", refFrame);
  }

  DOMElement* RigidContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    refFrame->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    refFrame->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  PointPropertyDialog::PointPropertyDialog(Element *point) : RigidContourPropertyDialog(point) {
    addTab("Visualization",1);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* PointPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* PointPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  LinePropertyDialog::LinePropertyDialog(Element *line) : RigidContourPropertyDialog(line) {
    addTab("Visualization",1);

    visu = new ExtWidget("Enable openMBV",new LineMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* LinePropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LinePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  PlanePropertyDialog::PlanePropertyDialog(Element *plane) : RigidContourPropertyDialog(plane) {
    addTab("Visualization",1);

    visu = new ExtWidget("Enable openMBV",new PlaneMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* PlanePropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* PlanePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  SpherePropertyDialog::SpherePropertyDialog(Element *sphere) : RigidContourPropertyDialog(sphere) {
    addTab("Visualization",1);

    radius = new ExtWidget("Radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"radius");
    addToTab("General", radius);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* SpherePropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    radius->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SpherePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    radius->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  CirclePropertyDialog::CirclePropertyDialog(Element *circle) : RigidContourPropertyDialog(circle) {
    addTab("Visualization",1);

    radius = new ExtWidget("Radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"radius");
    addToTab("General", radius);
    solid = new ExtWidget("Solid",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"solid");
    addToTab("General", solid);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* CirclePropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    radius->initializeUsingXML(item->getXMLElement());
    solid->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* CirclePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    radius->writeXMLFile(item->getXMLElement(),nullptr);
    solid->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  CylinderPropertyDialog::CylinderPropertyDialog(Element *circle) : RigidContourPropertyDialog(circle) {
    addTab("Visualization",1);

    radius = new ExtWidget("Radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"radius");
    addToTab("General", radius);
    height = new ExtWidget("Height",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"height");
    addToTab("General", height);
    solid = new ExtWidget("Solid",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"solid");
    addToTab("General", solid);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* CylinderPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    radius->initializeUsingXML(item->getXMLElement());
    height->initializeUsingXML(item->getXMLElement());
    solid->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* CylinderPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    radius->writeXMLFile(item->getXMLElement(),nullptr);
    height->writeXMLFile(item->getXMLElement(),nullptr);
    solid->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  CuboidPropertyDialog::CuboidPropertyDialog(Element *circle) : RigidContourPropertyDialog(circle) {
    addTab("Visualization",1);

    length = new ExtWidget("Length",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"length");
    addToTab("General", length);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* CuboidPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    length->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* CuboidPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    length->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  LineSegmentPropertyDialog::LineSegmentPropertyDialog(Element *line) : RigidContourPropertyDialog(line) {
    addTab("Visualization",1);

    length = new ExtWidget("Length",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"length");
    addToTab("General", length);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* LineSegmentPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    length->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LineSegmentPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    length->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  PlanarContourPropertyDialog::PlanarContourPropertyDialog(Element *contour) : RigidContourPropertyDialog(contour) {
    addTab("Visualization",1);

    nodes = new ExtWidget("Nodes",new ChoiceWidget(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),false,false,MBSIM%"nodes");
    addToTab("General", nodes);

    contourFunction = new ExtWidget("Contour function",new ChoiceWidget(new PlanarContourFunctionWidgetFactory(contour,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"contourFunction");
    addToTab("General", contourFunction);

    open = new ExtWidget("Open",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"open");
    addToTab("General", open);

    visu = new ExtWidget("Enable openMBV",new PlanarContourMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* PlanarContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    nodes->initializeUsingXML(item->getXMLElement());
    contourFunction->initializeUsingXML(item->getXMLElement());
    open->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* PlanarContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    nodes->writeXMLFile(item->getXMLElement(),nullptr);
    contourFunction->writeXMLFile(item->getXMLElement(),nullptr);
    open->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  PlanarNurbsContourPropertyDialog::PlanarNurbsContourPropertyDialog(Element *contour) : RigidContourPropertyDialog(contour) {
    addTab("Visualization",1);

    vector<QString> list;
    list.emplace_back("\"equallySpaced\"");
    list.emplace_back("\"chordLength\"");
    list.emplace_back("\"none\"");
    interpolation = new ExtWidget("Interpolation",new TextChoiceWidget(list,2,true),true,false,MBSIM%"interpolation");
    addToTab("General", interpolation);

    controlPoints = new ExtWidget("Control points",new ChoiceWidget(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"controlPoints");
    addToTab("General", controlPoints);

    numberOfControlPoints = new ExtWidget("Number of control points",new ChoiceWidget(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfControlPoints");
    addToTab("General", numberOfControlPoints);

    knotVector = new ExtWidget("Knot vector",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"knotVector");
    addToTab("General", knotVector);

    degree = new ExtWidget("Degree",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"degree");
    addToTab("General", degree);

    open = new ExtWidget("Open",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"open");
    addToTab("General", open);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* PlanarNurbsContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    interpolation->initializeUsingXML(item->getXMLElement());
    controlPoints->initializeUsingXML(item->getXMLElement());
    numberOfControlPoints->initializeUsingXML(item->getXMLElement());
    knotVector->initializeUsingXML(item->getXMLElement());
    degree->initializeUsingXML(item->getXMLElement());
    open->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* PlanarNurbsContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    interpolation->writeXMLFile(item->getXMLElement(),nullptr);
    controlPoints->writeXMLFile(item->getXMLElement(),nullptr);
    numberOfControlPoints->writeXMLFile(item->getXMLElement(),nullptr);
    knotVector->writeXMLFile(item->getXMLElement(),nullptr);
    degree->writeXMLFile(item->getXMLElement(),nullptr);
    open->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  SpatialContourPropertyDialog::SpatialContourPropertyDialog(Element *contour) : RigidContourPropertyDialog(contour) {
    addTab("Visualization",1);

    etaNodes = new ExtWidget("Eta nodes",new ChoiceWidget(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),false,false,MBSIM%"etaNodes");
    addToTab("General", etaNodes);

    xiNodes = new ExtWidget("Xi nodes",new ChoiceWidget(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),false,false,MBSIM%"xiNodes");
    addToTab("General", xiNodes);

    contourFunction = new ExtWidget("Contour function",new ChoiceWidget(new SpatialContourFunctionWidgetFactory(contour,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"contourFunction");
    addToTab("General", contourFunction);

    openEta = new ExtWidget("Open eta",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openEta");
    addToTab("General", openEta);

    openXi = new ExtWidget("Open xi",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openXi");
    addToTab("General", openXi);

    visu = new ExtWidget("Enable openMBV",new SpatialContourMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* SpatialContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    etaNodes->initializeUsingXML(item->getXMLElement());
    xiNodes->initializeUsingXML(item->getXMLElement());
    contourFunction->initializeUsingXML(item->getXMLElement());
    openEta->initializeUsingXML(item->getXMLElement());
    openXi->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SpatialContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    etaNodes->writeXMLFile(item->getXMLElement(),nullptr);
    xiNodes->writeXMLFile(item->getXMLElement(),nullptr);
    contourFunction->writeXMLFile(item->getXMLElement(),nullptr);
    openEta->writeXMLFile(item->getXMLElement(),nullptr);
    openXi->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  SpatialNurbsContourPropertyDialog::SpatialNurbsContourPropertyDialog(Element *contour) : RigidContourPropertyDialog(contour) {
    addTab("Visualization",1);

    vector<QString> list;
    list.emplace_back("\"equallySpaced\"");
    list.emplace_back("\"chordLength\"");
    list.emplace_back("\"none\"");
    interpolation = new ExtWidget("Interpolation",new TextChoiceWidget(list,2,true),true,false,MBSIM%"interpolation");
    addToTab("General", interpolation);

    controlPoints = new ExtWidget("Control points",new ChoiceWidget(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"controlPoints");
    addToTab("General", controlPoints);

    numberOfEtaControlPoints = new ExtWidget("Number of eta control points",new ChoiceWidget(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfEtaControlPoints");
    addToTab("General", numberOfEtaControlPoints);

    numberOfXiControlPoints = new ExtWidget("Number of xi control points",new ChoiceWidget(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfXiControlPoints");
    addToTab("General", numberOfXiControlPoints);

    etaKnotVector = new ExtWidget("Eta knot vector",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"etaKnotVector");
    addToTab("General", etaKnotVector);

    xiKnotVector = new ExtWidget("Xi knot vector",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"xiKnotVector");
    addToTab("General", xiKnotVector);

    etaDegree = new ExtWidget("Eta degree",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"etaDegree");
    addToTab("General", etaDegree);

    xiDegree = new ExtWidget("Xi degree",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"xiDegree");
    addToTab("General", xiDegree);

    openEta = new ExtWidget("Open eta",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openEta");
    addToTab("General", openEta);

    openXi = new ExtWidget("Open xi",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openXi");
    addToTab("General", openXi);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* SpatialNurbsContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    interpolation->initializeUsingXML(item->getXMLElement());
    controlPoints->initializeUsingXML(item->getXMLElement());
    numberOfEtaControlPoints->initializeUsingXML(item->getXMLElement());
    numberOfXiControlPoints->initializeUsingXML(item->getXMLElement());
    etaKnotVector->initializeUsingXML(item->getXMLElement());
    xiKnotVector->initializeUsingXML(item->getXMLElement());
    etaDegree->initializeUsingXML(item->getXMLElement());
    xiDegree->initializeUsingXML(item->getXMLElement());
    openEta->initializeUsingXML(item->getXMLElement());
    openXi->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SpatialNurbsContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    interpolation->writeXMLFile(item->getXMLElement(),nullptr);
    controlPoints->writeXMLFile(item->getXMLElement(),nullptr);
    numberOfEtaControlPoints->writeXMLFile(item->getXMLElement(),nullptr);
    numberOfXiControlPoints->writeXMLFile(item->getXMLElement(),nullptr);
    etaKnotVector->writeXMLFile(item->getXMLElement(),nullptr);
    xiKnotVector->writeXMLFile(item->getXMLElement(),nullptr);
    etaDegree->writeXMLFile(item->getXMLElement(),nullptr);
    xiDegree->writeXMLFile(item->getXMLElement(),nullptr);
    openEta->writeXMLFile(item->getXMLElement(),nullptr);
    openXi->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  DiskPropertyDialog::DiskPropertyDialog(Element *disk) : RigidContourPropertyDialog(disk) {
    addTab("Visualization",1);

    outerRadius = new ExtWidget("Outer radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"outerRadius");
    addToTab("General", outerRadius);
    width = new ExtWidget("Width",new ChoiceWidget(new ScalarWidgetFactory("0.1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"width");
    addToTab("General", width);
    innerRadius = new ExtWidget("Inner radius",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"innerRadius");
    addToTab("General", innerRadius);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* DiskPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    outerRadius->initializeUsingXML(item->getXMLElement());
    width->initializeUsingXML(item->getXMLElement());
    innerRadius->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DiskPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    outerRadius->writeXMLFile(item->getXMLElement(),nullptr);
    width->writeXMLFile(item->getXMLElement(),nullptr);
    innerRadius->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  CylindricalGearPropertyDialog::CylindricalGearPropertyDialog(Element *gear) : RigidContourPropertyDialog(gear) {
    addTab("Visualization",1);

    numberOfTeeth = new ExtWidget("Number of teeth",new ChoiceWidget(new ScalarWidgetFactory("15",vector<QStringList>(2,QStringList())),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfTeeth");
    addToTab("General", numberOfTeeth);
    width = new ExtWidget("Width",new ChoiceWidget(new ScalarWidgetFactory("50",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"width");
    addToTab("General", width);
    helixAngle = new ExtWidget("Helix angle",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"helixAngle");
    addToTab("General", helixAngle);
    module = new ExtWidget("Module",new ChoiceWidget(new ScalarWidgetFactory("16",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"module");
    addToTab("General", module);
    pressureAngle = new ExtWidget("Pressure angle",new ChoiceWidget(new ScalarWidgetFactory("20",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pressureAngle");
    addToTab("General", pressureAngle);
    backlash = new ExtWidget("Backlash",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"backlash");
    addToTab("General", backlash);
    externalToothed = new ExtWidget("External toothed",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"externalToothed");
    addToTab("General", externalToothed);
    outsideRadius = new ExtWidget("outsideRadius",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"outsideRadius");
    addToTab("General", outsideRadius);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* CylindricalGearPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    numberOfTeeth->initializeUsingXML(item->getXMLElement());
    width->initializeUsingXML(item->getXMLElement());
    helixAngle->initializeUsingXML(item->getXMLElement());
    module->initializeUsingXML(item->getXMLElement());
    pressureAngle->initializeUsingXML(item->getXMLElement());
    backlash->initializeUsingXML(item->getXMLElement());
    externalToothed->initializeUsingXML(item->getXMLElement());
    outsideRadius->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* CylindricalGearPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    numberOfTeeth->writeXMLFile(item->getXMLElement(),nullptr);
    width->writeXMLFile(item->getXMLElement(),nullptr);
    helixAngle->writeXMLFile(item->getXMLElement(),nullptr);
    module->writeXMLFile(item->getXMLElement(),nullptr);
    pressureAngle->writeXMLFile(item->getXMLElement(),nullptr);
    backlash->writeXMLFile(item->getXMLElement(),nullptr);
    externalToothed->writeXMLFile(item->getXMLElement(),nullptr);
    outsideRadius->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  RackPropertyDialog::RackPropertyDialog(Element *rack) : RigidContourPropertyDialog(rack) {
    addTab("Visualization",1);

    numberOfTeeth = new ExtWidget("Number of teeth",new ChoiceWidget(new ScalarWidgetFactory("15",vector<QStringList>(2,QStringList())),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfTeeth");
    addToTab("General", numberOfTeeth);
    height = new ExtWidget("Height",new ChoiceWidget(new ScalarWidgetFactory("50",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"height");
    addToTab("General", height);
    width = new ExtWidget("Width",new ChoiceWidget(new ScalarWidgetFactory("50",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"width");
    addToTab("General", width);
    helixAngle = new ExtWidget("Helix angle",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"helixAngle");
    addToTab("General", helixAngle);
    module = new ExtWidget("Module",new ChoiceWidget(new ScalarWidgetFactory("16",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"module");
    addToTab("General", module);
    pressureAngle = new ExtWidget("Pressure angle",new ChoiceWidget(new ScalarWidgetFactory("20",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pressureAngle");
    addToTab("General", pressureAngle);
    backlash = new ExtWidget("Backlash",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"backlash");
    addToTab("General", backlash);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* RackPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    numberOfTeeth->initializeUsingXML(item->getXMLElement());
    height->initializeUsingXML(item->getXMLElement());
    width->initializeUsingXML(item->getXMLElement());
    helixAngle->initializeUsingXML(item->getXMLElement());
    module->initializeUsingXML(item->getXMLElement());
    pressureAngle->initializeUsingXML(item->getXMLElement());
    backlash->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RackPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    numberOfTeeth->writeXMLFile(item->getXMLElement(),nullptr);
    height->writeXMLFile(item->getXMLElement(),nullptr);
    width->writeXMLFile(item->getXMLElement(),nullptr);
    helixAngle->writeXMLFile(item->getXMLElement(),nullptr);
    module->writeXMLFile(item->getXMLElement(),nullptr);
    pressureAngle->writeXMLFile(item->getXMLElement(),nullptr);
    backlash->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  BevelGearPropertyDialog::BevelGearPropertyDialog(Element *gear) : RigidContourPropertyDialog(gear) {
    addTab("Visualization",1);

    numberOfTeeth = new ExtWidget("Number of teeth",new ChoiceWidget(new ScalarWidgetFactory("15",vector<QStringList>(2,QStringList())),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfTeeth");
    addToTab("General", numberOfTeeth);
    width = new ExtWidget("Width",new ChoiceWidget(new ScalarWidgetFactory("50",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"width");
    addToTab("General", width);
    helixAngle = new ExtWidget("Helix angle",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"helixAngle");
    addToTab("General", helixAngle);
    pitchAngle = new ExtWidget("Pitch angle",new ChoiceWidget(new ScalarWidgetFactory("45",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pitchAngle");
    addToTab("General", pitchAngle);
    module = new ExtWidget("Module",new ChoiceWidget(new ScalarWidgetFactory("16",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"module");
    addToTab("General", module);
    pressureAngle = new ExtWidget("Pressure angle",new ChoiceWidget(new ScalarWidgetFactory("20",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pressureAngle");
    addToTab("General", pressureAngle);
    backlash = new ExtWidget("Backlash",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"backlash");
    addToTab("General", backlash);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* BevelGearPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    numberOfTeeth->initializeUsingXML(item->getXMLElement());
    width->initializeUsingXML(item->getXMLElement());
    helixAngle->initializeUsingXML(item->getXMLElement());
    pitchAngle->initializeUsingXML(item->getXMLElement());
    module->initializeUsingXML(item->getXMLElement());
    pressureAngle->initializeUsingXML(item->getXMLElement());
    backlash->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* BevelGearPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    numberOfTeeth->writeXMLFile(item->getXMLElement(),nullptr);
    width->writeXMLFile(item->getXMLElement(),nullptr);
    helixAngle->writeXMLFile(item->getXMLElement(),nullptr);
    pitchAngle->writeXMLFile(item->getXMLElement(),nullptr);
    module->writeXMLFile(item->getXMLElement(),nullptr);
    pressureAngle->writeXMLFile(item->getXMLElement(),nullptr);
    backlash->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  PlanarGearPropertyDialog::PlanarGearPropertyDialog(Element *gear) : RigidContourPropertyDialog(gear) {
    addTab("Visualization",1);

    numberOfTeeth = new ExtWidget("Number of teeth",new ChoiceWidget(new ScalarWidgetFactory("15",vector<QStringList>(2,QStringList())),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfTeeth");
    addToTab("General", numberOfTeeth);
    height = new ExtWidget("Height",new ChoiceWidget(new ScalarWidgetFactory("50",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"height");
    addToTab("General", height);
    width = new ExtWidget("Width",new ChoiceWidget(new ScalarWidgetFactory("50",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"width");
    addToTab("General", width);
    helixAngle = new ExtWidget("Helix angle",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"helixAngle");
    addToTab("General", helixAngle);
    module = new ExtWidget("Module",new ChoiceWidget(new ScalarWidgetFactory("16",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"module");
    addToTab("General", module);
    pressureAngle = new ExtWidget("Pressure angle",new ChoiceWidget(new ScalarWidgetFactory("20",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pressureAngle");
    addToTab("General", pressureAngle);
    backlash = new ExtWidget("Backlash",new ChoiceWidget(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"backlash");
    addToTab("General", backlash);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* PlanarGearPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    numberOfTeeth->initializeUsingXML(item->getXMLElement());
    height->initializeUsingXML(item->getXMLElement());
    width->initializeUsingXML(item->getXMLElement());
    helixAngle->initializeUsingXML(item->getXMLElement());
    module->initializeUsingXML(item->getXMLElement());
    pressureAngle->initializeUsingXML(item->getXMLElement());
    backlash->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* PlanarGearPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    numberOfTeeth->writeXMLFile(item->getXMLElement(),nullptr);
    height->writeXMLFile(item->getXMLElement(),nullptr);
    width->writeXMLFile(item->getXMLElement(),nullptr);
    helixAngle->writeXMLFile(item->getXMLElement(),nullptr);
    module->writeXMLFile(item->getXMLElement(),nullptr);
    pressureAngle->writeXMLFile(item->getXMLElement(),nullptr);
    backlash->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  TyrePropertyDialog::TyrePropertyDialog(Element *tyre) : RigidContourPropertyDialog(tyre) {
    addTab("Visualization",1);

    r = new ExtWidget("Radius",new ChoiceWidget(new ScalarWidgetFactory("0.3",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"radius");
    addToTab("General", r);

    rRim = new ExtWidget("Rim radius",new ChoiceWidget(new ScalarWidgetFactory("0.2",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"rimRadius");
    addToTab("General", rRim);

    w = new ExtWidget("Width",new ChoiceWidget(new ScalarWidgetFactory("0.2",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"width");
    addToTab("General", w);

    vector<QString> list;
    list.emplace_back("\"flat\"");
    list.emplace_back("\"circular\"");
    list.emplace_back("\"elliptical\"");
    shape = new ExtWidget("Shape of cross section contour",new TextChoiceWidget(list,0,false),true,false,MBSIM%"shapeOfCrossSectionContour");
    addToTab("General", shape);

    cp = new ExtWidget("Contour parameters",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"contourParameters");
    addToTab("General", cp);

    visu = new ExtWidget("Enable openMBV",new SpatialContourMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);

    connect(shape, &ExtWidget::widgetChanged, this, &TyrePropertyDialog::updateWidget);
  }

  DOMElement* TyrePropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    DOMElement *ele = E(item->getXMLElement())->getFirstElementChildNamed(MBSIM%"unloadedRadius");
    if(ele) ele->getOwnerDocument()->renameNode(ele,X()%MBSIM.getNamespaceURI(),X()%"radius");
    r->initializeUsingXML(item->getXMLElement());
    rRim->initializeUsingXML(item->getXMLElement());
    w->initializeUsingXML(item->getXMLElement());
    shape->blockSignals(true);
    shape->initializeUsingXML(item->getXMLElement());
    shape->blockSignals(false);
    cp->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* TyrePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    r->writeXMLFile(item->getXMLElement(),nullptr);
    rRim->writeXMLFile(item->getXMLElement(),nullptr);
    w->writeXMLFile(item->getXMLElement(),nullptr);
    shape->writeXMLFile(item->getXMLElement(),nullptr);
    cp->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  void TyrePropertyDialog::updateWidget() {
    if(not shape->isActive()) {
      cp->setActive(false);
      return;
    }
    auto shapeStr = shape->getWidget<TextChoiceWidget>()->getText();
    if(shapeStr=="\"flat\"") {
      cp->resize_(0,1);
      cp->setActive(false);
    }
    else {
      cp->setActive(true);
      if(shapeStr=="\"circular\"")
	cp->resize_(1,1);
      else if(shapeStr=="\"elliptical\"")
	cp->resize_(2,1);
    }
  }

  FlexiblePlanarNurbsContourPropertyDialog::FlexiblePlanarNurbsContourPropertyDialog(Element *contour) : ContourPropertyDialog(contour) {
    addTab("Visualization",1);

    interpolation = new ExtWidget("Interpolation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"interpolation");
    addToTab("General", interpolation);

    indices = new ExtWidget("Node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"nodeNumbers");
    addToTab("General", indices);

    knotVector = new ExtWidget("Knot vector",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"knotVector");
    addToTab("General", knotVector);

    degree = new ExtWidget("Degree",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"degree");
    addToTab("General", degree);

    open = new ExtWidget("Open",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"open");
    addToTab("General", open);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIMFLEX%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* FlexiblePlanarNurbsContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    interpolation->initializeUsingXML(item->getXMLElement());
    indices->initializeUsingXML(item->getXMLElement());
    knotVector->initializeUsingXML(item->getXMLElement());
    degree->initializeUsingXML(item->getXMLElement());
    open->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FlexiblePlanarNurbsContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    interpolation->writeXMLFile(item->getXMLElement(),nullptr);
    indices->writeXMLFile(item->getXMLElement(),nullptr);
    knotVector->writeXMLFile(item->getXMLElement(),nullptr);
    degree->writeXMLFile(item->getXMLElement(),nullptr);
    open->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  FlexibleSpatialNurbsContourPropertyDialog::FlexibleSpatialNurbsContourPropertyDialog(Element *contour) : ContourPropertyDialog(contour) {
    addTab("Visualization",1);

    interpolation = new ExtWidget("Interpolation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"interpolation");
    addToTab("General", interpolation);

    indices = new ExtWidget("Node numbers",new ChoiceWidget(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"nodeNumbers");
    addToTab("General", indices);

    etaKnotVector = new ExtWidget("Eta knot vector",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"etaKnotVector");
    addToTab("General", etaKnotVector);

    xiKnotVector = new ExtWidget("Xi knot vector",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"xiKnotVector");
    addToTab("General", xiKnotVector);

    etaDegree = new ExtWidget("Eta degree",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"etaDegree");
    addToTab("General", etaDegree);

    xiDegree = new ExtWidget("Xi degree",new ChoiceWidget(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"xiDegree");
    addToTab("General", xiDegree);

    openEta = new ExtWidget("Open eta",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"openEta");
    addToTab("General", openEta);

    openXi = new ExtWidget("Open xi",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"openXi");
    addToTab("General", openXi);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIMFLEX%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* FlexibleSpatialNurbsContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    interpolation->initializeUsingXML(item->getXMLElement());
    indices->initializeUsingXML(item->getXMLElement());
    etaKnotVector->initializeUsingXML(item->getXMLElement());
    xiKnotVector->initializeUsingXML(item->getXMLElement());
    etaDegree->initializeUsingXML(item->getXMLElement());
    xiDegree->initializeUsingXML(item->getXMLElement());
    openEta->initializeUsingXML(item->getXMLElement());
    openXi->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FlexibleSpatialNurbsContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    interpolation->writeXMLFile(item->getXMLElement(),nullptr);
    indices->writeXMLFile(item->getXMLElement(),nullptr);
    etaKnotVector->writeXMLFile(item->getXMLElement(),nullptr);
    xiKnotVector->writeXMLFile(item->getXMLElement(),nullptr);
    etaDegree->writeXMLFile(item->getXMLElement(),nullptr);
    xiDegree->writeXMLFile(item->getXMLElement(),nullptr);
    openEta->writeXMLFile(item->getXMLElement(),nullptr);
    openXi->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  NodesContourPropertyDialog::NodesContourPropertyDialog(Element *contour) : ContourPropertyDialog(contour) {
    addTab("Visualization",1);

    nodeNumbers = new ExtWidget("Node numbers",new ChoiceWidget(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"nodeNumbers");
    addToTab("General", nodeNumbers);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIMFLEX%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* NodesContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    nodeNumbers->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* NodesContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    nodeNumbers->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  FclContourPropertyDialog::FclContourPropertyDialog(Element *contour) : RigidContourPropertyDialog(contour) {
    computeLocalAABB = new ExtWidget("Compute local AABB",new ChoiceWidget(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIMFCL%"computeLocalAABB");
    addToTab("General", computeLocalAABB);
  }

  DOMElement* FclContourPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    computeLocalAABB->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FclContourPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    computeLocalAABB->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  FclBoxPropertyDialog::FclBoxPropertyDialog(Element *contour) : FclContourPropertyDialog(contour) {
    addTab("Visualization",1);

    length = new ExtWidget("Length",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"length");
    addToTab("General", length);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIMFCL%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* FclBoxPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FclContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    length->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FclBoxPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FclContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    length->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  FclSpherePropertyDialog::FclSpherePropertyDialog(Element *contour) : FclContourPropertyDialog(contour) {
    addTab("Visualization",1);

    radius = new ExtWidget("Radius",new ChoiceWidget(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"radius");
    addToTab("General", radius);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIMFCL%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* FclSpherePropertyDialog::initializeUsingXML(DOMElement *parent) {
    FclContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    radius->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FclSpherePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FclContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    radius->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  FclPlanePropertyDialog::FclPlanePropertyDialog(Element *contour) : FclContourPropertyDialog(contour) {
    addTab("Visualization",1);

    vector<QString> n = getVec<QString>(3,"0");
    n[0] = "1";
    normal = new ExtWidget("Normal",new ChoiceWidget(new VecWidgetFactory(n,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"normal");
    addToTab("General", normal);

    offset = new ExtWidget("Offset",new ChoiceWidget(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"offset");
    addToTab("General", offset);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIMFCL%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* FclPlanePropertyDialog::initializeUsingXML(DOMElement *parent) {
    FclContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    normal->initializeUsingXML(item->getXMLElement());
    offset->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FclPlanePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FclContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    normal->writeXMLFile(item->getXMLElement(),nullptr);
    offset->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  FclMeshPropertyDialog::FclMeshPropertyDialog(Element *contour) : FclContourPropertyDialog(contour) {
    addTab("Visualization",1);

    vertices = new ExtWidget("Vertices",new ChoiceWidget(new MatRowsVarWidgetFactory(1,3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"vertices");
    addToTab("General", vertices);

    triangles = new ExtWidget("Triangles",new ChoiceWidget(new MatRowsVarWidgetFactory(1,3,vector<QStringList>(3,QStringList()),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"triangles");
    addToTab("General", triangles);

    vector<QString> list;
    list.emplace_back("\"AABB\"");
    list.emplace_back("\"KDOP16\"");
    list.emplace_back("\"KDOP18\"");
    list.emplace_back("\"KDOP24\"");
    list.emplace_back("\"kIOS\"");
    list.emplace_back("\"OBB\"");
    list.emplace_back("\"OBBRSS\"");
    list.emplace_back("\"RSS\"");
    collisionStructure = new ExtWidget("Collision structure",new TextChoiceWidget(list,0,true),true,false,MBSIMFCL%"collisionStructure");
    addToTab("General", collisionStructure);

    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIMFCL%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* FclMeshPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FclContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    vertices->initializeUsingXML(item->getXMLElement());
    triangles->initializeUsingXML(item->getXMLElement());
    collisionStructure->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FclMeshPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FclContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    vertices->writeXMLFile(item->getXMLElement(),nullptr);
    triangles->writeXMLFile(item->getXMLElement(),nullptr);
    collisionStructure->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

}
