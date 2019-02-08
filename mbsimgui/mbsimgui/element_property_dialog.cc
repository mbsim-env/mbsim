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
#include "element_property_dialog.h"
#include "basic_widgets.h"
#include "variable_widgets.h"
#include "special_widgets.h"
#include "kinematics_widgets.h"
#include "kinetics_widgets.h"
#include "function_widgets.h"
#include "kinematic_functions_widgets.h"
#include "ombv_widgets.h"
#include "extended_widgets.h"
#include "dynamic_system_solver.h"
#include "frame.h"
#include "contour.h"
#include "rigid_body.h"
#include "constraint.h"
#include "contact.h"
#include "signal_.h"
#include "function_widget_factory.h"
#include <QPushButton>
#include <QDialogButtonBox>
#include <QFileInfo>
#include <QMessageBox>
#include <utility>
#include <mbxmlutilshelper/getinstallpath.h>
#include "mainwindow.h"
#include "dialogs.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  extern MainWindow *mw;

  class ConnectRigidBodiesWidgetFactory : public WidgetFactory {
    public:
      ConnectRigidBodiesWidgetFactory(Element *element_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      int getSize() const override { return name.size(); }
    protected:
      Element *element;
      std::vector<QString> name;
      QWidget *parent;
  };

  ConnectRigidBodiesWidgetFactory::ConnectRigidBodiesWidgetFactory(Element *element_, QWidget *parent_) : element(element_), parent(parent_) {
    name.emplace_back("1 rigid body");
    name.emplace_back("2 rigid bodies");
  }

  QWidget* ConnectRigidBodiesWidgetFactory::createWidget(int i) {
    return new ConnectElementsWidget<RigidBody>(i+1,element,parent);
  }

  ElementPropertyDialog::ElementPropertyDialog(Element *element) : EmbedItemPropertyDialog(element) {
    addTab("General");
    name = new ExtWidget("Name",new TextWidget(item->getName()));
    name->setToolTip("Set the name of the element");
    addToTab("General", name);
    addTab("Plot");
    plotFeature = new ExtWidget("Plot features",new PlotFeatureWidget(getElement()->getPlotFeatureType()));
    addToTab("Plot", plotFeature);
  }

  Element* ElementPropertyDialog::getElement() const {
    return static_cast<Element*>(item);
  }

  DOMElement* ElementPropertyDialog::initializeUsingXML(DOMElement *parent) {
    static_cast<TextWidget*>(name->getWidget())->setText(item->getName());
    plotFeature->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ElementPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->removeXMLElements();
    E(item->getXMLElement())->setAttribute("name",static_cast<TextWidget*>(name->getWidget())->getText().toStdString());
    plotFeature->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  void ElementPropertyDialog::showXMLHelp() {
    // generate url for current element
    string url="file://"+(MBXMLUtils::getInstallPath()/"share"/"mbxmlutils"/"doc").string();
    string ns=getElement()->getNameSpace().getNamespaceURI();
    replace(ns.begin(), ns.end(), ':', '_');
    replace(ns.begin(), ns.end(), '.', '_');
    replace(ns.begin(), ns.end(), '/', '_');
    url+="/"+ns+"/index.html#"+getElement()->getType().toStdString();
    // open in XML help dialog
    mw->xmlHelp(QString::fromStdString(url));
  }

  void ElementPropertyDialog::setName(const QString &str) {
    static_cast<TextWidget*>(name->getWidget())->setText(str);
  }

  void ElementPropertyDialog::setReadOnly(bool readOnly) {
    static_cast<TextWidget*>(name->getWidget())->setReadOnly(readOnly);
  }

  UnknownElementPropertyDialog::UnknownElementPropertyDialog(Element *element) : ElementPropertyDialog(element) {
    name->setDisabled(true);
    plotFeature->setDisabled(true);
    editor = new ExtWidget("XML Editor",new XMLEditorWidget);
    addToTab("General", editor);
  }

  DOMElement* UnknownElementPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    editor->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* UnknownElementPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->setXMLElement(editor->writeXMLFile(item->getXMLElement(),ref));
    return nullptr;
  }

  FramePropertyDialog::FramePropertyDialog(Element *frame) : ElementPropertyDialog(frame) {
    addTab("Visualization",1);
    visu = new ExtWidget("Enable openMBV",new FrameMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    visu->setToolTip("Set the visualisation parameters for the frame");
    addToTab("Visualization", visu);
  }

  DOMElement* FramePropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FramePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    visu->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  InternalFramePropertyDialog::InternalFramePropertyDialog(Element *frame) : ElementPropertyDialog(frame) {
    if(frame->getParent()->getEmbeded()) {
      buttonBox->button(QDialogButtonBox::Apply)->setDisabled(true);
      buttonBox->button(QDialogButtonBox::Ok)->setDisabled(true);
    }
    addTab("Visualization",1);
    visu = new ExtWidget("Enable openMBV",new FrameMBSOMBVWidget,true,true,static_cast<InternalFrame*>(frame)->getXMLFrameName());
    visu->setToolTip("Set the visualisation parameters for the frame");
    addToTab("Visualization", visu);
    setReadOnly(true);
    setName(frame->getName());
  }

  DOMElement* InternalFramePropertyDialog::initializeUsingXML(DOMElement *parent) {
    visu->initializeUsingXML(getElement()->getParent()->getXMLElement());
    static_cast<PlotFeatureWidget*>(plotFeature->getWidget())->initializeUsingXML2(getElement()->getParent()->getXMLElement());
    return parent;
  }

  DOMElement* InternalFramePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->removeXMLElements();
    DOMElement *ele = getElement()->getParent()->getXMLFrame();
    visu->writeXMLFile(getElement()->getParent()->getXMLElement(),ele);
    static_cast<PlotFeatureWidget*>(plotFeature->getWidget())->writeXMLFile2(getElement()->getParent()->getXMLElement(),ele);
    return nullptr;
  }

  FixedRelativeFramePropertyDialog::FixedRelativeFramePropertyDialog(Element *frame) : FramePropertyDialog(frame) {
    addTab("Kinematics",1);

    refFrame = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(frame,static_cast<Frame*>(frame)),true,false,MBSIM%"frameOfReference");
    addToTab("Kinematics", refFrame);

    position = new ExtWidget("Relative position",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"relativePosition");
    addToTab("Kinematics", position);

    orientation = new ExtWidget("Relative orientation",new ChoiceWidget2(new RotMatWidgetFactory,QBoxLayout::RightToLeft,5),true,false,MBSIM%"relativeOrientation");
    addToTab("Kinematics", orientation);
  }

  DOMElement* FixedRelativeFramePropertyDialog::initializeUsingXML(DOMElement *parent) {
    FramePropertyDialog::initializeUsingXML(item->getXMLElement());
    refFrame->initializeUsingXML(item->getXMLElement());
    position->initializeUsingXML(item->getXMLElement());
    orientation->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FixedRelativeFramePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FramePropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    refFrame->writeXMLFile(item->getXMLElement(),nullptr);
    position->writeXMLFile(item->getXMLElement(),nullptr);
    orientation->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  NodeFramePropertyDialog::NodeFramePropertyDialog(Element *frame) : FramePropertyDialog(frame) {

    nodeNumber = new ExtWidget("Node number",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"nodeNumber");
    addToTab("General", nodeNumber);
  }

  DOMElement* NodeFramePropertyDialog::initializeUsingXML(DOMElement *parent) {
    FramePropertyDialog::initializeUsingXML(item->getXMLElement());
    nodeNumber->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* NodeFramePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FramePropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    nodeNumber->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  ContourPropertyDialog::ContourPropertyDialog(Element *contour) : ElementPropertyDialog(contour) {
    thickness = new ExtWidget("Thickness",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"thickness");
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

    radius = new ExtWidget("Radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"radius");
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

    radius = new ExtWidget("Radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"radius");
    addToTab("General", radius);
    solid = new ExtWidget("Solid",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"solid");
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

  CuboidPropertyDialog::CuboidPropertyDialog(Element *circle) : RigidContourPropertyDialog(circle) {
    addTab("Visualization",1);

    length = new ExtWidget("Length",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"length");
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

    length = new ExtWidget("Length",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"length");
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

    nodes = new ExtWidget("Nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"nodes");
    addToTab("General", nodes);

    contourFunction = new ExtWidget("Contour function",new ChoiceWidget2(new PlanarContourFunctionWidgetFactory(contour,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"contourFunction");
    addToTab("General", contourFunction);

    open = new ExtWidget("Open",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"open");
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
    interpolation = new ExtWidget("Interpolation",new TextChoiceWidget(list,1,true),true,false,MBSIM%"interpolation");
    addToTab("General", interpolation);

    controlPoints = new ExtWidget("Control points",new ChoiceWidget2(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"controlPoints");
    addToTab("General", controlPoints);

    numberOfControlPoints = new ExtWidget("Number of control points",new ChoiceWidget2(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfControlPoints");
    addToTab("General", numberOfControlPoints);

    knotVector = new ExtWidget("Knot vector",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"knotVector");
    addToTab("General", knotVector);

    degree = new ExtWidget("Degree",new ChoiceWidget2(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"degree");
    addToTab("General", degree);

    open = new ExtWidget("Open",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"open");
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

    etaNodes = new ExtWidget("Eta nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"etaNodes");
    addToTab("General", etaNodes);

    xiNodes = new ExtWidget("Xi nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIM%"xiNodes");
    addToTab("General", xiNodes);

    contourFunction = new ExtWidget("Contour function",new ChoiceWidget2(new SpatialContourFunctionWidgetFactory(contour),QBoxLayout::TopToBottom,0),false,false,MBSIM%"contourFunction");
    addToTab("General", contourFunction);

    openEta = new ExtWidget("Open eta",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openEta");
    addToTab("General", openEta);

    openXi = new ExtWidget("Open xi",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openXi");
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
    interpolation = new ExtWidget("Interpolation",new TextChoiceWidget(list,1,true),true,false,MBSIM%"interpolation");
    addToTab("General", interpolation);

    controlPoints = new ExtWidget("Control points",new ChoiceWidget2(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"controlPoints");
    addToTab("General", controlPoints);

    numberOfEtaControlPoints = new ExtWidget("Number of eta control points",new ChoiceWidget2(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfEtaControlPoints");
    addToTab("General", numberOfEtaControlPoints);

    numberOfXiControlPoints = new ExtWidget("Number of xi control points",new ChoiceWidget2(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfXiControlPoints");
    addToTab("General", numberOfXiControlPoints);

    etaKnotVector = new ExtWidget("Eta knot vector",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"etaKnotVector");
    addToTab("General", etaKnotVector);

    xiKnotVector = new ExtWidget("Xi knot vector",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"xiKnotVector");
    addToTab("General", xiKnotVector);

    etaDegree = new ExtWidget("Eta degree",new ChoiceWidget2(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"etaDegree");
    addToTab("General", etaDegree);

    xiDegree = new ExtWidget("Xi degree",new ChoiceWidget2(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"xiDegree");
    addToTab("General", xiDegree);

    openEta = new ExtWidget("Open eta",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openEta");
    addToTab("General", openEta);

    openXi = new ExtWidget("Open xi",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"openXi");
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

    outerRadius = new ExtWidget("Outer radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"outerRadius");
    addToTab("General", outerRadius);
    width = new ExtWidget("Width",new ChoiceWidget2(new ScalarWidgetFactory("0.1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"width");
    addToTab("General", width);
    innerRadius = new ExtWidget("Inner radius",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"innerRadius");
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

  GearWheelPropertyDialog::GearWheelPropertyDialog(Element *disk) : RigidContourPropertyDialog(disk) {
    addTab("Visualization",1);

    numberOfTeeth = new ExtWidget("Number of teeth",new ChoiceWidget2(new ScalarWidgetFactory("15",vector<QStringList>(2,QStringList())),QBoxLayout::RightToLeft,5),false,false,MBSIM%"numberOfTeeth");
    addToTab("General", numberOfTeeth);
    width = new ExtWidget("Width",new ChoiceWidget2(new ScalarWidgetFactory("50",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"width");
    addToTab("General", width);
    helixAngle = new ExtWidget("Helix angle",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"helixAngle");
    addToTab("General", helixAngle);
    pitchAngle = new ExtWidget("Pitch angle",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pitchAngle");
    addToTab("General", pitchAngle);
    module = new ExtWidget("Module",new ChoiceWidget2(new ScalarWidgetFactory("16",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"module");
    addToTab("General", module);
    pressureAngle = new ExtWidget("Pressure angle",new ChoiceWidget2(new ScalarWidgetFactory("20",vector<QStringList>(2,angleUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"pressureAngle");
    addToTab("General", pressureAngle);
    backlash = new ExtWidget("Backlash",new ChoiceWidget2(new ScalarWidgetFactory("0",vector<QStringList>(2,lengthUnits()),vector<int>(2,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"backlash");
    addToTab("General", backlash);
    solid = new ExtWidget("Solid",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"solid");
    addToTab("General", solid);
    visu = new ExtWidget("Enable openMBV",new MBSOMBVColoreBodyWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", visu);
  }

  DOMElement* GearWheelPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidContourPropertyDialog::initializeUsingXML(item->getXMLElement());
    numberOfTeeth->initializeUsingXML(item->getXMLElement());
    width->initializeUsingXML(item->getXMLElement());
    helixAngle->initializeUsingXML(item->getXMLElement());
    pitchAngle->initializeUsingXML(item->getXMLElement());
    module->initializeUsingXML(item->getXMLElement());
    pressureAngle->initializeUsingXML(item->getXMLElement());
    backlash->initializeUsingXML(item->getXMLElement());
    solid->initializeUsingXML(item->getXMLElement());
    visu->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GearWheelPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidContourPropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    numberOfTeeth->writeXMLFile(item->getXMLElement(),nullptr);
    width->writeXMLFile(item->getXMLElement(),nullptr);
    helixAngle->writeXMLFile(item->getXMLElement(),nullptr);
    pitchAngle->writeXMLFile(item->getXMLElement(),nullptr);
    module->writeXMLFile(item->getXMLElement(),nullptr);
    pressureAngle->writeXMLFile(item->getXMLElement(),nullptr);
    backlash->writeXMLFile(item->getXMLElement(),nullptr);
    solid->writeXMLFile(item->getXMLElement(),nullptr);
    visu->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

  FlexiblePlanarNurbsContourPropertyDialog::FlexiblePlanarNurbsContourPropertyDialog(Element *contour) : ContourPropertyDialog(contour) {
    addTab("Visualization",1);

    interpolation = new ExtWidget("Interpolation",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"interpolation");
    addToTab("General", interpolation);

    indices = new ExtWidget("Indices",new ChoiceWidget2(new VecSizeVarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"indices");
    addToTab("General", indices);

    knotVector = new ExtWidget("Knot vector",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"knotVector");
    addToTab("General", knotVector);

    degree = new ExtWidget("Degree",new ChoiceWidget2(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"degree");
    addToTab("General", degree);

    open = new ExtWidget("Open",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"open");
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

    interpolation = new ExtWidget("Interpolation",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"interpolation");
    addToTab("General", interpolation);

    indices = new ExtWidget("Indices",new ChoiceWidget2(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"indices");
    addToTab("General", indices);

    etaKnotVector = new ExtWidget("Eta knot vector",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"etaKnotVector");
    addToTab("General", etaKnotVector);

    xiKnotVector = new ExtWidget("Xi knot vector",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"xiKnotVector");
    addToTab("General", xiKnotVector);

    etaDegree = new ExtWidget("Eta degree",new ChoiceWidget2(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"etaDegree");
    addToTab("General", etaDegree);

    xiDegree = new ExtWidget("Xi degree",new ChoiceWidget2(new ScalarWidgetFactory("3"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"xiDegree");
    addToTab("General", xiDegree);

    openEta = new ExtWidget("Open eta",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"openEta");
    addToTab("General", openEta);

    openXi = new ExtWidget("Open xi",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"openXi");
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

  FclContourPropertyDialog::FclContourPropertyDialog(Element *contour) : RigidContourPropertyDialog(contour) {
    computeLocalAABB = new ExtWidget("Compute local AABB",new ChoiceWidget2(new BoolWidgetFactory("true"),QBoxLayout::RightToLeft,5),true,false,MBSIMFCL%"computeLocalAABB");
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

    length = new ExtWidget("Length",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"length");
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

    radius = new ExtWidget("Radius",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,lengthUnits()),vector<int>(2,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"radius");
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
    normal = new ExtWidget("Normal",new ChoiceWidget2(new VecWidgetFactory(n,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"normal");
    addToTab("General", normal);

    offset = new ExtWidget("Offset",new ChoiceWidget2(new ScalarWidgetFactory(0),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"offset");
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

    vertices = new ExtWidget("Vertices",new ChoiceWidget2(new MatRowsVarWidgetFactory(1,3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"vertices");
    addToTab("General", vertices);

    triangles = new ExtWidget("Triangles",new ChoiceWidget2(new MatRowsVarWidgetFactory(1,3,vector<QStringList>(3,QStringList()),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMFCL%"triangles");
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

  GroupPropertyDialog::GroupPropertyDialog(Element *group, bool kinematics) : ElementPropertyDialog(group), frameOfReference(nullptr) {
    if(kinematics) {
      addTab("Kinematics",1);

      frameOfReference = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(group,nullptr),true,false,MBSIM%"frameOfReference");
      addToTab("Kinematics", frameOfReference);
    }
  }

  DOMElement* GroupPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    if(frameOfReference)
      frameOfReference->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GroupPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(parent,ref?ref:getElement()->getXMLFrames());
    if(frameOfReference)
      frameOfReference->writeXMLFile(item->getXMLElement(),ref?ref:getElement()->getXMLFrames());
    return nullptr;
  }

  DynamicSystemSolverPropertyDialog::DynamicSystemSolverPropertyDialog(Element *solver) : GroupPropertyDialog(solver,false) {
    addTab("Environment",1);
    addTab("Solver parameters",2);
    addTab("Extra");

    environment = new ExtWidget("Acceleration of gravity",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,accelerationUnits())),QBoxLayout::RightToLeft,5),false,false,MBSIM%"accelerationOfGravity");
    addToTab("Environment", environment);

    vector<QString> list;
    list.emplace_back("\"fixedpoint\"");
    list.emplace_back("\"GaussSeidel\"");
    list.emplace_back("\"direct\"");
    list.emplace_back("\"rootfinding\"");
    constraintSolver = new ExtWidget("Constraint solver",new TextChoiceWidget(list,0,true),true,false,MBSIM%"constraintSolver");
    addToTab("Solver parameters", constraintSolver);

    impactSolver = new ExtWidget("Impact solver",new TextChoiceWidget(list,0,true),true,false,MBSIM%"impactSolver");
    addToTab("Solver parameters", impactSolver);

    maxIter = new ExtWidget("Maximum iterations",new ChoiceWidget2(new ScalarWidgetFactory("10000"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumIterations");
    addToTab("Solver parameters", maxIter);

    numericalJacobian = new ExtWidget("Numerical jacobian",new ChoiceWidget2(new BoolWidgetFactory("false"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"numericalJacobian");
    addToTab("Solver parameters", numericalJacobian);

    projection = new ExtWidget("Projection tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-15"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"projectionTolerance");
    addToTab("Solver parameters", projection);

    gTol = new ExtWidget("Generalized relative position tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-8"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativePositionTolerance");
    addToTab("Solver parameters", gTol);

    gdTol = new ExtWidget("Generalized relative velocity tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativeVelocityTolerance");
    addToTab("Solver parameters", gdTol);

    gddTol = new ExtWidget("Generalized relative acceleration tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-12"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativeAccelerationTolerance");
    addToTab("Solver parameters", gddTol);

    laTol = new ExtWidget("Generalized force tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-12"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedForceTolerance");
    addToTab("Solver parameters", laTol);

    LaTol = new ExtWidget("Generalized impulse tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedImpulseTolerance");
    addToTab("Solver parameters", LaTol);

    gCorr = new ExtWidget("Generalized relative position correction value",new ChoiceWidget2(new ScalarWidgetFactory("1e-14"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativePositionCorrectionValue");
    addToTab("Solver parameters", gCorr);

    gdCorr = new ExtWidget("Generalized relative velocity correction value",new ChoiceWidget2(new ScalarWidgetFactory("1e-16"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedRelativeVelocityCorrectionValue");
    addToTab("Solver parameters", gdCorr);

    inverseKinetics = new ExtWidget("Inverse kinetics",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"inverseKinetics");
    addToTab("Extra", inverseKinetics);

    initialProjection = new ExtWidget("Initial projection",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialProjection");
    addToTab("Extra", initialProjection);

    determineEquilibriumState = new ExtWidget("Determine equilibrium state",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"determineEquilibriumState");
    addToTab("Extra",determineEquilibriumState);

    useConstraintSolverForSmoothMotion = new ExtWidget("Use constraint solver for smooth motion",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"useConstraintSolverForSmoothMotion");
    addToTab("Extra", useConstraintSolverForSmoothMotion);

    useConstraintSolverForPlot = new ExtWidget("Use constraint solver for plot",new ChoiceWidget2(new BoolWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"useConstraintSolverForPlot");
    addToTab("Extra", useConstraintSolverForPlot);
  }

  DOMElement* DynamicSystemSolverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GroupPropertyDialog::initializeUsingXML(item->getXMLElement());
    environment->initializeUsingXML(static_cast<DynamicSystemSolver*>(item)->getXMLEnvironments()->getFirstElementChild());
    constraintSolver->initializeUsingXML(item->getXMLElement());
    impactSolver->initializeUsingXML(item->getXMLElement());
    maxIter->initializeUsingXML(item->getXMLElement());
    numericalJacobian->initializeUsingXML(item->getXMLElement());
    projection->initializeUsingXML(item->getXMLElement());
    gTol->initializeUsingXML(item->getXMLElement());
    gdTol->initializeUsingXML(item->getXMLElement());
    gddTol->initializeUsingXML(item->getXMLElement());
    laTol->initializeUsingXML(item->getXMLElement());
    LaTol->initializeUsingXML(item->getXMLElement());
    gCorr->initializeUsingXML(item->getXMLElement());
    gdCorr->initializeUsingXML(item->getXMLElement());
    inverseKinetics->initializeUsingXML(item->getXMLElement());
    initialProjection->initializeUsingXML(item->getXMLElement());
    determineEquilibriumState->initializeUsingXML(item->getXMLElement());
    useConstraintSolverForSmoothMotion->initializeUsingXML(item->getXMLElement());
    useConstraintSolverForPlot->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DynamicSystemSolverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GroupPropertyDialog::writeXMLFile(parent,getElement()->getXMLFrames());
    environment->writeXMLFile(static_cast<DynamicSystemSolver*>(item)->getXMLEnvironments()->getFirstElementChild());
    constraintSolver->writeXMLFile(item->getXMLElement());
    impactSolver->writeXMLFile(item->getXMLElement());
    maxIter->writeXMLFile(item->getXMLElement());
    numericalJacobian->writeXMLFile(item->getXMLElement());
    projection->writeXMLFile(item->getXMLElement());
    gTol->writeXMLFile(item->getXMLElement());
    gdTol->writeXMLFile(item->getXMLElement());
    gddTol->writeXMLFile(item->getXMLElement());
    laTol->writeXMLFile(item->getXMLElement());
    LaTol->writeXMLFile(item->getXMLElement());
    gCorr->writeXMLFile(item->getXMLElement());
    gdCorr->writeXMLFile(item->getXMLElement());
    inverseKinetics->writeXMLFile(item->getXMLElement());
    initialProjection->writeXMLFile(item->getXMLElement());
    determineEquilibriumState->writeXMLFile(item->getXMLElement());
    useConstraintSolverForSmoothMotion->writeXMLFile(item->getXMLElement());
    useConstraintSolverForPlot->writeXMLFile(item->getXMLElement());
    return nullptr;
  }

  ObjectPropertyDialog::ObjectPropertyDialog(Element *object) : ElementPropertyDialog(object) {
    addTab("Initial conditions",1);

    q0 = new ExtWidget("Generalized initial position",new ChoiceWidget2(new VecWidgetFactory(0),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedInitialPosition");
    addToTab("Initial conditions", q0);

    u0 = new ExtWidget("Generalized initial velocity",new ChoiceWidget2(new VecWidgetFactory(0),QBoxLayout::RightToLeft,5),true,false,MBSIM%"generalizedInitialVelocity");
    addToTab("Initial conditions", u0);

    connect(q0, SIGNAL(widgetChanged()), this, SLOT(updateWidget()));
    connect(u0, SIGNAL(widgetChanged()), this, SLOT(updateWidget()));
  }

  DOMElement* ObjectPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    q0->initializeUsingXML(item->getXMLElement());
    u0->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ObjectPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    q0->writeXMLFile(item->getXMLElement(),ref);
    u0->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  BodyPropertyDialog::BodyPropertyDialog(Element *body) : ObjectPropertyDialog(body) {
    addTab("Kinematics",1);

    R = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(body,body->getParent()->getFrame(0),this),true,false,MBSIM%"frameOfReference");
    addToTab("Kinematics",R);
  }

  DOMElement* BodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObjectPropertyDialog::initializeUsingXML(item->getXMLElement());
    R->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* BodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObjectPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    R->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodyPropertyDialog::RigidBodyPropertyDialog(Element *body) : BodyPropertyDialog(body) {
    addTab("Visualization",3);

    K = new ExtWidget("Frame for kinematics",new LocalFrameOfReferenceWidget(body,nullptr),true,false,MBSIM%"frameForKinematics");
    addToTab("Kinematics",K);

    mass = new ExtWidget("Mass",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,massUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"mass");
    addToTab("General",mass);

    inertia = new ExtWidget("Inertia tensor",new ChoiceWidget2(new SymMatWidgetFactory(getEye<QString>(3,3,"0.01","0"),vector<QStringList>(3,inertiaUnits()),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIM%"inertiaTensor");
    addToTab("General",inertia);

    frameForInertiaTensor = new ExtWidget("Frame for inertia tensor",new LocalFrameOfReferenceWidget(body,nullptr),true,false,MBSIM%"frameForInertiaTensor");
    addToTab("General",frameForInertiaTensor);

    translation = new ExtWidget("Translation",new ChoiceWidget2(new TranslationWidgetFactory4(body,MBSIM,this),QBoxLayout::TopToBottom,3),true,false,"");
    addToTab("Kinematics", translation);
    connect(translation,SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    rotation = new ExtWidget("Rotation",new ChoiceWidget2(new RotationWidgetFactory4(body,MBSIM,this),QBoxLayout::TopToBottom,3),true,false,"");
    addToTab("Kinematics", rotation);
    connect(rotation,SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    translationDependentRotation = new ExtWidget("Translation dependent rotation",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"translationDependentRotation");
    addToTab("Kinematics", translationDependentRotation);

    vector<QString> list;
    list.emplace_back("\"derivativeOfGeneralizedPositionOfRotation\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameOfReference\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameForKinematics\"");
    generalizedVelocityOfRotation = new ExtWidget("Generalized velocity of rotation",new TextChoiceWidget(list,1,true),true,false,MBSIM%"generalizedVelocityOfRotation");
    addToTab("Kinematics", generalizedVelocityOfRotation);

    ombv = new ExtWidget("OpenMBV Body",new ChoiceWidget2(new OMBVRigidBodyWidgetFactory,QBoxLayout::TopToBottom,0),true,true,MBSIM%"openMBVRigidBody");
    addToTab("Visualization", ombv);

    ombvFrameRef=new ExtWidget("OpenMBV frame of reference",new LocalFrameOfReferenceWidget(body),true,false,MBSIM%"openMBVFrameOfReference");
    addToTab("Visualization", ombvFrameRef);
  }

  DOMElement* RigidBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    BodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    K->initializeUsingXML(item->getXMLElement());
    mass->initializeUsingXML(item->getXMLElement());
    inertia->initializeUsingXML(item->getXMLElement());
    frameForInertiaTensor->initializeUsingXML(item->getXMLElement());
    translation->initializeUsingXML(item->getXMLElement());
    rotation->initializeUsingXML(item->getXMLElement());
    translationDependentRotation->initializeUsingXML(item->getXMLElement());
    generalizedVelocityOfRotation->initializeUsingXML(item->getXMLElement());
    ombv->initializeUsingXML(item->getXMLElement());
    ombvFrameRef->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    BodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    mass->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    inertia->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    frameForInertiaTensor->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translationDependentRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    generalizedVelocityOfRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    DOMElement *ele =getElement()->getXMLContours()->getNextElementSibling();
    ombv->writeXMLFile(item->getXMLElement(),ele);
    ombvFrameRef->writeXMLFile(item->getXMLElement(),ele);
    return nullptr;
  }

  int RigidBodyPropertyDialog::getqRelSize() const {
    int nqT=0, nqR=0;
    if(translation->isActive()) {
      if(static_cast<ChoiceWidget2*>(translation->getWidget())->getIndex()!=1) {
        auto *trans = dynamic_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(static_cast<ChoiceWidget2*>(translation->getWidget())->getWidget())->getWidget());
        if(trans)
          nqT = trans->getArg1Size();
      }
    }
    if(rotation->isActive()) {
      if(static_cast<ChoiceWidget2*>(rotation->getWidget())->getIndex()!=1) {
        auto *rot = dynamic_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(static_cast<ChoiceWidget2*>(rotation->getWidget())->getWidget())->getWidget());
        if(rot)
          nqR = rot->getArg1Size();
      }
    }
    return nqT + nqR;
  }

  void RigidBodyPropertyDialog::resizeGeneralizedPosition() {
    int size =  getqRelSize();
    q0->resize_(size,1);
    if(not size) q0->setActive(false);
  }

  void RigidBodyPropertyDialog::resizeGeneralizedVelocity() {
    int size =  getuRelSize();
    u0->resize_(size,1);
    if(not size) u0->setActive(false);
  }

  GenericFlexibleFfrBodyPropertyDialog::GenericFlexibleFfrBodyPropertyDialog(Element *body) : BodyPropertyDialog(body) {

    translation = new ExtWidget("Translation",new ChoiceWidget2(new TranslationWidgetFactory4(body,MBSIMFLEX,this),QBoxLayout::TopToBottom,3),true,false,"");
    addToTab("Kinematics", translation);
    connect(translation,SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    rotation = new ExtWidget("Rotation",new ChoiceWidget2(new RotationWidgetFactory4(body,MBSIMFLEX,this),QBoxLayout::TopToBottom,3),true,false,"");
    addToTab("Kinematics", rotation);
    connect(rotation,SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    translationDependentRotation = new ExtWidget("Translation dependent rotation",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"translationDependentRotation");
    addToTab("Kinematics", translationDependentRotation);

    vector<QString> list;
    list.emplace_back("\"derivativeOfGeneralizedPositionOfRotation\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameOfReference\"");
    list.emplace_back("\"coordinatesOfAngularVelocityWrtFrameForKinematics\"");
    generalizedVelocityOfRotation = new ExtWidget("Generalized velocity of rotation",new TextChoiceWidget(list,1,true),true,false,MBSIM%"generalizedVelocityOfRotation");
    addToTab("Kinematics", generalizedVelocityOfRotation);
  }

  int GenericFlexibleFfrBodyPropertyDialog::getqRelSize() const {
    int nqT=0, nqR=0;
    if(translation->isActive()) {
      if(static_cast<ChoiceWidget2*>(translation->getWidget())->getIndex()!=1) {
        auto *trans = dynamic_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(static_cast<ChoiceWidget2*>(translation->getWidget())->getWidget())->getWidget());
        if(trans)
          nqT = trans->getArg1Size();
      }
    }
    if(rotation->isActive()) {
      if(static_cast<ChoiceWidget2*>(rotation->getWidget())->getIndex()!=1) {
        auto *rot = dynamic_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(static_cast<ChoiceWidget2*>(rotation->getWidget())->getWidget())->getWidget());
        if(rot)
          nqR = rot->getArg1Size();
      }
    }
    return nqT + nqR + getqERelSize();
  }

  void GenericFlexibleFfrBodyPropertyDialog::resizeGeneralizedPosition() {
    int size =  getqRelSize();
    q0->resize_(size,1);
    if(not size) q0->setActive(false);
  }

  void GenericFlexibleFfrBodyPropertyDialog::resizeGeneralizedVelocity() {
    int size =  getuRelSize();
    u0->resize_(size,1);
    if(not size) u0->setActive(false);
  }

  DOMElement* GenericFlexibleFfrBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    BodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    translation->initializeUsingXML(item->getXMLElement());
    rotation->initializeUsingXML(item->getXMLElement());
    translationDependentRotation->initializeUsingXML(item->getXMLElement());
    generalizedVelocityOfRotation->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GenericFlexibleFfrBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    BodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    return nullptr;
  }

  FlexibleFfrBodyPropertyDialog::FlexibleFfrBodyPropertyDialog(Element *body) : GenericFlexibleFfrBodyPropertyDialog(body) {
    addTab("Visualization",3);
    addTab("Nodal data");

    mass = new ExtWidget("Mass",new ChoiceWidget2(new ScalarWidgetFactory("1",vector<QStringList>(2,massUnits()),vector<int>(2,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"mass");
    addToTab("General",mass);

    rdm = new ExtWidget("Position integral",new ChoiceWidget2(new VecWidgetFactory(3,vector<QStringList>(3,QStringList()),vector<int>(3,0)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"positionIntegral");
    addToTab("General", rdm);

    rrdm = new ExtWidget("Position position integral",new ChoiceWidget2(new SymMatWidgetFactory(getEye<QString>(3,3,"0","0"),vector<QStringList>(3,inertiaUnits()),vector<int>(3,2)),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"positionPositionIntegral");
    addToTab("General",rrdm);

    Pdm = new ExtWidget("Shape function integral",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"shapeFunctionIntegral");
    addToTab("General",Pdm);

    rPdm = new ExtWidget("Position shape function integral",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"positionShapeFunctionIntegral",3,3,1),QBoxLayout::TopToBottom,3),true,false,"");
    addToTab("General",rPdm);

    PPdm = new ExtWidget("Shape function shape function integral",new ChoiceWidget2(new TwoDimMatArrayWidgetFactory(MBSIMFLEX%"shapeFunctionShapeFunctionIntegral",3,1,1),QBoxLayout::TopToBottom,3),true,false,"");
    addToTab("General",PPdm);

    Ke = new ExtWidget("Stiffness matrix",new ChoiceWidget2(new SymMatWidgetFactory(getMat<QString>(1,1,"0")),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"stiffnessMatrix");
    addToTab("General",Ke);

    De = new ExtWidget("Damping matrix",new ChoiceWidget2(new SymMatWidgetFactory(getMat<QString>(1,1,"0")),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"dampingMatrix");
    addToTab("General",De);

    beta = new ExtWidget("Proportional damping",new ChoiceWidget2(new VecWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"proportionalDamping");
    addToTab("General", beta);

    Knl1 = new ExtWidget("Nonlinear stiffness matrix of first order",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrder"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("General",Knl1);

    Knl2 = new ExtWidget("Nonlinear stiffness matrix of second order",new ChoiceWidget2(new TwoDimMatArrayWidgetFactory(MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrder"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("General",Knl2);

    ksigma0 = new ExtWidget("Initial stress integral",new ChoiceWidget2(new VecWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"initialStressIntegral");
    addToTab("General", ksigma0);

    ksigma1 = new ExtWidget("Nonlinear initial stress integral",new ChoiceWidget2(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"nonlinearInitialStressIntegral");
    addToTab("General", ksigma1);

    K0t = new ExtWidget("Geometric stiffness matrix due to acceleration",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"geometricStiffnessMatrixDueToAcceleration"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("General",K0t);

    K0r = new ExtWidget("Geometric stiffness matrix due to angular acceleration",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAcceleration"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("General",K0r);

    K0om = new ExtWidget("Geometric stiffness matrix due to angular velocity",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocity"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("General",K0om);

    r = new ExtWidget("Nodal relative position",new ChoiceWidget2(new OneDimVecArrayWidgetFactory(MBSIMFLEX%"nodalRelativePosition",1,3,true),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", r);

    A = new ExtWidget("Nodal relative orientation",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"nodalRelativeOrientation"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", A);

    Phi = new ExtWidget("Nodal shape matrix of translation",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"nodalShapeMatrixOfTranslation"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", Phi);

    Psi = new ExtWidget("Nodal shape matrix of rotation",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"nodalShapeMatrixOfRotation"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", Psi);

    sigmahel = new ExtWidget("Nodal stress matrix",new ChoiceWidget2(new OneDimMatArrayWidgetFactory(MBSIMFLEX%"nodalStressMatrix"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", sigmahel);

    sigmahen = new ExtWidget("Nodal nonlinear stress matrix",new ChoiceWidget2(new TwoDimMatArrayWidgetFactory(MBSIMFLEX%"nodalNonlinearStressMatrix"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", sigmahen);

    sigma0 = new ExtWidget("Nodal initial stress",new ChoiceWidget2(new OneDimVecArrayWidgetFactory(MBSIMFLEX%"nodalInitialStress"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", sigma0);

    K0F = new ExtWidget("Nodal geometric stiffness matrix due to force",new ChoiceWidget2(new TwoDimMatArrayWidgetFactory(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToForce"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", K0F);

    K0M = new ExtWidget("Nodal geometric stiffness matrix due to moment",new ChoiceWidget2(new TwoDimMatArrayWidgetFactory(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToMoment"),QBoxLayout::RightToLeft,3),true,false,"");
    addToTab("Nodal data", K0M);

    ombv = new ExtWidget("OpenMBV body",new ChoiceWidget2(new OMBVFlexibleBodyWidgetFactory,QBoxLayout::TopToBottom,0),true,true,MBSIMFLEX%"openMBVFlexibleBody");
    addToTab("Visualization", ombv);

    ombvNodes = new ExtWidget("OpenMBV nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"openMBVNodes");
    addToTab("Visualization", ombvNodes);

    vector<QString> list;
    list.emplace_back("\"none\"");
    list.emplace_back("\"xDisplacement\"");
    list.emplace_back("\"yDisplacement\"");
    list.emplace_back("\"zDisplacement\"");
    list.emplace_back("\"totalDisplacement\"");
    list.emplace_back("\"xxStress\"");
    list.emplace_back("\"yyStress\"");
    list.emplace_back("\"zzStress\"");
    list.emplace_back("\"xyStress\"");
    list.emplace_back("\"yzStress\"");
    list.emplace_back("\"zxStress\"");
    list.emplace_back("\"equivalentStress\"");
    ombvColorRepresentation = new ExtWidget("OpenMBV color representation",new TextChoiceWidget(list,0,true),true,false,MBSIMFLEX%"openMBVColorRepresentation");
    addToTab("Visualization", ombvColorRepresentation);

    plotNodes = new ExtWidget("Plot nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"plotNodes");
    addToTab("Visualization", plotNodes);

    connect(Pdm->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
//    connect(Knl1,SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
  }

  void FlexibleFfrBodyPropertyDialog::updateWidget() {
    BodyPropertyDialog::updateWidget();
    int size = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(Pdm->getWidget())->getWidget())->cols();
    if(static_cast<ChoiceWidget2*>(rPdm->getWidget())->getIndex()==0)
      rPdm->resize_(3,size);
    else
      rPdm->resize_(9,size);
    if(static_cast<ChoiceWidget2*>(PPdm->getWidget())->getIndex()==0)
      PPdm->resize_(size,size);
    else
      PPdm->resize_(9*size,size);
    Ke->resize_(size,size);
    De->resize_(size,size);
    if(Knl1->isActive()) {
      if(static_cast<ChoiceWidget2*>(Knl1->getWidget())->getIndex()==0)
        static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(Knl1->getWidget())->getWidget())->resize_(size,size,size);
      else
        Knl1->resize_(size*size,size);
    }
    if(Knl2->isActive()) {
      if(static_cast<ChoiceWidget2*>(Knl2->getWidget())->getIndex()==0)
        static_cast<TwoDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(Knl2->getWidget())->getWidget())->resize_(size,size,size,size);
      else
        Knl2->resize_(size*size*size,size);
    }
    ksigma0->resize_(size,1);
    ksigma1->resize_(size,size);
    if(K0t->isActive()) {
      if(static_cast<ChoiceWidget2*>(K0t->getWidget())->getIndex()==0)
        static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(K0t->getWidget())->getWidget())->resize_(3,size,size);
      else
        K0t->resize_(3*size,size);
    }
    if(K0r->isActive()) {
      if(static_cast<ChoiceWidget2*>(K0r->getWidget())->getIndex()==0)
        static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(K0r->getWidget())->getWidget())->resize_(3,size,size);
      else
        K0r->resize_(3*size,size);
    }
    if(K0om->isActive()) {
      if(static_cast<ChoiceWidget2*>(K0om->getWidget())->getIndex()==0)
        static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(K0om->getWidget())->getWidget())->resize_(3,size,size);
      else
        K0om->resize_(3*size,size);
    }
    if(r->isActive()) {
      int rsize;
      if(static_cast<ChoiceWidget2*>(r->getWidget())->getIndex()==0)
        rsize = static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(r->getWidget())->getWidget())->getArray().size();
      else
        rsize = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(static_cast<ChoiceWidget2*>(r->getWidget())->getWidget())->getWidget())->rows()/3;
      if(A->isActive()) {
        if(static_cast<ChoiceWidget2*>(A->getWidget())->getIndex()==0)
          static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(A->getWidget())->getWidget())->resize_(rsize,3,3);
        else
          A->resize_(3*rsize,3);
      }
      if(Phi->isActive()) {
        if(static_cast<ChoiceWidget2*>(Phi->getWidget())->getIndex()==0)
          static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(Phi->getWidget())->getWidget())->resize_(rsize,3,size);
        else
          Phi->resize_(3*rsize,size);
      }
      if(Psi->isActive()) {
        if(static_cast<ChoiceWidget2*>(Psi->getWidget())->getIndex()==0)
          static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(Psi->getWidget())->getWidget())->resize_(rsize,3,size);
        else
          Psi->resize_(3*rsize,size);
      }
      if(sigmahel->isActive()) {
        if(static_cast<ChoiceWidget2*>(sigmahel->getWidget())->getIndex()==0)
          static_cast<OneDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(sigmahel->getWidget())->getWidget())->resize_(rsize,6,size);
        else
          sigmahel->resize_(6*rsize,size);
      }
      if(sigmahen->isActive()) {
        if(static_cast<ChoiceWidget2*>(sigmahen->getWidget())->getIndex()==0)
          static_cast<TwoDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(sigmahen->getWidget())->getWidget())->resize_(rsize,size,6,size);
        else
          sigmahen->resize_(6*rsize*size,size);
      }
      if(sigma0->isActive()) {
        if(static_cast<ChoiceWidget2*>(sigma0->getWidget())->getIndex()==0)
          static_cast<OneDimVecArrayWidget*>(static_cast<ChoiceWidget2*>(sigma0->getWidget())->getWidget())->resize_(rsize,6,1);
        else
          sigma0->resize_(6*rsize,1);
      }
      if(K0F->isActive()) {
        if(static_cast<ChoiceWidget2*>(K0F->getWidget())->getIndex()==0)
          static_cast<TwoDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(K0F->getWidget())->getWidget())->resize_(rsize,size,size,size);
        else
          K0F->resize_(size*rsize*size,size);
      }
      if(K0M->isActive()) {
        if(static_cast<ChoiceWidget2*>(K0M->getWidget())->getIndex()==0)
          static_cast<TwoDimMatArrayWidget*>(static_cast<ChoiceWidget2*>(K0M->getWidget())->getWidget())->resize_(rsize,size,size,size);
        else
          K0M->resize_(size*rsize*size,size);
      }
    }
  }

  int FlexibleFfrBodyPropertyDialog::getqERelSize() const {
    int nqE=0;
    if(Pdm->isActive())
      nqE = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(Pdm->getWidget())->getWidget())->cols();
    return nqE;
  }

  DOMElement* FlexibleFfrBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GenericFlexibleFfrBodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    mass->initializeUsingXML(item->getXMLElement());
    rdm->initializeUsingXML(item->getXMLElement());
    rrdm->initializeUsingXML(item->getXMLElement());
    Pdm->initializeUsingXML(item->getXMLElement());
    rPdm->initializeUsingXML(item->getXMLElement());
    PPdm->initializeUsingXML(item->getXMLElement());
    Ke->initializeUsingXML(item->getXMLElement());
    De->initializeUsingXML(item->getXMLElement());
    beta->initializeUsingXML(item->getXMLElement());
    Knl1->initializeUsingXML(item->getXMLElement());
    Knl2->initializeUsingXML(item->getXMLElement());
    ksigma0->initializeUsingXML(item->getXMLElement());
    ksigma1->initializeUsingXML(item->getXMLElement());
    K0t->initializeUsingXML(item->getXMLElement());
    K0r->initializeUsingXML(item->getXMLElement());
    K0om->initializeUsingXML(item->getXMLElement());
    r->initializeUsingXML(item->getXMLElement());
    A->initializeUsingXML(item->getXMLElement());
    Phi->initializeUsingXML(item->getXMLElement());
    Psi->initializeUsingXML(item->getXMLElement());
    sigmahel->initializeUsingXML(item->getXMLElement());
    sigmahen->initializeUsingXML(item->getXMLElement());
    sigma0->initializeUsingXML(item->getXMLElement());
    K0F->initializeUsingXML(item->getXMLElement());
    K0M->initializeUsingXML(item->getXMLElement());
    ombv->initializeUsingXML(item->getXMLElement());
    ombvNodes->initializeUsingXML(item->getXMLElement());
    ombvColorRepresentation->initializeUsingXML(item->getXMLElement());
    plotNodes->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FlexibleFfrBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GenericFlexibleFfrBodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    mass->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rrdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Pdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rPdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    PPdm->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Ke->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    De->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    beta->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Knl1->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Knl2->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    ksigma0->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    ksigma1->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0t->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0r->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0om->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    r->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    A->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Phi->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    Psi->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    sigmahel->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    sigmahen->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    sigma0->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0F->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    K0M->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translationDependentRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    generalizedVelocityOfRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    DOMElement *ele =getElement()->getXMLContours()->getNextElementSibling();
    ombv->writeXMLFile(item->getXMLElement(),ele);
    ombvNodes->writeXMLFile(item->getXMLElement(),ele);
    ombvColorRepresentation->writeXMLFile(item->getXMLElement(),ele);
    plotNodes->writeXMLFile(item->getXMLElement(),ele);
    return nullptr;
  }

  CalculixBodyPropertyDialog::CalculixBodyPropertyDialog(Element *body) : GenericFlexibleFfrBodyPropertyDialog(body) {
    addTab("Visualization",3);

    resultFileName = new ExtWidget("Result file name",new FileWidget("", "Calculix result files", "frd files (*.frd)", 0, true),false,false,MBSIMFLEX%"resultFileName");
    addToTab("General",resultFileName);

    vector<QString> list;
    list.emplace_back("\"consistentMass\"");
    list.emplace_back("\"lumpedMass\"");
    formalism = new ExtWidget("Formalism",new TextChoiceWidget(list,1,true),true,false,MBSIMFLEX%"formalism");
    addToTab("General", formalism);

    beta = new ExtWidget("Proportional damping",new ChoiceWidget2(new VecWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"proportionalDamping");
    addToTab("General", beta);

    ombv = new ExtWidget("Enable openMBV",new CalculixBodyMBSOMBVWidget,true,true,MBSIMFLEX%"enableOpenMBV");
    addToTab("Visualization",ombv);

    plotNodes = new ExtWidget("Plot nodes",new ChoiceWidget2(new VecSizeVarWidgetFactory(2),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"plotNodes");
    addToTab("Visualization", plotNodes);
  }

  int CalculixBodyPropertyDialog::getqERelSize() const {
    int nqE=1;
    // TODO: read calculix result file and determine size of qE
    return nqE;
  }

  DOMElement* CalculixBodyPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GenericFlexibleFfrBodyPropertyDialog::initializeUsingXML(item->getXMLElement());
    resultFileName->initializeUsingXML(item->getXMLElement());
    formalism->initializeUsingXML(item->getXMLElement());
    beta->initializeUsingXML(item->getXMLElement());
    ombv->initializeUsingXML(item->getXMLElement());
    plotNodes->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* CalculixBodyPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GenericFlexibleFfrBodyPropertyDialog::writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    resultFileName->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    formalism->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    beta->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    rotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    translationDependentRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    generalizedVelocityOfRotation->writeXMLFile(item->getXMLElement(),getElement()->getXMLFrames());
    DOMElement *ele =getElement()->getXMLContours()->getNextElementSibling();
    ombv->writeXMLFile(item->getXMLElement(),ele);
    plotNodes->writeXMLFile(item->getXMLElement(),ele);
    return nullptr;
  }

  ConstraintPropertyDialog::ConstraintPropertyDialog(Element *constraint) : ElementPropertyDialog(constraint) {
  }

  MechanicalConstraintPropertyDialog::MechanicalConstraintPropertyDialog(Element *constraint) : ConstraintPropertyDialog(constraint) {
  }

  GeneralizedConstraintPropertyDialog::GeneralizedConstraintPropertyDialog(Element *constraint) : MechanicalConstraintPropertyDialog(constraint) {

    addTab("Visualization",2);

    support = new ExtWidget("Support frame",new ElementOfReferenceWidget<Frame>(constraint,nullptr,this),true,false,MBSIM%"supportFrame");
    addToTab("General",support);
  }

  DOMElement* GeneralizedConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    support->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    support->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedGearConstraintPropertyDialog::GeneralizedGearConstraintPropertyDialog(Element *constraint) : GeneralizedConstraintPropertyDialog(constraint) {

    dependentBody = new ExtWidget("Dependent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),false,false,MBSIM%"dependentRigidBody");
    addToTab("General", dependentBody);

    independentBodies = new ExtWidget("Independent rigid bodies",new ListWidget(new ElementOfReferenceWidgetFactory<RigidBody>(MBSIM%"independentRigidBody",constraint,true,this),"Independent body",0,2),false,false,"");
    addToTab("General",independentBodies);
  }

  DOMElement* GeneralizedGearConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    dependentBody->initializeUsingXML(item->getXMLElement());
    independentBodies->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedGearConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dependentBody->writeXMLFile(item->getXMLElement(),ref);
    independentBodies->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedDualConstraintPropertyDialog::GeneralizedDualConstraintPropertyDialog(Element *constraint) : GeneralizedConstraintPropertyDialog(constraint) {

    dependentBody = new ExtWidget("Dependent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),false,false,MBSIM%"dependentRigidBody");
    addToTab("General", dependentBody);

    independentBody = new ExtWidget("Independent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),true,false,MBSIM%"independentRigidBody");
    addToTab("General", independentBody);
  }

  DOMElement* GeneralizedDualConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    dependentBody->initializeUsingXML(item->getXMLElement());
    independentBody->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedDualConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dependentBody->writeXMLFile(item->getXMLElement(),ref);
    independentBody->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedPositionConstraintPropertyDialog::GeneralizedPositionConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {

    constraintFunction = new ExtWidget("Constraint function",new ChoiceWidget2(new FunctionWidgetFactory2(constraint,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"constraintFunction");
    addToTab("General", constraintFunction);
    connect(constraintFunction->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
  }

  void GeneralizedPositionConstraintPropertyDialog::updateWidget() {
//    RigidBody *refBody = static_cast<RigidBodyOfReferenceWidget*>(dependentBody->getWidget())->getSelectedBody();
//    int size = refBody?refBody->getqRelSize():0;
//    constraintFunction->resize_(size,1);
  }

  DOMElement* GeneralizedPositionConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedDualConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    constraintFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedPositionConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedDualConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    constraintFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedVelocityConstraintPropertyDialog::GeneralizedVelocityConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {
    addTab("Initial conditions",1);

    constraintFunction = new ExtWidget("Constraint function",new ChoiceWidget2(new ConstraintWidgetFactory(constraint,this),QBoxLayout::TopToBottom,3),false,false,"");
    addToTab("General", constraintFunction);
    connect(constraintFunction->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    x0 = new ExtWidget("Initial state",new ChoiceWidget2(new VecWidgetFactory(0,vector<QStringList>(3,QStringList())),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialState");
    addToTab("Initial conditions", x0);
  }

  void GeneralizedVelocityConstraintPropertyDialog::updateWidget() {
    cout << "GeneralizedVelocityConstraintPropertyDialog::updateWidget() not yet implemented" << endl;
  }

  DOMElement* GeneralizedVelocityConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedDualConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    x0->initializeUsingXML(item->getXMLElement());
    constraintFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedVelocityConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedDualConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    x0->writeXMLFile(item->getXMLElement(),ref);
    constraintFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedAccelerationConstraintPropertyDialog::GeneralizedAccelerationConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {
    addTab("Initial conditions",1);

    constraintFunction = new ExtWidget("Constraint function",new ChoiceWidget2(new ConstraintWidgetFactory(constraint,this),QBoxLayout::TopToBottom,3),false,false,"");
    addToTab("General", constraintFunction);
    connect(constraintFunction->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    x0 = new ExtWidget("Initial state",new ChoiceWidget2(new VecWidgetFactory(0,vector<QStringList>(3,QStringList())),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialState");
    addToTab("Initial conditions", x0);
  }

  void GeneralizedAccelerationConstraintPropertyDialog::updateWidget() {
//    RigidBody *refBody = static_cast<RigidBodyOfReferenceWidget*>(dependentBody->getWidget())->getSelectedBody();
//    int size = refBody?(refBody->getqRelSize()+refBody->getuRelSize()):0;
//    static_cast<ChoiceWidget2*>(constraintFunction->getWidget())->resize_(size,1);
//    if(x0_ && x0_->size() != size)
//      x0_->resize_(size);
//    static_cast<FunctionWidget*>(static_cast<ChoiceWidget2*>(constraintFunction->getWidget())->getWidget())->setArg1Size(size);
  }

  DOMElement* GeneralizedAccelerationConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    GeneralizedDualConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    x0->initializeUsingXML(item->getXMLElement());
    constraintFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedAccelerationConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    GeneralizedDualConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    x0->writeXMLFile(item->getXMLElement(),ref);
    constraintFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  JointConstraintPropertyDialog::JointConstraintPropertyDialog(Element *constraint) : MechanicalConstraintPropertyDialog(constraint) {

    addTab("Kinetics",1);
    addTab("Visualization",2);
    addTab("Initial conditions",2);

    dependentBodiesFirstSide = new ExtWidget("Dependent bodies on first side",new ListWidget(new ElementOfReferenceWidgetFactory<RigidBody>(MBSIM%"dependentRigidBodyOnFirstSide",constraint,this),"Body",0,2),false,false,"");
    addToTab("General",dependentBodiesFirstSide);
    connect(dependentBodiesFirstSide->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    dependentBodiesSecondSide = new ExtWidget("Dependent bodies on second side",new ListWidget(new ElementOfReferenceWidgetFactory<RigidBody>(MBSIM%"dependentRigidBodyOnSecondSide",constraint,this),"Body",0,2),false,false,"");
    addToTab("General",dependentBodiesSecondSide);
    connect(dependentBodiesSecondSide->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));

    independentBody = new ExtWidget("Independent rigid body",new ElementOfReferenceWidget<RigidBody>(constraint,nullptr,this),false,false,MBSIM%"independentRigidBody");
    addToTab("General", independentBody);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Frame>(2,constraint,this),false,false,MBSIM%"connect");
    addToTab("Kinetics", connections);

    vector<QString> list;
    list.emplace_back("\"firstFrame\"");
    list.emplace_back("\"secondFrame\"");
    refFrame = new ExtWidget("Frame of reference",new TextChoiceWidget(list,1,true),true,false,MBSIM%"frameOfReference");
    addToTab("Kinetics", refFrame);

    force = new ExtWidget("Force direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics", force);

    moment = new ExtWidget("Moment direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics", moment);

    q0 = new ExtWidget("Initial guess",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialGuess");
    addToTab("Initial conditions", q0);
  }

  void JointConstraintPropertyDialog::updateWidget() {
//    int size = 0;
//    ListWidget *list = static_cast<ListWidget*>(dependentBodiesFirstSide->getWidget());
//    for(int i=0; i<list->getSize(); i++) {
//      RigidBody *body = static_cast<RigidBodyOfReferenceWidget*>(list->getWidget(i))->getSelectedBody();
//      if(body)
//        size += body->getqRelSize();
//    }
//    list = static_cast<ListWidget*>(dependentBodiesSecondSide->getWidget());
//    for(int i=0; i<list->getSize(); i++) {
//      RigidBody *body = static_cast<RigidBodyOfReferenceWidget*>(list->getWidget(i))->getSelectedBody();
//      if(body)
//        size += body->getqRelSize();
//    }
//    if(q0_->size() != size)
//      q0_->resize_(size);
  }

  DOMElement* JointConstraintPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalConstraintPropertyDialog::initializeUsingXML(item->getXMLElement());
    dependentBodiesFirstSide->initializeUsingXML(item->getXMLElement());
    dependentBodiesSecondSide->initializeUsingXML(item->getXMLElement());
    independentBody->initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    refFrame->initializeUsingXML(item->getXMLElement());
    force->initializeUsingXML(item->getXMLElement());
    moment->initializeUsingXML(item->getXMLElement());
    q0->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* JointConstraintPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalConstraintPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    dependentBodiesFirstSide->writeXMLFile(item->getXMLElement(),ref);
    dependentBodiesSecondSide->writeXMLFile(item->getXMLElement(),ref);
    independentBody->writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    refFrame->writeXMLFile(item->getXMLElement(),ref);
    force->writeXMLFile(item->getXMLElement(),ref);
    moment->writeXMLFile(item->getXMLElement(),ref);
    q0->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedConnectionConstraintPropertyDialog::GeneralizedConnectionConstraintPropertyDialog(Element *constraint) : GeneralizedDualConstraintPropertyDialog(constraint) {
  }

  LinkPropertyDialog::LinkPropertyDialog(Element *link) : ElementPropertyDialog(link) {
  }

  MechanicalLinkPropertyDialog::MechanicalLinkPropertyDialog(Element *link) : LinkPropertyDialog(link) {
  }

  FrameLinkPropertyDialog::FrameLinkPropertyDialog(Element *link) : MechanicalLinkPropertyDialog(link) {
    addTab("Kinetics",1);
    addTab("Visualization",2);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Frame>(2,link,this),false,false,MBSIM%"connect");
    static_cast<ConnectElementsWidget<Frame>*>(connections->getWidget())->setDefaultElement("../Frame[I]");
    addToTab("Kinetics", connections);
  }

  DOMElement* FrameLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FrameLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  FixedFrameLinkPropertyDialog::FixedFrameLinkPropertyDialog(Element *link) : FrameLinkPropertyDialog(link) {
  }

  FloatingFrameLinkPropertyDialog::FloatingFrameLinkPropertyDialog(Element *link) : FrameLinkPropertyDialog(link) {
    vector<QString> list;
    list.emplace_back("\"firstFrame\"");
    list.emplace_back("\"secondFrame\"");
    refFrame = new ExtWidget("Frame of reference",new TextChoiceWidget(list,1,true),true,false,MBSIM%"frameOfReference");
    addToTab("Kinetics", refFrame);
  }

  DOMElement* FloatingFrameLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    refFrame->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FloatingFrameLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    refFrame->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodyLinkPropertyDialog::RigidBodyLinkPropertyDialog(Element *link) : MechanicalLinkPropertyDialog(link) {
    addTab("Visualization",2);

    support = new ExtWidget("Support frame",new ElementOfReferenceWidget<Frame>(link,nullptr,this),true,false,MBSIM%"supportFrame");
    addToTab("General",support);
  }

  DOMElement* RigidBodyLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    MechanicalLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    support->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodyLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    MechanicalLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    support->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DualRigidBodyLinkPropertyDialog::DualRigidBodyLinkPropertyDialog(Element *link) : RigidBodyLinkPropertyDialog(link) {
    addTab("Kinetics",1);

    connections = new ExtWidget("Connections",new ChoiceWidget2(new ConnectRigidBodiesWidgetFactory(link,this),QBoxLayout::RightToLeft,5),false,false,MBSIM%"connect");
    addToTab("Kinetics",connections);
  }

  DOMElement* DualRigidBodyLinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DualRigidBodyLinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  KineticExcitationPropertyDialog::KineticExcitationPropertyDialog(Element *kineticExcitation) : FloatingFrameLinkPropertyDialog(kineticExcitation) {

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics",forceDirection);

    forceFunction = new ExtWidget("Force function",new ChoiceWidget2(new FunctionWidgetFactory2(kineticExcitation,true,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"forceFunction");
    addToTab("Kinetics",forceFunction);

    momentDirection = new ExtWidget("Moment direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics",momentDirection);

    momentFunction = new ExtWidget("Moment function",new ChoiceWidget2(new FunctionWidgetFactory2(kineticExcitation,true,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"momentFunction");
    addToTab("Kinetics",momentFunction);

    arrow = new ExtWidget("Enable openMBV",new InteractionArrowMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization",arrow);

    connect(forceDirection->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    connect(forceFunction->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    connect(momentDirection->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    connect(momentFunction->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
  }

  void KineticExcitationPropertyDialog::updateWidget() {
    if(forceDirection->isActive()) {
      int size = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(forceDirection->getWidget())->getWidget())->cols();
      forceFunction->resize_(size,1);
    }
    if(momentDirection->isActive()) {
      int size = static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(momentDirection->getWidget())->getWidget())->cols();
      momentFunction->resize_(size,1);
    }
  }

  DOMElement* KineticExcitationPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    forceFunction->initializeUsingXML(item->getXMLElement());
    momentDirection->initializeUsingXML(item->getXMLElement());
    momentFunction->initializeUsingXML(item->getXMLElement());
    arrow->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* KineticExcitationPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    forceFunction->writeXMLFile(item->getXMLElement(),ref);
    momentDirection->writeXMLFile(item->getXMLElement(),ref);
    momentFunction->writeXMLFile(item->getXMLElement(),ref);
    arrow->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  SpringDamperPropertyDialog::SpringDamperPropertyDialog(Element *springDamper) : FixedFrameLinkPropertyDialog(springDamper) {

    forceFunction = new ExtWidget("Force function",new ChoiceWidget2(new SpringDamperWidgetFactory(springDamper,false,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    addToTab("Kinetics", forceFunction);

    unloadedLength = new ExtWidget("Unloaded length",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"unloadedLength");
    addToTab("General",unloadedLength);

    coilSpring = new ExtWidget("Enable openMBV",new CoilSpringMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", coilSpring);
  }

  DOMElement* SpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FixedFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceFunction->initializeUsingXML(item->getXMLElement());
    unloadedLength->initializeUsingXML(item->getXMLElement());
    coilSpring->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FixedFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceFunction->writeXMLFile(item->getXMLElement(),ref);
    unloadedLength->writeXMLFile(item->getXMLElement(),ref);
    coilSpring->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DirectionalSpringDamperPropertyDialog::DirectionalSpringDamperPropertyDialog(Element *springDamper) : FloatingFrameLinkPropertyDialog(springDamper) {

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget2(new VecWidgetFactory(3),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics",forceDirection);

    forceFunction = new ExtWidget("Force function",new ChoiceWidget2(new SpringDamperWidgetFactory(springDamper,false,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"forceFunction");
    addToTab("Kinetics", forceFunction);

    unloadedLength = new ExtWidget("Unloaded length",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"unloadedLength");
    addToTab("General",unloadedLength);

    coilSpring = new ExtWidget("Enable openMBV",new CoilSpringMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
    addToTab("Visualization", coilSpring);
  }

  DOMElement* DirectionalSpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    forceFunction->initializeUsingXML(item->getXMLElement());
    unloadedLength->initializeUsingXML(item->getXMLElement());
    coilSpring->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DirectionalSpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    forceFunction->writeXMLFile(item->getXMLElement(),ref);
    unloadedLength->writeXMLFile(item->getXMLElement(),ref);
    coilSpring->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  IsotropicRotationalSpringDamperPropertyDialog::IsotropicRotationalSpringDamperPropertyDialog(Element *springDamper) : FixedFrameLinkPropertyDialog(springDamper) {

    elasticMomentFunction = new ExtWidget("Elastic moment function",new ChoiceWidget2(new FunctionWidgetFactory2(springDamper,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"elasticMomentFunction");
    addToTab("Kinetics", elasticMomentFunction);

    dissipativeMomentFunction = new ExtWidget("Dissipative moment function",new ChoiceWidget2(new FunctionWidgetFactory2(springDamper,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"dissipativeMomentFunction");
    addToTab("Kinetics", dissipativeMomentFunction);
  }

  DOMElement* IsotropicRotationalSpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FixedFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    elasticMomentFunction->initializeUsingXML(item->getXMLElement());
    dissipativeMomentFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* IsotropicRotationalSpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FixedFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    elasticMomentFunction->writeXMLFile(item->getXMLElement(),ref);
    dissipativeMomentFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  JointPropertyDialog::JointPropertyDialog(Element *joint) : FloatingFrameLinkPropertyDialog(joint) {

    addTab("Extra");

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics",forceDirection);

    forceLaw = new ExtWidget("Force law",new ChoiceWidget2(new GeneralizedForceLawWidgetFactory,QBoxLayout::TopToBottom,0),true,false,MBSIM%"forceLaw");
    addToTab("Kinetics",forceLaw);

    momentDirection = new ExtWidget("Moment direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics",momentDirection);

    momentLaw = new ExtWidget("Moment law",new ChoiceWidget2(new GeneralizedForceLawWidgetFactory,QBoxLayout::TopToBottom,0),true,false,MBSIM%"momentLaw");
    addToTab("Kinetics",momentLaw);

    integrate = new ExtWidget("Integrate generalized relative velocity of rotation",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"integrateGeneralizedRelativeVelocityOfRotation");
    addToTab("Extra",integrate);
  }

  DOMElement* JointPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    forceLaw->initializeUsingXML(item->getXMLElement());
    momentDirection->initializeUsingXML(item->getXMLElement());
    momentLaw->initializeUsingXML(item->getXMLElement());
    integrate->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* JointPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    forceLaw->writeXMLFile(item->getXMLElement(),ref);
    momentDirection->writeXMLFile(item->getXMLElement(),ref);
    momentLaw->writeXMLFile(item->getXMLElement(),ref);
    integrate->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ElasticJointPropertyDialog::ElasticJointPropertyDialog(Element *joint) : FloatingFrameLinkPropertyDialog(joint) {

    addTab("Extra");

    forceDirection = new ExtWidget("Force direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"forceDirection");
    addToTab("Kinetics", forceDirection);

    momentDirection = new ExtWidget("Moment direction",new ChoiceWidget2(new MatColsVarWidgetFactory(3,1,vector<QStringList>(3,noUnitUnits()),vector<int>(3,1)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"momentDirection");
    addToTab("Kinetics", momentDirection);

    function = new ExtWidget("Generalized force function",new ChoiceWidget2(new SpringDamperWidgetFactory(joint,false,this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);

    integrate = new ExtWidget("Integrate generalized relative velocity of rotation",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"integrateGeneralizedRelativeVelocityOfRotation");
    addToTab("Extra", integrate);

    connect(forceDirection->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    connect(momentDirection->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    connect(function->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
  }

  void ElasticJointPropertyDialog::updateWidget() {
    int size = 0;
    if(forceDirection->isActive())
      size += static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(forceDirection->getWidget())->getWidget())->cols();
    if(momentDirection->isActive())
      size += static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(momentDirection->getWidget())->getWidget())->cols();
    function->resize_(size,1);
  }

  DOMElement* ElasticJointPropertyDialog::initializeUsingXML(DOMElement *parent) {
    FloatingFrameLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    forceDirection->initializeUsingXML(item->getXMLElement());
    momentDirection->initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    integrate->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ElasticJointPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FloatingFrameLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    forceDirection->writeXMLFile(item->getXMLElement(),ref);
    momentDirection->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    integrate->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedSpringDamperPropertyDialog::GeneralizedSpringDamperPropertyDialog(Element *springDamper) : DualRigidBodyLinkPropertyDialog(springDamper) {

    function = new ExtWidget("Generalized force function",new ChoiceWidget2(new SpringDamperWidgetFactory(springDamper,false,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);

    unloadedLength = new ExtWidget("Generalized Unloaded length",new ChoiceWidget2(new ScalarWidgetFactory("0"),QBoxLayout::RightToLeft,5),false,false,MBSIM%"generalizedUnloadedLength");
    addToTab("General",unloadedLength);
  }

  DOMElement* GeneralizedSpringDamperPropertyDialog::initializeUsingXML(DOMElement *parent) {
    DualRigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    unloadedLength->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedSpringDamperPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DualRigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    unloadedLength->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedFrictionPropertyDialog::GeneralizedFrictionPropertyDialog(Element *friction) : DualRigidBodyLinkPropertyDialog(friction) {

    frictionForceLaw = new ExtWidget("Generalized friction force law",new ChoiceWidget2(new FrictionForceLawWidgetFactory(this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedFrictionForceLaw");
    addToTab("Kinetics", frictionForceLaw);

    frictionImpactLaw = new ExtWidget("Generalized friction impact law",new ChoiceWidget2(new FrictionImpactLawWidgetFactory(this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"generalizedFrictionImpactLaw");
    addToTab("Kinetics", frictionImpactLaw);

    normalForceFunction = new ExtWidget("Generalized normal force function",new ChoiceWidget2(new FunctionWidgetFactory2(friction,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedNormalForceFunction");
    addToTab("Kinetics",normalForceFunction);
  }

  DOMElement* GeneralizedFrictionPropertyDialog::initializeUsingXML(DOMElement *parent) {
    DualRigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    frictionForceLaw->initializeUsingXML(item->getXMLElement());
    frictionImpactLaw->initializeUsingXML(item->getXMLElement());
    normalForceFunction->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedFrictionPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DualRigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frictionForceLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    normalForceFunction->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }


  GeneralizedGearPropertyDialog::GeneralizedGearPropertyDialog(Element *link) : RigidBodyLinkPropertyDialog(link) {
    addTab("Kinetics",1);
    addTab("Visualization",2);

    gearOutput = new ExtWidget("Gear output",new ElementOfReferenceWidget<RigidBody>(link,nullptr,this),false,false,MBSIM%"gearOutput");
    addToTab("General",gearOutput);

    gearInput = new ExtWidget("Gear inputs",new ListWidget(new ElementOfReferenceWidgetFactory<RigidBody>(MBSIM%"gearInput",link,true,this),"Gear input",0,2),false,false,"");
    addToTab("General",gearInput);

    function = new ExtWidget("Generalized force law",new ChoiceWidget2(new GeneralizedForceLawWidgetFactory,QBoxLayout::TopToBottom,0),true,false,MBSIM%"generalizedForceLaw");
    addToTab("Kinetics",function);
  }

  DOMElement* GeneralizedGearPropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    gearOutput->initializeUsingXML(item->getXMLElement());
    gearInput->initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedGearPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    gearOutput->writeXMLFile(item->getXMLElement(),ref);
    gearInput->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedElasticConnectionPropertyDialog::GeneralizedElasticConnectionPropertyDialog(Element *connection) : DualRigidBodyLinkPropertyDialog(connection) {

    function = new ExtWidget("Generalized force function",new ChoiceWidget2(new SpringDamperWidgetFactory(connection,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);

    connect(function,SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
    connect(connections->getWidget(),SIGNAL(widgetChanged()),this,SLOT(updateWidget()));
  }

  void GeneralizedElasticConnectionPropertyDialog::updateWidget() {
  }

  DOMElement* GeneralizedElasticConnectionPropertyDialog::initializeUsingXML(DOMElement *parent) {
    DualRigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedElasticConnectionPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DualRigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  GeneralizedElasticStructurePropertyDialog::GeneralizedElasticStructurePropertyDialog(Element *link) : RigidBodyLinkPropertyDialog(link) {
    addTab("Kinetics",1);

    rigidBody = new ExtWidget("Rigid bodies",new ListWidget(new ElementOfReferenceWidgetFactory<RigidBody>(MBSIM%"rigidBody",link,this),"Rigid body",0,2),false,false,"");
    addToTab("General",rigidBody);

    function = new ExtWidget("Generalized force function",new ChoiceWidget2(new SpringDamperWidgetFactory(link,true,this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"generalizedForceFunction");
    addToTab("Kinetics", function);
  }

  DOMElement* GeneralizedElasticStructurePropertyDialog::initializeUsingXML(DOMElement *parent) {
    RigidBodyLinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    rigidBody->initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GeneralizedElasticStructurePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    RigidBodyLinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    rigidBody->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ContactPropertyDialog::ContactPropertyDialog(Element *contact) : LinkPropertyDialog(contact) {

    addTab("Kinetics",1);
    addTab("Extra");

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Contour>(2,contact,this),false,false,MBSIM%"connect");
    addToTab("Kinetics", connections);

    contactForceLaw = new ExtWidget("Normal force law",new ChoiceWidget2(new GeneralizedForceLawWidgetFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"normalForceLaw");
    addToTab("Kinetics", contactForceLaw);

    contactImpactLaw = new ExtWidget("Normal impact law",new ChoiceWidget2(new GeneralizedImpactLawWidgetFactory,QBoxLayout::TopToBottom,0),true,false,MBSIM%"normalImpactLaw");
    addToTab("Kinetics", contactImpactLaw);

    frictionForceLaw = new ExtWidget("Tangential force law",new ChoiceWidget2(new FrictionForceLawWidgetFactory(this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"tangentialForceLaw");
    addToTab("Kinetics", frictionForceLaw);

    frictionImpactLaw = new ExtWidget("Tangential impact law",new ChoiceWidget2(new FrictionImpactLawWidgetFactory(this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"tangentialImpactLaw");
    addToTab("Kinetics", frictionImpactLaw);

    searchAllContactPoints = new ExtWidget("Search all contact points",new ChoiceWidget2(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"searchAllContactPoints");
    addToTab("Extra", searchAllContactPoints);

    initialGuess = new ExtWidget("Initial guess",new ChoiceWidget2(new MatRowsColsVarWidgetFactory(0,0),QBoxLayout::RightToLeft,5),true,false,MBSIM%"initialGuess");
    addToTab("Extra", initialGuess);

    tolerance = new ExtWidget("Tolerance",new ChoiceWidget2(new ScalarWidgetFactory("1e-10"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"tolerance");
    addToTab("Extra", tolerance);

    maxNumContacts = new ExtWidget("Maximum number of contacts",new ChoiceWidget2(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),true,false,MBSIM%"maximumNumberOfContacts");
    addToTab("Extra", maxNumContacts);
  }

  DOMElement* ContactPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    contactForceLaw->initializeUsingXML(item->getXMLElement());
    contactImpactLaw->initializeUsingXML(item->getXMLElement());
    frictionForceLaw->initializeUsingXML(item->getXMLElement());
    frictionImpactLaw->initializeUsingXML(item->getXMLElement());
    searchAllContactPoints->initializeUsingXML(item->getXMLElement());
    initialGuess->initializeUsingXML(item->getXMLElement());
    tolerance->initializeUsingXML(item->getXMLElement());
    maxNumContacts->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ContactPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    contactForceLaw->writeXMLFile(item->getXMLElement(),ref);
    contactImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionForceLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    searchAllContactPoints->writeXMLFile(item->getXMLElement(),ref);
    initialGuess->writeXMLFile(item->getXMLElement(),ref);
    tolerance->writeXMLFile(item->getXMLElement(),ref);
    maxNumContacts->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DiskContactPropertyDialog::DiskContactPropertyDialog(Element *contact) : LinkPropertyDialog(contact) {

    addTab("Kinetics",1);

    connections = new ExtWidget("Connections",new ConnectElementsWidget<Contour>(2,contact,this),false,false,MBSIM%"connect");
    addToTab("Kinetics", connections);

    contactForceLaw = new ExtWidget("Normal force law",new ChoiceWidget2(new GeneralizedForceLawWidgetFactory,QBoxLayout::TopToBottom,0),false,false,MBSIM%"normalForceLaw");
    addToTab("Kinetics", contactForceLaw);

    contactImpactLaw = new ExtWidget("Normal impact law",new ChoiceWidget2(new GeneralizedImpactLawWidgetFactory,QBoxLayout::TopToBottom,0),true,false,MBSIM%"normalImpactLaw");
    addToTab("Kinetics", contactImpactLaw);

    frictionForceLaw = new ExtWidget("Tangential force law",new ChoiceWidget2(new FrictionForceLawWidgetFactory(this),QBoxLayout::TopToBottom,0),false,false,MBSIM%"tangentialForceLaw");
    addToTab("Kinetics", frictionForceLaw);

    frictionImpactLaw = new ExtWidget("Tangential impact law",new ChoiceWidget2(new FrictionImpactLawWidgetFactory(this),QBoxLayout::TopToBottom,0),true,false,MBSIM%"tangentialImpactLaw");
    addToTab("Kinetics", frictionImpactLaw);
  }

  DOMElement* DiskContactPropertyDialog::initializeUsingXML(DOMElement *parent) {
    LinkPropertyDialog::initializeUsingXML(item->getXMLElement());
    connections->initializeUsingXML(item->getXMLElement());
    contactForceLaw->initializeUsingXML(item->getXMLElement());
    contactImpactLaw->initializeUsingXML(item->getXMLElement());
    frictionForceLaw->initializeUsingXML(item->getXMLElement());
    frictionImpactLaw->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DiskContactPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    LinkPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    connections->writeXMLFile(item->getXMLElement(),ref);
    contactForceLaw->writeXMLFile(item->getXMLElement(),ref);
    contactImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionForceLaw->writeXMLFile(item->getXMLElement(),ref);
    frictionImpactLaw->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ObserverPropertyDialog::ObserverPropertyDialog(Element *observer) : ElementPropertyDialog(observer) {
  }

  KinematicCoordinatesObserverPropertyDialog::KinematicCoordinatesObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    frame = new ExtWidget("Frame",new ElementOfReferenceWidget<Frame>(observer,nullptr,this),false,false,MBSIM%"frame");
    addToTab("General", frame);

    frameOfReference = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(observer,nullptr,this),true,false,MBSIM%"frameOfReference");
    addToTab("General", frameOfReference);

    position = new ExtWidget("Enable openMBV position",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVPosition");
    addToTab("Visualization",position);

    velocity = new ExtWidget("Enable openMBV velocity",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVVelocity");
    addToTab("Visualization",velocity);

    acceleration = new ExtWidget("Enable openMBV acceleration",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAcceleration");
    addToTab("Visualization",acceleration);
  }

  DOMElement* KinematicCoordinatesObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    frame->initializeUsingXML(item->getXMLElement());
    frameOfReference->initializeUsingXML(item->getXMLElement());
    position->initializeUsingXML(item->getXMLElement());
    velocity->initializeUsingXML(item->getXMLElement());
    acceleration->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* KinematicCoordinatesObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frame->writeXMLFile(item->getXMLElement(),ref);
    frameOfReference->writeXMLFile(item->getXMLElement(),ref);
    position->writeXMLFile(item->getXMLElement(),ref);
    velocity->writeXMLFile(item->getXMLElement(),ref);
    acceleration->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RelativeKinematicsObserverPropertyDialog::RelativeKinematicsObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    frame = new ExtWidget("Frame",new ElementOfReferenceWidget<Frame>(observer,nullptr,this),false,false,MBSIM%"frame");
    addToTab("General", frame);

    refFrame = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(observer,nullptr,this),true,false,MBSIM%"frameOfReference");
    addToTab("General", refFrame);

    position = new ExtWidget("Enable openMBV position",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVPosition");
    addToTab("Visualization",position);

    velocity = new ExtWidget("Enable openMBV velocity",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVVelocity");
    addToTab("Visualization",velocity);

    angularVelocity = new ExtWidget("Enable openMBV angular velocity",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAngularVelocity");
    addToTab("Visualization",angularVelocity);

    acceleration = new ExtWidget("Enable openMBV acceleration",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAcceleration");
    addToTab("Visualization",acceleration);

    angularAcceleration = new ExtWidget("Enable openMBV angular acceleration",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAngularAcceleration");
    addToTab("Visualization",angularAcceleration);
  }

  DOMElement* RelativeKinematicsObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    frame->initializeUsingXML(item->getXMLElement());
    refFrame->initializeUsingXML(item->getXMLElement());
    position->initializeUsingXML(item->getXMLElement());
    velocity->initializeUsingXML(item->getXMLElement());
    angularVelocity->initializeUsingXML(item->getXMLElement());
    acceleration->initializeUsingXML(item->getXMLElement());
    angularAcceleration->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RelativeKinematicsObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frame->writeXMLFile(item->getXMLElement(),ref);
    refFrame->writeXMLFile(item->getXMLElement(),ref);
    position->writeXMLFile(item->getXMLElement(),ref);
    velocity->writeXMLFile(item->getXMLElement(),ref);
    angularVelocity->writeXMLFile(item->getXMLElement(),ref);
    acceleration->writeXMLFile(item->getXMLElement(),ref);
    angularAcceleration->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  MechanicalLinkObserverPropertyDialog::MechanicalLinkObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    link = new ExtWidget("Mechanical link",new ElementOfReferenceWidget<Link>(observer,nullptr,this),false,false,MBSIM%"mechanicalLink");
    addToTab("General", link);

    forceArrow = new ExtWidget("Enable openMBV force",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVForce");
    addToTab("Visualization",forceArrow);

    momentArrow = new ExtWidget("Enable openMBV moment",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVMoment");
    addToTab("Visualization",momentArrow);
  }

  DOMElement* MechanicalLinkObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    link->initializeUsingXML(item->getXMLElement());
    forceArrow->initializeUsingXML(item->getXMLElement());
    momentArrow->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MechanicalLinkObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    link->writeXMLFile(item->getXMLElement(),ref);
    forceArrow->writeXMLFile(item->getXMLElement(),ref);
    momentArrow->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  MechanicalConstraintObserverPropertyDialog::MechanicalConstraintObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    constraint = new ExtWidget("Mechanical constraint",new ElementOfReferenceWidget<Constraint>(observer,nullptr,this),false,false,MBSIM%"mechanicalConstraint");
    addToTab("General", constraint);

    forceArrow = new ExtWidget("Enable openMBV force",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVForce");
    addToTab("Visualization",forceArrow);

    momentArrow = new ExtWidget("Enable openMBV moment",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVMoment");
    addToTab("Visualization",momentArrow);
  }

  DOMElement* MechanicalConstraintObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    constraint->initializeUsingXML(item->getXMLElement());
    forceArrow->initializeUsingXML(item->getXMLElement());
    momentArrow->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MechanicalConstraintObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    constraint->writeXMLFile(item->getXMLElement(),ref);
    forceArrow->writeXMLFile(item->getXMLElement(),ref);
    momentArrow->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ContactObserverPropertyDialog::ContactObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    link = new ExtWidget("Mechanical link",new ElementOfReferenceWidget<Link>(observer,nullptr,this),false,false,MBSIM%"contact");
    addToTab("General", link);

    forceArrow = new ExtWidget("Enable openMBV force",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVForce");
    addToTab("Visualization",forceArrow);

    momentArrow = new ExtWidget("Enable openMBV force",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVMoment");
    addToTab("Visualization",momentArrow);

    contactPoints = new ExtWidget("Enable openMBV contact points",new FrameMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVContactPoints");
    addToTab("Visualization",contactPoints);

    normalForceArrow = new ExtWidget("Enable openMBV normal force",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVNormalForce");
    addToTab("Visualization",normalForceArrow);

    frictionArrow = new ExtWidget("Enable openMBV tangential force",new FrictionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVTangentialForce");
    addToTab("Visualization",frictionArrow);
  }

  DOMElement* ContactObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    link->initializeUsingXML(item->getXMLElement());
    forceArrow->initializeUsingXML(item->getXMLElement());
    momentArrow->initializeUsingXML(item->getXMLElement());
    contactPoints->initializeUsingXML(item->getXMLElement());
    normalForceArrow->initializeUsingXML(item->getXMLElement());
    frictionArrow->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ContactObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    link->writeXMLFile(item->getXMLElement(),ref);
    forceArrow->writeXMLFile(item->getXMLElement(),ref);
    momentArrow->writeXMLFile(item->getXMLElement(),ref);
    contactPoints->writeXMLFile(item->getXMLElement(),ref);
    normalForceArrow->writeXMLFile(item->getXMLElement(),ref);
    frictionArrow->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  FrameObserverPropertyDialog::FrameObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    frame = new ExtWidget("Frame",new ElementOfReferenceWidget<Frame>(observer,nullptr,this),false,false,MBSIM%"frame");
    addToTab("General", frame);

    position = new ExtWidget("Enable openMBV position",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVPosition");
    addToTab("Visualization",position);

    velocity = new ExtWidget("Enable openMBV velocity",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVVelocity");
    addToTab("Visualization",velocity);

    angularVelocity = new ExtWidget("Enable openMBV angular velocity",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAngularVelocity");
    addToTab("Visualization",angularVelocity);

    acceleration = new ExtWidget("Enable openMBV acceleration",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAcceleration");
    addToTab("Visualization",acceleration);

    angularAcceleration = new ExtWidget("Enable openMBV angular acceleration",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAngularAcceleration");
    addToTab("Visualization",angularAcceleration);
  }

  DOMElement* FrameObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    frame->initializeUsingXML(item->getXMLElement());
    position->initializeUsingXML(item->getXMLElement());
    velocity->initializeUsingXML(item->getXMLElement());
    angularVelocity->initializeUsingXML(item->getXMLElement());
    acceleration->initializeUsingXML(item->getXMLElement());
    angularAcceleration->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FrameObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frame->writeXMLFile(item->getXMLElement(),ref);
    position->writeXMLFile(item->getXMLElement(),ref);
    velocity->writeXMLFile(item->getXMLElement(),ref);
    angularVelocity->writeXMLFile(item->getXMLElement(),ref);
    acceleration->writeXMLFile(item->getXMLElement(),ref);
    angularAcceleration->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodyObserverPropertyDialog::RigidBodyObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    body = new ExtWidget("Rigid body",new ElementOfReferenceWidget<RigidBody>(observer,nullptr,this),false,false,MBSIM%"rigidBody");
    addToTab("General", body);

    frameOfReference = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(observer,nullptr,this),true,false,MBSIM%"frameOfReference");
    addToTab("General", frameOfReference);

    weight = new ExtWidget("Enable openMBV weight",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVWeight");
    addToTab("Visualization",weight);

    jointForce = new ExtWidget("Enable openMBV joint force",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVJointForce");
    addToTab("Visualization",jointForce);

    jointMoment = new ExtWidget("Enable openMBV joint moment",new InteractionArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVJointMoment");
    addToTab("Visualization",jointMoment);

    axisOfRotation = new ExtWidget("Enable openMBV axis of rotation",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAxisOfRotation");
    addToTab("Visualization",axisOfRotation);

    momentum = new ExtWidget("Enable openMBV momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVMomentum");
    addToTab("Visualization",momentum);

    angularMomentum = new ExtWidget("Enable openMBV angular momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAngularMomentum");
    addToTab("Visualization",angularMomentum);

    derivativeOfMomentum = new ExtWidget("Enable openMBV derivative of momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVDerivativeOfMomentum");
    addToTab("Visualization",derivativeOfMomentum);

    derivativeOfAngularMomentum = new ExtWidget("Enable openMBV derivative of angular momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVDerivativeOfAngularMomentum");
    addToTab("Visualization",derivativeOfAngularMomentum);
  }

  DOMElement* RigidBodyObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    body->initializeUsingXML(item->getXMLElement());
    frameOfReference->initializeUsingXML(item->getXMLElement());
    weight->initializeUsingXML(item->getXMLElement());
    jointForce->initializeUsingXML(item->getXMLElement());
    jointMoment->initializeUsingXML(item->getXMLElement());
    axisOfRotation->initializeUsingXML(item->getXMLElement());
    momentum->initializeUsingXML(item->getXMLElement());
    angularMomentum->initializeUsingXML(item->getXMLElement());
    derivativeOfMomentum->initializeUsingXML(item->getXMLElement());
    derivativeOfAngularMomentum->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodyObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    body->writeXMLFile(item->getXMLElement(),ref);
    frameOfReference->writeXMLFile(item->getXMLElement(),ref);
    weight->writeXMLFile(item->getXMLElement(),ref);
    jointForce->writeXMLFile(item->getXMLElement(),ref);
    jointMoment->writeXMLFile(item->getXMLElement(),ref);
    axisOfRotation->writeXMLFile(item->getXMLElement(),ref);
    momentum->writeXMLFile(item->getXMLElement(),ref);
    angularMomentum->writeXMLFile(item->getXMLElement(),ref);
    derivativeOfMomentum->writeXMLFile(item->getXMLElement(),ref);
    derivativeOfAngularMomentum->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  RigidBodySystemObserverPropertyDialog::RigidBodySystemObserverPropertyDialog(Element *observer) : ObserverPropertyDialog(observer) {

    addTab("Visualization",1);

    bodies = new ExtWidget("Rigid bodies",new ListWidget(new ElementOfReferenceWidgetFactory<RigidBody>(MBSIM%"rigidBody",observer,this),"Rigid body",0,2),false,false,"");
    addToTab("General", bodies);

    frameOfReference = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(observer,nullptr,this),true,false,MBSIM%"frameOfReference");
    addToTab("General", frameOfReference);

    position = new ExtWidget("Enable openMBV position",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVPosition");
    addToTab("Visualization",position);

    velocity = new ExtWidget("Enable openMBV velocity",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVVelocity");
    addToTab("Visualization",velocity);

    acceleration = new ExtWidget("Enable openMBV acceleration",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAcceleration");
    addToTab("Visualization",acceleration);

    weight = new ExtWidget("Enable openMBV weight",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVWeight");
    addToTab("Visualization",weight);

    momentum = new ExtWidget("Enable openMBV momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVMomentum");
    addToTab("Visualization",momentum);

    angularMomentum = new ExtWidget("Enable openMBV angular momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVAngularMomentum");
    addToTab("Visualization",angularMomentum);

    derivativeOfMomentum = new ExtWidget("Enable openMBV derivative of momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVDerivativeOfMomentum");
    addToTab("Visualization",derivativeOfMomentum);

    derivativeOfAngularMomentum = new ExtWidget("Enable openMBV derivative of angular momentum",new ArrowMBSOMBVWidget,true,false,MBSIM%"enableOpenMBVDerivativeOfAngularMomentum");
    addToTab("Visualization",derivativeOfAngularMomentum);

  }

  DOMElement* RigidBodySystemObserverPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ObserverPropertyDialog::initializeUsingXML(item->getXMLElement());
    bodies->initializeUsingXML(item->getXMLElement());
    frameOfReference->initializeUsingXML(item->getXMLElement());
    position->initializeUsingXML(item->getXMLElement());
    velocity->initializeUsingXML(item->getXMLElement());
    acceleration->initializeUsingXML(item->getXMLElement());
    weight->initializeUsingXML(item->getXMLElement());
    momentum->initializeUsingXML(item->getXMLElement());
    angularMomentum->initializeUsingXML(item->getXMLElement());
    derivativeOfMomentum->initializeUsingXML(item->getXMLElement());
    derivativeOfAngularMomentum->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* RigidBodySystemObserverPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ObserverPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    bodies->writeXMLFile(item->getXMLElement(),ref);
    frameOfReference->writeXMLFile(item->getXMLElement(),ref);
    position->writeXMLFile(item->getXMLElement(),ref);
    velocity->writeXMLFile(item->getXMLElement(),ref);
    acceleration->writeXMLFile(item->getXMLElement(),ref);
    weight->writeXMLFile(item->getXMLElement(),ref);
    momentum->writeXMLFile(item->getXMLElement(),ref);
    angularMomentum->writeXMLFile(item->getXMLElement(),ref);
    derivativeOfMomentum->writeXMLFile(item->getXMLElement(),ref);
    derivativeOfAngularMomentum->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  SignalPropertyDialog::SignalPropertyDialog(Element *signal) : LinkPropertyDialog(signal) {
  }

  SensorPropertyDialog::SensorPropertyDialog(Element *sensor) : SignalPropertyDialog(sensor) {
  }

  ObjectSensorPropertyDialog::ObjectSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    object = new ExtWidget("Object of reference",new ElementOfReferenceWidget<Object>(sensor,nullptr,this),false,false,MBSIMCONTROL%"object");
    addToTab("General", object);
  }

  DOMElement* ObjectSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    object->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ObjectSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    object->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  LinkSensorPropertyDialog::LinkSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    link = new ExtWidget("Link of reference",new ElementOfReferenceWidget<Link>(sensor,nullptr,this),false,false,MBSIMCONTROL%"link");
    addToTab("General", link);
  }

  DOMElement* LinkSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    link->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LinkSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    link->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  FrameSensorPropertyDialog::FrameSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    frame = new ExtWidget("Frame of reference",new ElementOfReferenceWidget<Frame>(sensor,nullptr,this),false,false,MBSIMCONTROL%"frame");
    addToTab("General", frame);
  }

  DOMElement* FrameSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    frame->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FrameSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    frame->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  FunctionSensorPropertyDialog::FunctionSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    function = new ExtWidget("Function",new ChoiceWidget2(new FunctionWidgetFactory2(sensor,false,this),QBoxLayout::TopToBottom,0),false,false,MBSIMCONTROL%"function");
    addToTab("General", function);
  }

  DOMElement* FunctionSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SensorPropertyDialog::initializeUsingXML(item->getXMLElement());
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* FunctionSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SensorPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ContactSensorPropertyDialog::ContactSensorPropertyDialog(Element *sensor) : SensorPropertyDialog(sensor) {
    contact = new ExtWidget("Contact of reference",new ElementOfReferenceWidget<Contact>(sensor,nullptr,this),false,false,MBSIMCONTROL%"contact");
    addToTab("General", contact);
  }

  DOMElement* ContactSensorPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    contact->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ContactSensorPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    contact->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }


  MultiplexerPropertyDialog::MultiplexerPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signal",new ListWidget(new ElementOfReferenceWidgetFactory<Signal>(MBSIMCONTROL%"inputSignal",signal,this),"Signal",0,2),false,false,"");
    addToTab("General", inputSignal);
  }

  DOMElement* MultiplexerPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* MultiplexerPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  DemultiplexerPropertyDialog::DemultiplexerPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);
    indices = new ExtWidget("Indices",new ChoiceWidget2(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"indices");
    addToTab("General", indices);
  }

  DOMElement* DemultiplexerPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    indices->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* DemultiplexerPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    indices->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  LinearTransferSystemPropertyDialog::LinearTransferSystemPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);

    A = new ExtWidget("System matrix",new ChoiceWidget2(new SqrMatSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"systemMatrix");
    addToTab("General", A);

    B = new ExtWidget("Input matrix",new ChoiceWidget2(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"inputMatrix");
    addToTab("General", B);

    C = new ExtWidget("Output matrix",new ChoiceWidget2(new MatWidgetFactory(1,1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"outputMatrix");
    addToTab("General", C);

    D = new ExtWidget("Feedthrough matrix",new ChoiceWidget2(new SqrMatSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMCONTROL%"feedthroughMatrix");
    addToTab("General", D);
  }

  void LinearTransferSystemPropertyDialog::updateWidget() {
    int sizeA = A->isActive()?static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(A->getWidget())->getWidget())->rows():0;
    int sizeD = D->isActive()?static_cast<PhysicalVariableWidget*>(static_cast<ChoiceWidget2*>(D->getWidget())->getWidget())->rows():0;
    B->resize_(sizeA,sizeD);
    C->resize_(sizeD,sizeA);
  }

  DOMElement* LinearTransferSystemPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    A->initializeUsingXML(item->getXMLElement());
    B->initializeUsingXML(item->getXMLElement());
    C->initializeUsingXML(item->getXMLElement());
    D->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* LinearTransferSystemPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    A->writeXMLFile(item->getXMLElement(),ref);
    B->writeXMLFile(item->getXMLElement(),ref);
    C->writeXMLFile(item->getXMLElement(),ref);
    D->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  SignalOperationPropertyDialog::SignalOperationPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signal",new ListWidget(new ElementOfReferenceWidgetFactory<Signal>(MBSIMCONTROL%"inputSignal",signal,this),"Signal",1,2,false,1,2),false,false,"");
    addToTab("General", inputSignal);

    function = new ExtWidget("Function",new ChoiceWidget2(new SymbolicFunctionWidgetFactory3(signal,QStringList("x"),1,false),QBoxLayout::TopToBottom,0),false,false,MBSIMCONTROL%"function");
    addToTab("General", function);

    connect(inputSignal,SIGNAL(widgetChanged()),this,SLOT(updateFunctionFactory()));
  }

  void SignalOperationPropertyDialog::updateWidget() {
    function->updateWidget();
  }

  void SignalOperationPropertyDialog::updateFunctionFactory(bool defineWidget) {
    if(static_cast<ListWidget*>(inputSignal->getWidget())->getSize()==1)
      static_cast<ChoiceWidget2*>(function->getWidget())->setWidgetFactory(new SymbolicFunctionWidgetFactory3(getElement(),QStringList("x"),1,false));
    else
      static_cast<ChoiceWidget2*>(function->getWidget())->setWidgetFactory(new SymbolicFunctionWidgetFactory2(getElement(),QStringList("x")<<"y",1,false));
    if(defineWidget)
      static_cast<ChoiceWidget2*>(function->getWidget())->defineWidget(0);
  }

  DOMElement* SignalOperationPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    updateFunctionFactory(false);
    function->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* SignalOperationPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    function->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ExternSignalSourcePropertyDialog::ExternSignalSourcePropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    sourceSize = new ExtWidget("Length of input vector",new SpinBoxWidget(1,1,1000),false,false,MBSIMCONTROL%"sourceSize");
    addToTab("General", sourceSize);
  }

  DOMElement* ExternSignalSourcePropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    sourceSize->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ExternSignalSourcePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    sourceSize->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  ExternSignalSinkPropertyDialog::ExternSignalSinkPropertyDialog(Element *signal) : SignalPropertyDialog(signal) {
    inputSignal = new ExtWidget("Input signal",new ElementOfReferenceWidget<Signal>(signal,nullptr,this),false,false,MBSIMCONTROL%"inputSignal");
    addToTab("General", inputSignal);
  }

  DOMElement* ExternSignalSinkPropertyDialog::initializeUsingXML(DOMElement *parent) {
    SignalPropertyDialog::initializeUsingXML(item->getXMLElement());
    inputSignal->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ExternSignalSinkPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    SignalPropertyDialog::writeXMLFile(item->getXMLElement(),ref);
    inputSignal->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

}
