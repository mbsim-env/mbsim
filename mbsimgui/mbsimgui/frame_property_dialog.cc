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
#include "frame_property_dialog.h"
#include "variable_widgets.h"
#include "ombv_widgets.h"
#include "frame.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  FramePropertyDialog::FramePropertyDialog(Element *frame) : ElementPropertyDialog(frame) {
    addTab("Visualization",1);
    visu = new ExtWidget("Enable openMBV",new FrameMBSOMBVWidget,true,true,MBSIM%"enableOpenMBV");
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
    addTab("Visualization",1);
    visu = new ExtWidget("Enable openMBV",new FrameMBSOMBVWidget,true,true,static_cast<InternalFrame*>(frame)->getXMLFrameName());
    addToTab("Visualization", visu);
    static_cast<TextWidget*>(name->getWidget())->setReadOnly(true);
    static_cast<TextWidget*>(name->getWidget())->setText(frame->getName());
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

    position = new ExtWidget("Relative position",new ChoiceWidget(new VecWidgetFactory(3,vector<QStringList>(3,lengthUnits()),vector<int>(3,4)),QBoxLayout::RightToLeft,5),true,false,MBSIM%"relativePosition");
    addToTab("Kinematics", position);

    orientation = new ExtWidget("Relative orientation",new ChoiceWidget(new RotMatWidgetFactory,QBoxLayout::RightToLeft,5),true,false,MBSIM%"relativeOrientation");
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

    nodeNumber = new ExtWidget("Node number",new ChoiceWidget(new ScalarWidgetFactory("1"),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"nodeNumber");
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

  InterfaceNodeFramePropertyDialog::InterfaceNodeFramePropertyDialog(Element *frame, bool approx_) : FramePropertyDialog(frame), approx(nullptr) {

    nodeNumbers = new ExtWidget("Node numbers",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),false,false,MBSIMFLEX%"nodeNumbers");
    addToTab("General", nodeNumbers);

    weightingFactors = new ExtWidget("Weighting factors",new ChoiceWidget(new VecSizeVarWidgetFactory(1),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"weightingFactors");
    addToTab("General", weightingFactors);

    if(approx_) {
      approx = new ExtWidget("Approximate shape matrix of rotation",new ChoiceWidget(new BoolWidgetFactory("0"),QBoxLayout::RightToLeft,5),true,false,MBSIMFLEX%"approximateShapeMatrixOfRotation");
      addToTab("General", approx);
    }
  }

  DOMElement* InterfaceNodeFramePropertyDialog::initializeUsingXML(DOMElement *parent) {
    FramePropertyDialog::initializeUsingXML(item->getXMLElement());
    nodeNumbers->initializeUsingXML(item->getXMLElement());
    weightingFactors->initializeUsingXML(item->getXMLElement());
    if(approx) approx->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* InterfaceNodeFramePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    FramePropertyDialog::writeXMLFile(item->getXMLElement(),nullptr);
    nodeNumbers->writeXMLFile(item->getXMLElement(),nullptr);
    weightingFactors->writeXMLFile(item->getXMLElement(),nullptr);
    if(approx) approx->writeXMLFile(item->getXMLElement(),nullptr);
    return nullptr;
  }

}
