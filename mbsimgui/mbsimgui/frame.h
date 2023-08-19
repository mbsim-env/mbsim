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

#ifndef _FRAME__H_
#define _FRAME__H_

#include "element.h"

namespace MBSimGUI {

  class ExtWidget;

  class Frame : public Element {
    MBSIMGUI_OBJECTFACTORY_CLASS(Frame, Element, MBSIM%"Frame", "Frame");
    public:
      Frame();
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      PropertyDialog* createPropertyDialog() override { return new FramePropertyDialog(this); }
      QMenu* createContextMenu() override { return new FrameContextMenu(this); }
  };

  class InternalFrame : public Frame {
    MBSIMGUI_OBJECTFACTORY_CLASS(InternalFrame, Frame, MBSIM%"InternalFrame", "Internal frame");
    public:
      InternalFrame(const QString &name_, MBXMLUtils::FQN xmlFrameName_, const MBXMLUtils::FQN &plotFeatureType_="");
      QString getName() const override { return name; }
      PropertyDialog* createPropertyDialog() override { return new InternalFramePropertyDialog(this); }
      QMenu* createContextMenu() override { return new ElementContextMenu(this,nullptr,false,false); }
      void removeXMLElements() override;
      const MBXMLUtils::FQN& getXMLFrameName() const { return xmlFrameName; }
      MBXMLUtils::FQN getPlotFeatureType() const override { return plotFeatureType; }
    protected:
      QString name;
      MBXMLUtils::FQN xmlFrameName;
      MBXMLUtils::FQN plotFeatureType;
  };

  class FixedRelativeFrame : public Frame {
    MBSIMGUI_OBJECTFACTORY_CLASS(FixedRelativeFrame, Frame, MBSIM%"FixedRelativeFrame", "Fixed relative frame");
    public:
      PropertyDialog* createPropertyDialog() override { return new FixedRelativeFramePropertyDialog(this); }
  };

  class UnknownFixedRelativeFrame : public FixedRelativeFrame {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownFixedRelativeFrame, FixedRelativeFrame, MBSIM%"UnknownFixedRelativeFrame_dummy", "Unknown fixed relative frame");
    public:
      UnknownFixedRelativeFrame();
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class NodeFrame : public Frame {
    MBSIMGUI_OBJECTFACTORY_CLASS(NodeFrame, Frame, MBSIMFLEX%"NodeFrame", "Node frame");
    public:
      PropertyDialog* createPropertyDialog() override { return new NodeFramePropertyDialog(this); }
  };

  class UnknownNodeFrame : public NodeFrame {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownNodeFrame, NodeFrame, MBSIM%"UnknownNodeFrame_dummy", "Unknown node frame");
    public:
      UnknownNodeFrame();
      PropertyDialog* createPropertyDialog() override { return new UnknownItemPropertyDialog(this); }
  };

  class InterfaceNodeFrame : public NodeFrame {
    MBSIMGUI_OBJECTFACTORY_CLASS(InterfaceNodeFrame, NodeFrame, MBSIMFLEX%"InterfaceNodeFrame", "Interface node Frame");
    public:
      PropertyDialog* createPropertyDialog() override { return new InterfaceNodeFramePropertyDialog(this); }
  };

  class FfrInterfaceNodeFrame : public NodeFrame {
    MBSIMGUI_OBJECTFACTORY_CLASS(FfrInterfaceNodeFrame, NodeFrame, MBSIMFLEX%"FfrInterfaceNodeFrame", "Ffr interface node Frame");
    public:
      PropertyDialog* createPropertyDialog() override { return new InterfaceNodeFramePropertyDialog(this,true); }
  };

}

#endif
