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

#ifndef _FRAME__H_
#define _FRAME__H_

#include "element.h"

namespace MBSimGUI {

  class ExtWidget;

  class Frame : public Element {
    public:
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      ElementPropertyDialog* createPropertyDialog() override { return new FramePropertyDialog(this); }
      QMenu* createContextMenu() override { return new FrameContextMenu(this); }
  };

  class UnknownFrame : public Frame {
    public:
      QString getType() const override { return "UnknownFrame"; }
      ElementPropertyDialog* createPropertyDialog() override {return new UnknownElementPropertyDialog(this);}
  };

  class InternalFrame : public Frame {
    public:
      InternalFrame(const QString &name_, MBXMLUtils::FQN xmlFrameName_, const QString &plotFeatureType_="");
      QString getName() const override { return name; }
      QString getType() const override { return "InternalFrame"; }
      ElementPropertyDialog* createPropertyDialog() override { return new InternalFramePropertyDialog(this); }
      QMenu* createContextMenu() override { return new ElementContextMenu(this,nullptr,false,false); }
      void removeXMLElements() override;
      const MBXMLUtils::FQN& getXMLFrameName() const { return xmlFrameName; }
      QString getPlotFeatureType() const override { return plotFeatureType; }
    protected:
      QString name;
      MBXMLUtils::FQN xmlFrameName;
      QString plotFeatureType;
  };

  class FixedRelativeFrame : public Frame {
    public:
      QString getType() const override { return "FixedRelativeFrame"; }
      ElementPropertyDialog* createPropertyDialog() override { return new FixedRelativeFramePropertyDialog(this); }
  };

  class NodeFrame : public Frame {
    public:
      QString getType() const override { return "NodeFrame"; }
      MBXMLUtils::NamespaceURI getNameSpace() const override { return MBSIMFLEX; }
      ElementPropertyDialog* createPropertyDialog() override { return new NodeFramePropertyDialog(this); }
  };

}

#endif
