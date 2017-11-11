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
      xercesc::DOMElement* processFileID(xercesc::DOMElement* element);
      ElementPropertyDialog* createPropertyDialog() { return new FramePropertyDialog(this); }
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() { return new EmbeddingPropertyDialog(this,false); }
      QMenu* createContextMenu() { return new FrameContextMenu(this); }
  };

  class InternalFrame : public Frame {
    public:
      InternalFrame(const QString &name_, const MBXMLUtils::FQN &xmlFrameName_, const QString &plotFeatureType_="");
      QString getName() const { return name; }
      QString getType() const { return "InternalFrame"; }
      ElementPropertyDialog* createPropertyDialog() { return new InternalFramePropertyDialog(this); }
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() { return new EmbeddingPropertyDialog(this); }
      QMenu* createContextMenu() { return new ElementContextMenu(this,NULL,false); }
      void removeXMLElements();
      const MBXMLUtils::FQN& getXMLFrameName() const { return xmlFrameName; }
      QString getPlotFeatureType() const { return plotFeatureType; }
    protected:
      QString name;
      MBXMLUtils::FQN xmlFrameName;
      QString plotFeatureType;
  };

  class FixedRelativeFrame : public Frame {
    public:
      QString getType() const { return "FixedRelativeFrame"; }
      ElementPropertyDialog* createPropertyDialog() { return new FixedRelativeFramePropertyDialog(this); }
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() { return new EmbeddingPropertyDialog(this); }
  };

  class NodeFrame : public Frame {
    public:
      QString getType() const { return "NodeFrame"; }
      MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIMFLEX; }
      ElementPropertyDialog* createPropertyDialog() { return new NodeFramePropertyDialog(this); }
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() { return new EmbeddingPropertyDialog(this); }
  };

}

#endif
