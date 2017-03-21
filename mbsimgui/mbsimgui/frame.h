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
      Frame(const std::string &str="");
      ~Frame() { }
      virtual PropertyInterface* clone() const {return new Frame(*this);}
      std::string getType() const { return "Frame"; }
      static Frame* readXMLFile(const std::string &filename);
      xercesc::DOMElement* processFileID(xercesc::DOMElement* element);
      ElementPropertyDialog* createPropertyDialog() {return new FramePropertyDialog(this);}
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this,false);}
      QMenu* createContextMenu() {return new FrameContextMenu(this);}
  };

  class InternalFrame : public Frame {
    public:
      InternalFrame(const std::string &str, const MBXMLUtils::FQN &xmlFrameName_, const std::string &plotFeatureType_="") : Frame(str), xmlFrameName(xmlFrameName_), plotFeatureType(plotFeatureType_) { }
      virtual PropertyInterface* clone() const {return new InternalFrame(*this);}
      std::string getType() const { return "InternalFrame"; }
      ElementPropertyDialog* createPropertyDialog() {return new InternalFramePropertyDialog(this);}
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
      void removeXMLElements();
      const MBXMLUtils::FQN& getXMLFrameName() const { return xmlFrameName; }
      std::string getPlotFeatureType() const { return plotFeatureType; }
    protected:
      MBXMLUtils::FQN xmlFrameName;
      std::string plotFeatureType;
  };

  class FixedRelativeFrame : public Frame {
    public:
      FixedRelativeFrame(const std::string &str="");
      ~FixedRelativeFrame() { }
      virtual PropertyInterface* clone() const {return new FixedRelativeFrame(*this);}
      std::string getType() const { return "FixedRelativeFrame"; }
      ElementPropertyDialog* createPropertyDialog() {return new FixedRelativeFramePropertyDialog(this);}
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
      QMenu* createContextMenu() {return new FixedRelativeFrameContextMenu(this);}
  };

  class NodeFrame : public Frame {
    public:
      NodeFrame(const std::string &str="");
      ~NodeFrame() { }
      virtual PropertyInterface* clone() const {return new NodeFrame(*this);}
      std::string getType() const { return "NodeFrame"; }
      MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIMFLEX; }
      ElementPropertyDialog* createPropertyDialog() {return new NodeFramePropertyDialog(this);}
      EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
      QMenu* createContextMenu() {return new NodeFrameContextMenu(this);}
  };

}

#endif
