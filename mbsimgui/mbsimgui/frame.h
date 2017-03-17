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
#include "extended_properties.h"

namespace MBSimGUI {

  class ExtWidget;

  class Frame : public Element {
    friend class FramePropertyDialog;
    public:
    Frame(const std::string &str, Element *parent, bool grey=true, const std::vector<MBXMLUtils::FQN> &plotFeatureTypes=std::vector<MBXMLUtils::FQN>());
    ~Frame() { }
    virtual PropertyInterface* clone() const {return new Frame(*this);}
    std::string getType() const { return "Frame"; }
    static Frame* readXMLFile(const std::string &filename, Element *parent);    
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual void initializeUsingXML2(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile2(xercesc::DOMNode *element);
    virtual void initializeUsingXML3(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile3(xercesc::DOMNode *element);
    bool openMBVFrame() const {return visu.isActive();}
    void setOpenMBVFrame(bool flag) {visu.setActive(flag);}
    ElementPropertyDialog* createPropertyDialog() {return new FramePropertyDialog(this);}
    EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this,false);}
    QMenu* createContextMenu() {return new FrameContextMenu(this);}
    const MBXMLUtils::FQN& getXMLName() const { return xmlName; }
    void setXMLName(const MBXMLUtils::FQN &name) { xmlName = name; }
    protected:
    ExtProperty visu;
    MBXMLUtils::FQN xmlName;
  };

  class InternalFrame : public Frame {
    friend class InternalFramePropertyDialog;
    public:
    InternalFrame(const std::string &str, Element *parent, const std::vector<MBXMLUtils::FQN> &plotFeatureTypes=std::vector<MBXMLUtils::FQN>()) : Frame(str,parent,true,plotFeatureTypes) { }
    virtual PropertyInterface* clone() const {return new InternalFrame(*this);}
    std::string getType() const { return "InternalFrame"; }
    ElementPropertyDialog* createPropertyDialog() {return new InternalFramePropertyDialog(this);}
    EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
  };

  class FixedRelativeFrame : public Frame {
    friend class FixedRelativeFramePropertyDialog;
    public:
    FixedRelativeFrame(const std::string &str, Element *parent);
    ~FixedRelativeFrame() { }
    virtual PropertyInterface* clone() const {return new FixedRelativeFrame(*this);}
    std::string getType() const { return "FixedRelativeFrame"; }
    virtual xercesc::DOMElement* processFileID(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    virtual void initialize();
    ElementPropertyDialog* createPropertyDialog() {return new FixedRelativeFramePropertyDialog(this);}
    EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
    QMenu* createContextMenu() {return new FixedRelativeFrameContextMenu(this);}
    protected:
    ExtProperty refFrame, position, orientation;
  };

  class NodeFrame : public Frame {
    friend class NodeFramePropertyDialog;
    public:
    NodeFrame(const std::string &str, Element *parent);
    ~NodeFrame() { }
    virtual PropertyInterface* clone() const {return new NodeFrame(*this);}
    std::string getType() const { return "NodeFrame"; }
    MBXMLUtils::NamespaceURI getNameSpace() const { return MBSIMFLEX; }
    virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
    virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    ElementPropertyDialog* createPropertyDialog() {return new NodeFramePropertyDialog(this);}
    EmbeddingPropertyDialog* createEmbeddingPropertyDialog() {return new EmbeddingPropertyDialog(this);}
    QMenu* createContextMenu() {return new NodeFrameContextMenu(this);}
    protected:
    ExtProperty nodeNumber;
  };

}

#endif
