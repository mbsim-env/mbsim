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

#ifndef _LINK__H_
#define _LINK__H_

#include "element.h"

namespace MBSimGUI {

  class Link : public Element {
    public:
      Link(const std::string &str, Element *parent); 
      static Link* readXMLFile(const std::string &filename, Element *parent);
      virtual int getxSize() {return 0;}
  };

  class MechanicalLink : public Link {
    friend class MechanicalLinkPropertyDialog;
    public:
      MechanicalLink(const std::string &str, Element *parent) : Link(str, parent) { }
  };

  class FrameLink : public MechanicalLink {
    friend class FrameLinkPropertyDialog;
    public:
      FrameLink(const std::string &str, Element *parent);
      void initialize();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    protected:
      ExtProperty connections;
  };

  class FixedFrameLink : public FrameLink {
    friend class FixedFrameLinkPropertyDialog;
    public:
      FixedFrameLink(const std::string &str, Element *parent) : FrameLink(str,parent) { }
 };

  class FloatingFrameLink : public FrameLink {
    friend class FloatingFrameLinkPropertyDialog;
    public:
      FloatingFrameLink(const std::string &str, Element *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    protected:
      ExtProperty refFrameID;
 };

  class RigidBodyLink : public MechanicalLink {
    friend class RigidBodyLinkPropertyDialog;
    public:
      RigidBodyLink(const std::string &str, Element *parent);
      void initialize();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    protected:
      ExtProperty support;
 };

  class DualRigidBodyLink : public RigidBodyLink {
    friend class DualRigidBodyLinkPropertyDialog;
    public:
      DualRigidBodyLink(const std::string &str, Element *parent);
      void initialize();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
    protected:
      ExtProperty connections;
 };

}

#endif
