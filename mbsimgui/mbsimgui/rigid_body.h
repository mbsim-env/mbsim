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

#ifndef _RIGID_BODY__H_
#define _RIGID_BODY__H_

#include "body.h"

namespace MBSimGUI {

  class RigidBody : public Body {
    public:
      RigidBody(const QString &str="");
      QString getType() const { return "RigidBody"; }
      xercesc::DOMElement* getXMLFrames() { return frames; }
      xercesc::DOMElement* getXMLContours() { return contours; }
      void removeXMLElements();
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      xercesc::DOMElement* processFileID(xercesc::DOMElement* element);
      xercesc::DOMElement* processHref(xercesc::DOMElement* element);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      bool isConstrained() const {return constrained;}
      void setConstrained(bool b) {constrained = b;}
      ElementPropertyDialog* createPropertyDialog() {return new RigidBodyPropertyDialog(this);}
      QMenu* createFrameContextMenu() {return new FixedRelativeFramesContextMenu(this);}
    protected:
      bool constrained;
      xercesc::DOMElement *frames, *contours;
  };

}

#endif
