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
#include "extended_properties.h"

namespace MBSimGUI {

  class RigidBody : public Body {
    friend class RigidBodyPropertyDialog;
    public:
      RigidBody(const std::string &str, Element *parent);
      virtual PropertyInterface* clone() const {return new RigidBody(*this);}
      std::string getType() const { return "RigidBody"; }
      int getqSize() const {return constrained?0:getqRelSize();}
      int getquize() const {return constrained?0:getuRelSize();}
      int getqRelSize() const;
      int getuRelSize() const;
      xercesc::DOMElement* getXMLFrames();
      xercesc::DOMElement* getXMLFrame();
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent);
      xercesc::DOMElement* processFileID(xercesc::DOMElement *element);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);
      bool isConstrained() const {return constrained;}
      void setConstrained(bool b) {constrained = b;}
      void initialize();
      ElementPropertyDialog* createPropertyDialog() {return new RigidBodyPropertyDialog(this);}
      QMenu* createContextMenu() {return new RigidBodyContextMenu(this);}
      QMenu* createFrameContextMenu() {return new FixedRelativeFrameContextContextMenu(this);}
    protected:
      bool constrained;
      ExtProperty K, mass, inertia, frameForInertiaTensor, translation, rotation, translationDependentRotation, coordinateTransformationForRotation, bodyFixedRepresentationOfAngularVelocity, ombvEditor;
  };

}

#endif
