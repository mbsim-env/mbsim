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

#ifndef _RIGID_BODY__H_
#define _RIGID_BODY__H_

#include "body.h"

namespace MBSimGUI {

  class RigidBody : public Body {
    MBSIMGUI_OBJECTFACTORY_CLASS(RigidBody, Body, MBSIM%"RigidBody", "Rigid body");
    public:
      RigidBody();
      xercesc::DOMElement* getXMLFrames() override { return frames; }
      xercesc::DOMElement* getXMLContours() override { return contours; }
      void removeXMLElements() override;
      xercesc::DOMElement* createXMLElement(xercesc::DOMNode *parent) override;
      xercesc::DOMElement* processIDAndHref(xercesc::DOMElement* element) override;
      void create() override;
      void clear() override;
      void setDedicatedFileItem(FileItemData *dedicatedFileItem) override;
      void setDedicatedParameterFileItem(FileItemData *dedicatedFileItem) override;
      PropertyDialog* createPropertyDialog() override { return new RigidBodyPropertyDialog(this); }
      QMenu* createFrameContextMenu() override { return new FixedRelativeFramesContextMenu(this); }
      void updateNames() override;
      void updateValues() override;
      void createDiagramItem() override;
      void updateDiagramItem() override;
    protected:
      xercesc::DOMElement *frames, *contours;
  };

}

#endif
