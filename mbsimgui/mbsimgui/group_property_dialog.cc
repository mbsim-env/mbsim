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
#include "group_property_dialog.h"
#include "basic_widgets.h"
#include "extended_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  GroupPropertyDialog::GroupPropertyDialog(Element *group, bool kinematics) : ElementPropertyDialog(group), frameOfReference(nullptr) {
    if(kinematics) {
      addTab("Kinematics",1);

      frameOfReference = new ExtWidget("Frame of reference",new ParentFrameOfReferenceWidget(group,nullptr),true,false,MBSIM%"frameOfReference");
      addToTab("Kinematics", frameOfReference);
    }
  }

  DOMElement* GroupPropertyDialog::initializeUsingXML(DOMElement *parent) {
    ElementPropertyDialog::initializeUsingXML(item->getXMLElement());
    if(frameOfReference)
      frameOfReference->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* GroupPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    ElementPropertyDialog::writeXMLFile(parent,ref?ref:getElement()->getXMLFrames());
    if(frameOfReference)
      frameOfReference->writeXMLFile(item->getXMLElement(),ref?ref:getElement()->getXMLFrames());
    return nullptr;
  }

}
