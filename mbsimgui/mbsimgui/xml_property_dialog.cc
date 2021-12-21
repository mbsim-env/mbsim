/*
  MBSimGUI - A fronted for MBSim.
  Copyright (C) 2020 Martin Förg

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
#include "xml_property_dialog.h"
#include "basic_widgets.h"
#include "parameter.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  XMLPropertyDialog::XMLPropertyDialog(EmbedItemData *item) : EmbedItemPropertyDialog(item) {
    addTab("General");
    editor = new ExtWidget("XML editor",new XMLEditorWidget);
    addToTab("General", editor);
  }

  DOMElement* XMLPropertyDialog::initializeUsingXML(DOMElement *parent) {

    editor->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* XMLPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {

    item->setXMLElement(editor->writeXMLFile(item->getXMLElement(),ref));
    return nullptr;
  }

  ParameterXMLPropertyDialog::ParameterXMLPropertyDialog(EmbedItemData *item) : EmbedItemPropertyDialog(item) {
    addTab("General");
    editor = new ExtWidget("XML editor",new XMLEditorWidget);
    addToTab("General", editor);
  }

  DOMElement* ParameterXMLPropertyDialog::initializeUsingXML(DOMElement *parent) {

    editor->initializeUsingXML(static_cast<DOMElement*>(item->getParameter(0)->getXMLElement()->getParentNode()));
    return parent;
  }

  DOMElement* ParameterXMLPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {

    editor->writeXMLFile(static_cast<DOMElement*>(item->getParameter(0)->getXMLElement()->getParentNode()));
    return nullptr;
  }

}
