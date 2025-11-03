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

#ifndef _ELEMENT_PROPERTY_DIALOG_H_
#define _ELEMENT_PROPERTY_DIALOG_H_

#include "property_dialog.h"
#include "plot_attribute_store.h"

namespace MBSimGUI {

  class ExtWidget;
  class CommentWidget;
  class MBSimGUIContextAction;
  class Element;

  class ElementPropertyDialog : public EmbedItemPropertyDialog {

    public:
      ElementPropertyDialog(Element *element);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      Element* getElement() const;
    protected:
      ExtWidget *name, *plotFeature;
      CommentWidget *comment;
      MBSimGUIContextAction *mbsimguiContextAction;
      std::unique_ptr<PlotAttributeStore> plotAttribute;
  };

}

#endif
