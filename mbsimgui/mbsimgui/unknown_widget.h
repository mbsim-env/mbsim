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

#ifndef _UNKNOWN_WIDGET_H_
#define _UNKNOWN_WIDGET_H_

#include "widget.h"
#include "basic_widgets.h"

namespace MBSimGUI {

  class ExtWidget;

  template<class ContainerWidget>
  class UnknownWidget : public ContainerWidget {
    MBSIMGUI_OBJECTFACTORY_CLASS(UnknownWidget<ContainerWidget>, ContainerWidget,
      MBSIM%("Unknown"+ContainerWidget::getXMLTypeStatic().second), QString("Unknown ")+ContainerWidget::getTypeStatic());
    public:
      UnknownWidget();
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      MBXMLUtils::FQN tagName;
      ExtWidget *editor;
  };

  template<class ContainerWidget>
  UnknownWidget<ContainerWidget>::UnknownWidget() : tagName("http://www.mbsim-env.de/MBSimXML","Type") {
    auto *layout = new QVBoxLayout;
    layout->setMargin(0);
    this->setLayout(layout);
    editor = new ExtWidget("XML Editor",new XMLEditorWidget);
    layout->addWidget(editor);
  }

  template<class ContainerWidget>
  xercesc::DOMElement* UnknownWidget<ContainerWidget>::initializeUsingXML(xercesc::DOMElement *element) {
    tagName = MBXMLUtils::E(element)->getTagName();
    editor->initializeUsingXML(element);
    return element;
  }

  template<class ContainerWidget>
  xercesc::DOMElement* UnknownWidget<ContainerWidget>::writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref) {
    xercesc::DOMDocument *doc=parent->getOwnerDocument();
    xercesc::DOMElement *ele0 = MBXMLUtils::D(doc)->createElement(tagName);
    parent->insertBefore(ele0,ref);
    editor->writeXMLFile(ele0,ref);
    return ele0;
  }

}

#endif
