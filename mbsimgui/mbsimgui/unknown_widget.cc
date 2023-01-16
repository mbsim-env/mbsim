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

#include <config.h>
#include "unknown_widget.h"
#include "basic_widgets.h"
#include "extended_widgets.h"
#include "kinetics_widgets.h"
#include "environment_widgets.h"
#include "function_widget.h"
#include "function_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

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
    DOMDocument *doc=parent->getOwnerDocument();
    xercesc::DOMElement *ele0 = MBXMLUtils::D(doc)->createElement(tagName);
    parent->insertBefore(ele0,ref);
    editor->writeXMLFile(ele0,ref);
    return ele0;
  }

  // explizit instantiation
  #define X(Type) \
    template class UnknownWidget<Type>;
  MBSIMGUI_WIDGET_CONTAINERS
  #undef X

}
