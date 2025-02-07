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
#include "element_property_dialog.h"
#include "basic_widgets.h"
#include "extended_widgets.h"
#include <xercesc/dom/DOMProcessingInstruction.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ElementPropertyDialog::ElementPropertyDialog(Element *element) : EmbedItemPropertyDialog("Model-Element Properties", element) {
    addTab("General");
    name = new ExtWidget("Name",new TextWidget(item->getXMLElement()?QString::fromStdString(MBXMLUtils::E(item->getXMLElement())->getAttribute("name")):item->getName()));
    addToTab("General", name);
    addTab("Plot");
    plotFeature = new ExtWidget("Plot features",new PlotFeatureWidget(getElement()->getPlotFeatureType()));
    addToTab("Plot", plotFeature);
    plotAttribute = make_unique<PlotAttributeStore>();
    addTab("Comment");
    comment = new CommentWidget;
    addToTab("Comment", comment);
    addTab("Misc");
    mbsimguiContextAction = new MBSimGUIContextAction;
    addToTab("Misc", mbsimguiContextAction);
  }

  DOMElement* ElementPropertyDialog::initializeUsingXML(DOMElement *parent) {
    name->getWidget<TextWidget>()->setText(QString::fromStdString(MBXMLUtils::E(item->getXMLElement())->getAttribute("name")));
    comment->initializeUsingXML(item->getXMLElement());
    mbsimguiContextAction->initializeUsingXML(item->getXMLElement());
    plotFeature->initializeUsingXML(item->getXMLElement());
    plotAttribute->initializeUsingXML(item->getXMLElement());
    return parent;
  }

  DOMElement* ElementPropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    item->removeXMLElements();
    E(item->getXMLElement())->setAttribute("name",name->getWidget<TextWidget>()->getText().toStdString());
    comment->writeXMLFile(item->getXMLElement(),ref);
    mbsimguiContextAction->writeXMLFile(item->getXMLElement(),ref);
    item->updateName();
    plotFeature->writeXMLFile(item->getXMLElement(),ref);
    plotAttribute->writeXMLFile(item->getXMLElement(),ref);
    return nullptr;
  }

  Element* ElementPropertyDialog::getElement() const {
    return static_cast<Element*>(item);
  }

  vector<pair<string, string>> ElementPropertyDialog::getMBSimGUIContextActions(xercesc::DOMElement *parent) {
    // Model specific context actions
    vector<pair<string, string>> ret;

    // read all MBSIMGUI_CONTEXT_ACTION processing element instructions
    // (a mbsimgui context action is not part of the mbsimxml XML schema file -> that's why its a processing instructions)
    for(xercesc::DOMNode *pi=E(parent)->getFirstProcessingInstructionChildNamed("MBSIMGUI_CONTEXT_ACTION");
        pi!=nullptr; pi=pi->getNextSibling()) {
      // skip all none PI elements and PI elements with the wrong target
      if(pi->getNodeType()!=xercesc::DOMNode::PROCESSING_INSTRUCTION_NODE)
        continue;
      if(X()%static_cast<xercesc::DOMProcessingInstruction*>(pi)->getTarget()!="MBSIMGUI_CONTEXT_ACTION")
        continue;
      // get the name=... atttribute and skip this element if it does not exist
      auto data=X()%static_cast<xercesc::DOMProcessingInstruction*>(pi)->getData();
      std::string nameToken("name=\"");
      if(data.substr(0, nameToken.length())!=nameToken)
        continue;
      auto end=data.find("\" ", nameToken.length());
      if(end==std::string::npos) {
        end=data.find("\"\n", nameToken.length());
        if(end==std::string::npos)
          continue;
      }

      // add the context action
      std::string name=data.substr(nameToken.length(), end-nameToken.length());
      std::string code=data.substr(end+2);
      ret.emplace_back(std::move(name), std::move(code));
    }
    return ret;
  }

}
