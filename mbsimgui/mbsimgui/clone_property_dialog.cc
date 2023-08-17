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
#include "clone_property_dialog.h"
#include "element.h"
#include "basic_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  ClonePropertyDialog::ClonePropertyDialog(Element *element) : EmbedItemPropertyDialog("Array/Pattern Properties", element) {
    addTab("General");
    name = new ExtWidget("Name",new TextWidget(QString::fromStdString(MBXMLUtils::E(item->getXMLElement())->getAttribute("name"))));
    name->setToolTip("Set the name of the element");
    addToTab("General", name);
    clone = new ExtWidget("Array/Pattern",new CloneWidget,true,false);
    connect(clone,&Widget::widgetChanged,this,&ClonePropertyDialog::updateName);
    addToTab("General",clone);
  }

  DOMElement* ClonePropertyDialog::initializeUsingXML(DOMElement *parent) {
    static_cast<TextWidget*>(name->getWidget())->setText(QString::fromStdString(MBXMLUtils::E(item->getXMLElement())->getAttribute("name")));
    DOMElement *embed = item->getEmbedXMLElement();
    if(embed) {
      clone->setActive(E(embed)->hasAttribute("count"));
      if(E(embed)->hasAttribute("count")) static_cast<CloneWidget*>(clone->getWidget())->setCount(QString::fromStdString(E(embed)->getAttribute("count")));
      if(E(embed)->hasAttribute("counterName")) static_cast<CloneWidget*>(clone->getWidget())->setCounterName(QString::fromStdString(E(embed)->getAttribute("counterName")));
      if(E(embed)->hasAttribute("onlyif")) static_cast<CloneWidget*>(clone->getWidget())->setOnlyif(QString::fromStdString(E(embed)->getAttribute("onlyif")));
    }
    return parent;
  }

  DOMElement* ClonePropertyDialog::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    E(item->getXMLElement())->setAttribute("name",static_cast<TextWidget*>(name->getWidget())->getText().toStdString());
    item->updateName();
    DOMElement *embedNode = item->getEmbedXMLElement();
    if(clone->isActive()) {
      if(not embedNode) embedNode = item->createEmbedXMLElement();
      E(embedNode)->setAttribute("count",static_cast<CloneWidget*>(clone->getWidget())->getCount().toStdString());
      E(embedNode)->setAttribute("counterName",static_cast<CloneWidget*>(clone->getWidget())->getCounterName().toStdString());
      E(embedNode)->setAttribute("onlyif",static_cast<CloneWidget*>(clone->getWidget())->getOnlyif().toStdString());
    }
    else if(embedNode) {
      E(embedNode)->removeAttribute("count");
      E(embedNode)->removeAttribute("counterName");
      E(embedNode)->removeAttribute("onlyif");
    }
    item->maybeRemoveEmbedXMLElement();
    item->updateStatus();
    return nullptr;
  }

  void ClonePropertyDialog::updateName() {
    TextWidget *textWidget = static_cast<TextWidget*>(name->getWidget());
    if(clone->isActive()) {
      QString text = textWidget->getText();
      int i1 = text.indexOf("{");
      cout<<"mfmf active "<<i1<<endl;
      if(i1==-1)
        textWidget->setText(textWidget->getText()+"{"+static_cast<CloneWidget*>(clone->getWidget())->getCounterName()+"}");
    }
    else {
      QString text = textWidget->getText();
      cout<<"mfmf deact "<<text.toStdString()<<" "<<"{ *" + static_cast<CloneWidget*>(clone->getWidget())->getCounterName().toStdString() + " *}"<<endl;
      QRegularExpression re("{ *" + static_cast<CloneWidget*>(clone->getWidget())->getCounterName() + " *}");
      text.remove(re);
      cout<<"mfmf deact2 "<<text.toStdString()<<endl;
      textWidget->setText(text);
    }
  }

}
