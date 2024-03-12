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
