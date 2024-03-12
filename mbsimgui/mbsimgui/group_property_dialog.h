#ifndef _GROUP_PROPERTY_DIALOG_H_
#define _GROUP_PROPERTY_DIALOG_H_

#include "element_property_dialog.h"

namespace MBSimGUI {

  class ExtWidget;
  class CommentWidget;
  class Element;

  class GroupPropertyDialog : public ElementPropertyDialog {

    public:
      GroupPropertyDialog(Element *group, bool kinematics=true);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *frameOfReference;
  };
}

#endif
