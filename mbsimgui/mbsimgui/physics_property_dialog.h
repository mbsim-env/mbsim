#ifndef _PHYSICS_PROPERTY_DIALOG_H_
#define _PHYSICS_PROPERTY_DIALOG_H_

#include "link_property_dialog.h"

namespace MBSimGUI {

  class UniversalGravitationPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      UniversalGravitationPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections, *gravitationalConstant, *enableOpenMBV;
  };

  class WeightPropertyDialog : public MechanicalLinkPropertyDialog {

    public:
      WeightPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *connections, *gravityFunction, *enableOpenMBV;
  };

  class BuoyancyPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      BuoyancyPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *displacedVolume, *densityFunction, *gravityFunction, *enableOpenMBV;
  };

  class DragPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      DragPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *dragFunction, *enableOpenMBV;
  };

  class AerodynamicsPropertyDialog : public FloatingFrameLinkPropertyDialog {

    public:
      AerodynamicsPropertyDialog(Element *link);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *densityFunction, *coefficientFunction, *referenceSurface, *windSpeed, *enableOpenMBV;
  };

}

#endif
