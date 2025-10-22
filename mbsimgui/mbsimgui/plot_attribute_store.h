#ifndef _PLOT_ATTRIBUTE_STORE_
#define _PLOT_ATTRIBUTE_STORE_

#include <memory>
#include <string>
#include <vector>
#include <xercesc/util/XercesDefs.hpp>

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
  class DOMDocument;
}

namespace MBSimGUI {

  // For now we just store the plotAttribute's in ElementPropertyDialog to be able to write it back when an element is changed and save.
  // This needs to be extended to view and edit the plotAttribute's in MBSimGUI (its a missing feature for now).
  // If done so it should be done similarly to plotFeature's: new ExtWidget("Plot attributes",new PlotAttributeWidget(...));
  class PlotAttributeStore {
    public:
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent);
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref);
    private:
      std::vector<std::shared_ptr<xercesc::DOMDocument>> pas;
  };

}

#endif
