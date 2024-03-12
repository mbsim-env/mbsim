#include "plot_attribute_store.h"
#include "namespace.h"
#include <mbxmlutilshelper/dom.h>

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;

namespace MBSimGUI {

  DOMElement* PlotAttributeStore::initializeUsingXML(DOMElement *parent) {
    DOMElement *e=parent->getFirstElementChild();
    while(e && (E(e)->getTagName()==MBSIM%"plotAttribute" ||
                E(e)->getTagName()==MBSIM%"plotAttributeInt" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloat" ||
                E(e)->getTagName()==MBSIM%"plotAttributeString" ||
                E(e)->getTagName()==MBSIM%"plotAttributeIntVector" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloatVector" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloatMatrix")) {
      string type = E(e)->getTagName().second;
      string name = E(e)->getAttribute("name");
      string code = E(e)->getTagName()==MBSIM%"plotAttribute" ? "" : E(e)->getText<string>();
      pas.emplace_back(PA{type, name, code});
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotAttributeStore::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(auto pa : pas) {
      DOMElement *ele = D(doc)->createElement(MBSIM%pa.type);
      E(ele)->setAttribute("name", pa.name);
      if(pa.type != "plotAttribute")
        ele->insertBefore(doc->createTextNode(X()%pa.code), nullptr);
      parent->insertBefore(ele, ref);
    }
    return nullptr;
  }

}
