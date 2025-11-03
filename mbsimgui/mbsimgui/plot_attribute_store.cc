#include "plot_attribute_store.h"
#include "mainwindow.h"
#include "namespace.h"
#include <mbxmlutilshelper/dom.h>

using namespace std;
using namespace xercesc;
using namespace MBXMLUtils;

namespace MBSimGUI {

  extern MainWindow *mw;

  DOMElement* PlotAttributeStore::initializeUsingXML(DOMElement *parent) {
    DOMElement *e=parent->getFirstElementChild();
    while(e && (E(e)->getTagName()==MBSIM%"plotAttribute" ||
                E(e)->getTagName()==MBSIM%"plotAttributeInt" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloat" ||
                E(e)->getTagName()==MBSIM%"plotAttributeString" ||
                E(e)->getTagName()==MBSIM%"plotAttributeIntVector" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloatVector" ||
                E(e)->getTagName()==MBSIM%"plotAttributeFloatMatrix")) {
      auto doc = mw->mbxmlparserNoVal->createDocument();
      pas.emplace_back(doc);
      auto root = doc->importNode(e, true);
      doc->insertBefore(root, nullptr);
      e=e->getNextElementSibling();
    }
    return e;
  }

  DOMElement* PlotAttributeStore::writeXMLFile(DOMNode *parent, DOMNode *ref) {
    DOMDocument *doc=parent->getOwnerDocument();
    for(auto pa : pas) {
      auto root = doc->importNode(pa->getDocumentElement(), true);
      parent->insertBefore(root, ref);
    }
    return nullptr;
  }

}
