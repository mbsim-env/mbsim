#include <config.h>
#include "node_frame.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void NodeFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumber");
    setNodeNumber(getInt(e));
  }

  DOMElement* NodeFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
    addElementText(ele0, MBSIMFLEX%"nodeNumber", getNodeNumber());
    return ele0;
  }

}
