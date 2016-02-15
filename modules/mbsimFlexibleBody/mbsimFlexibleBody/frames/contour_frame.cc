#include <config.h>
#include "contour_frame.h"

#include <mbsimFlexibleBody/flexible_body.h>

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void ContourFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"contourParameters");
    setContourParameters(getVec(e));
  }

  DOMElement* ContourFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
//    addElementText(ele0, MBSIMFLEX%"contourParameters",
//    int(getContourParameters()));
    return ele0;
  }

}
