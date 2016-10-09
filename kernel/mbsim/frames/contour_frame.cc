#include <config.h>
#include "contour_frame.h"

using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void ContourFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"zeta");
    setZeta(getVec(e));
  }

}
