#include <config.h>
#include "frame_1s.h"
#include <mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h>

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void Frame1s::updatePositions(double t) {
    static_cast<FlexibleBody1s21RCM*>(parent)->updatePositions(t,this);
    updatePos = false;
  }

  void Frame1s::updateVelocities(double t) {
    static_cast<FlexibleBody1s21RCM*>(parent)->updateVelocities(t,this);
    updateVel = false;
  }

  void Frame1s::updateAccelerations(double t) {
    static_cast<FlexibleBody1s21RCM*>(parent)->updateAccelerations(t,this);
    updateAcc = true;
  }

  void Frame1s::updateJacobians(double t, int j) {
    static_cast<FlexibleBody1s21RCM*>(parent)->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void Frame1s::updateGyroscopicAccelerations(double t) {
    static_cast<FlexibleBody1s21RCM*>(parent)->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

  void Frame1s::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"parameter");
    setParameter(getDouble(e));
  }

  DOMElement* Frame1s::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
    addElementText(ele0, MBSIMFLEX%"parameter", getParameter());
    return ele0;
  }

}
