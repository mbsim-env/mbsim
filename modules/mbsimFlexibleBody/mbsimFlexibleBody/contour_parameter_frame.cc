#include <config.h>
#include "contour_parameter_frame.h"

#include <mbsimFlexibleBody/flexible_body.h>

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void ContourParameterFrame::updatePositions(double t) {
    WrOP = static_cast<FlexibleBody*>(parent)->getVelocity(t,zeta);
    AWP = static_cast<FlexibleBody*>(parent)->getAngularVelocity(t,zeta);
    updatePos = false;
  }

  void ContourParameterFrame::updateVelocities(double t) {
    WvP = static_cast<FlexibleBody*>(parent)->getVelocity(t,zeta);
    WomegaP = static_cast<FlexibleBody*>(parent)->getAngularVelocity(t,zeta);
    updateVel = false;
  }

  void ContourParameterFrame::updateAccelerations(double t) {
    WaP = static_cast<FlexibleBody*>(parent)->getAcceleration(t,zeta);
    WpsiP = static_cast<FlexibleBody*>(parent)->getAngularAcceleration(t,zeta);
    updateAcc = false;
  }

  void ContourParameterFrame::updateJacobians(double t, int j) {
    WJP[j] = static_cast<FlexibleBody*>(parent)->getJacobianOfTranslation(t,zeta);
    WJR[j] = static_cast<FlexibleBody*>(parent)->getJacobianOfRotation(t,zeta);
    updateJac[j] = false;
  }

  void ContourParameterFrame::updateGyroscopicAccelerations(double t) {
    WjP = static_cast<FlexibleBody*>(parent)->getGyroscopicAccelerationOfTranslation(t,zeta);
    WjR = static_cast<FlexibleBody*>(parent)->getGyroscopicAccelerationOfRotation(t,zeta);
    updateGA = false;
  }

  void ContourParameterFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);

    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"contourParameters");
    setContourParameters(getVec(e));
  }

  DOMElement* ContourParameterFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
//    addElementText(ele0, MBSIMFLEX%"contourParameters",
//    int(getContourParameters()));
    return ele0;
  }

}
