#include <config.h>
#include "contour_parameter_frame.h"

#include <mbsimFlexibleBody/flexible_body.h>

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void ContourParameterFrame::updatePositions(double t) {
    static_cast<FlexibleBody*>(parent)->updatePositions(t,this);
    updatePos = false;
  }

  void ContourParameterFrame::updateVelocities(double t) {
    static_cast<FlexibleBody*>(parent)->updateVelocities(t,this);
    updateVel = false;
  }

  void ContourParameterFrame::updateAccelerations(double t) {
    static_cast<FlexibleBody*>(parent)->updateAccelerations(t,this);
    updateAcc = false;
  }

  void ContourParameterFrame::updateJacobians(double t, int j) {
    static_cast<FlexibleBody*>(parent)->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void ContourParameterFrame::updateGyroscopicAccelerations(double t) {
    static_cast<FlexibleBody*>(parent)->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

}
