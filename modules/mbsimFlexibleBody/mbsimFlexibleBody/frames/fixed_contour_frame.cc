#include <config.h>
#include "mbsimFlexibleBody/frames/fixed_contour_frame.h"
#include "mbsimFlexibleBody/flexible_body.h"

using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void FixedContourFrame::updatePositions(double t) {
    static_cast<FlexibleBody*>(parent)->updatePositions(t,this);
    updatePos = false;
  }

  void FixedContourFrame::updateVelocities(double t) {
    static_cast<FlexibleBody*>(parent)->updateVelocities(t,this);
    updateVel = false;
  }

  void FixedContourFrame::updateAccelerations(double t) {
    static_cast<FlexibleBody*>(parent)->updateAccelerations(t,this);
    updateAcc = false;
  }

  void FixedContourFrame::updateJacobians(double t, int j) {
    static_cast<FlexibleBody*>(parent)->updateJacobians(t,this,j);
    updateJac[j] = false;
  }

  void FixedContourFrame::updateGyroscopicAccelerations(double t) {
    static_cast<FlexibleBody*>(parent)->updateGyroscopicAccelerations(t,this);
    updateGA = false;
  }

}
