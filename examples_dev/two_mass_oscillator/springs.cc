#include "springs.h"
#include "coordinate_system.h"

namespace MBSim {

  Spring::Spring(const string &name) : FlexibleConnection(name) {
    setForceDirection("[0;1;0]");
  }

  void Spring::updateStage1(double t) {
    WrP0P1=port[1]->getPosition() - port[0]->getPosition();
    Vec forceDir;
    forceDir = WrP0P1/nrm2(WrP0P1);
    Wf = forceDir;
    g(0) = trans(forceDir)*WrP0P1;
    gd(0) = trans(forceDir)*(port[1]->getVelocity() - port[0]->getVelocity());  

  } 

  void Spring::updateStage2(double t) {
    la(0) = (cT*(g(0)-l0) + dT*gd(0));
    WF[0] = Wf*la;
    WF[1] = -WF[0];
  }    

}
