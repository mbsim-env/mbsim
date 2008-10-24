#include "springs.h"
#include "coordinate_system.h"

namespace MBSim {

  Spring::Spring(const string &name) : FlexibleConnection(name) {
    setForceDirection("[0;1;0]");
  }

  void Spring::updateg(double t) {
    WrP0P1=port[1]->getPosition() - port[0]->getPosition();
    forceDir = WrP0P1/nrm2(WrP0P1);
    Wf = forceDir;
    g(0) = trans(forceDir)*WrP0P1;
  } 

  void Spring::updategd(double t) {
    gd(0) = trans(forceDir)*(port[1]->getVelocity() - port[0]->getVelocity());  
  }    

  void Spring::updateh(double t) {
    la(0) = (cT*(g(0)-l0) + dT*gd(0));
    WF[0] = Wf*la;
    WF[1] = -WF[0];
    for(unsigned int i=0; i<port.size(); i++)
      h[i] += trans(port[i]->getJacobianOfTranslation())*WF[i];
  }    

}
