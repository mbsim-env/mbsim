#ifndef SPRINGS_H_
#define SPRINGS_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/data_interface_base.h"

#ifdef HAVE_AMVIS
namespace AMVis {class CoilSpring;}
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/coilspring.h>
#endif

namespace MBSim {

  class Spring : public LinkMechanics {
    protected:
      double l0, cT, dT;
      fmatvec::Vec forceDir;
#ifdef HAVE_AMVIS
      AMVis::CoilSpring *coilspringAMVis;
      DataInterfaceBase *coilspringAMVisUserFunctionColor;
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::CoilSpring *coilspringAMVis;
#endif
    public:
      Spring(const std::string &name);
      void init();
      void initPlot();
      void updateg(double t);
      void updategd(double t); 
      void updateh(double t); 
      void setl0(double l0_) {l0=l0_;}
      void setStiffness(double c) {cT = c;}
      void setDamping(double d) {dT = d;}
      bool isActive() const {return true;}
      bool gActiveChanged() {return false;}
      virtual void connect(Frame *port1, Frame* port2);
      void plot(double t,double dt=1); 
#ifdef HAVE_AMVIS
      void setAMVisSpring(AMVis::CoilSpring *spring_, DataInterfaceBase* funcColor=0) {coilspringAMVis= spring_; coilspringAMVisUserFunctionColor= funcColor;}
#endif
#ifdef HAVE_AMVISCPPINTERFACE
      void setAMVisSpring(AMVis::CoilSpring *spring_) {coilspringAMVis=spring_;}
#endif

  };

}

#endif
