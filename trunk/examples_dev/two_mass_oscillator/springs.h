#ifndef SPRINGS_H_
#define SPRINGS_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/data_interface_base.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#endif

namespace MBSim {

  class Spring : public LinkMechanics {
    protected:
      double l0, cT, dT;
      fmatvec::Vec forceDir;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::CoilSpring *coilspringOpenMBV;
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
      virtual void connect(FrameInterface *frame1, FrameInterface* frame2);
      void plot(double t,double dt=1); 
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpring(OpenMBV::CoilSpring *spring_) {coilspringOpenMBV=spring_;}
#endif

  };

}

#endif
