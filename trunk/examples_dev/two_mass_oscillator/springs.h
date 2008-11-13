#ifndef SPRINGS_H_
#define SPRINGS_H_

#include "link.h"
#include "data_interface_base.h"


#ifdef HAVE_AMVIS
namespace AMVis {class CoilSpring;}
#endif

namespace MBSim {

class Spring : public Link {
  protected:
    double l0, cT, dT;
    Vec forceDir;
#ifdef HAVE_AMVIS
      AMVis::CoilSpring *coilspringAMVis;
      DataInterfaceBase *coilspringAMVisUserFunctionColor;
#endif
  public:
    Spring(const string &name);
    void init();
    void initPlotFiles();
    void updateg(double t);
    void updategd(double t); 
    void updateh(double t); 
    void setl0(double l0_) {l0=l0_;}
    void setStiffness(double c) {cT = c;}
    void setDamping(double d) {dT = d;}
    bool isActive() const {return true;}
    bool gActiveChanged() {return false;}
    virtual void connect(CoordinateSystem *port1, CoordinateSystem* port2);
    void plot(double t,double dt=1); 
 #ifdef HAVE_AMVIS
      void setAMVisSpring(AMVis::CoilSpring *spring_, DataInterfaceBase* funcColor=0) {coilspringAMVis= spring_; coilspringAMVisUserFunctionColor= funcColor;}
#endif
  };
  
}

#endif
