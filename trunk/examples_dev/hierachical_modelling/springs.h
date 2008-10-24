#ifndef SPRINGS_H_
#define SPRINGS_H_

#include "flexible_connection.h"
#include "data_interface_base.h"

namespace MBSim {
class Spring : public FlexibleConnection {
  protected:
    double l0, cT, dT;
    Vec forceDir;
  public:
    Spring(const string &name);
    void updateg(double t);
    void updategd(double t); 
    void updateh(double t); 
    void setl0(double l0_) {l0=l0_;}
    void setStiffness(double c) {cT = c;}
    void setDamping(double d) {dT = d;}
};
   
}

#endif
