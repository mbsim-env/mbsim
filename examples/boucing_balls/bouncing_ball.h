#ifndef _ROCKING_ROD_H_
#define _ROCKING_ROD_H_

#include "multi_body_system.h"
#include "impact_rigid.h"
#include <string>

using namespace std;
using namespace MBSim;

class BouncingBall : public MultiBodySystem {
  private:
    double mu;
    ImpactRigid *ir;
  public:
    BouncingBall(const string &projectName); 
    void setmu(double mu_) {mu=mu_;}
    void init();
};

#endif
