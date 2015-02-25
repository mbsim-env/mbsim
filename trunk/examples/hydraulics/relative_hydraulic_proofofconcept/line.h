#ifndef LINE_H
#define LINE_H

#include <mbsim/object.h>

namespace MBSim {

class Line : public Object {
  protected:
    fmatvec::Mat J;
    double flowrate;
  public:
    Line(std::string name);
    void updateStateDependentVariables(double);
    void updateInverseKineticsJacobians(double) {}
#ifdef HAVE_OPENMBVCPPINTERFACE
    boost::shared_ptr<OpenMBV::Group> getOpenMBVGrp() { return boost::shared_ptr<OpenMBV::Group>(); }
#endif

    void calcqSize() { qSize=0; }
    void calcuSize(int j);
    void updateM(double, int k);
    fmatvec::Mat getJ() { return J; }
    void updateJacobians(double,int k);
    double getFlowrate() { return flowrate; }
    void init(InitStage stage);
    void plot(double t, double dt=1);
};

}

#endif
