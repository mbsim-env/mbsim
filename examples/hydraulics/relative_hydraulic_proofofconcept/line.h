#ifndef LINE_H
#define LINE_H

#include <mbsim/object.h>

namespace MBSim {

class Line : public Object {
  protected:
    fmatvec::Mat J;
    double flowrate;
    bool updJ;
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
    fmatvec::Mat getJ(double t) { if(updJ) updateJacobians(t); return J; }
    void updateJacobians(double t ,int k=0);
    double getFlowrate() { return flowrate; }
    void init(InitStage stage);
    void plot(double t, double dt=1);
    void resetUpToDate() { Object::resetUpToDate(); updJ = true; }
};

}

#endif
