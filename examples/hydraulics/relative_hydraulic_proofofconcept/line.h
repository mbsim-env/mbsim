#ifndef LINE_H
#define LINE_H

#include <mbsim/objects/object.h>

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
    std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() { return std::shared_ptr<OpenMBV::Group>(); }

    void calcqSize() { qSize=0; }
    void calcuSize(int j);
    void updateM();
    fmatvec::Mat evalJ(int k=0) { if(updJ) updateJacobians(k); return J; }
    void updateJacobians(int k=0);
    double getFlowrate() { return flowrate; }
    void init(InitStage stage);
    void plot();
    void resetUpToDate() { Object::resetUpToDate(); updJ = true; }
};

}

#endif
