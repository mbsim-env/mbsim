#ifndef PRESS_BOUND_H
#define PRESS_BOUND_H

#include <mbsim/link.h>
#include <vector>
#include "line.h"

namespace MBSim {

class PressBound : public Link {
  protected:
    double p;
    std::vector<Line*> conIn, conOut;
  public:
    PressBound(std::string name);
    void updateg(double) {}
    void updategd(double) {}
    void updateWRef(const fmatvec::Mat&, int) {}
    void updateVRef(const fmatvec::Mat&, int) {}
    void updatehRef(const fmatvec::Vec&, int) {}
    void updatedhdqRef(const fmatvec::Mat&, int) {}
    void updatedhduRef(const fmatvec::SqrMat&, int) {}
    void updatedhdtRef(const fmatvec::Vec&, int) {}
    void updaterRef(const fmatvec::Vec&, int) {}
    void updaterRef(const fmatvec::Vec&) {}
    bool isActive() const { return false; }
    bool gActiveChanged() { return false; }
    virtual bool isSingleValued() const { return true; }
    void setPressure(double p_) { p=p_; }
    void addInConnection(Line *l) { conIn.push_back(l); }
    void addOutConnection(Line *l) { conOut.push_back(l); }
    void updateh(double);
};

}

#endif
