#ifndef RIGID_CONTOUR_FUNCTION1S_H
#define RIGID_CONTOUR_FUNCTION1S_H

#include <mbsim/utils/ppolynom.h>
#include <mbsim/utils/contour_functions.h>

class FuncCrPC : public MBSim::ContourFunction1s {
  private:
    fmatvec::Vec Cb;
    MBSim::PPolynom pp_y;
    MBSim::PPolynom pp_z;
  
  public:
    FuncCrPC() {
      Cb = fmatvec::Vec("[1.0; 0.0; 0.0]");
    }
    
    void setYZ(const fmatvec::Mat& YZ, int discretization=1, fmatvec::Vec rYZ=fmatvec::Vec(3, fmatvec::INIT, 0));
    
    fmatvec::Vec operator()(const double& alpha, const void * =NULL); // Vektor vom P nach C
    fmatvec::Vec diff1(const double& alpha); // Tangente in C
    fmatvec::Vec diff2(const double& alpha); // 2. Ableitung in C

    void init(double alpha);

    fmatvec::Vec computeT(const double& alpha) {fmatvec::Vec T = -diff1(alpha); return T/nrm2(T); } // Tangente immer so, dass t n b ein rechtssystem bilden
    fmatvec::Vec computeB(const double& alpha) {return Cb; } // immer konstant
    fmatvec::Vec computeN(const double& alpha) {fmatvec::Vec N = crossProduct(diff1(alpha), Cb); return N/nrm2(N); } // normale immer nach außen
};  

#endif
