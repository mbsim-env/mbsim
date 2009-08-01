#ifndef RIGID_CONTOUR_FUNCTION1S_H
#define RIGID_CONTOUR_FUNCTION1S_H

#include <mbsim/utils/ppolynom.h>
#include <mbsim/userfunction_contour.h>

class FuncCrPC : public MBSim::UserFunctionContour1s {
  private:
    fmatvec::Vec Cb;
    MBSim::PPolynom pp_y;
    MBSim::PPolynom pp_z;
  
  public:
    FuncCrPC() {
      Cb = fmatvec::Vec("[1.0; 0.0; 0.0]");
    }
    
    void setYZ(const fmatvec::Mat& YZ, int discretization=1, fmatvec::Vec rYZ=fmatvec::Vec(3, fmatvec::INIT, 0));
    
    fmatvec::Vec operator()(double alpha); // Vektor vom P nach C
    fmatvec::Vec diff1(double alpha); // Tangente in C
    fmatvec::Vec diff2(double alpha); // 2. Ableitung in C
    // double computeR(double alpha); // Krümmungsradius

    void init(double alpha);

    fmatvec::Vec computeT(double alpha) {fmatvec::Vec T = -diff1(alpha); return T/nrm2(T); } // Tangente immer so, dass t n b ein rechtssystem bilden
    fmatvec::Vec computeB(double alpha) {return Cb; } // immer konstant
    fmatvec::Vec computeN(double alpha) {fmatvec::Vec N = crossProduct(diff1(alpha), Cb); return N/nrm2(N); } // normale immer nach außen
};  

#endif
