#include <mbsim/utils/nonlinear_algebra.h>

using namespace MBSim;
using namespace std;

class Res : public Function<fmatvec::Vec(fmatvec::Vec)> {
  public:
    Res() = default;
    fmatvec::Vec operator()(const fmatvec::Vec &x_) override {
      double x = x_(0);
      double r = (1-alpha)*(0.5*x-0.1) + alpha*(x*x*x-x-2);
      return fmatvec::Vec(1, fmatvec::INIT, r);
    }
    void setHomotopyParameter(double a) { alpha = a; }
  private:
    double alpha = 1; // homotopy-parameter
};

class Jac : public Function<fmatvec::SqrMat(fmatvec::Vec)> {
  public:
    Jac() = default;
    fmatvec::SqrMat operator()(const fmatvec::Vec &x_) override {
      double x = x_(0);
      double drdx = (1-alpha)*0.5 + alpha*(2*x*x-1);
      return fmatvec::SqrMat(1, fmatvec::INIT, drdx);
    }
    void setHomotopyParameter(double a) { alpha = a; }
  private:
    double alpha = 1; // homotopy-parameter
};

int main() {
  int ret = 0;

  Res res;
  Jac jac;
  MultiDimNewtonMethod solver(&res, &jac);
  solver.setInitialLineSearchStepSizeFactor(0.1);

  // initial solution with alpha = 0
  double x = 0.2;
  fmatvec::Vec x_(1, fmatvec::INIT, x);

  // homotopy iteration
  const double dalphaMin = 0.05;
  const int homotopyIterMax = 20;
  int homotopyIter = 0;
  double alphaLastSuccess = 0;
  double alpha = 1;
  while(true) {
    res.setHomotopyParameter(alpha);
    jac.setHomotopyParameter(alpha);
    if(alpha != 1)
      fmatvec::Atom::msgStatic(fmatvec::Atom::Info) << "Intermediate homotopy-parameter "<<alpha<<" needed in DynamicSystemSolver::solveConstraintsNonlinearEquations for convergence."<<endl;
    x_ = solver.solve(x_);
    cout<<"info = "<<solver.getInfo()<<endl;

    // solution found -> break
    if(solver.getInfo() == 0 && alpha == 1)
      break;
    // diverge -> decrease alpha
    if(solver.getInfo() == -2 || solver.getInfo() == -3) {
      if(alpha-alphaLastSuccess < dalphaMin) {
        ret++;
        break;
      }
      alpha = (alphaLastSuccess + alpha)/2;
    }
    // simplified solution found -> increase alpha to 1
    else if(solver.getInfo() == 0) {
      alphaLastSuccess = alpha;
      alpha = 1;
    }
    else {
      ret++;
      break;
    }

    homotopyIter++;
    if(homotopyIter > homotopyIterMax) {
      ret++;
      break;
    }
  }
  cout<<"ret = "<<ret<<" solution="<<x_<<endl;

  return ret;
}
