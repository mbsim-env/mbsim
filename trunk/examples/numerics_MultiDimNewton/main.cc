#include <numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/utils/stopwatch.h>
#include <fmatvec.h>

#include <iostream>

using namespace fmatvec;
using namespace MBSim;
using namespace MBSimNumerics;
using namespace std;


class TestFunction : public MBSimNumerics::Function1<fmatvec::Vec, fmatvec::Vec> {

  public:
    /**
     * \brief Constructor
     */
  TestFunction(){
  }

    /**
     * \brief Destructor
     */
    virtual ~TestFunction() {
    }

    virtual fmatvec::Vec operator ()(const fmatvec::Vec & vector, const void * = NULL) {
      Vec result(vector.size(), INIT, 0.);

      for(int i=0; i< result.size(); i++) {
        result(i) = pow(sin(i*2*M_PI/vector.size()) + vector(i), 2);
      }

      return result;
    }

};

class TestFunction2 : public MBSim::Function1<fmatvec::Vec, fmatvec::Vec> {

  public:
    /**
     * \brief Constructor
     */
  TestFunction2(){
  }

    /**
     * \brief Destructor
     */
    virtual ~TestFunction2() {
    }

    virtual fmatvec::Vec operator ()(const fmatvec::Vec & vector, const void * = NULL) {
      Vec result(vector.size(), INIT, 0.);

      for(int i=0; i< result.size(); i++) {
        result(i) = pow(sin(i*2*M_PI/vector.size()) + vector(i), 2);
      }

      return result;
    }

};

int main (int argc, char* argv[]) {

  int dimension = 3000;

  TestFunction * function = new TestFunction();

  MultiDimensionalNewtonMethod newton(function);

  TestFunction2 * function2 = new TestFunction2();

  MultiDimNewtonMethod newton2(function2);

  map<Index, double> tolerances;
  tolerances.insert(pair<Index, double>(Index(0,dimension/2-1), 1e-10));
  tolerances.insert(pair<Index, double>(Index(dimension/2,dimension-1), 1e-10));

  newton.setCriteriaFunction(new LocalResidualCriteriaFunction(tolerances));

  Vec initialSolution(dimension,INIT,0.0);
  for(int i =0; i< dimension; i++) {
    initialSolution(i) = 5*i;
  }
  Vec test1 = initialSolution.copy();
  Vec test2 = initialSolution.copy();

  StopWatch sw;

  for(int i =0 ; i < 2; i++) {
    if(i==1)
      newton.setCriteriaFunction(new GlobalResidualCriteriaFunction());
    sw.start();
    test1 = newton.solve(initialSolution);

    cout << "Time = " << sw.stop(true) << endl;
  }

  cout << "Info of new = " << newton.getInfo() << endl;

  cout << "iter = " << newton.getNumberOfIterations() << endl;
  cout << "itermax = " << newton.getNumberOfMaximalIterations() << endl;

  //cout << "test1 = "  << test1 << endl;

  sw.start();

  test2 = newton2.solve(test2);

  cout << "Time = " << sw.stop(true) << endl;

  cout << "Info of old = " << newton2.getInfo() << endl;

  cout << "iter = " << newton2.getNumberOfIterations() << endl;
  cout << "itermax = " << newton2.getNumberOfMaximalIterations() << endl;

  //cout << "test2 = "  << test2 << endl;

  //cout << "test1 - test2" << test1 - test2 << endl;

  return 0;

}
