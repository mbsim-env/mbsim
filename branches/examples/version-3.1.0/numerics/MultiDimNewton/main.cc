#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/utils/stopwatch.h>
#include <fmatvec.h>

#include <iostream>

using namespace fmatvec;
using namespace MBSim;
using namespace std;

const int dimension = 11;


template<class type>
class TestFunction : public Function1<fmatvec::Vector<type, double>, fmatvec::Vector<type, double> > {

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

    virtual fmatvec::Vector<type, double> operator ()(const fmatvec::Vector<type, double> & vector, const void * = NULL) {
      Vector<type, double> result(vector.size(), NONINIT);

      for(int i=0; i< result.size(); i++) {
        result(i) = pow(sin(i*2*M_PI/vector.size()) + vector(i), 2);
      }

      return result;
    }

};

int main (int argc, char* argv[]) {

  Vec initialSolution(dimension,INIT,0.0);
  for(int i =0; i< dimension; i++) {
    initialSolution(i) = 5*i;
  }
  Vec test1 = initialSolution.copy();
  Vec test2 = initialSolution.copy();
  Vector<Fixed<dimension>, double > initFixed(initialSolution.copy());
  Vector<Fixed<dimension>, double> test3;

  StopWatch sw;

  cout << "Solving system of dimension " << dimension << endl;


  /*New Newton as Reference*/
  TestFunction<Ref> * function = new TestFunction<Ref>();
  MultiDimensionalNewtonMethod<Ref, double> newtonNew;
  newtonNew.setFunction(function);
  NumericalNewtonJacobianFunction<Ref, double> jac;
  newtonNew.setJacobianFunction(&jac);
  StandardDampingFunction<Ref, double> damp;
  newtonNew.setDampingFunction(&damp);

  map<Index, double> tolerances;
  tolerances.insert(pair<Index, double>(Index(0,dimension/2-1), 1e-10));
  tolerances.insert(pair<Index, double>(Index(dimension/2,dimension-1), 1e-8));

  for(int i =0 ; i < 1; i++) {
    if(i==0) {
      cout << "Solving with GlobalResidualCriteriaFunction with tolerance of 1e-10 ... " << endl;
      newtonNew.setCriteriaFunction(new GlobalResidualCriteriaFunction<Ref, double>(1e-10));
    }
    else {
      cout << "Solving with LocalResidualCriteriaFunction with tolerance for first half of solution vector of 1e-10 and second half of 1e-8 ... " << endl;
      newtonNew.setCriteriaFunction(new LocalResidualCriteriaFunction<Ref, double>(tolerances));
    }
    sw.start();
    test1 = newtonNew.solve(initialSolution);

    cout << "Precision = " << nrmInf((*function)(test1)) << endl;
    cout << "Time = " << sw.stop(true) << endl;
    cout << "Iterations = " << newtonNew.getNumberOfIterations() << endl << endl;
  }

  /*New Newton as Fixed-Size*/
  TestFunction<Fixed<dimension> > functionFixed;
  MultiDimensionalNewtonMethod<Fixed<dimension>, double> newtonNewFixed;
  newtonNewFixed.setFunction(&functionFixed);
  NumericalNewtonJacobianFunction<Fixed<dimension>, double> jacFixed;
  newtonNewFixed.setJacobianFunction(&jacFixed);
  StandardDampingFunction<Fixed<dimension>, double> dampFixed;
  newtonNewFixed.setDampingFunction(&dampFixed);

  map<Index, double> tolerancesFixed;
  tolerancesFixed.insert(pair<Index, double>(Index(0,dimension/2-1), 1e-10));
  tolerancesFixed.insert(pair<Index, double>(Index(dimension/2,dimension-1), 1e-8));

  for(int i =0 ; i < 1; i++) {
    if(i==0) {
      cout << "Solving with GlobalResidualCriteriaFunction with tolerance of 1e-10 ... " << endl;
      newtonNewFixed.setCriteriaFunction(new GlobalResidualCriteriaFunction<Fixed<dimension>, double>(1e-10));
    }
    else {
      cout << "Solving with LocalResidualCriteriaFunction with tolerance for first half of solution vector of 1e-10 and second half of 1e-8 ... " << endl;
      newtonNewFixed.setCriteriaFunction(new LocalResidualCriteriaFunction<Fixed<dimension>, double>(tolerancesFixed));
    }
    sw.start();
    test3 = newtonNewFixed.solve(initFixed);

    cout << "Precision = " << nrmInf((functionFixed)(test3)) << endl;
    cout << "Time = " << sw.stop(true) << endl;
    cout << "Iterations = " << newtonNewFixed.getNumberOfIterations() << endl << endl;
  }

  /*Old newton*/
  cout << "Solving system with old newton algorithm as reference with tolerance of 1e-10 ... " << endl;
  TestFunction<Ref> * function2 = new TestFunction<Ref>();
  MultiDimNewtonMethod newtonOld(function2);
  newtonOld.setTolerance(1e-10);

  sw.start();

  test2 = newtonOld.solve(test2);

  cout << "Precision = " << nrmInf((*function2)(test2)) << endl;
  cout << "Time = " << sw.stop(true) << endl;
  cout << "Iterations = " << newtonOld.getNumberOfIterations() << "(+1)" << endl;
  cout << "REAMARK: The iterations are counted differently (one less for the old Newton)..." << endl;

  return 0;

}
