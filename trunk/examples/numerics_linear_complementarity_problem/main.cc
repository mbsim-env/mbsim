#include <mbsim/numerics/linear_complementarity_problem/linear_complementarity_problem.h>
#include <fmatvec.h>

#include <fmatvecTestbench/matgenerator.h>

#include <iostream>

using namespace fmatvec;
using namespace MBSim;
using namespace std;

SymMat makeInfluenceMat(int dim, double influenceDisk, double influenceElement) {
  SymMat A(dim, NONINIT);

  for(int i=0; i<dim; i++)
    for(int j =i; j<dim; j++) {
      int diff = j-i;
      double elementInfluence = 0;
      double DiskInfluence = 0;
      if(diff == 0 or (diff <= 1 and (j%2 == 1)))
        elementInfluence = influenceElement;
      DiskInfluence = influenceDisk * cos((double)diff/dim * M_PI);
      A(i,j) = elementInfluence + DiskInfluence;
    }


  return A;
}

void solve(SymMat& C, Vec& q, int dim) {
  LinearComplementarityProblem problem(C, q);
  problem.setDebugLevel(1);
//  map<Index, double> tolerances;
//  tolerances.insert(pair<Index, double>(Index(0,dim-1), 1e-8));
//  tolerances.insert(pair<Index, double>(Index(dim,2*dim-1), 1e-3));
//  LocalResidualCriteriaFunction * residualCriteria = new LocalResidualCriteriaFunction(tolerances);
//  problem.setNewtonCriteriaFunction(residualCriteria);
  //
  //  LocalShiftCriteriaFunction * shiftCriteria = new LocalShiftCriteriaFunction(tolerances);
  //  problem.setFixpointCriteriaFunction(shiftCriteria);
  Vec result;
  for (int strategy = 0; strategy < 5; strategy++) {
    problem.setStrategy(LCPSolvingStrategy(strategy));
    if (LCPSolvingStrategy(strategy) == 2) {
      for (int jactype = 0; jactype < 2; jactype++) {
        problem.setJacobianType(JacobianType(jactype));
        result = problem.solve();
      }
    }
    else {
      result = problem.solve();
    }
    //cout << result << endl;
    //cout << C * result(dim, 2*dim-1) + q - result(0, dim-1) << endl;
  }
  
}

int main (int argc, char* argv[]) {

  int dim = 300;

  Matgenerator MatGen;
  MatGen.init();

  MatGen.setDecimalPlaces(4);
  MatGen.setRange(1);

  Vec q = MatGen.getVec(dim);

  for(int i=0; i<q.size(); i++) {
    q(i) /= 100;
    if(q(i) > 0)
      q(i) *= -1;
  }


  SymMat C = makeInfluenceMat(dim, 10e8, 10e5);

  if(dim < 20) {
    cout << "C = " << C << endl;

    cout << "q = " << q << endl;
  }

  solve(C, q, dim);

  return 0;

}


