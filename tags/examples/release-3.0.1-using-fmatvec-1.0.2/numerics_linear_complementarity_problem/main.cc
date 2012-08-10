#include <mbsim/numerics/linear_complementarity_problem/linear_complementarity_problem.h>
#include <fmatvec.h>

#include <iostream>

using namespace fmatvec;
using namespace MBSim;
using namespace std;

SymMat makeInfluenceMat(int dim, double influenceDisk, double influenceElement) {
  SymMat A(dim, NONINIT);

  for (int i = 0; i < dim; i++)
    for (int j = i; j < dim; j++) {
      int diff = j - i;
      double elementInfluence = 0;
      double DiskInfluence = 0;
      if (diff == 0)
        elementInfluence = influenceElement;
      DiskInfluence = influenceDisk * cos((double) diff / dim * M_PI);
      A(i, j) = elementInfluence + DiskInfluence;
    }

  return A;
}

void solve(SymMat& C, Vec& q, int dim) {
  LinearComplementarityProblem problem(C, q);
  problem.setDebugLevel(1);
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
  }

}

int main(int argc, char* argv[]) {

  int dim = 100;

  Vec q(dim, NONINIT);

  for (int i = 0; i < dim; i++) {
    q(i) = -cos(-M_PI / 2 + M_PI * i / dim);
  }

  SymMat C = makeInfluenceMat(dim, 10e8, 10e5);

  if (dim < 20) {
    cout << "C = " << C << endl;

    cout << "q = " << q << endl;
  }

  solve(C, q, dim);

  return 0;

}

