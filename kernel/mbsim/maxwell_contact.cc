#include <config.h>
#include <time.h>

#include <maxwell_contact.h>

#include <fmatvec.h>

#include <mbsim/contour.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/nonsmooth_algebra.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  /* MaxwellContact   ***************************************************************************************************************************/

  MaxwellContact::MaxwellContact(const string &name) :
      LinkMechanics(name), contourPairing(0), possibleContactPoints(0), C(SymMat(0,NONINIT)), matConst(1.), matConstSetted(false), DEBUGLEVEL(0)
  {
    gTol = 1;
  }

  /**
   * \brief Sets up the MFL and solves it to compute the entries for the h-vectors of all contours
   */
  void MaxwellContact::updateh(double t, int k) {

      if (DEBUGLEVEL >= 3) {
        cout << name << endl;
        cout << __func__ << endl;
      }

    updatePossibleContactPoints();
    updategd(t);
    updateJacobians(t,k);

    if (possibleContactPoints.size() > 0) {

      /*compute Influence Matrix*/
      updateInfluenceMatrix(t);

      computeMaterialConstant(t);

      /*save rigidBodyGaps in vector*/
      Vec rigidBodyGap(possibleContactPoints.size(), INIT, 0.);
      for (size_t i = 0; i < possibleContactPoints.size(); i++) {
        rigidBodyGap(i) = contourPairing[possibleContactPoints[i]]->getRelativeDistance()(0);
      }

      if (DEBUGLEVEL >= 5)
        cout << "rigidBodyGap: " << rigidBodyGap << endl;

      /*solve Linear Complementary Problem*/
      Vec lambda = solveLCP(C, rigidBodyGap);

      if (DEBUGLEVEL >= 3) {
        cout << "lambda = " << lambda << endl;
      }

      //index for active contact (to assign possible contact to active contact)
      int activeContact = 0;

      /*create h - vector(s)*/
      //loop over all contacts
      for (size_t potentialContact = 0; potentialContact < contourPairing.size(); potentialContact++) {
        ContourPairing* pair = contourPairing[potentialContact];
        if (pair->isActive()) { //if contact is active
          /*compute forces with directions for contact*/
          pair->getContactForce()(0) = lambda(activeContact); //normal force
          pair->getForce(1) = pair->getContourPointData()[0].getFrameOfReference().getOrientation().col(0) * pair->getContactForce()(0);
          if (pair->getFrictionForceLaw()) { //if friction is activated
            int frictionDirections = pair->getFrictionDirections();
            pair->getContactForce()(1, frictionDirections) = (*pair->getFrictionForceLaw())(pair->getRelativeVelocity()(1, frictionDirections), fabs(pair->getContactForce()(0)));

            pair->getForce(1) += pair->getContourPointData()[0].getFrameOfReference().getOrientation().col(1) * pair->getContactForce()(1);
            if (frictionDirections > 1)
              pair->getForce(1) += pair->getContourPointData()[0].getFrameOfReference().getOrientation().col(2) * pair->getContactForce()(2);
          }
          pair->getForce(0) = -pair->getForce(1);
          for (size_t i = 0; i < 2; i++) { //add forces to h-vector of both contours of the current contact point
            h[k][2 * potentialContact + i] += pair->getContourPointData()[i].getFrameOfReference().getJacobianOfTranslation(k).T() * pair->getForce(i);
          }

          //increase for next active contacts
          activeContact++;
        }
      }
    }
  }

  /**
   * \brief updates the rigid-body distances for all contact-points
   */
  void MaxwellContact::updateg(double t) {
    for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
      (*iter)->updateg(t);
    }
  }

  /**
   * \brief updates the velocities on active contact points
   */
  void MaxwellContact::updategd(double t) {
    for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
      (*iter)->updategd(t);
    }
  }

  void MaxwellContact::updateJacobians(double t, int j) {
    for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); ++iter) {
      (*iter)->updateJacobians(t,j);
    }
  }

  /**
   * \brief references the h and hLink vectors of the single contours to the global vector
   */
  void MaxwellContact::updatehRef(const Vec& hParent, int j) {
    for (size_t i = 0; i < contour.size(); i++) {
      int hInd = contour[i]->gethInd(j);
      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
      h[j][i].resize() >> hParent(I);
    }
  }

  void MaxwellContact::init(InitStage stage) {
    //    LinkMechanics::init(stage); //TODO: Ist Reihenfolge wichtig, wenn ja, wo ist das dokumentiert --> Kommentar in extradynamic-interface !
    if (stage == MBSim::preInit) {
      //initialise each contour-pairing
      for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); ++iter) {
        (*iter)->init(stage);
        //(*iter)->setFrictionForceLaw(fdf);
      }
    }
    else if (stage == resize) {
      gdSize = 0;
      for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
        gdSize  += 1 + (*iter)->getFrictionDirections();
      }

      g.resize(contourPairing.size());
      gd.resize(gdSize);
      la.resize(gdSize);

      for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); ++iter) {
        (*iter)->init(stage);
      }
    }
    else if (stage == MBSim::plot) {
      updatePlotFeatures();
      for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
        (*iter)->setParent(parent);
      }
      if (getPlotFeature(plotRecursive) == enabled) {
        for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++)
          (*iter)->init(stage);
      }
    }
    else if (stage == MBSim::unknownStage) {

      // reference to the positions of (full) vectors of link-class
      int forceIndex = 0; //index that adds all force directions together
      for (size_t k = 0; k < contourPairing.size(); k++) {
        contourPairing[k]->getRelativeDistance().resize() >> g(k, k);
        contourPairing[k]->getRelativeVelocity().resize() >> gd(forceIndex, forceIndex + 1 + contourPairing[k]->getFrictionDirections() - 1);
        contourPairing[k]->getContactForce().resize() >> la(forceIndex, forceIndex + 1 + contourPairing[k]->getFrictionDirections() - 1);

        forceIndex += 1 + contourPairing[k]->getFrictionDirections();
      }
    }
    //As last step: do init-step of LinkMechanics
    LinkMechanics::init(stage);
  }

  void MaxwellContact::checkActiveg() {
    for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
      (*iter)->checkActiveg();
    }
  }

  void MaxwellContact::setParent(DynamicSystem* sys) {
    Link::setParent(sys);
    for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
      (*iter)->setParent(sys);
    }
  }

  /**
   * \brief plot-routine for output
   */
  void MaxwellContact::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
      for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
        (*iter)->plot(t, dt);
      }
    }
  LinkMechanics::plot(t, dt);
}

  void MaxwellContact::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      for (vector<ContourPairing*>::iterator iter = contourPairing.begin(); iter != contourPairing.end(); iter++) {
        (*iter)->closePlot();
      }
      LinkMechanics::closePlot();
    }
  }

  void MaxwellContact::addContourPairing(ContourPairing* contourPairing_) { //(Contour *contour1, Contour *contour2, string name_, bool plotContact /*= true*/, ContactKinematics * contactKinematics_ /* = 0*/) {
    //append contactKinematics to vector
    contourPairing.push_back(contourPairing_);

    //append contours 1 and 2 to list
    //TODO: optimization may be possible by adding one contour just once --> Problem: referencing to the contour gets harder
    LinkMechanics::connect(contourPairing_->getContour(0));
    LinkMechanics::connect(contourPairing_->getContour(1));
  }

  void MaxwellContact::addContourCoupling(Contour *contour1, Contour *contour2, InfluenceFunction *fct) {
    pair<Contour*, Contour*> Pair(contour1, contour2);
    if (contour2 < contour1)
      Pair = pair<Contour*, Contour*>(contour2, contour1);
    if (!influenceFunctions.count(Pair)) {
      influenceFunctions[Pair] = fct;
    }
    else {
      cout << "WARNING: Function existed for contour-pair: \"" << contour1->getName() << "\" + \"" << contour2->getName() << "\"." << endl;
      cout << "         No Function has been added." << endl;
    }

  }

  void MaxwellContact::updatePossibleContactPoints() {
    checkActiveg(); //TODO: maybe is another location more reasonable. checkActiveg is normally called "global" in the dynamic_system_solver-class for all SetValued-Force laws. This is not ideal for the MaxwellContact so it calls this function on its own. why shouldn't call each link its own update-routine and the dynamic-system-solve is not forced to do it?
    possibleContactPoints.clear();
    for (size_t i =0; i < contourPairing.size(); i++) {
      if (contourPairing[i]->isActive()) {
        possibleContactPoints.push_back(i);
      }
    }
  }

  void MaxwellContact::updateInfluenceMatrix(const double t) {
    C.resize(possibleContactPoints.size());

    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      //get contours of current possible contactPoint
      int currentContactNumber = possibleContactPoints[i];

      C(i, i) = computeInfluenceCoefficient(currentContactNumber);

      for (size_t j = i + 1; j < possibleContactPoints.size(); j++) {
        //get coupled contours
        int coupledContactNumber = possibleContactPoints[j];

        /*coupling of first current contour*/
        C(i, j) = computeInfluenceCoefficient(currentContactNumber, coupledContactNumber);
      }
    }

    if (DEBUGLEVEL >= 5) {
      cout << "The InfluenceMatrix is: " << C << endl;
      cout << "With eigenvalues: " << eigval(C) << endl;
    }
  }

  double MaxwellContact::computeInfluenceCoefficient(const int &currentContactNumber) {
    double FactorC = 0.;

    int currentContourNumber = 2 * currentContactNumber;

    for (int i = 0; i < 2; i++) {

      //get involved contours
      Contour *contour1 = contour[currentContourNumber + i];
      Contour *contour2 = contour[currentContourNumber + i];
      pair<Contour*, Contour*> Pair;

      if (contour1 < contour2)
        Pair = pair<Contour*, Contour*>(contour1, contour2);
      else
        Pair = pair<Contour*, Contour*>(contour2, contour1);

      if (influenceFunctions.count(Pair)) { //If there is a function, there is a coupling between these contours
        InfluenceFunction *fct = influenceFunctions[Pair];
        /*TODO_grundl: solve the problem of the ordering input parameters for the function (for now it isn't really solved ..)
         maybe add a MaxwellCouplingFunction class that is derived from Function2 but has these identifiers --> Identifiers are now the (short)names*/
        Vec firstLagrangeParameter;
        Vec secondLagrangeParameter;
        if (contour1->getShortName() == (*fct).getFirstContourName() and contour2->getShortName() == (*fct).getSecondContourName()) {
          firstLagrangeParameter.resize() = contour1->computeLagrangeParameter(contourPairing[currentContactNumber]->getContourPointData()[i].getFrameOfReference().getPosition());
          secondLagrangeParameter.resize() = contour2->computeLagrangeParameter(contourPairing[currentContactNumber]->getContourPointData()[i].getFrameOfReference().getPosition());
        }
        else
          throw MBSimError("MaxwellContact::computeFactorC: The contours \"" + contour1->getShortName() + "\" and \"" + contour2->getShortName() + " don't fit with the function's contour names: \"" + (*fct).getFirstContourName() + "\" and \"" + (*fct).getSecondContourName() + "\"");

        if (DEBUGLEVEL >= 3) {
          cout << "First LagrangeParameter of contour \"" << contour1->getShortName() << "\" is:" << firstLagrangeParameter << endl;
          cout << "Second LagrangeParameter contour \"" << contour2->getShortName() << "\" is:" << secondLagrangeParameter << endl;
        }

        FactorC += (*fct)(firstLagrangeParameter, secondLagrangeParameter);
      }
    }

    if (fabs(FactorC) <= macheps()) {
      throw MBSimError("No elasticity is given for one of the following contours:\n  -" + contour[currentContourNumber]->getShortName() + "\n  -" + contour[currentContourNumber + 1]->getShortName() + "\nThat is not an option!");
    }

    return FactorC;
  }

  double MaxwellContact::computeInfluenceCoefficient(const int &affectedContactNumber, const int &coupledContactNumber) {
    double FactorC = 0;

    int affectedContourNumber = 2 * affectedContactNumber;
    int coupledContourNumber = 2 * coupledContactNumber;

    for (int affectedContourIterator = 0; affectedContourIterator < 2; affectedContourIterator++) {
      for (int coupledContourIterator = 0; coupledContourIterator < 2; coupledContourIterator++) {
        //get involved contours
        Contour *contour1 = contour[affectedContourNumber + affectedContourIterator];
        Contour *contour2 = contour[coupledContourNumber + coupledContourIterator];

        pair<Contour*, Contour*> Pair;

        if (contour1 < contour2)
          Pair = pair<Contour*, Contour*>(contour1, contour2);
        else
          Pair = pair<Contour*, Contour*>(contour2, contour1);

        if (influenceFunctions.count(Pair)) { //If there is a function, there is a coupling between these contours
          InfluenceFunction *fct = influenceFunctions[Pair];
          /*TODO_grundl: solve the problem of the ordering for the function input parameters (for now it isn't really solved ..)
           maybe add a MaxwellCouplingFunction class that is derived from Function2 but has these identifiers*/
          Vec firstLagrangeParameter = Vec(2, NONINIT);
          Vec secondLagrangeParameter = Vec(2, NONINIT);
          if (contour1->getShortName() == (*fct).getFirstContourName() and contour2->getShortName() == (*fct).getSecondContourName()) {
            firstLagrangeParameter = contour1->computeLagrangeParameter(contourPairing[affectedContactNumber]->getContourPointData()[affectedContourIterator].getFrameOfReference().getPosition());
            secondLagrangeParameter = contour2->computeLagrangeParameter(contourPairing[coupledContactNumber]->getContourPointData()[coupledContourIterator].getFrameOfReference().getPosition());
          }
          else if (contour2->getShortName() == (*fct).getFirstContourName() and contour1->getShortName() == (*fct).getSecondContourName()) {
            secondLagrangeParameter = contour1->computeLagrangeParameter(contourPairing[affectedContactNumber]->getContourPointData()[affectedContourIterator].getFrameOfReference().getPosition());
            firstLagrangeParameter = contour2->computeLagrangeParameter(contourPairing[coupledContactNumber]->getContourPointData()[coupledContourIterator].getFrameOfReference().getPosition());
          }
          else
            throw MBSimError("MaxwellContact::computeFactorC: The contours \"" + contour1->getShortName() + "\" and \"" + contour2->getShortName() + " don't fit with the function's contour names: \"" + (*fct).getFirstContourName() + "\" and \"" + (*fct).getSecondContourName() + "\"");

          if (DEBUGLEVEL >= 3) {
            cout << "First LagrangeParameter of contour \"" << contour1->getShortName() << "\" is:" << firstLagrangeParameter << endl;
            cout << "Second LagrangeParameter contour \"" << contour2->getShortName() << "\" is:" << secondLagrangeParameter << endl;
          }

          FactorC += (*fct)(firstLagrangeParameter, secondLagrangeParameter);
        }
      }
    }
    return FactorC;
  }

  void MaxwellContact::computeMaterialConstant(const double & t) {
    if (!matConstSetted && t > 0.) {
      /*update Material constant*/
      Vec Eigvals = eigval(C);
      double eigvalSum = 0;
      for (int i = 0; i < Eigvals.size(); i++) {
        eigvalSum += Eigvals(i);
      }
      matConst = eigvalSum / Eigvals.size();

      matConstSetted = true;
    }
  }

  Vec MaxwellContact::solveLCP(const SymMat & M, const Vec & q, const unsigned int & LemkeSteps /* = 10000 */) {

    clock_t t_start = clock();
    /*dimension of the system*/
    size_t dimension = q.size();

	  if(DEBUGLEVEL >= 1) {
		  cout << "*****" << name << ":" << __func__ << "*****" << endl;
		  if(DEBUGLEVEL >= 2) {
		    cout << "Systemdimension is: " << dimension << endl;
		  }
	  }

    /*create the Maxwell Function*/
    MaxwellFunction func = MaxwellFunction(q, M);
    func.setSolverType(NewtonMethodSolver);

    //Define different solvers
    MultiDimNewtonMethod NewtonSolver(&func);
    NewtonSolver.setMaximumNumberOfIterations((int)1e4);
    MultiDimFixPointIteration FixpointIterator(&func);
    FixpointIterator.setNumberOfMaximalIterations((int)1e6);
    FixpointIterator.setTolerance(1e-4);
    LemkeAlgorithm LemkeSolver;

    /*Initialize gapLamba*/
    Vec solution0(dimension * 2, INIT, 0.);
    for (size_t i = 0; i < dimension; i++) {
      if (q(i) > 0)
        solution0(i) = q(i);
      else
        solution0(i) = 0;
    }

    for (size_t i = dimension; i < 2 * dimension; i++) {
      if (q(i - dimension) > 0)
        solution0(i) = 0;
      else
        solution0(i) = -q(i - dimension) / matConst;
    }

    if (DEBUGLEVEL >= 5) {
      cout << "solution0: " << solution0 << endl;
      cout << "nrm2(f(solution0)): " << nrm2(func(solution0)) << endl;
    }

    /*create vector for solution*/
    Vec solution(solution0.copy());

    //TODO_grundl build Jacobian, if it is possible at all --> if it is computed numerically it gets more unstable?
    MaxwellJacobian jac;

    //NOTE: Fortran-NewtonSolver seems to work worse than the "native" NewtonSolver of the utils in mbsim
    //      MultiDimNewtonMethodFortran NewtonSolverFortran;
    //      NewtonSolverFortran.setRootFunction(&func);
    //      Vec gapLambdaFortran = NewtonSolverFortran.solve(gapLambda0);

    /* solve the LCP */
    bool converged = false;
    bool stop = false;

    //use Lemke-Solver for small systems

    LemkeSolver.setSystem(M, q);

    clock_t t_start_Lemke1 = clock();
    solution = LemkeSolver.solve(LemkeSteps);


    if (LemkeSolver.getInfo() == 0) {
      converged = true;
      if (DEBUGLEVEL >= 1) {
        cout << "LemkerSolver found solution" << endl;
        if (DEBUGLEVEL >= 2) {
          double cpuTime = double(clock()-t_start_Lemke1)/CLOCKS_PER_SEC;
          cout << "... in: " << cpuTime << "s = " << cpuTime/3600 << "h" << endl;
          if (DEBUGLEVEL >= 4) { //Lemke-Solver Info
            cout << "solution: " << solution << endl;
            cout << "gaps       forces   " << endl;
            for (int i = 0; i < solution.size() / 2; i++) {
              cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
            }
          }
        }
      }
    }
    else {
      solution = solution0;
      if (DEBUGLEVEL >= 1) {
        cout << "No Solution for Lemke-Algorithm. Now Trying combination of Newton + FixpointIteration" << endl;
      }
    }

    Vec lambda;
    lambda >> solution(dimension, 2 * dimension - 1);

    clock_t t_start_Iterative = clock();

    while (!stop and !converged) {

      //use Fixpoint-Iteration only once
      if (func.getSolverType() == FixPointIterationSolver) {
        stop = true;
        func.setSolverType(NewtonMethodSolver);
        NewtonSolver.setTolerance(1e-4);
      }

      solution = NewtonSolver.solve(solution);

      if (DEBUGLEVEL >= 3) {
        cout << "Info about NewtonSolver" << endl;
        cout << "nrm2(f(solution)):  " << nrm2(func(solution)) << endl;
        if (DEBUGLEVEL >= 4) { //Newton-Solver Info
          cout << "solution: " << solution << endl;
          cout << "gaps       forces   " << endl;
          for (int i = 0; i < solution.size() / 2; i++) {
            cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
          }
        }
      }

      switch (NewtonSolver.getInfo()) {
        case 0:
          converged = true;
          if (DEBUGLEVEL >= 1) {
            cout << "Newton-Solver found solution" << endl;
            if (DEBUGLEVEL >= 2) {
              double cpuTime = double(clock()-t_start_Iterative)/CLOCKS_PER_SEC;
              cout << "... in: " << cpuTime << "s = " << cpuTime/3600 << "h" << endl;
              if (DEBUGLEVEL >= 4) { //Newton-Solver Info
                cout << "solution: " << solution << endl;
                cout << "gaps       forces   " << endl;
                for (int i = 0; i < solution.size() / 2; i++) {
                  cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
                }
              }
            }
          }
        break;
        case 1: //convergence is active, but not enough steps
          if (DEBUGLEVEL >= 3) {
            cout << "Newton scheme seems to converge but has'nt finished" << endl;
            cout << "Increasing number of maximal iterations ... " << endl;
            vector<double> norms = NewtonSolver.getNorms();
            cout << "Current Norm is: " << norms[norms.size() - 1] << endl;
          }
          NewtonSolver.setMaximumNumberOfIterations(10 * NewtonSolver.getNumberOfMaximalIterations()); //raise number of iterations (if there is convergence ...)
        break;
        case -1:
          if (!stop) {
            if (DEBUGLEVEL >= 3) {
              cout << "MaxwellContact::solveLCP(double t): No convergence of Newton scheme during calculation of contact forces" << endl;
              cout << "Trying a Fixpoint-solver (at least for a good starting value for the Newton scheme) ..." << endl;
            }

            func.setSolverType(FixPointIterationSolver);
            solution = FixpointIterator.solve(solution);

            if (DEBUGLEVEL >= 3) {
              cout << "Info about FixpointSolver" << endl;
              func.setSolverType(NewtonMethodSolver);
              cout << "nrm2(f(solution)):  " << nrm2(func(solution)) << endl;
              if (DEBUGLEVEL >= 4) {
                cout << "solution: " << solution << endl;
                cout << "gaps       forces   " << endl;
                for (int i = 0; i < solution.size() / 2; i++) {
                  cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
                }
              }
              func.setSolverType(FixPointIterationSolver);
            }

            if (FixpointIterator.getInfo() == 0) {
              converged = true;
              if (DEBUGLEVEL >= 1) {
                cout << "Fixpoint-Iterator found solution" << endl;
                if (DEBUGLEVEL >= 2) {
                  double cpuTime = double(clock()-t_start_Iterative)/CLOCKS_PER_SEC;
                  cout << "... in: " << cpuTime << "s = " << cpuTime/3600 << "h" << endl;
                  if (DEBUGLEVEL >= 4) {
                    cout << "solution: " << solution << endl;
                    cout << "gaps       forces   " << endl;
                    for (int i = 0; i < solution.size() / 2; i++) {
                      cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
                    }
                  }
                }
              }
            }
          }

        break;
        default:
          throw MBSimError("ERROR MaxwellContact::solveLCP(double t): No convergence during calculation of contact forces with Newton scheme!");
        break;
      }
    }

    if (!converged) {
      if (DEBUGLEVEL >= 1) {
        cout << "No convergence during calculation of contact forces with Newton scheme!" << endl;
        cout << "Now using Lemke Algorithm..." << endl;
      }

      clock_t t_start_Lemke1 = clock();
      solution = LemkeSolver.solve();

      if (LemkeSolver.getInfo() == 0) {
        converged = true;
        if (DEBUGLEVEL >= 1) {
          cout << "LemkerSolver found solution with the second try ..." << endl;
          if (DEBUGLEVEL >= 2) {
            double cpuTime = double(clock()-t_start_Lemke1)/CLOCKS_PER_SEC;
            cout << "... in: " << cpuTime << "s = " << cpuTime/3600 << "h" << endl;
            if (DEBUGLEVEL >= 4) { //Newton-Solver Info
              cout << "solution: " << solution << endl;
              cout << "gaps       forces   " << endl;
              for (int i = 0; i < solution.size() / 2; i++) {
                cout << solution(i) << " | " << solution(i + solution.size() / 2) << endl;
              }
            }
          }
        }
      }

    }

    if (!converged) {
      throw MBSimError("ERROR MaxwellContact::solveLCP(): No Solution found for this LCP");
    }

    if (DEBUGLEVEL >= 1) {
      double cpuTime = double(clock()-t_start)/CLOCKS_PER_SEC;
      cout << "Solution found in: " << cpuTime << "s = " << cpuTime/3600 << "h" << endl;
    }


    return lambda;
  }

  /* END - MaxwellContact   ***************************************************************************************************************************/

  /* MaxwellFunction   ***************************************************************************************************************************/

  MaxwellFunction::MaxwellFunction(const fmatvec::Vec & rigidBodyGap_, const SymMat &C_, const double &r_ /*= 10*/, bool INFO_ /*= false*/) :
      NumberOfContacts(rigidBodyGap_.size()), rigidBodyGap(rigidBodyGap_), C(C_), r(r_), solverType(NewtonMethodSolver), INFO(INFO_) {

    //dimensions have to be equal
    assert(C_.size() == NumberOfContacts);
  }

  MaxwellFunction::~MaxwellFunction() {

  }

  Vec MaxwellFunction::operator ()(const Vec &gapLambda, const void *) {

    //check dimensions
    assert(gapLambda.size() == 2 * NumberOfContacts);

    Vec returnVec(2 * NumberOfContacts, INIT, 0.);

    //reference to gap and lambda
    Vec gap;
    Vec lambda;
    gap << gapLambda(0, NumberOfContacts - 1);
    lambda << gapLambda(NumberOfContacts, 2 * NumberOfContacts - 1);

    if (INFO) {

      cout << "gap is: " << gap << endl;
      cout << "lambda is: " << lambda << endl;

      cout << "C is: " << C << endl;
    }

    //compute first part
    if (solverType == NewtonMethodSolver) {
      returnVec(0, NumberOfContacts - 1) = rigidBodyGap + C * lambda - gap;

      //loop for the prox-functions
      for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
        returnVec(NumberOfContacts + contactIterator) = proxCN(lambda(contactIterator) - r * gap(contactIterator)) - lambda(contactIterator);
        if (INFO) {
          cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(lambda(contactIterator) - r * gap(contactIterator)) << "- " << lambda(contactIterator) << endl;
          cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << lambda(contactIterator) << "-" << r << "*" << gap(contactIterator) << endl;
        }
      }
    }
    else if (solverType == FixPointIterationSolver) {
      returnVec(0, NumberOfContacts - 1) = rigidBodyGap + C * lambda;
      //loop for the prox-functions
      for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
        returnVec(NumberOfContacts + contactIterator) = proxCN(lambda(contactIterator) - r * gap(contactIterator));
        if (INFO) {
          cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(lambda(contactIterator) - r * gap(contactIterator)) << "- " << lambda(contactIterator) << endl;
          cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << lambda(contactIterator) << "-" << r << "*" << gap(contactIterator) << endl;
        }
      }
    }

    if (INFO)
      cout << "returnVec is: " << returnVec << endl;

    return returnVec;

  }

  /* END - MaxwellFunction   ***************************************************************************************************************************/

  /* MaxwellJacobian   ***************************************************************************************************************************/

  MaxwellJacobian::MaxwellJacobian() {

  }

  MaxwellJacobian::~MaxwellJacobian() {

  }

  SqrMat MaxwellJacobian::operator ()(const Vec &distance, const void *) {
    SqrMat returnMat(3, INIT, 0.);

    return returnMat;
  }

/* END - MaxwellJacobian   ***************************************************************************************************************************/
}

