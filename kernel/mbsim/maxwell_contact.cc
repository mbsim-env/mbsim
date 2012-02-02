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
#include <mbsim/utils/linear_complementarity_problem.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

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
      Vec lambda = solveLCP(C, rigidBodyGap, MBSim::Standard, matConst)(rigidBodyGap.size(), 2 * rigidBodyGap.size()-1);

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
}

