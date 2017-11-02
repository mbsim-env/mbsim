/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h> 
#include "maxwell_contact.h"

#include <mbsim/contours/contour.h>
#include <mbsim/constitutive_laws/maxwell_unilateral_constraint.h>
#include <mbsim/functions/kinetics/influence_function.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/utils.h>
#include <mbsim/objectfactory.h>
#include <openmbvcppinterface/group.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>

#include <algorithm>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MaxwellContact)

  MaxwellContact::MaxwellContact(const string &name) : Contact(name), LCP(SymMat(0,NONINIT), Vec(0,NONINIT)), dampingCoefficient(0.), gLim(0.), matConst(0), matConstSetted(false), DEBUGLEVEL(0) {
  }

  MaxwellContact::~MaxwellContact() {
    for (map<pair<Contour*, Contour*>,InfluenceFunction*>::iterator it=influenceFunctions.begin(); it!=influenceFunctions.end(); ++it)
      delete it->second;
  }

  void MaxwellContact::initializeContourCouplings() {
    for(size_t i = 0; i < referenceXML.size(); i++) {
      Contour* contour1 = getByPath<Contour>(referenceXML[i].name1);
      Contour* contour2 = getByPath<Contour>(referenceXML[i].name2);
      addContourCoupling(contour1, contour2, referenceXML[i].function);
    }
  }

  void MaxwellContact::addContourCoupling(Contour *contour1, Contour *contour2, InfluenceFunction *fct) {
    pair<Contour*, Contour*> Pair(contour1, contour2);
    if (contour2 < contour1)
      Pair = pair<Contour*, Contour*>(contour2, contour1);
    if (!influenceFunctions.count(Pair)) {
      influenceFunctions[Pair] = fct;
      influenceFunctions[Pair]->setParent(this);
    }
    else {
      msg(Warn) << "Function existed for contour-pair: \"" << contour1->getPath() << "\" + \"" << contour2->getPath() << "\".\n" <<
                   "No Function has been added." << endl;
    }

  }

  void MaxwellContact::init(InitStage stage, const InitConfigSet &config) {
    if (stage ==resolveStringRef) {

      // initialize all contour couplings if generalized force law is of maxwell-type
      initializeContourCouplings();

      Contact::init(stage, config);
    }
    else if (stage == preInit) {
      Contact::init(stage, config);
      setNormalForceLaw(new MaxwellUnilateralConstraint);
      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->setNormalForceLaw(fcl);
      }
   }
    else
      Contact::init(stage, config);
  }

  void MaxwellContact::updateGeneralizedNormalForce() {

    Contact::updateGeneralizedNormalForce();

    updatePossibleContactPoints();

    //Apply damping force
    //TODO: use damping function for that (to be more flexible...)
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        if ((*jter).evalGeneralizedRelativePosition()(0) < gLim and (*jter).evalGeneralizedRelativeVelocity()(0) < 0)
          (*jter).getGeneralizedNormalForce(false) = -dampingCoefficient * (*jter).evalGeneralizedRelativeVelocity()(0);
        else
          (*jter).getGeneralizedNormalForce(false) = 0;
      }
    }

    if (possibleContactPoints.size()) {
      updateInfluenceMatrix();
      updateRigidBodyGap();

      LCP.setSystem(C, rigidBodyGap);

      map<RangeV, double> tolerances;
      tolerances.insert(pair<RangeV, double>(RangeV(0, possibleContactPoints.size() - 1), 1e-8)); //tolerances for distances
      tolerances.insert(pair<RangeV, double>(RangeV(possibleContactPoints.size(), 2 * possibleContactPoints.size() - 1), 1e-3)); //tolerances for forces
      LocalResidualCriteriaFunction critfunc(tolerances);
      LCP.setNewtonCriteriaFunction(&critfunc);
      LCP.setDebugLevel(0);

      solution0.resize() = LCP.solve(solution0);

      Vec lambda = solution0(rigidBodyGap.size(), 2 * rigidBodyGap.size() - 1);

      if (DEBUGLEVEL >= 3) {
        cout << "lambda = " << lambda << endl;
      }

      for (size_t i = 0; i < possibleContactPoints.size(); ++i) {
        contacts[possibleContactPoints[i].first][possibleContactPoints[i].second].getGeneralizedNormalForce(false) += lambda(i);
      }
    }
  }

  void MaxwellContact::updatePossibleContactPoints() {
    possibleContactPoints.clear();
    for (size_t i = 0; i < contacts.size(); ++i) {
      for (size_t j = 0; j < contacts[i].size(); ++j) {
        if (contacts[i][j].evalGeneralizedRelativePosition()(0) < 0) { //TODO: use gActive, but only at timestep No 2...
          possibleContactPoints.push_back(pair<int,int>(i,j));
        }
      }
    }
  }

  void MaxwellContact::updateInfluenceMatrix() {
    C.resize(possibleContactPoints.size());

    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      //get index of contours of current possible contactPoint
      const std::pair<int, int> & currentContactIndex = possibleContactPoints[i];

      C(i, i) = computeInfluenceCoefficient(currentContactIndex);

      for (size_t j = i + 1; j < possibleContactPoints.size(); j++) {
        //get index of coupled contour
        const std::pair<int, int> & coupledContactIndex = possibleContactPoints[j];

        C(i, j) = computeInfluenceCoefficient(currentContactIndex, coupledContactIndex);
      }
    }

    if (DEBUGLEVEL >= 5) {
      cout << "The InfluenceMatrix is: " << C << endl;
      cout << "With eigenvalues: " << eigval(C) << endl;
    }
  }

  void MaxwellContact::updateRigidBodyGap() {
    /*save rigidBodyGaps in vector*/
    rigidBodyGap.resize(possibleContactPoints.size());
    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      rigidBodyGap(i) = contacts[possibleContactPoints[i].first][possibleContactPoints[i].second].evalGeneralizedRelativePosition()(0);
    }

    if (DEBUGLEVEL >= 5)
      cout << "rigidBodyGap: " << rigidBodyGap << endl;
  }

  double MaxwellContact::computeInfluenceCoefficient(const std::pair<int, int> & contactIndex) {
    double FactorC = 0.;

    for (int i = 0; i < 2; i++) {

      //get involved contours
      Contour * contour = contacts[contactIndex.first][contactIndex.second].getContour(i);
      pair<Contour*, Contour*> contourPair = pair<Contour*, Contour*>(contour, contour);

      if (influenceFunctions.count(contourPair)) { //If there is a function, there is a coupling between these contours
        InfluenceFunction *fct = influenceFunctions[contourPair];
        pair<Contour*, ContourFrame*> contInfo(contour, contacts[contactIndex.first][contactIndex.second].getContourFrame(i));
        //        Vec2 lagrangeParameter = contour->getLagrangeParameter(contacts[contactIndex.first][contactIndex.second].getcpData()[i].getFrameOfReference().getPosition());

        //        if (DEBUGLEVEL >= 3) {
        //          cout << "LagrangeParameter of contour \"" << contour->getPath() << "\" is:" << lagrangeParameter << endl;
        //        }

        FactorC += (*fct)(contInfo, contInfo);
      }
    }

    if (fabs(FactorC) <= macheps()) {
      throw MBSimError("No elasticity is given for one of the following contours:\n  -" + contacts[contactIndex.first][contactIndex.second].getContour(0)->getPath() + "\n  -" + contacts[contactIndex.first][contactIndex.second].getContour(0)->getPath() + "\nThat is not an option!");
    }

    return FactorC;
  }

  double MaxwellContact::computeInfluenceCoefficient(const std::pair<int, int> & contactIndex, const std::pair<int, int> & coupledContactIndex) {
    double FactorC = 0;

    for (int affectedContourIterator = 0; affectedContourIterator < 2; affectedContourIterator++) {
      for (int coupledContourIterator = 0; coupledContourIterator < 2; coupledContourIterator++) {
        //get involved contours
        Contour *contour1 = contacts[contactIndex.first][contactIndex.second].getContour(affectedContourIterator);
        Contour *contour2 = contacts[coupledContactIndex.first][coupledContactIndex.second].getContour(coupledContourIterator);

        pair<Contour*, Contour*> Pair;

        if (contour1 < contour2)
          Pair = pair<Contour*, Contour*>(contour1, contour2);
        else
          Pair = pair<Contour*, Contour*>(contour2, contour1);

        if (influenceFunctions.count(Pair)) { //If there is a function, there is a coupling between these contours
          InfluenceFunction *fct = influenceFunctions[Pair];
          //          Vec2 firstLagrangeParameter = contour1->getLagrangeParameter(contacts[contactIndex.first][contactIndex.second].getcpData()[affectedContourIterator].getFrameOfReference().getPosition());
          //          Vec2 secondLagrangeParameter = contour2->getLagrangeParameter(contacts[coupledContactIndex.first][coupledContactIndex.second].getcpData()[coupledContourIterator].getFrameOfReference().getPosition());
          pair<Contour*, ContourFrame*> cont1Info(contour1, contacts[contactIndex.first][contactIndex.second].getContourFrame(affectedContourIterator));
          pair<Contour*, ContourFrame*> cont2Info(contour2, contacts[coupledContactIndex.first][coupledContactIndex.second].getContourFrame(coupledContourIterator));

          //          if (DEBUGLEVEL >= 3) {
          //            cout << "First LagrangeParameter of contour \"" << contour1->getPath() << "\" is:" << firstLagrangeParameter << endl;
          //            cout << "Second LagrangeParameter contour \"" << contour2->getPath() << "\" is:" << secondLagrangeParameter << endl;
          //          }

          FactorC += (*fct)(cont1Info, cont2Info);
        }
      }
    }
    return FactorC;
  }

  void MaxwellContact::computeMaterialConstant() {
    if (!matConstSetted and possibleContactPoints.size()) {
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

  void MaxwellContact::initializeUsingXML(DOMElement *element) {
    Contact::initializeUsingXML(element);
    DOMElement *e;
    e = E(element)->getFirstElementChildNamed(MBSIM%"InfluenceFunction");
    while(e) {
      xmlInfo info;
      info.function = ObjectFactory::createAndInit<InfluenceFunction>(e->getFirstElementChild());
      info.name1 = E(e->getFirstElementChild())->getAttribute("contourName1");
      info.name2 = E(e->getFirstElementChild())->getAttribute("contourName2");

      referenceXML.push_back(info);

      e = e->getNextElementSibling();
    }
  }

}
