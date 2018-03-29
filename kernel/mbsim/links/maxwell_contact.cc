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
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/constitutive_laws/friction_force_law.h>
#include <mbsim/constitutive_laws/generalized_impact_law.h>
#include <mbsim/constitutive_laws/friction_impact_law.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
#include <mbsim/objectfactory.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/constitutive_laws/maxwell_unilateral_constraint.h>
#include <mbsim/functions/kinetics/influence_function.h>
#include <algorithm>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MaxwellContact)

  MaxwellContact::MaxwellContact(const string &name) : Link(name), contacts(0), contactKinematics(0), contour(2), ckNames(0), plotFeatureMap(), fcl(0), fdf(0), fnil(0), ftil(0), searchAllCP(false), tol(1e-10), LCP(SymMat(0,NONINIT), Vec(0,NONINIT)), dampingCoefficient(0.), gLim(0.), matConst(0), matConstSetted(false), saved_ref(0) {
  }

  MaxwellContact::~MaxwellContact() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
//      for (int k = 0; k < contactKinematics[cK]->getMaximumNumberOfContacts(); ++k)
//        delete contactKinematics[cK]->getContactKinematics(k);
      delete contactKinematics[cK];
    }
    delete fcl;
    delete fdf;
    delete fnil;
    delete ftil;
    for (map<pair<Contour*, Contour*>,InfluenceFunction*>::iterator it=influenceFunctions.begin(); it!=influenceFunctions.end(); ++it)
      delete it->second;
  }

  void MaxwellContact::setPlotFeatureContactKinematics(std::string cKName, std::size_t pf, bool value) {
    if (ckNames.end() != find(ckNames.begin(), ckNames.end(), cKName)) {
      pair<string, std::size_t> Pair(cKName, pf);
      plotFeatureMap.insert(pair<pair<string, std::size_t>, bool>(Pair, value));
    }
  }

  void MaxwellContact::updatewb() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter)
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatewb();
  }

  void MaxwellContact::updateW(int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateW(j);
    }
  }

  void MaxwellContact::updateV(int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateV(j);
    }
  }

  void MaxwellContact::updateh(int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateh(j);
    }
  }

  void MaxwellContact::updateg() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateg();
    }
  }

  void MaxwellContact::updategd() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updategd();
    }
  }

  void MaxwellContact::updateStopVector() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateStopVector();
    }
  }

  void MaxwellContact::updateJacobians(int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateJacobians(j);
    }
  }

  void MaxwellContact::updateWRef(const Mat& WParent, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateWRef(WParent, j);
    }
  }

  void MaxwellContact::updateVRef(const Mat& VParent, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateVRef(VParent, j);
    }
  }

  void MaxwellContact::updatehRef(const Vec& hParent, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatehRef(hParent, j);
    }
  }

  void MaxwellContact::updaterRef(const Vec& rParent, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updaterRef(rParent, j);
    }
  }

  void MaxwellContact::updatewbRef(const Vec& wbParent) {
    Link::updatewbRef(wbParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatewbRef(wbParent);
    }
  }

  void MaxwellContact::updatelaRef(const Vec& laParent) {
    Link::updatelaRef(laParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatelaRef(laParent);
    }
  }

  void MaxwellContact::updateLaRef(const Vec& LaParent) {
    Link::updateLaRef(LaParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateLaRef(LaParent);
    }
  }

  void MaxwellContact::updategRef(const Vec& gParent) {
    Link::updategRef(gParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->updategRef(gParent);
      }
    }
  }

  void MaxwellContact::updategdRef(const Vec& gdParent) {
    Link::updategdRef(gdParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updategdRef(gdParent);
    }
  }

  void MaxwellContact::updateresRef(const Vec& resParent) {
    Link::updateresRef(resParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateresRef(resParent);
    }
  }

  void MaxwellContact::updaterFactorRef(const Vec& rFactorParent) {
    Link::updaterFactorRef(rFactorParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updaterFactorRef(rFactorParent);
    }
  }

  void MaxwellContact::updatesvRef(const Vec &svParent) {
    Link::updatesvRef(svParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatesvRef(svParent);
    }
  }

  void MaxwellContact::updatejsvRef(const VecInt &jsvParent) {
    Link::updatejsvRef(jsvParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatejsvRef(jsvParent);
    }
  }

  void MaxwellContact::updateLinkStatusRef(const VecInt &LinkStatusParent) {
    Link::updateLinkStatusRef(LinkStatusParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateLinkStatusRef(LinkStatusParent);
    }
  }

  void MaxwellContact::updateLinkStatusRegRef(const VecInt &LinkStatusRegParent) {
    Link::updateLinkStatusRegRef(LinkStatusRegParent);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateLinkStatusRegRef(LinkStatusRegParent);
    }
  }

  void MaxwellContact::calclaSize(int j) {
    Link::calclaSize(j);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calclaSize(j);
        laSize += jter->getlaSize();
      }
    }
  }

  void MaxwellContact::calcgSize(int j) {
    Link::calcgSize(j);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcgSize(j);
        gSize += jter->getgSize();
      }
    }
  }

  void MaxwellContact::calcgdSize(int j) {
    Link::calcgdSize(j);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcgdSize(j);
        gdSize += jter->getgdSize();
      }
    }
  }

  void MaxwellContact::calcrFactorSize(int j) {
    Link::calcrFactorSize(j);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcrFactorSize(j);
        rFactorSize += jter->getrFactorSize();
      }
    }
  }

  void MaxwellContact::calcsvSize() {
    Link::calcsvSize();
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcsvSize();
        svSize += jter->getsvSize();
      }
    }
  }

  void MaxwellContact::calcLinkStatusSize() {
    Link::calcLinkStatusSize();
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcLinkStatusSize();
        LinkStatusSize += jter->getLinkStatusSize();
      }
    }
    LinkStatus.resize(LinkStatusSize);
  }

  void MaxwellContact::calcLinkStatusRegSize() {
    Link::calcLinkStatusRegSize();
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcLinkStatusRegSize();
        LinkStatusRegSize += jter->getLinkStatusRegSize();
      }
    }
    LinkStatusReg.resize(LinkStatusRegSize);
  }

  void MaxwellContact::init(InitStage stage, const InitConfigSet &config) {
    if (stage ==resolveStringRef) {
      // initialize all contour couplings if generalized force law is of maxwell-type
      initializeContourCouplings();

      //connect all contours given in xml file
      for (size_t i = 0; i < saved_ref.size(); i++) {
        if (saved_ref[i].name1 != "" && saved_ref[i].name2 != "")
          connect(getByPath<Contour>(saved_ref[i].name1), getByPath<Contour>(saved_ref[i].name2), 0, saved_ref[i].contourPairingName);
        //TODO: add option to specifiy contact_kinematics
      }

      if(not(contour.size()))
        throwError("no connection given!");

      Link::init(stage, config);
    }
    else if (stage == preInit) {
      Link::init(stage, config);
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        contactKinematics[cK]->setSearchAllContactPoints(searchAllCP);
        contactKinematics[cK]->setInitialGuess(zeta0);
        contactKinematics[cK]->setTolerance(tol);
        contactKinematics[cK]->assignContours(contour[0][cK], contour[1][cK]);
        contacts.push_back(vector<SingleContact>());
        for (int k = 0; k < contactKinematics[cK]->getMaximumNumberOfContacts(); ++k) {
          stringstream contactName;
          contactName << ckNames[cK];
          if (contactKinematics[cK]->getMaximumNumberOfContacts() > 1)
            contactName << "_" << k;
          contacts[cK].push_back(SingleContact(contactName.str()));
          contacts[cK][k].connect(contour[0][cK], contour[1][cK]);
          contacts[cK][k].plotFeature = plotFeature;
          contacts[cK][k].plotFeatureForChildren = plotFeatureForChildren;

          //set the tolerances for the single contacts
          contacts[cK][k].setGeneralizedRelativePositionTolerance(gTol);
          contacts[cK][k].setGeneralizedRelativeVelocityTolerance(gdTol);
          contacts[cK][k].setGeneralizedRelativeAccelerationTolerance(gddTol);
          contacts[cK][k].setGeneralizedForceTolerance(laTol);
          contacts[cK][k].setGeneralizedImpulseTolerance(LaTol);
          contacts[cK][k].setrMax(rMax);
        }
      }

      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
          //set parent
          jter->setParent(this);

          //set contact laws for children
          jter->setNormalForceLaw(fcl);
          jter->setNormalImpactLaw(fnil);
          jter->setTangentialForceLaw(fdf);
          jter->setTangentialImpactLaw(ftil);

          jter->init(stage, config);
        }
      }
      setNormalForceLaw(new MaxwellUnilateralConstraint);
      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->setNormalForceLaw(fcl);
      }
    }
    else if (stage == plotting) {
      Element::init(stage, config);
      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->init(stage, config);
      }
    }
    else {
      Link::init(stage, config);
      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->init(stage, config);
      }
    }
    //Don't call init()-routines for "sub"-contacts with stage "LASTINITSTAGE" as here is checked if contactKinematics has more than one possible contact point, which is only possible in multi-contact
    if(fcl) fcl->init(stage, config);
    if(fdf) fdf->init(stage, config);
    if(fnil) fnil->init(stage, config);
    if(ftil) ftil->init(stage, config);
  }

  bool MaxwellContact::isSetValued() const {
    bool flag = fcl and fcl->isSetValued();
    if (fdf)
      flag |= fdf->isSetValued();
    return flag;
  }

  bool MaxwellContact::isSingleValued() const {
    if (fcl and fcl->isSetValued()) {
      if (fdf) {
        return not fdf->isSetValued();
      }
      return false;
    }
    return true;
  }

  void MaxwellContact::updateLinkStatus() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->updateLinkStatus();
      }
    }
  }

  void MaxwellContact::updateLinkStatusReg() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->updateLinkStatusReg();
      }
    }
  }

  bool MaxwellContact::isActive() const {
    for (std::vector<std::vector<SingleContact> >::const_iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::const_iterator jter = iter->begin(); jter != iter->end(); ++jter)
        if (jter->isActive())
          return true;
    }
    return false;
  }

  bool MaxwellContact::gActiveChanged() {
    bool changed = false;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        if (jter->gActiveChanged())
          changed = true;
    }
    return changed;
  }

  bool MaxwellContact::detectImpact() {
    bool impact = false;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        if (jter->detectImpact())
          impact = true;
    }
    return impact;
  }

  void MaxwellContact::plot() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->plot();
    }
  }

  void MaxwellContact::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Link::setDynamicSystemSolver(sys);

    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->setDynamicSystemSolver(ds);
    }
  }

  void MaxwellContact::setNormalForceLaw(GeneralizedForceLaw *fcl_) {
    fcl = fcl_;
    fcl->setParent(this);
  }

  void MaxwellContact::setNormalImpactLaw(GeneralizedImpactLaw *fnil_) {
    fnil = fnil_;
    fnil->setParent(this);
  }

  void MaxwellContact::setTangentialForceLaw(FrictionForceLaw *fdf_) {
    fdf = fdf_;
    fdf->setParent(this);
  }

  void MaxwellContact::setTangentialImpactLaw(FrictionImpactLaw *ftil_) {
    ftil = ftil_;
    ftil->setParent(this);
  }

  void MaxwellContact::setgInd(int gInd_) {
    Link::setgInd(gInd_);
    int nextgInd = gInd_;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setgInd(nextgInd);
        nextgInd += jter->getgSize();
      }
    }
  }

  void MaxwellContact::setgdInd(int gdInd_) {
    Link::setgdInd(gdInd_);
    int nextgdInd = gdInd_;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setgdInd(nextgdInd);
        nextgdInd += jter->getgdSize();
      }
    }
  }

  void MaxwellContact::setsvInd(int svInd_) {
    Link::setsvInd(svInd_);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setsvInd(svInd_);
        svInd_ += jter->getsvSize();
      }
    }
  }

  void MaxwellContact::setlaInd(int laInd_) {
    Link::setlaInd(laInd_);
    int nextlaInd = laInd_;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setlaInd(nextlaInd);
        nextlaInd += jter->getlaSize();
      }
    }
  }

  void MaxwellContact::setrFactorInd(int rFactorInd_) {
    Link::setrFactorInd(rFactorInd_);
    int nextrFactorInd = rFactorInd_;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setrFactorInd(nextrFactorInd);
        nextrFactorInd += jter->getrFactorSize();
      }
    }
  }

  void MaxwellContact::setcorrInd(int corrInd_) {
    Link::setcorrInd(corrInd_);
    int nextcorrInd = corrInd_;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setcorrInd(nextcorrInd);
        nextcorrInd += jter->getcorrSize();
      }
    }
  }

  void MaxwellContact::setLinkStatusInd(int LinkStatusInd_) {
    Link::setLinkStatusInd(LinkStatusInd_);
    int nextLinkStatusInd = LinkStatusInd_;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setLinkStatusInd(nextLinkStatusInd);
        nextLinkStatusInd += jter->getLinkStatusSize();
      }
    }
  }

  void MaxwellContact::setLinkStatusRegInd(int LinkStatusRegInd_) {
    Link::setLinkStatusRegInd(LinkStatusRegInd_);
    int nextLinkStatusRegInd = LinkStatusRegInd_;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setLinkStatusRegInd(nextLinkStatusRegInd);
        nextLinkStatusRegInd += jter->getLinkStatusRegSize();
      }
    }
  }

  void MaxwellContact::solveImpactsFixpointSingle() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsFixpointSingle();
    }
  }

  void MaxwellContact::solveConstraintsFixpointSingle() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsFixpointSingle();
    }
  }

  void MaxwellContact::solveImpactsGaussSeidel() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsGaussSeidel();
    }
  }

  void MaxwellContact::solveConstraintsGaussSeidel() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsGaussSeidel();
    }
  }

  void MaxwellContact::solveImpactsRootFinding() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsRootFinding();
    }
  }

  void MaxwellContact::solveConstraintsRootFinding() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsRootFinding();
    }
  }

  void MaxwellContact::jacobianConstraints() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->jacobianConstraints();
    }
  }

  void MaxwellContact::jacobianImpacts() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->jacobianImpacts();
    }
  }

  void MaxwellContact::updaterFactors() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updaterFactors();
    }
  }

  void MaxwellContact::checkConstraintsForTermination() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkConstraintsForTermination();
    }
  }

  void MaxwellContact::checkImpactsForTermination() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkImpactsForTermination();
    }
  }

  void MaxwellContact::checkActive(int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkActive(j);
    }
  }

  int MaxwellContact::getFrictionDirections() {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void MaxwellContact::connect(Contour *contour0, Contour* contour1, ContactKinematics* contactKinematics_ /*=0*/, const string & name_ /* ="" */) {
    contour[0].push_back(contour0);
    contour[1].push_back(contour1);
    contactKinematics.push_back(contactKinematics_);

    int cK = contactKinematics.size() - 1;

    if (contactKinematics[cK] == 0) {
      contactKinematics[cK] = contour0->findContactPairingWith(typeid(*contour0), typeid(*contour1));
      if (contactKinematics[cK] == 0) {
        contactKinematics[cK] = contour1->findContactPairingWith(typeid(*contour1), typeid(*contour0));
        if (contactKinematics[cK] == 0) {
          contactKinematics[cK] = contour0->findContactPairingWith(typeid(*contour1), typeid(*contour0));
          if (contactKinematics[cK] == 0) {
            contactKinematics[cK] = contour1->findContactPairingWith(typeid(*contour0), typeid(*contour1));
            if (contactKinematics[cK] == 0) {
              throwError("(MaxwellContact::init): Unknown contact pairing between Contour \"" + boost::core::demangle(typeid(*contour0).name()) + "\" and Contour \"" + boost::core::demangle(typeid(*contour1).name()) + "\"!");
            }
          }
        }
      }
    }

    //Create a single contact(with all the information) for every sub contact of each contact kinematics that is part of the multiple contact
    if (name_ == "")
      ckNames.push_back(name + "_" + to_string(cK));
    else
      ckNames.push_back(name_);

  }

  void MaxwellContact::LinearImpactEstimation(double t, Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->LinearImpactEstimation(t, gInActive_, gdInActive_, IndInActive_, gAct_, IndActive_);
    }
  }

  void MaxwellContact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->SizeLinearImpactEstimation(sizeInActive_, sizeActive_);
    }
  }

  void MaxwellContact::setGeneralizedForceTolerance(double tol) {
    Link::setGeneralizedForceTolerance(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setGeneralizedForceTolerance(tol);
      }
    }
  }

  void MaxwellContact::setGeneralizedImpulseTolerance(double tol) {
    Link::setGeneralizedImpulseTolerance(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setGeneralizedImpulseTolerance(tol);
      }
    }
  }

  void MaxwellContact::setGeneralizedRelativePositionTolerance(double tol) {
    Link::setGeneralizedRelativePositionTolerance(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setGeneralizedRelativePositionTolerance(tol);
      }
    }
  }

  void MaxwellContact::setGeneralizedRelativeVelocityTolerance(double tol) {
    Link::setGeneralizedRelativeVelocityTolerance(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setGeneralizedRelativeVelocityTolerance(tol);
      }
    }
  }

  void MaxwellContact::setGeneralizedRelativeAccelerationTolerance(double tol) {
    Link::setGeneralizedRelativeAccelerationTolerance(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setGeneralizedRelativeAccelerationTolerance(tol);
      }
    }
  }

  void MaxwellContact::setrMax(double rMax_) {
    Link::setrMax(rMax_);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setrMax(rMax_);
      }
    }
  }

  void MaxwellContact::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    DOMElement *e;

    /*Read all contour pairings*/
    //Get all contours, that should be connected
    e = E(element)->getFirstElementChildNamed(MBSIM%"connect"); //TODO: all connects must be in a row (is that okay?)
    while (e) { //As long as there are siblings read them and save them
      if (E(e)->getTagName() == MBSIM%"connect") {
        saved_references ref;
        ref.name1 = E(e)->getAttribute("ref1");
        ref.name2 = E(e)->getAttribute("ref2");
        if (E(e)->hasAttribute("name"))
          ref.contourPairingName = E(e)->getAttribute("name");
        else
          ref.contourPairingName = "";
        //TODO: add possibility of defining own contactKinematics? (also in Contact-class)

        saved_ref.push_back(ref);
        e = e->getNextElementSibling();
      }
      else {
        break;
      }
    }

    //Set contact law
    e = E(element)->getFirstElementChildNamed(MBSIM%"normalForceLaw");
    if(e) {
      GeneralizedForceLaw *gfl = ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild());
      setNormalForceLaw(gfl);
    }

    //Get Impact law
    e = E(element)->getFirstElementChildNamed(MBSIM%"normalImpactLaw");
    if (e) {
      GeneralizedImpactLaw *gifl = ObjectFactory::createAndInit<GeneralizedImpactLaw>(e->getFirstElementChild());
      setNormalImpactLaw(gifl);
    }

    //Get Friction Force Law
    e = E(element)->getFirstElementChildNamed(MBSIM%"tangentialForceLaw");
    if (e) {
      FrictionForceLaw *ffl = ObjectFactory::createAndInit<FrictionForceLaw>(e->getFirstElementChild());
      setTangentialForceLaw(ffl);
    }

    //Get Friction Impact Law
    e = E(element)->getFirstElementChildNamed(MBSIM%"tangentialImpactLaw");
    if (e) {
      FrictionImpactLaw *fil = ObjectFactory::createAndInit<FrictionImpactLaw>(e->getFirstElementChild());
      setTangentialImpactLaw(fil);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"searchAllContactPoints");
    if (e) setSearchAllContactPoints(E(e)->getText<bool>());

    e = E(element)->getFirstElementChildNamed(MBSIM%"initialGuess");
    if (e) setInitialGuess(E(e)->getText<Vec>());

    e = E(element)->getFirstElementChildNamed(MBSIM%"tolerance");
    if (e) setTolerance(E(e)->getText<double>());

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

  void MaxwellContact::updatecorrRef(const Vec& corrParent) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatecorrRef(corrParent);
    }
  }

  void MaxwellContact::updatecorr(int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatecorr(j);
    }
  }

  void MaxwellContact::calccorrSize(int j) {
    Link::calccorrSize(j);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calccorrSize(j);
        corrSize += jter->getcorrSize();
      }
    }
  }

  void MaxwellContact::checkRoot() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkRoot();
    }
  }

  void MaxwellContact::resetUpToDate() {
    updrrel = true;
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->resetUpToDate();
    }
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

  void MaxwellContact::updateGeneralizedForces() {

    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter)
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updlaN = false;

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

      solution0.resize() = LCP.solve(solution0);

      Vec lambda = solution0(rigidBodyGap.size(), 2 * rigidBodyGap.size() - 1);

      if (msg(Debug)) {
        msg(Debug) << "lambda = " << lambda << endl;
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

    if (msg(Debug)) {
      msg(Debug) << "The InfluenceMatrix is: " << C << endl;
      msg(Debug) << "With eigenvalues: " << eigval(C) << endl;
    }
  }

  void MaxwellContact::updateRigidBodyGap() {
    /*save rigidBodyGaps in vector*/
    rigidBodyGap.resize(possibleContactPoints.size());
    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      rigidBodyGap(i) = contacts[possibleContactPoints[i].first][possibleContactPoints[i].second].evalGeneralizedRelativePosition()(0);
    }

    if (msg(Debug))
      msg(Debug) << "rigidBodyGap: " << rigidBodyGap << endl;
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

        //        if (msg(Debug)) {
        //          msg(Debug) << "LagrangeParameter of contour \"" << contour->getPath() << "\" is:" << lagrangeParameter << endl;
        //        }

        FactorC += (*fct)(contInfo, contInfo);
      }
    }

    if (fabs(FactorC) <= macheps) {
      throwError("No elasticity is given for one of the following contours:\n  -" + contacts[contactIndex.first][contactIndex.second].getContour(0)->getPath() + "\n  -" + contacts[contactIndex.first][contactIndex.second].getContour(0)->getPath() + "\nThat is not an option!");
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

          //          if (msg(Debug)) {
          //            msg(Debug) << "First LagrangeParameter of contour \"" << contour1->getPath() << "\" is:" << firstLagrangeParameter << endl;
          //            msg(Debug) << "Second LagrangeParameter contour \"" << contour2->getPath() << "\" is:" << secondLagrangeParameter << endl;
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

  void MaxwellContact::updateGeneralizedPositions() {
    for(size_t i=0; i<contactKinematics.size(); i++)
    contactKinematics[i]->updateg(contacts[i]);
    updrrel = false;
  }

}
