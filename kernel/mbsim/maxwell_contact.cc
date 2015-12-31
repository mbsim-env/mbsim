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

#include <mbsim/contour.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
#include <fmatvec/function.h>
#include <mbsim/utils/utils.h>
#include <mbsim/objectfactory.h>
#include <mbsim/functions/kinetic_functions.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>
#endif

#include <algorithm>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(MaxwellContact, MBSIM%"MaxwellContact")

  MaxwellContact::MaxwellContact(const string &name) : Link(name), contacts(0), contactKinematics(0), ckNames(0), plotFeatureMap(), fcl(0), fdf(0), fnil(0), ftil(0), LCP(SymMat(0,NONINIT), Vec(0,NONINIT)), dampingCoefficient(0.), gLim(0.), matConst(0), matConstSetted(false), DEBUGLEVEL(0), saved_ref(0) {
  }

  MaxwellContact::~MaxwellContact() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); ++k)
        delete contactKinematics[cK]->getContactKinematics(k);
      delete contactKinematics[cK];
    }
    delete fcl;
    delete fdf;
    delete fnil;
    delete ftil;
    for (map<pair<Contour*, Contour*>,InfluenceFunction*>::iterator it=influenceFunctions.begin(); it!=influenceFunctions.end(); ++it)
      delete it->second;
  }

  void MaxwellContact::setDynamicSystemSolver(DynamicSystemSolver * sys) {
    ds = sys;
  }

  void MaxwellContact::setPlotFeatureContactKinematics(std::string cKName, PlotFeature pf, PlotFeatureStatus value) {
    if (ckNames.end() != find(ckNames.begin(), ckNames.end(), cKName)) {
      pair<string, PlotFeature> Pair(cKName, pf);
      plotFeatureMap.insert(pair<pair<string, PlotFeature>, PlotFeatureStatus>(Pair, value));
    }
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  shared_ptr<OpenMBV::Group> MaxwellContact::getOpenMBVGrp() {
    return openMBVGrp;
  }
#endif

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

  void MaxwellContact::updatewb(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter)
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatewb(t);
  }

  void MaxwellContact::updateW(double t, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateW(t, j);
    }
  }

  void MaxwellContact::updateV(double t, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateV(t, j);
    }
  }

  void MaxwellContact::updateh(double t, int j) {
    computeSmoothForces(t);

    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateh(t, j);
    }
  }

  void MaxwellContact::updateg(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateg(t);
    }
  }

  void MaxwellContact::updategd(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updategd(t);
    }
  }

  void MaxwellContact::updateStopVector(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateStopVector(t);
    }
  }

  void MaxwellContact::updateJacobians(double t, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateJacobians(t, j);
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

  void MaxwellContact::calcxSize() {
    Link::calcxSize();
    xSize = 0;
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

  void MaxwellContact::init(InitStage stage) {
    if (stage == resolveXMLPath) {
      //connect all contours given in xml file
      for (size_t i = 0; i < saved_ref.size(); i++) {
        if (saved_ref[i].name1 != "" && saved_ref[i].name2 != "")
          connect(getByPath<Contour>(saved_ref[i].name1), getByPath<Contour>(saved_ref[i].name2), 0, saved_ref[i].contourPairingName);
        //TODO: add option to specifiy contact_kinematics
      }

      // initialize all contour couplings if generalized force law is of maxwell-type
      initializeContourCouplings();

      Link::init(stage);
    }
    else if (stage == preInit) {
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        contactKinematics[cK]->assignContours(contour[0][cK], contour[1][cK]);

        contacts.push_back(vector<SingleContact>());

        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); ++k) {
          stringstream contactName;
          contactName << ckNames[cK];
          if (contactKinematics[cK]->getNumberOfPotentialContactPoints() > 1)
            contactName << "_" << k;
          contacts[cK].push_back(SingleContact(contactName.str()));
          contacts[cK][k].setContactKinematics(contactKinematics[cK]->getContactKinematics(k) ? contactKinematics[cK]->getContactKinematics(k) : contactKinematics[cK]);
          contacts[cK][k].connect(contour[0][cK], contour[1][cK]);
          //Applies the plot feature to all children (make it possible to set only some children...)
          for (int i = plotRecursive; i != LASTPLOTFEATURE; i++) {
            PlotFeature pf = static_cast<PlotFeature>(i);
            PlotFeatureStatus pfS = getPlotFeature(pf);

            pair<string, PlotFeature> Pair(ckNames[cK], pf);
            if (plotFeatureMap.find(Pair) != plotFeatureMap.end()) {
              pfS = plotFeatureMap.find(Pair)->second;
            }

            contacts[cK][k].setPlotFeature(pf, pfS);
          }

          //set the tolerances for the single contacts
          contacts[cK][k].setgTol(gTol);
          contacts[cK][k].setgdTol(gdTol);
          contacts[cK][k].setgddTol(gddTol);
          contacts[cK][k].setlaTol(laTol);
          contacts[cK][k].setLaTol(LaTol);
          contacts[cK][k].setrMax(rMax);
        }
      }

      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
          //set parent
          jter->setParent(this);

          //Set dynamic system solver for children
          jter->setDynamicSystemSolver(ds);

          //set contact laws for children
          jter->setNormalForceLaw(fcl);
          jter->setNormalImpactLaw(fnil);
          jter->setTangentialForceLaw(fdf);
          jter->setTangentialImpactLaw(ftil);

#ifdef HAVE_OPENMBVCPPINTERFACE
          //Set OpenMBV-Properties to single contacts
          if (openMBVFrame)
            jter->setOpenMBVContactPoints((iter==contacts.begin() and jter==iter->begin())?openMBVFrame:OpenMBV::ObjectFactory::create(openMBVFrame));
          if (contactArrow)
            jter->setOpenMBVNormalForce((iter==contacts.begin() and jter==iter->begin())?contactArrow:OpenMBV::ObjectFactory::create(contactArrow));
          if (frictionArrow)
            jter->setOpenMBVTangentialForce((iter==contacts.begin() and jter==iter->begin())?frictionArrow:OpenMBV::ObjectFactory::create(frictionArrow));
#endif
          jter->init(stage);
        }
      }
      Link::init(stage);
    }
    else if (stage == resize) {
      Link::init(stage);

      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->init(stage);
      }
    }
    else if (stage == plotting) {
      Element::init(stage);
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
        openMBVGrp->setName(name + "_ContactGroup");
        openMBVGrp->setExpand(false);
        parent->getOpenMBVGrp()->addObject(openMBVGrp);
#endif
        for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
          for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
            jter->init(stage);
        }
      }
    }
    else if (stage == unknownStage) {
      Link::init(stage);

      for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->init(stage);
      }
    }
    else
      Link::init(stage);
    //Don't call init()-routines for "sub"-contacts with stage "LASTINITSTAGE" as here is checked if contactKinematics has more than one possible contact point, which is only possible in multi-contact
    if(fcl) fcl->init(stage);
    if(fdf) fdf->init(stage);
    if(fnil) fnil->init(stage);
    if(ftil) ftil->init(stage);
  }

  bool MaxwellContact::isSetValued() const {
    bool flag = false;
    if (fdf)
      flag |= fdf->isSetValued();
    return flag;
  }

  bool MaxwellContact::isSingleValued() const {
    if (false) {
      if (fdf) {
        return not fdf->isSetValued();
      }
      return false;
    }
    return true;
  }

  void MaxwellContact::updateLinkStatus(double dt) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->updateLinkStatus(dt);
      }
    }
  }

  void MaxwellContact::updateLinkStatusReg(double dt) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->updateLinkStatusReg(dt);
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

  void MaxwellContact::plot(double t, double dt) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->plot(t, dt);
    }
  }

  void MaxwellContact::closePlot() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->closePlot();
    }
    if (getPlotFeature(plotRecursive) == enabled) {
      Element::closePlot();
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

  void MaxwellContact::solveImpactsFixpointSingle(double t, double dt) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsFixpointSingle(t,dt);
    }
  }

  void MaxwellContact::solveConstraintsFixpointSingle(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsFixpointSingle(t);
    }
  }

  void MaxwellContact::solveImpactsGaussSeidel(double t, double dt) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsGaussSeidel(t,dt);
    }
  }

  void MaxwellContact::solveConstraintsGaussSeidel(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsGaussSeidel(t);
    }
  }

  void MaxwellContact::solveImpactsRootFinding(double t, double dt) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsRootFinding(t,dt);
    }
  }

  void MaxwellContact::solveConstraintsRootFinding(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsRootFinding(t);
    }
  }

  void MaxwellContact::jacobianConstraints(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->jacobianConstraints(t);
    }
  }

  void MaxwellContact::jacobianImpacts(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->jacobianImpacts(t);
    }
  }

  void MaxwellContact::updaterFactors(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updaterFactors(t);
    }
  }

  void MaxwellContact::checkConstraintsForTermination(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkConstraintsForTermination(t);
    }
  }

  void MaxwellContact::checkImpactsForTermination(double t, double dt) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkImpactsForTermination(t,dt);
    }
  }

  void MaxwellContact::checkActive(double t, int j) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkActive(t,j);
    }
  }

  int MaxwellContact::getFrictionDirections() {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void MaxwellContact::connect(Contour *contour0, Contour* contour1, ContactKinematics* contactKinematics_ /*=0*/, const string & name_ /* ="" */) {
//    Link::connect(contour0);
//    Link::connect(contour1);
    contour[0].push_back(contour0);
    contour[1].push_back(contour1);
    contactKinematics.push_back(contactKinematics_);

    int cK = contactKinematics.size() - 1;

    if (contactKinematics[cK] == 0) {
      contactKinematics[cK] = contour0->findContactPairingWith(contour0->getType(), contour1->getType());
      if (contactKinematics[cK] == 0) {
        contactKinematics[cK] = contour1->findContactPairingWith(contour1->getType(), contour0->getType());
        if (contactKinematics[cK] == 0) {
          contactKinematics[cK] = contour0->findContactPairingWith(contour1->getType(), contour0->getType());
          if (contactKinematics[cK] == 0) {
            contactKinematics[cK] = contour1->findContactPairingWith(contour0->getType(), contour1->getType());
            if (contactKinematics[cK] == 0) {
              THROW_MBSIMERROR("(MaxwellContact::init): Unknown contact pairing between Contour \"" + contour0->getType() + "\" and Contour \"" + contour1->getType() + "\"!");
            }
          }
        }
      }
    }

    //Create a single contact(with all the information) for every sub contact of each contact kinematics that is part of the multiple contact
    if (name_ == "")
      ckNames.push_back(name + "_" + numtostr(cK));
    else
      ckNames.push_back(name_);

  }

  void MaxwellContact::getCurvatures(Vec & r, int contactKinematicsIndex) const {
    THROW_MBSIMERROR("Not implemented");
    //TODO
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

  void MaxwellContact::setlaTol(double tol) {
    Link::setlaTol(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setlaTol(tol);
      }
    }
  }

  void MaxwellContact::setLaTol(double tol) {
    Link::setLaTol(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setLaTol(tol);
      }
    }
  }

  void MaxwellContact::setgTol(double tol) {
    Link::setgTol(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setgTol(tol);
      }
    }
  }

  void MaxwellContact::setgdTol(double tol) {
    Link::setgdTol(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setgdTol(tol);
      }
    }
  }

  void MaxwellContact::setgddTol(double tol) {
    Link::setgddTol(tol);
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setgddTol(tol);
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

  void MaxwellContact::computeSmoothForces(double t) {
    updatePossibleContactPoints(t);

    //Apply damping force
    //TODO: use damping function for that (to be more flexible...)
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        if ((*jter).getGeneralizedRelativePosition(t)(0) < gLim and (*jter).getGeneralizedRelativeVelocity(t)(0) < 0)
          (*jter).getGeneralizedSingleValuedForce(false)(0) = -dampingCoefficient * (*jter).getGeneralizedRelativeVelocity(t)(0);
        else
          (*jter).getGeneralizedSingleValuedForce(false)(0) = 0;
      }
    }

    if (possibleContactPoints.size()) {
      updateInfluenceMatrix(t);
      updateRigidBodyGap(t);

      LCP.setSystem(C, rigidBodyGap);

      map<Index, double> tolerances;
      tolerances.insert(pair<Index, double>(Index(0, possibleContactPoints.size() - 1), 1e-8)); //tolerances for distances
      tolerances.insert(pair<Index, double>(Index(possibleContactPoints.size(), 2 * possibleContactPoints.size() - 1), 1e-3)); //tolerances for forces
      LocalResidualCriteriaFunction critfunc(tolerances);
      LCP.setNewtonCriteriaFunction(&critfunc);
      LCP.setDebugLevel(0);

      solution0.resize() = LCP.solve(solution0);

      Vec lambda = solution0(rigidBodyGap.size(), 2 * rigidBodyGap.size() - 1);

      if (DEBUGLEVEL >= 3) {
        cout << "lambda = " << lambda << endl;
      }

      for (size_t i = 0; i < possibleContactPoints.size(); ++i) {
        contacts[possibleContactPoints[i].first][possibleContactPoints[i].second].getGeneralizedSingleValuedForce(false)(0) += lambda(i);
      }
    }
  }

  void MaxwellContact::updatePossibleContactPoints(double t) {
    possibleContactPoints.clear();
    for (size_t i = 0; i < contacts.size(); ++i) {
      for (size_t j = 0; j < contacts[i].size(); ++j) {
        if (contacts[i][j].getGeneralizedRelativePosition(t)(0) < 0) { //TODO: use gActive, but only at timestep No 2...
          possibleContactPoints.push_back(pair<int,int>(i,j));
        }
      }
    }
  }

  void MaxwellContact::updateInfluenceMatrix(double t) {
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

  void MaxwellContact::updateRigidBodyGap(double t) {
    /*save rigidBodyGaps in vector*/
    rigidBodyGap.resize(possibleContactPoints.size());
    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      rigidBodyGap(i) = contacts[possibleContactPoints[i].first][possibleContactPoints[i].second].getGeneralizedRelativePosition(t)(0);
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
        pair<Contour*, ContourPointData> contInfo(contour, contacts[contactIndex.first][contactIndex.second].getcpData(i));
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
          pair<Contour*, ContourPointData> cont1Info(contour1, contacts[contactIndex.first][contactIndex.second].getcpData(affectedContourIterator));
          pair<Contour*, ContourPointData> cont2Info(contour2, contacts[coupledContactIndex.first][coupledContactIndex.second].getcpData(coupledContourIterator));

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
    Link::initializeUsingXML(element);
    DOMElement *e;

    //Set contact law
    e = E(element)->getFirstElementChildNamed(MBSIM%"normalForceLaw");
    GeneralizedForceLaw *gfl = ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild());
    setNormalForceLaw(gfl);

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

    e = E(element)->getFirstElementChildNamed(MBSIM%"InfluenceFunction");
    while(e) {
      xmlInfo info;
      info.function = ObjectFactory::createAndInit<InfluenceFunction>(e->getFirstElementChild());
      info.name1 = E(e->getFirstElementChild())->getAttribute("contourName1");
      info.name2 = E(e->getFirstElementChild())->getAttribute("contourName2");

      referenceXML.push_back(info);

      e = e->getNextElementSibling();
    }


#ifdef HAVE_OPENMBVCPPINTERFACE
    //Get all drawing thingies
    if (E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints")) {
      OpenMBVFrame ombv;
      openMBVFrame = ombv.createOpenMBV(e);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      contactArrow = ombv.createOpenMBV(e);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      frictionArrow = ombv.createOpenMBV(e);
    }
#endif
  }

  DOMElement* MaxwellContact::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Link::writeXMLFile(parent);
    //    DOMElement *ele1;
    //    ele1 = new DOMElement(MBSIM%"normalForceLaw");
    //    if (fcl)
    //      fcl->writeXMLFile(ele1);
    //    ele0->LinkEndChild(ele1);
    //    if (fnil) {
    //      ele1 = new DOMElement(MBSIM%"normalImpactLaw");
    //      fnil->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //    if (fdf) {
    //      ele1 = new DOMElement(MBSIM%"tangentialForceLaw");
    //      fdf->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //    if (ftil) {
    //      ele1 = new DOMElement(MBSIM%"tangentialImpactLaw");
    //      ftil->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //    ele1 = new DOMElement(MBSIM%"connect");
    //    //for(unsigned int i=0; i<saved_ref.size(); i++) {
    //    ele1->SetAttribute("ref1", contour[0]->getXMLPath(this, true)); // relative path
    //    ele1->SetAttribute("ref2", contour[1]->getXMLPath(this, true)); // relative path
    //    //}
    //    ele0->LinkEndChild(ele1);
    //#ifdef HAVE_OPENMBVCPPINTERFACE
    ////    if(openMBVContactFrameSize>0)
    ////      addElementText(ele0,MBSIM%"enableOpenMBVContactPoints",openMBVContactFrameSize);
    //    if (contactArrow) {
    //      ele1 = new DOMElement(MBSIM%"openMBVNormalForceArrow");
    //      contactArrow->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //    if (frictionArrow) {
    //      ele1 = new DOMElement(MBSIM%"openMBVTangentialForceArrow");
    //      frictionArrow->writeXMLFile(ele1);
    //      ele0->LinkEndChild(ele1);
    //    }
    //#endif
    return ele0;
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

  void MaxwellContact::checkRoot(double t) {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkRoot(t);
    }
  }

  void MaxwellContact::resetUpToDate() {
    for (std::vector<std::vector<SingleContact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<SingleContact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->resetUpToDate();
    }
  }
}

