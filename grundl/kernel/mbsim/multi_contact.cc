/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "multi_contact.h"

#include <mbsim/contact.h>
#include <mbsim/contour.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/utils.h>
#include <mbsim/objectfactory.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/objectfactory.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  MultiContact::MultiContact(const string &name) :
      LinkMechanics(name), fcl(0), fdf(0), fnil(0), ftil(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , openMBVGrp(0), openMBVContactFrameSize(0), openMBVContactFrameEnabled(false), contactArrow(NULL), frictionArrow(NULL)
#endif
          , saved_ref1(""), saved_ref2("") {
  }

  MultiContact::~MultiContact() {

  }

  void MultiContact::setDynamicSystemSolver(DynamicSystemSolver * sys) {
    ds = sys;
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Group* MultiContact::getOpenMBVGrp() {
    return openMBVGrp;
  }
#endif

  void MultiContact::updatewb(double t, int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
            jter->updatewb(t, j);  //TODO: contact kinematics in sub-contact!!
    }
  }

  void MultiContact::updateW(double t, int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateW(t, j);
    }
  }

  void MultiContact::updateV(double t, int j) {
    if (getFrictionDirections()) {
      if (fdf->isSetValued()) {
        for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
          for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
            jter->updateV(t, j);
        }
      }
    }
  }

  void MultiContact::updateh(double t, int j) {
    (*fcl).computeSmoothForces(contacts);

    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->applyh(t, j);
    }
  }

  void MultiContact::updateg(double t) {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      //for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); ++k) {
        contactKinematics[cK]->updateg(contacts[cK]);
      //}
    }
  }

  void MultiContact::updategd(double t) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updategd(t);
    }
  }

  void MultiContact::updateStopVector(double t) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateStopVector(t);
    }
  }

  void MultiContact::updateJacobians(double t, int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateJacobians(t, j);
    }
  }

  void MultiContact::updateWRef(const Mat& WParent, int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateWRef(WParent, j);
    }
  }

  void MultiContact::updateVRef(const Mat& VParent, int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateVRef(VParent, j);
    }
  }

  void MultiContact::updatehRef(const Vec& hParent, int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatehRef(hParent, j);
    }
  }

  void MultiContact::updatewbRef(const Vec& wbParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatewbRef(wbParent);
    }
  }

  void MultiContact::updatelaRef(const Vec& laParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatelaRef(laParent);
    }
  }

  void MultiContact::updategRef(const Vec& gParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->updategRef(gParent);
      }
    }
  }


  void MultiContact::updategdRef(const Vec& gdParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updategdRef(gdParent);
    }
  }

  void MultiContact::updateresRef(const Vec& resParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updateresRef(resParent);
    }
  }

  void MultiContact::updaterFactorRef(const Vec& rFactorParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updaterFactorRef(rFactorParent);
    }
  }

  void MultiContact::updatesvRef(const Vec &svParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatesvRef(svParent);
    }
  }

  void MultiContact::updatejsvRef(const VecInt &jsvParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatejsvRef(jsvParent);
    }
  }

  void MultiContact::calcxSize() {
    LinkMechanics::calcxSize();
    xSize = 0;
  }

  void MultiContact::calclaSize(int j) {
    LinkMechanics::calclaSize(j);
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calclaSize(j);
        laSize += jter->getlaSize();
      }
    }
  }

  void MultiContact::calcgSize(int j) {
    LinkMechanics::calcgSize(j);
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcgSize(j);
        gSize += jter->getgSize();
      }
    }
  }

  void MultiContact::calcgdSize(int j) {
    LinkMechanics::calcgdSize(j);
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcgdSize(j);
        gdSize += jter->getgdSize();
      }
    }
  }

  void MultiContact::calcrFactorSize(int j) {
    LinkMechanics::calcrFactorSize(j);
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->calcrFactorSize(j);
        rFactorSize += jter->getrFactorSize();
      }
    }
  }

  void MultiContact::calcsvSize() {
    LinkMechanics::calcsvSize();
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setsvInd(svInd + svSize);
        jter->calcsvSize();
        svSize += jter->getsvSize();
      }
    }
  }

  void MultiContact::calcLinkStatusSize() {
    LinkMechanics::calcLinkStatusSize();
    int n = 0;
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      n += contactKinematics[cK]->getNumberOfPotentialContactPoints();
    }
    LinkStatusSize = n;
    LinkStatus.resize(LinkStatusSize);
  }

  void MultiContact::calcLinkStatusRegSize() {
    LinkMechanics::calcLinkStatusRegSize();
    int n = 0;
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      n += contactKinematics[cK]->getNumberOfPotentialContactPoints();
    }
    LinkStatusRegSize = n;
    LinkStatusReg.resize(LinkStatusRegSize);
  }

  void MultiContact::init(InitStage stage) {
    if (stage == resolveXMLPath) {
      //TODO XMLinitialize for multi-contact
//      if (saved_ref1 != "" && saved_ref2 != "")
//        connect(getByPath<Contour>(saved_ref1), getByPath<Contour>(saved_ref2));
//      LinkMechanics::init(stage);
    }
    else if (stage == preInit) {
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        Contour* contour0 = contour[2 * cK];
        Contour* contour1 = contour[2 * cK + 1];
        contactKinematics[cK]->assignContours(contour0, contour1);

        contacts.push_back(vector<Contact>());

        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); ++k) {
          stringstream contactName;
          contactName << ckNames[cK];
          if(contactKinematics[cK]->getNumberOfPotentialContactPoints() > 1)
            contactName << "_" <<  k;
          contacts[cK].push_back(Contact(contactName.str()));
          contacts[cK][k].setContactKinematics(contactKinematics[cK]);
          contacts[cK][k].connect(contour0);
          contacts[cK][k].connect(contour1);
        }
      }

      for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
          //set parent
          jter->setParent(this);

          //Set dynamic system solver for children
          jter->setDynamicSystemSolver(ds);

          //set contact laws for children
          jter->setContactForceLaw(fcl);
          jter->setContactImpactLaw(fnil);
          jter->setFrictionForceLaw(fdf);
          jter->setFrictionImpactLaw(ftil);

#ifdef HAVE_OPENMBVCPPINTERFACE
          for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
            for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
              jter->setOpenMBVNormalForceArrow(contactArrow);
              jter->setOpenMBVFrictionForceArrow(frictionArrow);
            }
          }
#endif
          jter->init(stage);
        }
      }
    }
    else if (stage == resize) {
      LinkMechanics::init(stage);

      for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->init(stage);
      }
    }
    else if (stage == MBSim::plot) {
      Element::init(stage);
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVGrp = new OpenMBV::Group();
        openMBVGrp->setName(name + "_ContactGroup");
        openMBVGrp->setExpand(false);
        parent->getOpenMBVGrp()->addObject(openMBVGrp);
#endif
        for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
          for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
            jter->init(stage);
        }
      }
    }
    else if (stage == unknownStage) {
      LinkMechanics::init(stage);

      for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
        for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
          jter->init(stage);
      }
    }

    else
      LinkMechanics::init(stage);
  }

  bool MultiContact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if (fdf)
      flag |= fdf->isSetValued();
    return flag;
  }

  bool MultiContact::isSingleValued() const {
    if (fcl->isSetValued()) {
      if (fdf) {
        return not fdf->isSetValued();
      }
      return false;
    }
    return true;
  }

  void MultiContact::updateLinkStatus(double t) {
    //TODO: Problem here would be the mapping between the contact (which is identified by the std::pair) and the linkStatusIndex (which is identified by an integer)
//    int linkStatusIndex = 0;
//    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
//      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
//        if (gActive[cK][k]) {
//          LinkStatus(linkStatusIndex) = 2;
//          if (ftil) {
//            if (ftil->isSticking(lakT[cK][k], gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0), LaTol, gdTol))
//              LinkStatus(linkStatusIndex) = 3;
//            else
//              LinkStatus(linkStatusIndex) = 4;
//          }
//        }
//        else
//          LinkStatus(linkStatusIndex) = 1;
//        linkStatusIndex++;
//      }
//    }
    throw;
  }

  void MultiContact::updateLinkStatusReg(double t) {
    //TODO: Problem here would be the mapping between the contact (which is identified by the std::pair) and the linkStatusIndex (which is identified by an integer)
//    int linkStatusIndex = 0;
//    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
//      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
//        if (gActive[cK][k]) {
//          LinkStatusReg(linkStatusIndex) = 2;
//        }
//        else {
//          LinkStatusReg(linkStatusIndex) = 1;
//        }
//        linkStatusIndex++;
//      }
//    }
    throw;
  }

  bool MultiContact::isActive() const {
    for (std::vector<std::vector<Contact> >::const_iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::const_iterator jter = iter->begin(); jter != iter->end(); ++jter)
        if (jter->isActive())
          return true;
    }
    return false;
  }

  bool MultiContact::gActiveChanged() {
    bool changed = false;
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        if (jter->gActiveChanged())
          changed = true;
    }
    return changed;
  }

  void MultiContact::plot(double t, double dt) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->plot(t, dt);
    }
  }

  void MultiContact::closePlot() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->closePlot();
    }
    if (getPlotFeature(plotRecursive) == enabled) {
      Element::closePlot();
    }
  }

  void MultiContact::setgInd(int gInd_) {
    LinkMechanics::setgInd(gInd_);
    int nextgInd = gInd_;
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setgInd(nextgInd);
        nextgInd += jter->getgSize();
      }
    }
  }

  void MultiContact::setgdInd(int gdInd_) {
    LinkMechanics::setgdInd(gdInd_);
    int nextgdInd = gdInd_;
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setgdInd(nextgdInd);
        nextgdInd += jter->getgdSize();
      }
    }
  }

  void MultiContact::setlaInd(int laInd_) {
    LinkMechanics::setlaInd(laInd_);
    int nextlaInd = laInd_;
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setlaInd(nextlaInd);
        nextlaInd += jter->getlaSize();
      }
    }
  }

  void MultiContact::setrFactorInd(int rFactorInd_) {
    LinkMechanics::setrFactorInd(rFactorInd_);
    int nextrFactorInd = rFactorInd_;
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setrFactorInd(nextrFactorInd);
        nextrFactorInd += jter->getrFactorSize();
      }
    }
  }

  void MultiContact::solveImpactsFixpointSingle(double dt) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsFixpointSingle(dt);
    }
  }

  void MultiContact::solveConstraintsFixpointSingle() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsFixpointSingle();
    }
  }

  void MultiContact::solveImpactsGaussSeidel(double dt) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsGaussSeidel(dt);
    }
  }

  void MultiContact::solveConstraintsGaussSeidel() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsGaussSeidel();
    }
  }

  void MultiContact::solveImpactsRootFinding(double dt) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveImpactsRootFinding(dt);
    }
  }

  void MultiContact::solveConstraintsRootFinding() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->solveConstraintsRootFinding();
    }
  }

  void MultiContact::jacobianConstraints() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->jacobianConstraints();
    }
  }

  void MultiContact::jacobianImpacts() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->jacobianImpacts();
    }
  }

  void MultiContact::updaterFactors() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updaterFactors();
    }
  }

  void MultiContact::checkConstraintsForTermination() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkConstraintsForTermination();
    }
  }

  void MultiContact::checkImpactsForTermination(double dt) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkImpactsForTermination(dt);
    }
  }

  void MultiContact::checkActive(int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkActive(j);
    }
  }

  int MultiContact::getFrictionDirections() {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void MultiContact::connect(Contour *contour0, Contour* contour1, ContactKinematics* contactKinematics_ /*=0*/, const string & name) {
    LinkMechanics::connect(contour0);
    LinkMechanics::connect(contour1);
    contactKinematics.push_back(contactKinematics_);

    int cK = contactKinematics.size() - 1;

    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour0->findContactPairingWith(contour0->getType(), contour1->getType());
    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour1->findContactPairingWith(contour1->getType(), contour0->getType());
    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour0->findContactPairingWith(contour1->getType(), contour0->getType());
    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour1->findContactPairingWith(contour0->getType(), contour1->getType());
    if (contactKinematics[cK] == 0)
      throw MBSimError("ERROR in " + getName() + " (Contact::init): Unknown contact pairing between Contour \"" + contour0->getType() + "\" and Contour\"" + contour1->getType() + "\"!");

    //Create a single contact(with all the information) for every sub contact of each contact kinematics that is part of the multiple contact
    if (name == "")
      ckNames.push_back(numtostr(cK));
    else
      ckNames.push_back(name);

  }

  void MultiContact::setPlotFeature(PlotFeature pf, PlotFeatureStatus value, size_t indexKinematics) {
    assert(indexKinematics >= 0);
    assert(indexKinematics < contactKinematics.size());
    for(int i = 0; i < contactKinematics[indexKinematics]->getNumberOfPotentialContactPoints(); ++i)
      contacts[indexKinematics][i].setPlotFeature(pf, value);
  }

  void MultiContact::computeCurvatures(Vec & r, int contactKinematicsIndex) const {
    throw;
    //TODO
  }

  void MultiContact::LinearImpactEstimation(Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->LinearImpactEstimation(gInActive_, gdInActive_, IndInActive_, gAct_, IndActive_);
    }
  }

  void MultiContact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->SizeLinearImpactEstimation(sizeInActive_, sizeActive_);
    }
  }

  void MultiContact::initializeUsingXML(TiXmlElement *element) {
    throw; //TODO?!
    LinkMechanics::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"contactForceLaw");
    GeneralizedForceLaw *gfl = ObjectFactory::getInstance()->createGeneralizedForceLaw(e->FirstChildElement());
    setContactForceLaw(gfl);
    gfl->initializeUsingXML(e->FirstChildElement());
    e = e->NextSiblingElement();
    GeneralizedImpactLaw *gifl = ObjectFactory::getInstance()->createGeneralizedImpactLaw(e->FirstChildElement());
    if (gifl) {
      setContactImpactLaw(gifl);
      gifl->initializeUsingXML(e->FirstChildElement());
      e = e->NextSiblingElement();
    }
    FrictionForceLaw *ffl = ObjectFactory::getInstance()->createFrictionForceLaw(e->FirstChildElement());
    if (ffl) {
      setFrictionForceLaw(ffl);
      ffl->initializeUsingXML(e->FirstChildElement());
      e = e->NextSiblingElement();
    }
    FrictionImpactLaw *fil = ObjectFactory::getInstance()->createFrictionImpactLaw(e->FirstChildElement());
    if (fil) {
      setFrictionImpactLaw(fil);
      fil->initializeUsingXML(e->FirstChildElement());
    }
    e = element->FirstChildElement(MBSIMNS"connect");
    saved_ref1 = e->Attribute("ref1");
    saved_ref2 = e->Attribute("ref2");
#ifdef HAVE_OPENMBVCPPINTERFACE
    if (element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints"))
      enableOpenMBVContactPoints(getDouble(element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints")));
    e = element->FirstChildElement(MBSIMNS"openMBVNormalForceArrow");
    if (e) {
      OpenMBV::Arrow *arrow = dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVNormalForceArrow(arrow);
      e = e->NextSiblingElement();
    }
    e = element->FirstChildElement(MBSIMNS"openMBVFrictionArrow");
    if (e) {
      OpenMBV::Arrow *arrow = dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVFrictionForceArrow(arrow);
      e = e->NextSiblingElement();
    }
#endif
  }

  void MultiContact::updatecorrRef(const Vec& corrParent) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatecorrRef(corrParent);
    }
  }

  void MultiContact::updatecorr(int j) {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->updatecorr(j);
    }
  }

  void MultiContact::calccorrSize(int j) {
    LinkMechanics::calccorrSize(j);
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter) {
        jter->setcorrInd(corrInd + corrSize);
        jter->calccorrSize(j);
        corrSize += jter->getcorrSize();
      }
    }
  }

  void MultiContact::checkRoot() {
    for (std::vector<std::vector<Contact> >::iterator iter = contacts.begin(); iter != contacts.end(); ++iter) {
      for (std::vector<Contact>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
        jter->checkRoot();
    }
  }
}

