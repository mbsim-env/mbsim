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
#include <mbsim/contact.h>
#include <mbsim/frame.h>
#include <mbsim/contour.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
#include <fmatvec/function.h>
#include <mbsim/utils/utils.h>
#include <mbsim/objectfactory.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/objectfactory.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SingleContact, MBSIM%"SingleContact")

  SingleContact::SingleContact(const string &name) : ContourLink(name), contactKinematics(0), fcl(0), fdf(0), fnil(0), ftil(0), gActive(0), gActive0(0), gdActive(0), gddActive(0), updlaN(true), updlaT(true)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , openMBVContactFrame(2)
#endif
          , rootID(0), saved_ref1(""), saved_ref2("") {
  }

  SingleContact::~SingleContact() {
    if (gdActive)
      delete[] gdActive;
    if (gddActive)
      delete[] gddActive;
  }

  void SingleContact::updatewb(double t) {
    if(gdActive[0]) {
      wb -= getGlobalForceDirection(t)(Index(0,2),Index(0,laSize-1)).T() * cFrame[0]->getGyroscopicAccelerationOfTranslation(t);
      wb += getGlobalForceDirection(t)(Index(0,2),Index(0,laSize-1)).T() * cFrame[1]->getGyroscopicAccelerationOfTranslation(t);

      contactKinematics->updatewb(t, wb, getGeneralizedRelativePosition(t)(0), cFrame);
    }
  }

  void SingleContact::resetUpToDate() {
    ContourLink::resetUpToDate();
    updlaN = true;
    updlaT = true;
  }

  void SingleContact::updateGeneralizedNormalForceM(double t) {
    if(gdActive[0])
      lambdaN = laN(0);
    else
      lambdaN = 0;
  }

  void SingleContact::updateGeneralizedNormalForceS(double t) {
    lambdaN = (*fcl)(getGeneralizedRelativePosition(t)(0), getGeneralizedRelativeVelocity(t)(0));
  }

  void SingleContact::updateGeneralizedNormalForceP(double t) {
    static_cast<Contact*>(parent)->updateGeneralizedNormalForce(t);
  }

  void SingleContact::updateGeneralizedTangentialForceM(double t) {
    if(gdActive[1])
      lambdaT = laT;
    else if(gdActive[0])
      lambdaT = fdf->dlaTdlaN(getGeneralizedRelativeVelocity(t)(Index(1,getFrictionDirections()))) * getGeneralizedNormalForce(t);
    else
      lambdaT.init(0);
  }

  void SingleContact::updateGeneralizedTangentialForceS(double t) {
    lambdaT = (*fdf)(getGeneralizedRelativeVelocity(t)(Index(1,getFrictionDirections())), fabs(getGeneralizedNormalForce(t)));
  }

  void SingleContact::updateGeneralizedForces(double t) {
    lambda(0) = getGeneralizedNormalForce(t);
    lambda.set(Index(1,lambda.size()-1),getGeneralizedTangentialForce(t));
    updla = false;
  }

  void SingleContact::updateGeneralizedPositions(double t) {
    updatePositions(t);
    updrrel = false;
  }

  void SingleContact::updateGeneralizedVelocities(double t) {
    updateVelocities(t);
    updvrel = false;
  }

  void SingleContact::updatePositions(double t) {
    contactKinematics->updateg(t, rrel(0), cFrame);
    updPos = false;
  }

  void SingleContact::updatePositions(double t, Frame *frame) {
    if(updPos) contactKinematics->updateg(t, rrel(0), cFrame);
  }

  void SingleContact::updateVelocities(double t) {
    if ((fcl->isSetValued() and gdActive[0]) or (not fcl->isSetValued() and fcl->isClosed(getGeneralizedRelativePosition(t)(0), 0))) { // TODO: nicer implementation
      Vec3 Wn = cFrame[0]->getOrientation(t).col(0);

      Vec3 WvD = cFrame[1]->getVelocity(t) - cFrame[0]->getVelocity(t);

      vrel(0) = Wn.T() * WvD;

      if (getFrictionDirections()) {
        Mat3xV Wt(getFrictionDirections());
        Wt.set(0, cFrame[0]->getOrientation().col(1));
        if (getFrictionDirections() > 1)
          Wt.set(1, cFrame[0]->getOrientation().col(2));

        vrel.set(Index(1,getFrictionDirections()), Wt.T() * WvD);
      }
    }
    else
      vrel.init(0);
    updVel = false;
  }

  void SingleContact::updateg(double t) {
    g = getGeneralizedRelativePosition(t)(Index(0,gSize-1));
  }

  void SingleContact::updategd(double t) {
    int addIndexnormal = fcl->isSetValued()?0:1;
    gd = getGeneralizedRelativeVelocity(t)(Index(addIndexnormal,gdSize+addIndexnormal-1));
  }

  void SingleContact::updateh(double t, int j) {
    Vec3 F = getGlobalForceDirection(t).col(0)*getGeneralizedNormalForce(t);
    if(fdf and not fdf->isSetValued())
      F += getGlobalForceDirection(t)(Range<Fixed<0>,Fixed<2> >(),Range<Var,Var>(1,getFrictionDirections()))*getGeneralizedTangentialForce(t);

    h[j][0] -= cFrame[0]->getJacobianOfTranslation(t,j).T() * F;
    h[j][1] += cFrame[1]->getJacobianOfTranslation(t,j).T() * F;
  }

  void SingleContact::updateW(double t, int j) {
    int i = fcl->isSetValued()?0:1;
    Mat3xV RF = getGlobalForceDirection(t)(Range<Fixed<0>,Fixed<2> >(),Range<Var,Var>(i,i+laSize-1));

    W[j][0] -= cFrame[0]->getJacobianOfTranslation(t,j).T() * RF;
    W[j][1] += cFrame[1]->getJacobianOfTranslation(t,j).T() * RF;
  }

  void SingleContact::updateV(double t, int j) {
    if (getFrictionDirections()) {
      if (fdf->isSetValued()) {
        if (gdActive[0] and not gdActive[1]) { // with this if-statement for the timestepping integrator it is V=W as it just evaluates checkActive(1)
          Mat3xV RF = getGlobalForceDirection(t)(Index(0,2),Index(1, getFrictionDirections()));
          V[j][0] -= cFrame[0]->getJacobianOfTranslation(t,j).T() * RF * fdf->dlaTdlaN(getGeneralizedRelativeVelocity(t)(Index(1,getFrictionDirections())));
          V[j][1] += cFrame[1]->getJacobianOfTranslation(t,j).T() * RF * fdf->dlaTdlaN(getGeneralizedRelativeVelocity(t)(Index(1,getFrictionDirections())));
        }
      }
    }
  }

  void SingleContact::updateStopVector(double t) {
    // TODO account for regularized normal force
    if (gActive != gdActive[0])
      THROW_MBSIMERROR("Internal error");
    if (gActive) {
      sv(0) = gddN(0) - gddTol;
      if (gdActive[1]) {
        if (getFrictionDirections()) {
          sv(1) = nrm2(gddT) - gddTol;
          if (sv(1) > 0) {
            gddNBuf = gddN;
            gddTBuf = gddT;
          }
        }
      }
      else if(fdf and fdf->isSetValued()) {
        if (getFrictionDirections() == 1)
          sv(1) = getGeneralizedRelativeVelocity(t)(1);
        else {
          sv(1) = getGeneralizedRelativeVelocity(t)(1) + getGeneralizedRelativeVelocity(t)(2); // TODO: is there a better concept?
        }
      }
    }
    else {
      int i=0;
      if(fcl->isSetValued()) sv(i++) = getGeneralizedRelativePosition(t)(0);
      if (fdf and fdf->isSetValued())
        sv(i) = 1;
    }
  }

  void SingleContact::updatelaRef(const Vec& laParent) {
    ContourLink::updatelaRef(laParent);
    if (laSize) {
      int laIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        laN >> la(0, 0);
        laIndSizeNormal++;
      }

      if (fdf and fdf->isSetValued())
        laT >> la(laIndSizeNormal, laSize - 1);
    }
  }

  void SingleContact::updateLaRef(const Vec& LaParent) {
    ContourLink::updateLaRef(LaParent);
    if (laSize) {
      int laIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        LaN >> La(0, 0);
        laIndSizeNormal++;
      }

      if (fdf and fdf->isSetValued())
        LaT >> La(laIndSizeNormal, laSize - 1);
    }
  }

  void SingleContact::updategdRef(const Vec& gdParent) {
    ContourLink::updategdRef(gdParent);
    if (gdSize) {
      int gdIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        gdN >> gd(0, 0);
        gdIndSizeNormal++;
      }
      if (fdf and fdf->isSetValued())
        gdT >> gd(gdIndSizeNormal, gdSize - 1);
    }
  }

  void SingleContact::calcxSize() {
    ContourLink::calcxSize();
    xSize = 0;
  }

  void SingleContact::calclaSize(int j) {
    ContourLink::calclaSize(j);
    if (j == 0) { // IA
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections();

    }
    else if (j == 1) { // IG
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections();

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive;
    }
    else if (j == 2) { // IB
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections();

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[0];

    }
    else if (j == 3) { // IH
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued and active
      if (fdf and fdf->isSetValued())
        laSize += getFrictionDirections() * gdActive[1];

      //check if contact is active --> else lambda Size will get zero...
      laSize *= gActive * gdActive[0];

    }
    else if (j == 4) { // IG
      //Add 1 to lambda size if normal force law is setValued and active
      if (fcl->isSetValued())
        laSize = gActive;
    }
    else if (j == 5) { // IB
      laSize = gActive * gdActive[0];
    }
    else
      THROW_MBSIMERROR("Internal error");
  }

  void SingleContact::calcgSize(int j) {
    ContourLink::calcgSize(j);
    if (j == 0) { // IA
      gSize = 1;
    }
    else if (j == 1) { // IG
      gSize = gActive;
    }
    else if (j == 2) { // IB
      gSize = gActive * gdActive[0];
    }
    else
      THROW_MBSIMERROR("Internal error");
  }

  void SingleContact::calcgdSize(int j) {
    // TODO: avoid code duplication for maintenance
    ContourLink::calcgdSize(j);
    if (j == 0) { // all contacts
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += getFrictionDirections();

    }
    else if (j == 1) { // closed contacts
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += getFrictionDirections();

      gdSize *= gActive;

    }
    else if (j == 2) { // contacts which stay closed
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += getFrictionDirections();

      gdSize *= gActive * gdActive[0];

    }
    else if (j == 3) { // sticking contacts
      // add 1 to gdSize if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      // add number of friction directions to gdSize if friction force law is setValued
      if (fdf and fdf->isSetValued())
        gdSize += gdActive[1] * getFrictionDirections();

      gdSize *= gActive * gdActive[0];

    }
    else
      THROW_MBSIMERROR("Internal error");
  }

  void SingleContact::calcrFactorSize(int j) {
    ContourLink::calcrFactorSize(j);
    int addition = 0;
    if(fcl->isSetValued())
      addition += 1;
    if (j == 0) { // IA
      rFactorSize = addition + min(getFrictionDirections(), 1);
    }
    else if (j == 1) { // IG
      rFactorSize = gActive * (addition + min(getFrictionDirections(), 1));
    }
    else if (j == 2) { // IB
      rFactorSize = gActive * gdActive[0] * (addition + min(getFrictionDirections(), 1));
    }
    else if (j == 3) { // IB
      rFactorSize = gActive * gdActive[0] * (addition + gdActive[1] * min(getFrictionDirections(), 1));
    }
  }

  void SingleContact::calcsvSize() {
    ContourLink::calcsvSize();

    //Add length due to normal direction
    svSize = fcl->isSetValued() ? 1 : 0;

    //Add length due to tangentinal direction
    if (fdf)
      svSize += fdf->isSetValued() ? min(getFrictionDirections(), 1) : 0;
  }

  void SingleContact::calcLinkStatusSize() {
    ContourLink::calcLinkStatusSize();
    //assert(contactKinematics->getNumberOfPotentialContactPoints() == 1); //not necessary anymore as SingleContact has only one contact point
    LinkStatusSize = 1;
    LinkStatus.resize(LinkStatusSize);
  }

  void SingleContact::calcLinkStatusRegSize() {
    ContourLink::calcLinkStatusRegSize();
    //assert(contactKinematics->getNumberOfPotentialContactPoints() == 1); //not necessary anymore as SingleContact has only one contact point
    LinkStatusRegSize = 1;
    LinkStatusReg.resize(LinkStatusRegSize);
  }

  void SingleContact::init(InitStage stage) {
    if (stage == resolveXMLPath) {
      if (saved_ref1 != "" && saved_ref2 != "")
        connect(getByPath<Contour>(saved_ref1), getByPath<Contour>(saved_ref2));
      ContourLink::init(stage);
    }
    else if (stage == resize) {
      ContourLink::init(stage);

      RF.resize(1+getFrictionDirections());
      RM.resize(1+getFrictionDirections());

      iF = Index(0,1+getFrictionDirections()-1);
      iM = Index(0,-1);
      DF.resize(1+getFrictionDirections(),NONINIT);

      lambdaT.resize(getFrictionDirections());

      //TODO: Change this if la should be the vector of nonsmooth forces
      la.resize(1 + getFrictionDirections());
      g.resize(1);
      gd.resize(1 + getFrictionDirections());
      gddN.resize(1);
      gddT.resize(getFrictionDirections());
      gdnN.resize(1);
      gdnT.resize(getFrictionDirections());
      rrel.resize(1);
      vrel.resize(1 + getFrictionDirections());
      lambda.resize(1 + getFrictionDirections());

      if (getFrictionDirections() == 0)
        gdActive[1] = false;
    }
    else if (stage == unknownStage) {
      ContourLink::init(stage);

      if(contour[0]==NULL or contour[1]==NULL)
        THROW_MBSIMERROR("Not all connections are given!");

      if (contactKinematics == 0) {
        contactKinematics = contour[0]->findContactPairingWith(contour[0]->getType(), contour[1]->getType());
        if (contactKinematics == 0) {
          contactKinematics = contour[1]->findContactPairingWith(contour[1]->getType(), contour[0]->getType());
          if (contactKinematics == 0) {
            contactKinematics = contour[0]->findContactPairingWith(contour[1]->getType(), contour[0]->getType());
            if (contactKinematics == 0) {
              contactKinematics = contour[1]->findContactPairingWith(contour[0]->getType(), contour[1]->getType());
              if (contactKinematics == 0)
                THROW_MBSIMERROR("(Contact::init): Unknown contact pairing between Contour \"" + contour[0]->getType() + "\" and Contour\"" + contour[1]->getType() + "\"!");
            }
          }
        }
      }

      //TODO: check if indices are set correctly?
      laN.resize() >> la(0, 0);
      laT.resize() >> la(1, getFrictionDirections());

      gdN.resize() >> gd(0, 0);
      gdT.resize() >> gd(1, getFrictionDirections());

      gddNBuf.resize(1);
      gddTBuf.resize(getFrictionDirections());
    }
    else if (stage == preInit) {
      ContourLink::init(stage);

      gActive = 1;
      gActive0 = 1;
      gdActive = new unsigned int[2];
      gddActive = new unsigned int[2];

      for (int j = 0; j < 1 + min(1, getFrictionDirections()); j++)
        gdActive[j] = 1;
      for (int j = 1 + min(1, getFrictionDirections()); j < 2; j++)
        gdActive[j] = 0;
      for (int j = 0; j < 1 + min(1, getFrictionDirections()); j++)
        gddActive[j] = 1;
      for (int j = 1 + min(1, getFrictionDirections()); j < 2; j++)
        gddActive[j] = 0;

      if(not fcl)
        updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceP;
      else if(fcl->isSetValued())
        updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceM;
      else
        updateGeneralizedNormalForce_ = &SingleContact::updateGeneralizedNormalForceS;

      if(not fdf)
        updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForce0;
      else if(fdf->isSetValued())
        updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForceM;
      else
        updateGeneralizedTangentialForce_ = &SingleContact::updateGeneralizedTangentialForceS;
    }
    else if (stage == plotting) {
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (getPlotFeature(openMBV) == enabled && (openMBVContactFrame[0] || contactArrow || frictionArrow)) {
          openMBVContactGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openMBVContactGrp->setName(name + "_ContactGroup");
          openMBVContactGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVContactGrp);

          if (openMBVContactFrame[0]) {
            for (unsigned int i = 0; i < 2; i++) { // frames
              string name = string((i == 0 ? "A" : "B")) + string("_") + string(contour[i]->getName());
              openMBVContactFrame[i]->setName(name);
              openMBVContactGrp->addObject(openMBVContactFrame[i]);
            }
          }
          // arrows
          if (contactArrow) {
            contactArrow->setName("NormalForce_B");
            openMBVContactGrp->addObject(contactArrow);
          }
          if (frictionArrow) { // friction force
            frictionArrow->setName("FrictionForce_B");
            openMBVContactGrp->addObject(frictionArrow);
          }
        }
#endif
        ContourLink::init(stage);
      }

    }
    else if(stage == LASTINITSTAGE) {
      if(contactKinematics->getNumberOfPotentialContactPoints() > 1)
        throw new MBSimError("Contact has contact kinematics with more than one possible contact point. Use Multi-Contact for that!");
    }
    else {
      ContourLink::init(stage);
    }
    if(fcl) fcl->init(stage);
    if(fdf) fdf->init(stage);
    if(fnil) fnil->init(stage);
    if(ftil) ftil->init(stage);
  }

  bool SingleContact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if (fdf)
      flag |= fdf->isSetValued();
    return flag;
  }

  bool SingleContact::isSingleValued() const {
    if (fcl->isSetValued()) {
      if (fdf)
        return not fdf->isSetValued();
      return false;
    }
    return true;
  }

  void SingleContact::updateLinkStatus(double t) {
    if (gActive) {
      LinkStatus(0) = 2;
      if (ftil) {
        if (ftil->isSticking(laT, gdnT, gdT, laN(0), LaTol, gdTol))
          LinkStatus(0) = 3;
        else
          LinkStatus(0) = 4;
      }
    }
    else
      LinkStatus(0) = 1;
  }

  void SingleContact::updateLinkStatusReg(double t) {
    if (gActive) {
      LinkStatusReg(0) = 2;
    }
    else {
      LinkStatusReg(0) = 1;
    }
  }

  bool SingleContact::isActive() const {
    return gActive ? true : false;
  }

  bool SingleContact::gActiveChanged() {
    bool changed = (gActive0 != gActive ? true : false);
    gActive0 = gActive;
    return changed;
  }

  bool SingleContact::detectImpact() {
    return gActive0 < gActive ? true : false;
  }

  void SingleContact::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (getPlotFeature(openMBV) == enabled && (openMBVContactFrame[0] || contactArrow || frictionArrow)) {
        // frames
        if (openMBVContactFrame[0]) {
          for (unsigned int i = 0; i < 2; i++) {
            vector<double> data;
            data.push_back(t);
            data.push_back(cFrame[i]->getPosition(t)(0));
            data.push_back(cFrame[i]->getPosition()(1));
            data.push_back(cFrame[i]->getPosition()(2));
            Vec3 cardan = AIK2Cardan(cFrame[i]->getOrientation(t));
            data.push_back(cardan(0));
            data.push_back(cardan(1));
            data.push_back(cardan(2));
            data.push_back(0);
            openMBVContactFrame[i]->append(data);
          }
        }
        // arrows
        vector<double> data;
        if (contactArrow) {
          data.push_back(t);
          data.push_back(cFrame[1]->getPosition(t)(0));
          data.push_back(cFrame[1]->getPosition()(1));
          data.push_back(cFrame[1]->getPosition()(2));
          Vec3 F = getGlobalForceDirection(t).col(0)*getGeneralizedNormalForce(t);
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back(nrm2(F));
          contactArrow->append(data);
        }
        if (frictionArrow && getFrictionDirections() > 0) { // friction force
          data.clear();
          data.push_back(t);
          data.push_back(cFrame[1]->getPosition()(0));
          data.push_back(cFrame[1]->getPosition()(1));
          data.push_back(cFrame[1]->getPosition()(2));
          Vec3 F = getGlobalForceDirection(t)(Index(0,2),Index(1, getFrictionDirections()))*getGeneralizedTangentialForce(t);
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          // TODO fdf->isSticking(getGeneralizedRelativeVelocity(t)(Index(1,getFrictionDirections())), gdTol)
          data.push_back((fdf->isSetValued() && laT.size()) ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
          frictionArrow->append(data);
        }
      }
#endif
      ContourLink::plot(t, dt);
    }
  }

  void SingleContact::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      ContourLink::closePlot();
    }
  }

  void SingleContact::setNormalForceLaw(GeneralizedForceLaw *fcl_) { 
    fcl = fcl_; 
    if(fcl) 
      fcl->setParent(this);
  }

  void SingleContact::setNormalImpactLaw(GeneralizedImpactLaw *fnil_) { 
    fnil = fnil_; 
    if(fnil) 
      fnil->setParent(this);
  }

  void SingleContact::setTangentialForceLaw(FrictionForceLaw *fdf_) { 
    fdf = fdf_; 
    if(fdf) 
      fdf->setParent(this);
  }

  void SingleContact::setTangentialImpactLaw(FrictionImpactLaw *ftil_) { 
    ftil = ftil_; 
    if(ftil) 
      ftil->setParent(this);
  }

  void SingleContact::solveImpactsFixpointSingle(double t, double dt) {
    if (gActive) {
      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa();
      const Vec &b = ds->getb(false);

      int addIndexNormal = 0;
      if (fcl->isSetValued()) {
        addIndexNormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        LaN(0) = fnil->project(LaN(0), gdnN(0), gdN(0), rFactor(0));
      }

      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + addIndexNormal + i);
          for (int j = ia[laInd + i + addIndexNormal]; j < ia[laInd + 1 + i + addIndexNormal]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }

        //            if (ftil) //There must be a ftil coming with a setValued fdf
        LaT = ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*dt, rFactor(addIndexNormal));
      }
    }
  }

  void SingleContact::solveConstraintsFixpointSingle(double t) {
    if (gdActive[0]) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb(false);

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        laN(0) = fcl->project(laN(0), gddN(0), rFactor(0));
      }

      if (fdf and fdf->isSetValued() and gdActive[1]) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gddT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }

        laT = fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexnormal));
      }
    }
  }

  void SingleContact::solveImpactsGaussSeidel(double t, double dt) {
    assert(getFrictionDirections() <= 1);
    if (gActive) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa();
      const Vec &b = ds->getb(false);

      //TODO: check indices (in other solution algorithms too!)
      const double om = 1.0;
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        const double buf = fnil->solve(a[ia[laInd]], gdnN(0), gdN(0));
        LaN(0) += om * (buf - LaN(0));
      }

      if (ftil) {
        gdnT(0) = b(laInd + addIndexnormal);
        for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
          gdnT(0) += a[j] * LaMBS(ja[j]);

        Vec buf = ftil->solve(ds->getG()(Index(laInd + addIndexnormal, laInd + getFrictionDirections())), gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*dt);
        LaT += om * (buf - LaT);
      }
    }
  }

  void SingleContact::solveConstraintsGaussSeidel(double t) {
    assert(getFrictionDirections() <= 1);

    if (gdActive[0]) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb(false);

      const double om = 1.0; // relaxation parameter omega (cf. Foerg, dissertation, p. 102)

      int addIndexNormal = 0;
      if (fcl->isSetValued()) {
        addIndexNormal++;
        gddN(0) = b(laInd);
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        const double buf = fcl->solve(a[ia[laInd]], gddN(0));
        laN(0) += om * (buf - laN(0));
      }

      if (fdf and fdf->isSetValued() and gdActive[1]) {
        gddT(0) = b(laInd + addIndexNormal);
        for (int j = ia[laInd + addIndexNormal] + 1; j < ia[laInd + addIndexNormal + 1]; j++)
          gddT(0) += a[j] * laMBS(ja[j]);

        Vec buf = fdf->solve(ds->getG()(Index(laInd + addIndexNormal, laInd + addIndexNormal + getFrictionDirections() - 1)), gddT, fcl->isSetValued()?laN(0):lambdaN);
        laT += om * (buf - laT);
      }
    }
  }

  void SingleContact::solveImpactsRootFinding(double t, double dt) {
    if (gActive) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa();
      const Vec &b = ds->getb(false);

      //compute residuum for normal direction
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);

        res(0) = LaN(0) - fnil->project(LaN(0), gdnN(0), gdN(0), rFactor(0));
      }

      //compute residuum for tangential directions
      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }
        //            if (ftil) There must be a frictional impact law if fdf is set valued!
        res(addIndexnormal, addIndexnormal + getFrictionDirections() - 1) = LaT - ftil->project(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*dt, rFactor(addIndexnormal));
      }
    }
  }

  void SingleContact::solveConstraintsRootFinding(double t) {
    if (gdActive[0]) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb(false);

      //compute residuum for normal direction
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * laMBS(ja[j]);

        res(0) = laN(0) - fcl->project(laN(0), gddN(0), rFactor(0));
      }

      //compute residuum for tangential directions
      if (fdf and fdf->isSetValued()) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gdnT(i) += a[j] * laMBS(ja[j]);
        }
        //            if (ftil) There must be a frictional impact law if fdf is set valued!
        res(addIndexnormal, addIndexnormal + getFrictionDirections() - 1) = laT - fdf->project(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexnormal));
      }
    }
  }

  void SingleContact::jacobianConstraints(double t) {
    if (gdActive[0]) {

      const SqrMat Jprox = ds->getJprox();
      const SqrMat G = ds->getG(t);

      //TODO: separate normal and tangential

      RowVec jp1 = Jprox.row(laInd);
      RowVec e1(jp1.size());
      e1(laInd) = 1;

      int addIndexNormal = 0;
      if(fcl->isSetValued()) {
    	  addIndexNormal++;
    	  Vec diff = fcl->diff(laN(0), gddN(0), rFactor(0));

    	  jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk)
    	  for (int i = 0; i < G.size(); i++)
    		  jp1(i) -= diff(1) * G(laInd, i);
      }

      if (getFrictionDirections() == 1) {
        Mat diff = fdf->diff(laT, gddT(0, 0), fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexNormal));
        RowVec jp2 = Jprox.row(laInd + addIndexNormal);
        RowVec e2(jp2.size());
        e2(laInd + 1) = 1;
        Mat e(2, jp2.size());
        e(0, laInd) = 1;
        e(1, laInd + addIndexNormal) = 1;
        jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
        //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp2(i) -= diff(0, 1) * G(laInd + addIndexNormal, i);

      }
      else if (getFrictionDirections() == 2) {
        Mat diff = ftil->diff(laT, gddT, gdT, fcl->isSetValued()?laN(0):lambdaN, rFactor(addIndexNormal));
        Mat jp2 = Jprox(Index(laInd + addIndexNormal, laInd + addIndexNormal + 1), Index(0, Jprox.cols() - 1));
        Mat e2(2, jp2.cols());
        e2(0, laInd + addIndexNormal) = 1;
        e2(1, laInd + addIndexNormal + 1) = 1;
        jp2 = e2 - diff(Index(0, 1), Index(4, 4)) * e1 - diff(Index(0, 1), Index(0, 1)) * e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk+1,laInd+laIndk+2),Index(0,G.size()-1))
        for (int i = 0; i < G.size(); i++) {
          jp2(0, i) = diff(0, 2) * G(laInd + addIndexNormal, i) + diff(0, 3) * G(laInd + addIndexNormal + 1, i);
          jp2(1, i) = diff(1, 2) * G(laInd + addIndexNormal, i) + diff(1, 3) * G(laInd + addIndexNormal + 1, i);
        }
      }
    }
  }

  void SingleContact::jacobianImpacts(double t, double dt) {
    if (gActive) {

      const SqrMat Jprox = ds->getJprox();
      const SqrMat G = ds->getG(t);

      RowVec jp1 = Jprox.row(laInd);
      RowVec e1(jp1.size());
      e1(laInd) = 1;

      int addIndexNormal = 0;
      if (fcl->isSetValued()) {
    	  addIndexNormal++;

    	  Vec diff = fnil->diff(LaN(0), gdnN(0), gdN(0), rFactor(0));

    	  jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk)
    	  for (int i = 0; i < G.size(); i++)
    		  jp1(i) -= diff(1) * G(laInd, i);
      }

      if (getFrictionDirections() == 1) {
        Mat diff = ftil->diff(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*dt, rFactor(addIndexNormal));
        RowVec jp2 = Jprox.row(laInd + addIndexNormal);
        RowVec e2(jp2.size());
        e2(laInd + addIndexNormal) = 1;
        Mat e(2, jp2.size());
        e(0, laInd) = 1;
        e(1, laInd + addIndexNormal) = 1;
        jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk)
        //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk)
        for (int i = 0; i < G.size(); i++)
          jp2(i) -= diff(0, 1) * G(laInd + addIndexNormal, i);

      }
      else if (getFrictionDirections() == 2) {
        Mat diff = ftil->diff(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*dt, rFactor(addIndexNormal));
        Mat jp2 = Jprox(Index(laInd + addIndexNormal, laInd + addIndexNormal + 1), Index(0, Jprox.cols() - 1));
        Mat e2(2, jp2.cols());
        e2(0, laInd + addIndexNormal) = 1;
        e2(1, laInd + addIndexNormal + 1) = 1;
        jp2 = e2 - diff(Index(0, 1), Index(4, 4)) * e1 - diff(Index(0, 1), Index(0, 1)) * e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk+1,laInd+laIndk+2),Index(0,G.size()-1))
        for (int i = 0; i < G.size(); i++) {
          jp2(0, i) = diff(0, 2) * G(laInd + addIndexNormal, i) + diff(0, 3) * G(laInd + addIndexNormal + 1, i);
          jp2(1, i) = diff(1, 2) * G(laInd + addIndexNormal, i) + diff(1, 3) * G(laInd + addIndexNormal + 1, i);
        }
      }
    }
  }

  void SingleContact::updaterFactors(double t) {
    if (gdActive[0]) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        double sumN = 0;
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          sumN += fabs(a[j]);
        const double aN = a[ia[laInd]];
        if (aN > sumN) {
          rFactorUnsure(0) = 0;
          rFactor(0) = 1.0 / aN;
        }
        else {
          rFactorUnsure(0) = 1;
          rFactor(0) = rMax / aN;
        }
      }

      if (fdf and gdActive[1] and fdf->isSetValued()) {
        double sumT1 = 0;
        double sumT2 = 0;
        double aT1, aT2;
        if (getFrictionDirections() == 1) {
          for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
            sumT1 += fabs(a[j]);
          aT1 = a[ia[laInd + addIndexnormal]];
          if (aT1 > sumT1) {
            rFactorUnsure(addIndexnormal) = 0;
            rFactor(addIndexnormal) = 1.0 / aT1;
          }
          else {
            rFactorUnsure(addIndexnormal) = 1;
            rFactor(addIndexnormal) = rMax / aT1;
          }
        }
        else if (getFrictionDirections() == 2) {
          for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
            sumT1 += fabs(a[j]);
          for (int j = ia[laInd + addIndexnormal + 1] + 1; j < ia[laInd + addIndexnormal + 2]; j++)
            sumT2 += fabs(a[j]);
          aT1 = a[ia[laInd + addIndexnormal]];
          aT2 = a[ia[laInd + addIndexnormal + 1]];

          // TODO rFactorUnsure
          if (aT1 - sumT1 >= aT2 - sumT2)
            if (aT1 + sumT1 >= aT2 + sumT2)
              rFactor(addIndexnormal) = 2.0 / (aT1 + aT2 + sumT1 - sumT2);
            else
              rFactor(addIndexnormal) = 1.0 / aT2;
          else if (aT1 + sumT1 < aT2 + sumT2)
            rFactor(addIndexnormal) = 2.0 / (aT1 + aT2 - sumT1 + sumT2);
          else
            rFactor(addIndexnormal) = 1.0 / aT1;
        }
      }
    }
  }

  void SingleContact::checkConstraintsForTermination(double t) {
    if (gdActive[0]) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb(false);

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;

        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        if (!fcl->isFulfilled(laN(0), gddN(0), laTol, gddTol)) {
          ds->setTermination(false);
          return;
        }
      }

      if (fdf && gdActive[1]) {

        for (unsigned int i = 0; i < gdActive[1] * getFrictionDirections(); i++) { //TODO: Is there any other number than 0 or one for gdActive? otherwithe the multiplication could be deleted again...
          gddT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gddT(i) += a[j] * laMBS(ja[j]);
        }

        if (!fdf->isFulfilled(laT, gddT, fcl->isSetValued()?laN(0):lambdaN, laTol, gddTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void SingleContact::checkImpactsForTermination(double t, double dt) {
    if (gActive) {

      const double *a = ds->getGs(t)();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa();
      const Vec &b = ds->getb(false);

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * LaMBS(ja[j]);
        if (!fnil->isFulfilled(LaN(0), gdnN(0), gdN(0), LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }

      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gdnT(i) += a[j] * LaMBS(ja[j]);
        }
        if (!ftil->isFulfilled(LaT, gdnT, gdT, fcl->isSetValued()?LaN(0):lambdaN*dt, LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void SingleContact::checkActive(double t, int j) {
    if (j == 1) { // formerly checkActiveg()
      gActive = fcl->isClosed(getGeneralizedRelativePosition(t)(0), gTol) ? 1 : 0;
      gdActive[0] = gActive;
      gdActive[1] = gdActive[0];
    }
    else if (j == 2) { // formerly checkActivegd()
      gdActive[0] = gActive ? (fcl->remainsClosed(getGeneralizedRelativeVelocity(t)(0), gdTol) ? 1 : 0) : 0;
      gdActive[1] = getFrictionDirections() && gdActive[0] ? (fdf->isSticking(getGeneralizedRelativeVelocity(t)(Index(1,getFrictionDirections())), gdTol) ? 1 : 0) : 0;
      gddActive[0] = gdActive[0];
      gddActive[1] = gdActive[1];
    }
    else if (j == 3) { // formerly checkActivegdn() (new gap velocities)
      if (gActive) { // contact is closed
        if (gdnN(0) <= gdTol) { // contact stays closed // TODO bilateral contact
          gdActive[0] = true;
          gddActive[0] = true;
          if (getFrictionDirections()) {
            if (nrm2(gdnT) <= gdTol) {
              gdActive[1] = true;
              gddActive[1] = true;
            }
            else {
              gdActive[1] = false;
              gddActive[1] = false;
            }
          }
        }
        else { // contact will open
          gdActive[0] = false;
          gdActive[1] = false;
          gddActive[0] = false;
          gddActive[1] = false;
        }
      }
    }
    else if (j == 4) { // formerly checkActivegdd()
      if (gActive) {
        if (gdActive[0]) {
          if (gddN(0) <= gddTol) { // contact stays closed on velocity level
            gddActive[0] = true;
            if (getFrictionDirections()) {
              if (gdActive[1]) {
                if (nrm2(gddT) <= gddTol)
                  gddActive[1] = true;
                else
                  gddActive[1] = false;
              }
            }
          }
          else { // contact will open on velocity level
            gddActive[0] = false;
            gddActive[1] = false;
          }
        }
      }
    }
    else if (j == 5) { // activity clean-up, if there is no activity on acceleration or velocity level, also more basic levels are set to non-active
      if (gActive) {
        if (gdActive[0]) {
          if (gdActive[1]) {
            if (!gddActive[1])
              gdActive[1] = false;
          }
          if (!gddActive[0]) {
            gActive = false;
            gdActive[0] = false;
            gdActive[1] = false;
          }
        }
        else
          gActive = false;
      }
    }
    else if (j == 6) { // just observe closing contact
      if (rootID == 3) {
        gActive = true;
        gdActive[0] = true;
        gdActive[1] = true;
        gddActive[0] = true;
        gddActive[1] = true;
      }
    }
    else if (j == 7) { // just observe slip-stick transitions
      if (getFrictionDirections()) {
        if (rootID == 2) {
          gdActive[1] = true;
          gddActive[1] = true;
        }
      }
    }
    else if (j == 8) { // just observe opening contacts and stick-slip transitions
      if (jsv(0) && rootID == 1) { // opening contact
        gddActive[0] = false;
        gddActive[1] = false;
      }
      if (getFrictionDirections()) {
        if (jsv(1) && rootID == 1) { // stick-slip transition
          gddActive[1] = false;
        }
      }
    }
    else
      THROW_MBSIMERROR("Internal error");
  }

  int SingleContact::getFrictionDirections() const {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void SingleContact::initializeUsingXML(DOMElement *element) {
    ContourLink::initializeUsingXML(element);
    DOMElement *e;

    //Set contact law
    e = E(element)->getFirstElementChildNamed(MBSIM%"normalForceLaw");
    GeneralizedForceLaw *gfl = ObjectFactory::createAndInit<GeneralizedForceLaw>(e->getFirstElementChild());
    setNormalForceLaw(gfl);

    //Set impact law (if given)
    e = E(element)->getFirstElementChildNamed(MBSIM%"normalImpactLaw");
    if (e) {
      GeneralizedImpactLaw *gifl = ObjectFactory::createAndInit<GeneralizedImpactLaw>(e->getFirstElementChild());
      setNormalImpactLaw(gifl);
    }

    //Set friction law (if given)
    e = E(element)->getFirstElementChildNamed(MBSIM%"tangentialForceLaw");
    if (e) {
      FrictionForceLaw *ffl = ObjectFactory::createAndInit<FrictionForceLaw>(e->getFirstElementChild());
      setTangentialForceLaw(ffl);
    }

    //Set friction impact law (if given)
    e = E(element)->getFirstElementChildNamed(MBSIM%"tangentialImpactLaw");
    if (e) {
      FrictionImpactLaw *fil = ObjectFactory::createAndInit<FrictionImpactLaw>(e->getFirstElementChild());
      setTangentialImpactLaw(fil);
    }

    //Save contour names for initialization
    e = E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1 = E(e)->getAttribute("ref1");
    saved_ref2 = E(e)->getAttribute("ref2");

#ifdef HAVE_OPENMBVCPPINTERFACE
    //Contact points
    if (E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVContactPoints")) {
      OpenMBVFrame ombv;
      openMBVContactFrame[0]=ombv.createOpenMBV(e); 
      openMBVContactFrame[1]=OpenMBV::ObjectFactory::create(openMBVContactFrame[0]);
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVNormalForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      contactArrow=ombv.createOpenMBV(e); 
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVTangentialForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      frictionArrow=ombv.createOpenMBV(e); 
    }
#endif
  }

  void SingleContact::updatecorr(int j) {
    if (j == 1) { // IG position
      if (gActive) { // Contact was closed
        if (gdActive[0])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = 1e-14; // Contact opens, projection to positive normal distance
      }
    }
    else if (j == 2) {
      if (gActive && gdActive[0]) { // Contact was closed
        if (gddActive[0])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = 1e-14; // Contact opens, projection to positive normal distance
      }
    }
    else if (j == 4) {
      if (rootID == 1) {
        gddN = gddNBuf;
        gddT = gddTBuf;
      }
      if (gActive && gdActive[0]) { // Contact was closed
        if (gddActive[0])
          corr(0) = 0; // Contact stays closed, regular projection
        else
          corr(0) = 1e-16; // Contact opens, projection to positive normal distance
        if (getFrictionDirections()) {
          if (gdActive[1]) { // Contact was sticking
            if (gddActive[1]) {
              corr(1) = 0; // Contact stays sticking, regular projection
              if (getFrictionDirections() > 1)
                corr(2) = 0; // Contact stays sticking, regular projection
            }
            else {
              corr(1) = gddT(0) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
              if (getFrictionDirections() > 1)
                corr(2) = gddT(1) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
            }
          }
        }
      }
    }
    else
      THROW_MBSIMERROR("Internal error");
  }

  void SingleContact::calccorrSize(int j) {
    ContourLink::calccorrSize(j);
    if (j == 1) { // IG
      corrSize = gActive;
    }
    else if (j == 2) { // IB
      corrSize = gActive * gdActive[0];
    }
    //    else if(j==3) { // IG
    //      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
    //        corrIndk = corrSize;
    //        corrSizek = gActive[i]*(1+getFrictionDirections());
    //        corrSize = corrSizek;
    //      }
    //    }
    else if (j == 4) { // IH
      corrSize = gActive * gdActive[0] * (1 + gdActive[1] * getFrictionDirections());
    }
    else
      THROW_MBSIMERROR("Internal error");
  }

  void SingleContact::checkRoot(double t) {
    rootID = 0;
    if (jsv(0)) {
      if (gActive)
        rootID = 1; // contact was closed -> opening
      else
        rootID = 3; // contact was open -> impact
    }
    if (getFrictionDirections()) {
      if (jsv(1)) {
        if (gdActive[1])
          rootID = 1; // contact was sticking -> sliding
        else {
          if (getFrictionDirections() == 1)
            rootID = 2; // contact was sliding -> sticking
          else if (nrm2(getGeneralizedRelativeVelocity(t)((Index(1,getFrictionDirections())))) <= gdTol)
            rootID = 2; // contact was sliding -> sticking
        }
      }
    }
    ds->setRootID(max(ds->getRootID(), rootID));
  }

  void SingleContact::LinearImpactEstimation(double t, Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
    if (gActive) {
      gAct_(*IndActive_) = getGeneralizedRelativePosition(t)(0);
      (*IndActive_)++;
    }
    else {
      // TODO check if already computed
      Vec3 Wn = cFrame[0]->getOrientation(t).col(0);
      // TODO check if already computed
      Vec3 WvD = cFrame[1]->getVelocity(t) - cFrame[0]->getVelocity(t);
      gdInActive_(*IndInActive_) = Wn.T() * WvD;
      gInActive_(*IndInActive_) = getGeneralizedRelativePosition(t)(0);
      (*IndInActive_)++;
    }
  }

  void SingleContact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
    if (gActive)
      (*sizeActive_)++;
    else
      (*sizeInActive_)++;
  }

}
