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
using namespace MBXMLUtils;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  SingleContact::SingleContact(const string &name) :
      LinkMechanics(name), contactKinematics(0), fcl(0), fdf(0), fnil(0), ftil(0), cpData(0), gActive(0), gActive0(0), gdActive(0), gddActive(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , openMBVContactGrp(0), openMBVContactFrameSize(0), openMBVContactFrameEnabled(true), contactArrow(NULL), frictionArrow(NULL)
#endif
          , saved_ref1(""), saved_ref2("") {
  }

  SingleContact::~SingleContact() {

    //TODO: who deletes contactKinamtics if contact is used in a multi-contact?
//    if (contactKinematics) {
//      delete contactKinematics;
//      contactKinematics = 0;
//    }
    /* Delete will fail if the same object is used for more than one Contact.
     * TODO: A delete concept (who deletes what) is still missing in MBSim.
     if(fcl) { delete fcl; fcl=0; }
     if(fdf) { delete fdf; fdf=0; }
     if(fnil) { delete fnil; fnil=0; }
     if(ftil) { delete ftil; ftil=0; }*/

    if (cpData)
      delete[] cpData;
    if (gdActive)
      delete[] gdActive;
    if (gddActive)
      delete[] gddActive;
  }

  void SingleContact::updatewb(double t, int j) {
    if (gdActive[j]) {
      for (unsigned i = 0; i < 2; ++i) //TODO: only two contours are interacting
        wb += fF[i](Range<Fixed<0>,Fixed<2> >(), Range<Var,Var>(0,laSize-1)).T() * cpData[i].getFrameOfReference().getGyroscopicAccelerationOfTranslation(j);

      contactKinematics->updatewb(wb, g, cpData);
    }
  }

  void SingleContact::updateW(double t, int j) {
    if (gActive) {

      int fFTangCol = 1;
      if (fcl->isSetValued())
        fF[1].set(0,cpData[0].getFrameOfReference().getOrientation().col(0));
      else
        fFTangCol = 0;

      if (getFrictionDirections()) {
        if (fdf->isSetValued()) {
          fF[1].set(fFTangCol, cpData[0].getFrameOfReference().getOrientation().col(1));
          if (getFrictionDirections() > 1)
            fF[1].set(fFTangCol+1, cpData[0].getFrameOfReference().getOrientation().col(2));
        }
      }

      fF[0] = -fF[1];

      for (unsigned int i = 0; i < 2; i++) //TODO: only two contours are interacting at one time?
        W[j][i] += cpData[i].getFrameOfReference().getJacobianOfTranslation(j).T() * fF[i](Range<Fixed<0>,Fixed<2> >(),Range<Var,Var>(0,laSize-1));
    }
  }

  void SingleContact::updateV(double t, int j) {
    if (getFrictionDirections()) {
      if (fdf->isSetValued()) {
        if (gdActive[0] and not gdActive[1]) {
          for (unsigned int i = 0; i < 2; i++) { //TODO: only two contours are interacting at one time?
            V[j][i] += cpData[i].getFrameOfReference().getJacobianOfTranslation(j).T() * fF[i](Range<Fixed<0>,Fixed<2> >(), iT) * fdf->dlaTdlaN(gdT, laN(0));
          }
        }
      }
    }
  }

  void SingleContact::updateh(double t, int j) {
    laN(0) = (*fcl)(g(0), gdN(0));

    applyh(t, j);
  }

  void SingleContact::updateg(double t) {
	  if (g.size())
	    contactKinematics->updateg(g, cpData);

  }

  void SingleContact::updategd(double t) {
    if ((fcl->isSetValued() and gdActive[0]) or (not fcl->isSetValued() and fcl->isActive(g(0), 0))) { // TODO: nicer implementation
      for (unsigned int i = 0; i < 2; i++)
        contour[i]->updateKinematicsForFrame(cpData[i], velocities); // angular velocity necessary e.g. see ContactKinematicsSpherePlane::updatewb

      Vec3 Wn = cpData[0].getFrameOfReference().getOrientation().col(0);

      Vec3 WvD = cpData[1].getFrameOfReference().getVelocity() - cpData[0].getFrameOfReference().getVelocity();

      gdN(0) = Wn.T() * WvD;

      if (gdT.size()) {
        Mat3xV Wt(gdT.size());
        Wt.set(0, cpData[0].getFrameOfReference().getOrientation().col(1));
        if (gdT.size() > 1)
          Wt.set(1, cpData[0].getFrameOfReference().getOrientation().col(2));

        gdT = Wt.T() * WvD;
      }
    }
  }

  void SingleContact::updateStopVector(double t) {
    if (gActive != gdActive[0])
      throw;
    if (gActive) {
      sv(0) = gddN(0) > gddTol ? -1 : 1;
      if (gdActive[1]) {
        if (getFrictionDirections()) {
          sv(1) = nrm2(gddT) > gddTol ? -1 : 1;
          if ((int) sv(1) == -1) {
            gddNBuf = gddN;
            gddTBuf = gddT;
          }
        }
      }
      else {
        if (getFrictionDirections() == 1)
          sv(1) = gdT(0) > 0 ? 1 : -1;
        else if (getFrictionDirections() == 2) {
          sv(1) = gdT(0) + gdT(1); // TODO: is there a better concept?
        }
      }
    }
    else {
      sv(0) = g(0) > 0 ? 1 : -1;
      if (getFrictionDirections())
        sv(1) = 1;
    }
  }

  void SingleContact::updateJacobians(double t, int j) {
    if (gActive)
      for (unsigned int i = 0; i < 2; i++)
        contour[i]->updateJacobiansForFrame(cpData[i], j);
  }

  void SingleContact::updateWRef(const Mat& WParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      int hInd = contour[i]->gethInd(j);
      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
      Index J = Index(laInd, laInd + laSize - 1);
      W[j][i] >> WParent(I, J);
    }
  }

  void SingleContact::updateVRef(const Mat& VParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      int hInd = contour[i]->gethInd(j);
      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
      Index J = Index(laInd, laInd + laSize - 1);
      V[j][i] >> VParent(I, J);
    }
  }

  void SingleContact::updatehRef(const Vec& hParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      int hInd = contour[i]->gethInd(j);
      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
      h[j][i] >> hParent(I);
    }
  }

  void SingleContact::updatelaRef(const Vec& laParent) {
    LinkMechanics::updatelaRef(laParent);
    if (laSize) {
      int laIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        laN >> la(0, 0);
        laIndSizeNormal++;
      }

      if (fdf)
        if (fdf->isSetValued())
          laT >> la(laIndSizeNormal, laSize - 1);
    }
  }

  void SingleContact::updategdRef(const Vec& gdParent) {
    LinkMechanics::updategdRef(gdParent);
    if (gdSize) {
      int gdIndSizeNormal = 0;
      if (fcl->isSetValued()) {
        gdN >> gd(0, 0);
        gdIndSizeNormal++;
      }
      if (fdf)
        if (fdf->isSetValued())
          gdT >> gd(gdIndSizeNormal, gdSize - 1);
    }
  }

  void SingleContact::calcxSize() {
    LinkMechanics::calcxSize();
    xSize = 0;
  }

  void SingleContact::calclaSize(int j) {
    LinkMechanics::calclaSize(j);
    if (j == 0) { // IA
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf)
        if (fdf->isSetValued())
          laSize += getFrictionDirections();

    }
    else if (j == 1) { // IG
      //Add 1 to lambda size if normal force law is setValued
      if (fcl->isSetValued())
        laSize = 1;
      else
        laSize = 0;

      //Add number of friction directions to lambda size if friction force law is setValued
      if (fdf)
        if (fdf->isSetValued())
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
      if (fdf)
        if (fdf->isSetValued())
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
      if (fdf)
        if (fdf->isSetValued())
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
      throw;
  }

  void SingleContact::calcgSize(int j) {
    LinkMechanics::calcgSize(j);
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
      throw;
  }

  void SingleContact::calcgdSize(int j) {
    LinkMechanics::calcgdSize(j);
    if (j == 0) { // IA
      //Add 1 to gd size if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      //Add number of friction directions to gd size if friction force law is setValued
      if (fdf)
        if (fdf->isSetValued())
          gdSize += getFrictionDirections();

    }
    else if (j == 1) { // IG
      //Add 1 to gd size if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      //Add number of friction directions to gd size if friction force law is setValued
      if (fdf)
        if (fdf->isSetValued())
          gdSize += getFrictionDirections();

      gdSize *= gActive;

    }
    else if (j == 2) { // IB
      //Add 1 to gd size if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      //Add number of friction directions to gd size if friction force law is setValued
      if (fdf)
        if (fdf->isSetValued())
          gdSize += getFrictionDirections();

      gdSize *= gActive * gdActive[0];

    }
    else if (j == 3) { // IH
      //Add 1 to gd size if normal force law is setValued
      if (fcl->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;

      //Add number of friction directions to gd size if friction force law is setValued
      if (fdf)
        if (fdf->isSetValued())
          gdSize += gdActive[1] * getFrictionDirections();

      gdSize *= gActive * gdActive[0];

    }
    else
      throw;
  }

  void SingleContact::calcrFactorSize(int j) {
    LinkMechanics::calcrFactorSize(j);
    if (j == 0) { // IA
      rFactorSize = 1 + min(getFrictionDirections(), 1);
    }
    else if (j == 1) { // IG
      rFactorSize = gActive * (1 + min(getFrictionDirections(), 1));
    }
    else if (j == 2) { // IB
      rFactorSize = gActive * gdActive[0] * (1 + min(getFrictionDirections(), 1));
    }
    else if (j == 3) { // IB
      rFactorSize = gActive * gdActive[0] * (1 + gdActive[1] * min(getFrictionDirections(), 1));
    }
  }

  void SingleContact::calcsvSize() {
    LinkMechanics::calcsvSize();

    //Add length due to normal direction
    svSize = fcl->isSetValued() ? 1 : 0;

    //Add length due to tangentinal direction
    if (fdf)
      svSize += fdf->isSetValued() ? min(getFrictionDirections(), 1) : 0;
  }

  void SingleContact::calcLinkStatusSize() {
    LinkMechanics::calcLinkStatusSize();
    //assert(contactKinematics->getNumberOfPotentialContactPoints() == 1); //not necessary anymore as SingleContact has only one contact point
    LinkStatusSize = 1;
    LinkStatus.resize(LinkStatusSize);
  }

  void SingleContact::calcLinkStatusRegSize() {
    LinkMechanics::calcLinkStatusRegSize();
    //assert(contactKinematics->getNumberOfPotentialContactPoints() == 1); //not necessary anymore as SingleContact has only one contact point
    LinkStatusRegSize = 1;
    LinkStatusReg.resize(LinkStatusRegSize);
  }

  void SingleContact::init(InitStage stage) {
    if (stage == resolveXMLPath) {
      if (saved_ref1 != "" && saved_ref2 != "")
        connect(getByPath<Contour>(saved_ref1), getByPath<Contour>(saved_ref2));
      LinkMechanics::init(stage);
    }
    else if (stage == resize) {
      LinkMechanics::init(stage);

      //TODO: Change this if la should be the vector of nonsmooth forces
      la.resize(1 + getFrictionDirections());
      g.resize(1);
      gd.resize(1 + getFrictionDirections());
      gddN.resize(1);
      gddT.resize(getFrictionDirections());
      gdnN.resize(1);
      gdnT.resize(getFrictionDirections());

      // clear container first, because InitStage resize is called twice (before and after the reorganization)
      if (cpData)
        delete[] cpData;

      if (getFrictionDirections() == 0)
        gdActive[1] = false;

      cpData = new ContourPointData[2];

      cpData[0].getFrameOfReference().setName("0");
      cpData[1].getFrameOfReference().setName("1");

      cpData[0].getFrameOfReference().sethSize(contour[0]->gethSize(0), 0);
      cpData[0].getFrameOfReference().sethSize(contour[0]->gethSize(1), 1);
      cpData[1].getFrameOfReference().sethSize(contour[1]->gethSize(0), 0);
      cpData[1].getFrameOfReference().sethSize(contour[1]->gethSize(1), 1);

      cpData[0].getFrameOfReference().init(stage);
      cpData[1].getFrameOfReference().init(stage);
    }
    else if (stage == unknownStage) {
      LinkMechanics::init(stage);

      if(fcl->isSetValued())
        iT = Index(1, getFrictionDirections());
      else
        iT = Index(0, getFrictionDirections() - 1);

      //TODO: check if indices are set correctly?
      laN.resize() >> la(0, 0);
      laT.resize() >> la(1, getFrictionDirections());

      gdN.resize() >> gd(0, 0);
      gdT.resize() >> gd(1, getFrictionDirections());

      gddNBuf.resize(1);
      gddTBuf.resize(getFrictionDirections());
    }
    else if (stage == preInit) {
      LinkMechanics::init(stage);

      /*Set the sizes of the different vectors*/
      contactKinematics->assignContours(contour[0], contour[1]);

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

    }
    else if (stage == MBSim::plot) {
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || contactArrow || frictionArrow)) {
          openMBVContactGrp = new OpenMBV::Group();
          openMBVContactGrp->setName(name + "_ContactGroup");
          openMBVContactGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVContactGrp);

          if (openMBVContactFrameSize > epsroot()) {
            for (unsigned int i = 0; i < 2; i++) { // frames
              openMBVContactFrame.push_back(new OpenMBV::Frame);
              openMBVContactFrame[i]->setOffset(1.);
              openMBVContactFrame[i]->setSize(openMBVContactFrameSize);
              openMBVContactFrame[i]->setName(string("ContactPoint_") + (i == 0 ? "A" : "B"));
              openMBVContactFrame[i]->setEnable(openMBVContactFrameEnabled);
              openMBVContactGrp->addObject(openMBVContactFrame[i]);
            }
          }
          // arrows
          if (contactArrow) {
            openMBVNormalForceArrow = new OpenMBV::Arrow(*contactArrow);
            openMBVNormalForceArrow->setName("NormalForce_B");
            openMBVContactGrp->addObject(openMBVNormalForceArrow);
          }
          if (frictionArrow && getFrictionDirections() > 0) { // friction force
            openMBVFrictionArrow = new OpenMBV::Arrow(*frictionArrow);
            openMBVFrictionArrow->setName("FrictionForce_B");
            openMBVContactGrp->addObject(openMBVFrictionArrow);
          }
        }
#endif
        LinkMechanics::init(stage);
//        if (getPlotFeature(linkKinematics) == enabled) {
//          plotColumns.push_back("g[" + numtostr(i) + "](" + numtostr(0) + ")");
//          for (int j = 0; j < 1 + getFrictionDirections(); ++j)
//            plotColumns.push_back("gd[" + numtostr(i) + "](" + numtostr(j) + ")");
//        }
//        if (getPlotFeature(generalizedLinkForce) == enabled) {
//          for (int j = 0; j < 1 + getFrictionDirections(); ++j)
//            plotColumns.push_back("la[" + numtostr(i) + "](" + numtostr(j) + ")");
//        }
//        PlotFeatureStatus pfKinematics = getPlotFeature(linkKinematics);
//        PlotFeatureStatus pfKinetics = getPlotFeature(generalizedLinkForce);
//        setPlotFeature(linkKinematics, disabled);
//        setPlotFeature(generalizedLinkForce, disabled);
//        LinkMechanics::init(stage);
//        setPlotFeature(linkKinematics, pfKinematics);
//        setPlotFeature(generalizedLinkForce, pfKinetics);
      }

    }
    else if(stage == LASTINITSTAGE) {
      if(contactKinematics->getNumberOfPotentialContactPoints() > 1)
        throw new MBSimError("ERROR: Contact has contact kinematics with more than one possible contact point. Use Multi-Contact for that!");
    }
    else {
      LinkMechanics::init(stage);
    }
  }

  bool SingleContact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if (fdf)
      flag |= fdf->isSetValued();
    return flag;
  }

  bool SingleContact::isSingleValued() const {
    if (fcl->isSetValued()) {
      if (fdf) {
        return not fdf->isSetValued();
      }
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

  void SingleContact::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || contactArrow || frictionArrow)) {
        // frames
        if (openMBVContactFrameSize > epsroot()) {
          for (unsigned int i = 0; i < 2; i++) {
            vector<double> data;
            data.push_back(i);
            data.push_back(cpData[i].getFrameOfReference().getPosition()(0));
            data.push_back(cpData[i].getFrameOfReference().getPosition()(1));
            data.push_back(cpData[i].getFrameOfReference().getPosition()(2));
            Vec3 cardan = AIK2Cardan(cpData[i].getFrameOfReference().getOrientation());
            data.push_back(cardan(0));
            data.push_back(cardan(1));
            data.push_back(cardan(2));
            data.push_back(0);
            openMBVContactFrame[i]->append(data);
          }
        }
        // arrows
        // normal force
        vector<double> data;
        if (contactArrow) {
          data.push_back(t);
          data.push_back(cpData[1].getFrameOfReference().getPosition()(0));
          data.push_back(cpData[1].getFrameOfReference().getPosition()(1));
          data.push_back(cpData[1].getFrameOfReference().getPosition()(2));
          Vec3 F;
          if (fcl->isSetValued()) {
            if (gActive)
//              F = fF[1].col(0) * laN / dt;
              F = cpData[0].getFrameOfReference().getOrientation().col(0) * laN / dt;
          }
          else
            F = cpData[0].getFrameOfReference().getOrientation().col(0) * laN;
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back(nrm2(F));
          openMBVNormalForceArrow->append(data);
        }
        if (frictionArrow && getFrictionDirections() > 0) { // friction force
          data.clear();
          data.push_back(t);
          data.push_back(cpData[1].getFrameOfReference().getPosition()(0));
          data.push_back(cpData[1].getFrameOfReference().getPosition()(1));
          data.push_back(cpData[1].getFrameOfReference().getPosition()(2));
          Vec3 F;
          if (fdf->isSetValued()) {                    // TODO switch between stick and slip not possible with TimeStepper
            if (gActive && laT.size()) { // stick friction
              F = cpData[0].getFrameOfReference().getOrientation().col(1) * laT(0) / dt;
              if (getFrictionDirections() > 1)
                F += cpData[0].getFrameOfReference().getOrientation().col(2) * laT(1) / dt;
            }
            if (gActive && laT.size() == 0) // slip friction
              F = fF[1](Index(0, 2), iT) * fdf->dlaTdlaN(gdT, laN(0)) * laN(0) / dt;
          }
          else {
            F = cpData[0].getFrameOfReference().getOrientation().col(1) * laT(0);
            if (getFrictionDirections() > 1)
              F += cpData[0].getFrameOfReference().getOrientation().col(2) * laT(1);
          }
          data.push_back(F(0));
          data.push_back(F(1));
          data.push_back(F(2));
          data.push_back((fdf->isSetValued() && laT.size()) ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
          openMBVFrictionArrow->append(data);
        }
      }
#endif
      if (getPlotFeature(linkKinematics) == enabled) {
        bool flag = fcl->isSetValued();
        plotVector.push_back(g(0)); //gN
        if ((flag && gActive) || (!flag && fcl->isActive(g(0), 0))) {
          plotVector.push_back(gdN(0)); //gd-Normal
          for (int j = 0; j < getFrictionDirections(); j++)
            plotVector.push_back(gdT(j)); //gd-Tangential
        }
        else {
          for (int j = 0; j < 1 + getFrictionDirections(); j++)
            plotVector.push_back(NAN); //gd
        }
      }
      if (getPlotFeature(generalizedLinkForce) == enabled) {
        if (gActive && gdActive[0]) {
          plotVector.push_back(laN(0) / (fcl->isSetValued() ? dt : 1.));
          if (gdActive[1]) {
            for (int j = 0; j < getFrictionDirections(); j++)
              plotVector.push_back(laT(j) / (fdf->isSetValued() ? dt : 1.));
          }
          else {
            if (fdf) {
              Vec buf = fdf->dlaTdlaN(gdT, laN(0)) * laN(0);
              for (int j = 0; j < getFrictionDirections(); j++)
                plotVector.push_back(buf(j) / (fdf->isSetValued() ? dt : 1.));
            }
          }
        }
        else {
          for (int j = 0; j < 1 + getFrictionDirections(); j++)
            plotVector.push_back(0);
        }
      }
      PlotFeatureStatus pfKinematics = getPlotFeature(linkKinematics);
      PlotFeatureStatus pfKinetics = getPlotFeature(generalizedLinkForce);
      setPlotFeature(linkKinematics, disabled);
      setPlotFeature(generalizedLinkForce, disabled);
      LinkMechanics::plot(t, dt);
      setPlotFeature(linkKinematics, pfKinematics);
      setPlotFeature(generalizedLinkForce, pfKinetics);
    }
  }

  void SingleContact::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      LinkMechanics::closePlot();
    }
  }

  void SingleContact::solveImpactsFixpointSingle(double dt) {
    if (gActive) {
      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

      int addIndexNormal = 0;
      double scaleFactorN = dt;
      if (fcl->isSetValued()) {
        scaleFactorN = 1;
        addIndexNormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * laMBS(ja[j]);

        laN(0) = fnil->project(laN(0), gdnN(0), gdN(0), rFactor(0));
      }

      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + addIndexNormal + i);
          for (int j = ia[laInd + i + addIndexNormal]; j < ia[laInd + 1 + i + addIndexNormal]; j++)
            gdnT(i) += a[j] * laMBS(ja[j]);
        }

        //            if (ftil) //There must be a ftil coming with a setValued fdf
        laT = ftil->project(laT, gdnT, gdT, laN(0) * scaleFactorN, rFactor(1));
      }
    }
  }

  void SingleContact::solveConstraintsFixpointSingle() {
    if (gdActive[0]) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gddN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gddN(0) += a[j] * laMBS(ja[j]);

        laN(0) = fcl->project(laN(0), gddN(0), rFactor(0));
      }

      if (fdf) {
        if (fdf->isSetValued() and gdActive[1]) {
          for (int i = 0; i < getFrictionDirections(); i++) {
            gddT(i) = b(laInd + i + addIndexnormal);
            for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
              gddT(i) += a[j] * laMBS(ja[j]);
          }

          laT = fdf->project(laT, gddT, laN(0), rFactor(1));
        }
      }
    }
  }

  void SingleContact::solveImpactsGaussSeidel(double dt) {
    assert(getFrictionDirections() <= 1);
    if (gActive) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

      //TODO: check indices (in other solution algorithms too!)
      const double om = 1.0;
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * laMBS(ja[j]);

        const double buf = fnil->solve(a[ia[laInd]], gdnN(0), gdN(0));
        laN(0) += om * (buf - laN(0));
      }

      if (ftil) {
        gdnT(0) = b(laInd + addIndexnormal);
        for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
          gdnT(0) += a[j] * laMBS(ja[j]);

        Vec buf = ftil->solve(ds->getG()(Index(laInd + addIndexnormal, laInd + getFrictionDirections())), gdnT, gdT, laN(0));
        laT += om * (buf - laT);
      }
    }
  }

  void SingleContact::solveConstraintsGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    if (gdActive[0]) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

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

      if (fdf) {
        if (fdf->isSetValued() && gdActive[1]) {
          gddT(0) = b(laInd + addIndexNormal);
          for (int j = ia[laInd + addIndexNormal] + 1; j < ia[laInd + addIndexNormal + 1]; j++)
            gddT(0) += a[j] * laMBS(ja[j]);

          Vec buf = fdf->solve(ds->getG()(Index(laInd + addIndexNormal, laInd + addIndexNormal + getFrictionDirections() - 1)), gddT, laN(0));
          laT += om * (buf - laT);
        }
      }
    }
  }

  void SingleContact::solveImpactsRootFinding(double dt) {
    if (gActive) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

      //compute residuum for normal direction
      int addIndexnormal = 0;
      if (fcl->isSetValued()) {
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * laMBS(ja[j]);

        res(0) = laN(0) - fnil->project(laN(0), gdnN(0), gdN(0), rFactor(0));
      }

      //compute residuum for tangential directions
      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gdnT(i) += a[j] * laMBS(ja[j]);
        }
        //            if (ftil) There must be a frictional impact law if fdf is set valued!
        res(addIndexnormal, addIndexnormal + getFrictionDirections() - 1) = laT - ftil->project(laT, gdnT, gdT, laN(0), rFactor(1));
      }
    }
  }

  void SingleContact::solveConstraintsRootFinding() {
    if (gdActive[0]) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

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
      if (fdf) {
        if (fdf->isSetValued()) {
          for (int i = 0; i < getFrictionDirections(); i++) {
            gdnT(i) = b(laInd + i + addIndexnormal);
            for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
              gdnT(i) += a[j] * laMBS(ja[j]);
          }
          //            if (ftil) There must be a frictional impact law if fdf is set valued!
          res(addIndexnormal, addIndexnormal + getFrictionDirections() - 1) = laT - fdf->project(laT, gddT, laN(0), rFactor(1));
        }
      }
    }
  }

  void SingleContact::jacobianConstraints() {
    if (gdActive[0]) {

      const SqrMat Jprox = ds->getJprox();
      const SqrMat G = ds->getG();

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
        Mat diff = fdf->diff(laT, gddT(0, 0), laN(0), rFactor(1));
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
        Mat diff = ftil->diff(laT, gddT, gdT, laN(0), rFactor(1));
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

  void SingleContact::jacobianImpacts() {
    if (gActive) {

      const SqrMat Jprox = ds->getJprox();
      const SqrMat G = ds->getG();

      RowVec jp1 = Jprox.row(laInd);
      RowVec e1(jp1.size());
      e1(laInd) = 1;

      int addIndexNormal = 0;
      if (fcl->isSetValued()) {
    	  addIndexNormal++;

    	  Vec diff = fnil->diff(laN(0), gdnN(0), gdN(0), rFactor(0));

    	  jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk)
    	  for (int i = 0; i < G.size(); i++)
    		  jp1(i) -= diff(1) * G(laInd, i);
      }

      if (getFrictionDirections() == 1) {
        Mat diff = ftil->diff(laT, gdnT, gdT, laN(0), rFactor(1));
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
        Mat diff = ftil->diff(laT, gdnT, gdT, laN(0), rFactor(1));
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

  void SingleContact::updaterFactors() {
    if (gdActive[0]) {

      const double *a = ds->getGs()();
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

      if (fdf && gdActive[1]) {
        if (fdf->isSetValued()) {
          double sumT1 = 0;
          double sumT2 = 0;
          double aT1, aT2;
          if (getFrictionDirections() == 1) {
            for (int j = ia[laInd + addIndexnormal] + 1; j < ia[laInd + addIndexnormal + 1]; j++)
              sumT1 += fabs(a[j]);
            aT1 = a[ia[laInd + addIndexnormal]];
            if (aT1 > sumT1) {
              rFactorUnsure(1) = 0;
              rFactor(1) = 1.0 / aT1;
            }
            else {
              rFactorUnsure(1) = 1;
              rFactor(1) = rMax / aT1;
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
                rFactor(1) = 2.0 / (aT1 + aT2 + sumT1 - sumT2);
              else
                rFactor(1) = 1.0 / aT2;
            else if (aT1 + sumT1 < aT2 + sumT2)
              rFactor(1) = 2.0 / (aT1 + aT2 - sumT1 + sumT2);
            else
              rFactor(1) = 1.0 / aT1;
          }
        }
      }
    }
  }

  void SingleContact::checkConstraintsForTermination() {
    if (gdActive[0]) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

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

        if (!fdf->isFulfilled(laT, gddT, laN(0), laTol, gddTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void SingleContact::checkImpactsForTermination(double dt) {
    if (gActive) {

      const double *a = ds->getGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla();
      const Vec &b = ds->getb();

      int addIndexnormal = 0;
      double scaleFactorN = dt;
      if (fcl->isSetValued()) {
        scaleFactorN = 1.;
        addIndexnormal++;
        gdnN(0) = b(laInd);
        for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
          gdnN(0) += a[j] * laMBS(ja[j]);
        if (!fnil->isFulfilled(laN(0), gdnN(0), gdN(0), LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }

      if (ftil) {
        for (int i = 0; i < getFrictionDirections(); i++) {
          gdnT(i) = b(laInd + i + addIndexnormal);
          for (int j = ia[laInd + i + addIndexnormal]; j < ia[laInd + 1 + i + addIndexnormal]; j++)
            gdnT(i) += a[j] * laMBS(ja[j]);
        }
        if (!ftil->isFulfilled(laT, gdnT, gdT, laN(0) * scaleFactorN, LaTol, gdTol)) {
          ds->setTermination(false);
          return;
        }
      }
    }
  }

  void SingleContact::checkActive(int j) {
    if (j == 1) { // formerly checkActiveg()
      gActive = fcl->isActive(g(0), gTol) ? 1 : 0;
      gdActive[0] = gActive;
      gdActive[1] = gdActive[0];
    }
    else if (j == 2) { // formerly checkActivegd()
      gdActive[0] = gActive ? (fcl->remainsActive(gdN(0), gdTol) ? 1 : 0) : 0;
      gdActive[1] = getFrictionDirections() && gdActive[0] ? (fdf->isSticking(gdT, gdTol) ? 1 : 0) : 0;
      gddActive[0] = gdActive[0];
      gddActive[1] = gdActive[1];
    }
    else if (j == 3) { // formerly checkActivegdn()
      if (gActive) { // Contact was closed
        if (gdnN(0) <= gdTol) { // Contact stays closed // TODO bilateral contact
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
        else { // Contact will open
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
          if (gddN(0) <= gddTol) { // Contact stays closed
            gddActive[0] = true;
            if (getFrictionDirections()) {
              if (gdActive[1]) {
                if (nrm2(gddT) <= gddTol)
                  gddActive[1] = true;
                else {
                  gddActive[1] = false;
                }
              }
            }
          }
          else { // Contact will open
            gddActive[0] = false;
            gddActive[1] = false;
          }
        }
      }
    }
    else if (j == 5) {
      if (gActive) {
        if (gdActive[0]) {
          if (gdActive[1]) {
            if (!gddActive[1]) {
              gdActive[1] = false;
            }
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
    else if (j == 6) { // nur nach schließenden Kontakten schauen
      if (rootID == 3) {
        gActive = true;
        gdActive[0] = true;
        gdActive[1] = true;
        gddActive[0] = true;
        gddActive[1] = true;
      }
    }
    else if (j == 7) { // nur nach Gleit-Haft-Übergängen schauen
      if (getFrictionDirections()) {
        if (rootID == 2) {
          gdActive[1] = true;
          gddActive[1] = true;
        }
      }
    }
    else if (j == 8) { // nur nach öffnenden Kontakten und Haft-Gleit-Übergängen schauen
      if (jsv(0) && rootID == 1) { // Kontakt öffnet
        gddActive[0] = false;
        gddActive[1] = false;
      }
      if (getFrictionDirections()) {
        if (jsv(1) && rootID == 1) { // Haft-Gleitübergang
          gddActive[1] = false;
        }
      }
    }
    else
      throw;
  }

  int SingleContact::getFrictionDirections() {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void SingleContact::connect(Contour *contour0, Contour* contour1, ContactKinematics* contactKinematics_ /*=0*/) {
    LinkMechanics::connect(contour0);
    LinkMechanics::connect(contour1);
    contactKinematics = contactKinematics_;

    if (contactKinematics == 0)
      contactKinematics = contour0->findContactPairingWith(contour0->getType(), contour1->getType());
    if (contactKinematics == 0)
      contactKinematics = contour1->findContactPairingWith(contour1->getType(), contour0->getType());
    if (contactKinematics == 0)
      contactKinematics = contour0->findContactPairingWith(contour1->getType(), contour0->getType());
    if (contactKinematics == 0)
      contactKinematics = contour1->findContactPairingWith(contour0->getType(), contour1->getType());
    if (contactKinematics == 0)
      throw MBSimError("ERROR in " + getName() + " (Contact::init): Unknown contact pairing between Contour \"" + contour0->getType() + "\" and Contour\"" + contour1->getType() + "\"!");

  }

  void SingleContact::applyh(int t, int j) {
    WF[1] = cpData[0].getFrameOfReference().getOrientation().col(0) * laN(0);

    if (fdf)
      if (not fdf->isSetValued()) {
        laT = (*fdf)(gdT, fabs(laN(0)));
        WF[1] += cpData[0].getFrameOfReference().getOrientation().col(1) * laT(0);
        if (getFrictionDirections() > 1)
          WF[1] += cpData[0].getFrameOfReference().getOrientation().col(2) * laT(1);
      }

    WF[0] = -WF[1];
    for (unsigned int i = 0; i < 2; i++) { //TODO only two contours are interacting at one time?
      h[j][i] += cpData[i].getFrameOfReference().getJacobianOfTranslation(j).T() * WF[i];
    }
  }

  void SingleContact::computeCurvatures(Vec & r) const {
    contactKinematics->computeCurvatures(r, cpData);
  }

  void SingleContact::LinearImpactEstimation(Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
    if (gActive) {
      gAct_(*IndActive_) = g(0);
      (*IndActive_)++;
    }
    else {
      for (unsigned int i = 0; i < 2; i++)
        contour[i]->updateKinematicsForFrame(cpData[i], velocities);
      Vec3 Wn = cpData[0].getFrameOfReference().getOrientation().col(0);
      Vec3 WvD = cpData[1].getFrameOfReference().getVelocity() - cpData[0].getFrameOfReference().getVelocity();
      gdInActive_(*IndInActive_) = Wn.T() * WvD;
      gInActive_(*IndInActive_) = g(0);
      (*IndInActive_)++;
    }
  }

  void SingleContact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
    if (gActive)
      (*sizeActive_)++;
    else
      (*sizeInActive_)++;
  }

  void SingleContact::initializeUsingXML(TiXmlElement *element) {
    LinkMechanics::initializeUsingXML(element);
    TiXmlElement *e;

    //Set contact law
    e = element->FirstChildElement(MBSIMNS"contactForceLaw");
    GeneralizedForceLaw *gfl = ObjectFactory<GeneralizedForceLaw>::create(e->FirstChildElement());
    setContactForceLaw(gfl);
    gfl->initializeUsingXML(e->FirstChildElement());

    //Set impact law (if given)
    e = e->NextSiblingElement();
    GeneralizedImpactLaw *gifl = ObjectFactory<GeneralizedImpactLaw>::create(e->FirstChildElement());
    if (gifl) {
      setContactImpactLaw(gifl);
      gifl->initializeUsingXML(e->FirstChildElement());
    }

    //Set friction law (if given)
    e = e->NextSiblingElement();
    FrictionForceLaw *ffl = ObjectFactory<FrictionForceLaw>::create(e->FirstChildElement());
    if (ffl) {
      setFrictionForceLaw(ffl);
      ffl->initializeUsingXML(e->FirstChildElement());
    }

    //Set friction impact law (if given)
    e = e->NextSiblingElement();
    FrictionImpactLaw *fil = ObjectFactory<FrictionImpactLaw>::create(e->FirstChildElement());
    if (fil) {
      setFrictionImpactLaw(fil);
      fil->initializeUsingXML(e->FirstChildElement());
    }

    //Save contour names for initialization
    e = element->FirstChildElement(MBSIMNS"connect");
    saved_ref1 = e->Attribute("ref1");
    saved_ref2 = e->Attribute("ref2");

#ifdef HAVE_OPENMBVCPPINTERFACE
    //test what should be drawn to OpenMBV

    //Contact points
    if (element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints"))
      enableOpenMBVContactPoints(getDouble(element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints")));

    //Normal force
    e = element->FirstChildElement(MBSIMNS"openMBVNormalForceArrow");
    if (e) {
      OpenMBV::Arrow *arrow = dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVNormalForceArrow(arrow);
      e = e->NextSiblingElement();
    }

    //Friction force
    e = element->FirstChildElement(MBSIMNS"openMBVFrictionArrow");
    if (e) {
      OpenMBV::Arrow *arrow = dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVFrictionArrow(arrow);
      e = e->NextSiblingElement();
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
      throw;
  }

  void SingleContact::calccorrSize(int j) {
    LinkMechanics::calccorrSize(j);
    if (j == 1) { // IG
      corrSize += gActive;
    }
    else if (j == 2) { // IB
      corrSize += gActive * gdActive[0];
    }
    //    else if(j==3) { // IG
    //      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
    //        corrIndk = corrSize;
    //        corrSizek = gActive[i]*(1+getFrictionDirections());
    //        corrSize += corrSizek;
    //      }
    //    }
    else if (j == 4) { // IH
      corrSize += gActive * gdActive[0] * (1 + gdActive[1] * getFrictionDirections());
    }
    else
      throw;
  }

  void SingleContact::checkRoot() {
    rootID = 0;
    if (jsv(0)) {
      if (gActive)
        rootID = 1; // Contact was closed -> opening
      else
        rootID = 3; // Contact was open -> impact
    }
    if (getFrictionDirections()) {
      if (jsv(1)) {
        if (gdActive[1])
          rootID = 1; // Contact was sticking -> sliding
        else {
          if (getFrictionDirections() == 1)
            rootID = 2; // Contact was sliding -> sticking
          else if (nrm2(gdT) <= gdTol)
            rootID = 2; // Contact was sliding -> sticking
        }
      }
    }
    ds->setRootID(max(ds->getRootID(), rootID));
  }

}

