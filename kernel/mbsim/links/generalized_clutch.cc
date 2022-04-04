/* Copyright (C) 2004-2009 MBSim Development Team
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/links/generalized_clutch.h"
#include "mbsim/objectfactory.h"
#include "mbsim/objects/rigid_body.h"
#include <mbsim/constitutive_laws/friction_force_law.h>
#include <mbsim/constitutive_laws/friction_impact_law.h>
#include <mbsim/dynamic_system_solver.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedClutch)

  GeneralizedClutch::~GeneralizedClutch() {
    delete laT;
    delete LaT;
    delete laN;
  }

  bool GeneralizedClutch::isSetValued() const {
    return laT->isSetValued();
  }

  void GeneralizedClutch::updateGeneralizedForces() {
    if(isSetValued()) {
      if(gdActive)
        lambda = evalla();
      else if(gActive) {
        Vec gd = evalGeneralizedRelativeVelocity();
        lambda = laT->dlaTdlaN(gd)*(*laN)(getTime());
        if(gd(0)*gdDir<0) lambda*=-1.0;
      }
      else
	lambda.init(0);
    }
    else
      lambda = (*laT)(evalGeneralizedRelativeVelocity(),(*laN)(getTime()));
    updla = false;
  }

  void GeneralizedClutch::updateh(int j) {
    if(not(isSetValued() and gdActive))
      DualRigidBodyLink::updateh(j);
  }

  void GeneralizedClutch::updateW(int j) {
    if(isSetValued() and gdActive)
      DualRigidBodyLink::updateW(j);
  }

  const double& GeneralizedClutch::evalgdn() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdn(0);
  }

  const double& GeneralizedClutch::evalgdd() {
    if(ds->getUpdatela()) ds->updatela();
    return gdd(0);
  }

  bool GeneralizedClutch::isActive() const {
    return gActive ? true : false;
  }

  bool GeneralizedClutch::gActiveChanged() {
    bool changed = (gActive0 != gActive ? true : false);
    gActive0 = gActive;
    return changed;
  }

  void GeneralizedClutch::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(laT->isSetValued() and not LaT)
        throwError("Friction impact law must be defined!");
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[generalizedRelativePosition]) {
	  if(e)
	    addToPlot("engagement");
	}
	if(plotFeature[generalizedForce])
	  addToPlot("generalizedNormalForce");
      }
    }
    else if(stage==unknownStage) {
      if(body[0]->getGeneralizedVelocitySize()!=1)
        throwError("rigid bodies must have 1 dof!");
    }
    DualRigidBodyLink::init(stage, config);
    laT->init(stage, config);
    if(LaT) LaT->init(stage, config);
    if(e) e->init(stage, config);
    laN->init(stage, config);
  }

  void GeneralizedClutch::plot() {
    if(plotFeature[plotRecursive]) {
      if(plotFeature[generalizedRelativePosition]) {
	if(e)
	  Element::plot((*e)(getTime()));
      }
      if(plotFeature[generalizedForce])
	Element::plot((*laN)(getTime()));
    }
    DualRigidBodyLink::plot();
  }

  void GeneralizedClutch::setGeneralizedFrictionForceLaw(FrictionForceLaw *laT_) {
    laT = laT_;
    laT->setParent(this);
  }

  void GeneralizedClutch::setGeneralizedFrictionImpactLaw(FrictionImpactLaw *LaT_) {
    LaT = LaT_;
    LaT->setParent(this);
  }

  void GeneralizedClutch::updateStopVector() {
    if(e)
      sv(0) = (*e)(getTime());
    if(gActive) {
      if(gdActive)
	sv(e?1:0) = fabs(evalgdd()) - gddTol;
      else
	sv(e?1:0) = evalGeneralizedRelativeVelocity()(0);
    }
    else
      sv(e?1:0) = 1;
  }

  void GeneralizedClutch::calclaSize(int j) {
    DualRigidBodyLink::calclaSize(j);
    if (j <= 2) { // IA
      if (laT->isSetValued())
        laSize = gActive * 1;
      else
        laSize = 0;
    }
    else if (j == 3) { // IB
      if (laT->isSetValued() and gdActive)
        laSize = gActive * 1;
      else
        laSize = 0;
    }
    else if (j <= 5) { // IG
      if (laT->isSetValued())
        laSize = gActive * 1;
      else
        laSize = 0;
    }
    else
      throwError("Internal error");
  }

  void GeneralizedClutch::calcgSize(int j) {
    DualRigidBodyLink::calcgSize(j);
    gSize = 0;
  }

  void GeneralizedClutch::calcgdSize(int j) {
    DualRigidBodyLink::calcgdSize(j);
    if (j <= 2) { // all contacts
      if (laT->isSetValued())
        gdSize = gActive * 1;
      else
        gdSize = 0;
    }
    else if (j == 3) { // sticking contacts
      if (laT->isSetValued() and gdActive)
        gdSize = gActive * 1;
      else
        gdSize = 0;
    }
    else
      throwError("Internal error");
  }

  void GeneralizedClutch::calcrFactorSize(int j) {
    DualRigidBodyLink::calcrFactorSize(j);
    if (j <= 2) // IA
      rFactorSize = gActive * 1;
    else if (j == 3) // IB
      rFactorSize = gActive * gdActive;
  }

  void GeneralizedClutch::calcsvSize() {
    DualRigidBodyLink::calcsvSize();
    svSize = laT->isSetValued() ? (e?2:1) : 0;
  }

  void GeneralizedClutch::checkActive(int j) {
    if (j == 1) {
      if(e)
	gActive = (*e)(getTime())<0 ? 0 : 1;
      gdActive = gActive;
    }
    else if (j == 2) {
      Vec gd = evalGeneralizedRelativeVelocity();
      gdActive = gActive ? (laT->isSticking(gd, gdTol) ? 1 : 0) : 0;
      gddActive = gdActive;
      if(not gdActive)
        gdDir = gd(0)>0?1:-1;
    }
    else if (j == 3) {
      if(gActive) {
	if(fabs(evalgdn()) <= gdTol) {
	  gdActive = true;
	  gddActive = true;
	}
	else {
	  gdActive = false;
	  gddActive = false;
	  gdDir = gdn(0)>0?1:-1;
	}
      }
    }
    else if (j == 4) {
      if(gActive) {
	if(gdActive) {
	  if(fabs(evalgdd()) <= gddTol)
	    gddActive = true;
	  else {
	    gddActive = false;
	    gdDir = gdd(0)>0?1:-1;
	  }
	}
      }
    }
    else if (j == 5) {
      if(gActive) {
	if(gdActive) {
	  if(!gddActive)
	    gdActive = false;
	}
      }
    }
    else if (j == 6) {
     if(gActive and not gdActive) {
        gdActive = true;
        gddActive = true;
      }
    }
    else if (j == 7) {
      if(rootID == 2) {
        gdActive = true;
        gddActive = true;
      }
    }
    else if (j == 8) {
      if(e and jsv(0) and rootID == 1) { // opening or closing contact
	gActive = not gActive;
	Vec gd = evalGeneralizedRelativeVelocity();
	gdActive = gActive ? (laT->isSticking(gd, gdTol) ? 1 : 0) : 0;
	gddActive = gdActive;
	if(gActive and not gddActive)
	  gdDir = gd(0)>0?1:-1;
      }
      if(jsv(e?1:0) && rootID == 1) { // stick-slip transition
        gddActive = false;
        gdDir = gdd(0)>0?1:-1;
      }
    }
    else
      throwError("Internal error");
  }

  void GeneralizedClutch::updatecorr(int j) {
    if(j <= 2) { // IG position
    }
    else if (j == 4) {
      if(gActive and gdActive) { // Contact was sticking
        if(gddActive) {
          corr(0) = 0; // Contact stays sticking, regular projection
        }
        else {
          corr(0) = gdd(0) > 0 ? gdCorr : -gdCorr; // Contact slides, projection to valid tangential velocity
        }
      }
    }
    else
      throwError("Internal error");
  }

  void GeneralizedClutch::calccorrSize(int j) {
    DualRigidBodyLink::calccorrSize(j);
    if (j <= 2) { // IG
      corrSize = 0;
    }
    else if (j == 4) { // IH
      corrSize = gActive * gdActive;
    }
    else
      throwError("Internal error");
  }

  void GeneralizedClutch::checkRoot() {
    rootID = 0;
    if(e and jsv(0))
      rootID = 1; // clutch was closed -> opening
    if(jsv(e?1:0)) {
      if(gdActive)
        rootID = 1; // contact was sticking -> sliding
      else
        rootID = 2; // contact was sliding -> sticking
    }
    ds->setRootID(max(ds->getRootID(), rootID));
  }

  void GeneralizedClutch::updaterFactors() {
    if (gdActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();

      double sumT1 = 0;
      double aT1;
      for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
        sumT1 += fabs(a[j]);
      aT1 = a[ia[laInd]];
      if (aT1 > sumT1) {
        rFactorUnsure(0) = 0;
        rFactor(0) = 1.0 / aT1;
      }
      else {
        rFactorUnsure(0) = 1;
        rFactor(0) = rMax / aT1;
      }
    }
  }

  void GeneralizedClutch::solveConstraintsFixpointSingle() {
    if (gdActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      gdd(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdd(0) += a[j] * laMBS(ja[j]);

      la = laT->project(la, gdd, (*laN)(getTime()), rFactor(0));
    }
  }

  void GeneralizedClutch::solveImpactsFixpointSingle() {
    if (gdActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      gdn(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdn(0) += a[j] * LaMBS(ja[j]);

      La = LaT->project(La, gdn, gd, (*laN)(getTime())*getStepSize(), rFactor(0));
    }
  }

  void GeneralizedClutch::checkConstraintsForTermination() {
    if (gdActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      gdd(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdd(0) += a[j] * laMBS(ja[j]);

      if (!laT->isFulfilled(la, gdd, (*laN)(getTime()), laTol, gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void GeneralizedClutch::checkImpactsForTermination() {
    if (gdActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      gdn(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdn(0) += a[j] * LaMBS(ja[j]);

      if (!LaT->isFulfilled(La, gdn, gd, (*laN)(getTime())*getStepSize(), LaTol, gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void GeneralizedClutch::initializeUsingXML(DOMElement *element) {
    DualRigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedFrictionForceLaw");
    setGeneralizedFrictionForceLaw(ObjectFactory::createAndInit<FrictionForceLaw>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedFrictionImpactLaw");
    if(e) setGeneralizedFrictionImpactLaw(ObjectFactory::createAndInit<FrictionImpactLaw>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"engagementFunction");
    if(e) setEngagementFunction(ObjectFactory::createAndInit<Function<double(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedNormalForceFunction");
    setGeneralizedNormalForceFunction(ObjectFactory::createAndInit<Function<double(double)>>(e->getFirstElementChild()));
  }

}
