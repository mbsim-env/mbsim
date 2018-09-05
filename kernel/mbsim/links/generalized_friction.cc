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
#include "mbsim/links/generalized_friction.h"
#include "mbsim/objectfactory.h"
#include "mbsim/objects/rigid_body.h"
#include <mbsim/constitutive_laws/friction_force_law.h>
#include <mbsim/dynamic_system_solver.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedFriction)

  GeneralizedFriction::~GeneralizedFriction() {
    delete func;
    delete laN;
  }

  bool GeneralizedFriction::isSetValued() const {
    return func->isSetValued();
  }

  void GeneralizedFriction::updateGeneralizedForces() {
    if(isSetValued()) {
      if(gdActive)
        lambda = la;
      else {
        Vec gd = evalGeneralizedRelativeVelocity();
        lambda = func->dlaTdlaN(gd)*(*laN)(getTime());
        if(gd(0)*gdDir<0) lambda*=-1.0;
      }
    }
    else
      lambda = (*func)(evalGeneralizedRelativeVelocity(),(*laN)(getTime()));
    updla = false;
  }

  void GeneralizedFriction::updateh(int j) {
    if(not(isSetValued() and gdActive))
      DualRigidBodyLink::updateh(j);
  }

  void GeneralizedFriction::updateW(int j) {
    if(isSetValued() and gdActive)
      DualRigidBodyLink::updateW(j);
  }

  const double& GeneralizedFriction::evalgdn() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdn(0);
  }

  const double& GeneralizedFriction::evalgdd() {
    if(ds->getUpdatela()) ds->updatela();
    return gdd(0);
  }

  void GeneralizedFriction::init(InitStage stage, const InitConfigSet &config) {
    if(stage==unknownStage) {
      if(body[0]->getGeneralizedVelocitySize()!=1)
        throwError("rigid bodies must have 1 dof!");
    }
    DualRigidBodyLink::init(stage, config);
    func->init(stage, config);
    laN->init(stage, config);
  }

  void GeneralizedFriction::setGeneralizedFrictionForceLaw(FrictionForceLaw *func_) { 
    func = func_; 
    func->setParent(this);
  }

  void GeneralizedFriction::updateStopVector() {
    if (gdActive)
      sv(0) = fabs(evalgdd()) - gddTol;
    else
      sv(0) = evalGeneralizedRelativeVelocity()(0);
  }

  void GeneralizedFriction::calclaSize(int j) {
    DualRigidBodyLink::calclaSize(j);
    if (j <= 2) { // IA
      if (func->isSetValued())
        laSize = 1;
      else
        laSize = 0;
    }
    else if (j == 3) { // IB
      if (func->isSetValued() and gdActive)
        laSize = 1;
      else
        laSize = 0;
    }
    else if (j <= 5) { // IG
      if (func->isSetValued())
        laSize = 1;
      else
        laSize = 0;
    }
    else
      throwError("Internal error");
  }

  void GeneralizedFriction::calcgSize(int j) {
    DualRigidBodyLink::calcgSize(j);
    gSize = 0;
  }

  void GeneralizedFriction::calcgdSize(int j) {
    DualRigidBodyLink::calcgdSize(j);
    if (j <= 2) { // all contacts
      if (func->isSetValued())
        gdSize = 1;
      else
        gdSize = 0;
    }
    else if (j == 3) { // sticking contacts
      if (func->isSetValued() and gdActive)
        gdSize = 1;
      else
        gdSize = 0;
    }
    else
      throwError("Internal error");
  }

  void GeneralizedFriction::calcrFactorSize(int j) {
    DualRigidBodyLink::calcrFactorSize(j);
    if (j <= 2) // IA
      rFactorSize = 1;
    else if (j == 3) // IB
      rFactorSize = gdActive;
  }

  void GeneralizedFriction::calcsvSize() {
    DualRigidBodyLink::calcsvSize();
    svSize = func->isSetValued() ? 1 : 0;
  }

  void GeneralizedFriction::checkActive(int j) {
    if (j == 1)
      gdActive = 1;
    else if (j == 2) {
      Vec gd = evalGeneralizedRelativeVelocity();
      gdActive = func->isSticking(gd, gdTol) ? 1 : 0;
      gddActive = gdActive;
      if(not gdActive)
        gdDir = gd(0)>0?1:-1;
    }
    else if (j == 3) {
      if (fabs(evalgdn()) <= gdTol) {
        gdActive = true;
        gddActive = true;
      }
      else {
        gdActive = false;
        gddActive = false;
        gdDir = gdn(0)>0?1:-1;
      }
    }
    else if (j == 4) {
      if (gdActive) {
        if (fabs(evalgdd()) <= gddTol)
          gddActive = true;
        else {
          gddActive = false;
          gdDir = gdd(0)>0?1:-1;
        }
      }
    }
    else if (j == 5) {
      if (gdActive) {
        if (!gddActive)
          gdActive = false;
      }
    }
//    else if (j == 6) {
//      if (rootID == 3) {
//        gdActive = true;
//        gddActive = true;
//      }
//    }
    else if (j == 7) {
      if (rootID == 2) {
        gdActive = true;
        gddActive = true;
      }
    }
    else if (j == 8) {
      if (jsv(0) && rootID == 1) { // stick-slip transition
        gddActive = false;
        gdDir = gdd(0)>0?1:-1;
      }
    }
    else
      throwError("Internal error");
  }

  void GeneralizedFriction::updatecorr(int j) {
    if (j <= 2) { // IG position
    }
    else if (j == 4) {
      if (gdActive) { // Contact was sticking
        if (gddActive) {
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

  void GeneralizedFriction::calccorrSize(int j) {
    DualRigidBodyLink::calccorrSize(j);
    if (j <= 2) { // IG
      corrSize = 0;
    }
    else if (j == 4) { // IH
      corrSize = gdActive;
    }
    else
      throwError("Internal error");
  }

  void GeneralizedFriction::checkRoot() {
    rootID = 0;
    if (jsv(0)) {
      if (gdActive)
        rootID = 1; // contact was sticking -> sliding
      else
        rootID = 2; // contact was sliding -> sticking
    }
    ds->setRootID(max(ds->getRootID(), rootID));
  }

  void GeneralizedFriction::updaterFactors() {
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

  void GeneralizedFriction::solveConstraintsFixpointSingle() {
    if (gdActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      gdd(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdd(0) += a[j] * laMBS(ja[j]);

      la = func->project(la, gdd, (*laN)(getTime()), rFactor(0));
    }
  }

  void GeneralizedFriction::checkConstraintsForTermination() {
    if (gdActive) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      gdd(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdd(0) += a[j] * laMBS(ja[j]);

      if (!func->isFulfilled(la, gdd, (*laN)(getTime()), laTol, gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void GeneralizedFriction::initializeUsingXML(DOMElement *element) {
    DualRigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedFrictionForceLaw");
    setGeneralizedFrictionForceLaw(ObjectFactory::createAndInit<FrictionForceLaw>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedNormalForceFunction");
    setGeneralizedNormalForceFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
  }

}
