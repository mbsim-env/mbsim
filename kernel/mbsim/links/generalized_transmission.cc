/* Copyright (C) 2004-2026 MBSim Development Team
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
#include "mbsim/links/generalized_transmission.h"
#include "mbsim/objectfactory.h"
#include "mbsim/objects/rigid_body.h"
#include <mbsim/dynamic_system_solver.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedTransmission)

  GeneralizedTransmission::~GeneralizedTransmission() {
    delete i;
  }

  void GeneralizedTransmission::updateGeneralizedPositions() {
    ratio[1] = (*i)(getTime());
    DualRigidBodyLink::updateGeneralizedPositions();
  }

  void GeneralizedTransmission::updateGeneralizedVelocities() {
    ratio[1] = (*i)(getTime());
    DualRigidBodyLink::updateGeneralizedVelocities();
  }

  void GeneralizedTransmission::updateForce() {
    ratio[1] = (*i)(getTime());
    DualRigidBodyLink::updateForce();
  }

  void GeneralizedTransmission::updateMoment() {
    ratio[1] = (*i)(getTime());
    DualRigidBodyLink::updateMoment();
  }

  void GeneralizedTransmission::updateR() {
    ratio[1] = (*i)(getTime());
    DualRigidBodyLink::updateR();
  }

  void GeneralizedTransmission::updateGeneralizedForces() {
    if(active)
      lambda = evalla();
    else if(fabs(i0) <= gddTol)
      lambda(0) = 0;
    else {
      Vec gd = evalGeneralizedRelativeVelocity();
      lambda(0) = (gd(0)>0?-1:1)*sForce;
      if(gd(0)*gdDir<0) lambda(0)*=-1.0;
    }
    updla = false;
  }

  void GeneralizedTransmission::updateh(int j) {
    if(not(active)) {
      ratio[1] = (*i)(getTime());
      DualRigidBodyLink::updateh(j);
    }
  }

  void GeneralizedTransmission::updateW(int j) {
    if(laSize) {
      ratio[1] = (*i)(getTime());
      DualRigidBodyLink::updateW(j);
    }
  }

  void GeneralizedTransmission::updatewb() {
    if(wb.size()) {
      ratio[1] = (*i)(getTime());
      DualRigidBodyLink::updatewb();
    }
  }

  void GeneralizedTransmission::updategd() {
    if(gdSize) gd = evalGeneralizedRelativeVelocity();
  }

  const double& GeneralizedTransmission::evalgdn() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdn(0);
  }

  const double& GeneralizedTransmission::evalgdd() {
    if(ds->getUpdatela()) ds->updatela();
    return gdd(0);
  }

  void GeneralizedTransmission::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(body.size() == 1)
	throwError("two rigid bodies must be given!");
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[generalizedRelativeVelocity]) {
	  addToPlot("transmission");
	}
      }
    }
    else if(stage==unknownStage) {
      if(body[0]->getGeneralizedVelocitySize()!=1)
        throwError("rigid bodies must have 1 dof!");
    }
    DualRigidBodyLink::init(stage, config);
    if(i) i->init(stage, config);
  }

  void GeneralizedTransmission::plot() {
    if(plotFeature[plotRecursive]) {
      if(plotFeature[generalizedRelativeVelocity]) {
	Element::plot((*i)(getTime()));
      }
    }
    DualRigidBodyLink::plot();
  }

  void GeneralizedTransmission::updateStopVector() {
    if(active or fabs(i0) <= gddTol)
      sv(0) = (fabs((*i)(getTime())-i0)<=gddTol)?1:-1;
    else
      sv(0) = evalGeneralizedRelativeVelocity()(0);
//    else
 //     sv(0) = 1;
  }

  void GeneralizedTransmission::calclaSize(int j) {
    DualRigidBodyLink::calclaSize(j);
    if (j <= 2) // IA, IG, IB
      laSize = 1;
    else if (j == 3) // IH, IG
      laSize = active;
    else if (j <= 5) // IG, IB
      laSize = 0;
  }

  void GeneralizedTransmission::calcgSize(int j) {
    DualRigidBodyLink::calcgSize(j);
    gSize = 0;
  }

  void GeneralizedTransmission::calcgdSize(int j) {
    DualRigidBodyLink::calcgdSize(j);
    if (j <= 2) //  IA, IG, IB
      gdSize = 1;
    else if (j == 3) // IH, IG
      gdSize = active;
    else if (j <= 5) // IG, IB
      gdSize = 0;
  }

  void GeneralizedTransmission::calcrFactorSize(int j) {
    DualRigidBodyLink::calcrFactorSize(j);
    if (j <= 2) // IA
      rFactorSize = 1;
    else if (j == 3) // IB
      rFactorSize = active;
  }

  void GeneralizedTransmission::calcsvSize() {
    DualRigidBodyLink::calcsvSize();
    svSize = 1;
  }

  void GeneralizedTransmission::checkActive(int j) {
    i0 = (*i)(getTime());
    if (j == 1) {
      Vec gd = evalGeneralizedRelativeVelocity();
      active = (fabs(i0) <= gddTol) ? false : (iSync ? true : ((fabs(gd(0)) <= gdTol) ? 1 : 0));
      if (not active)
	gdDir = gd(0)>0?1:-1;
    }
    else if (j == 6) {
      if (rootID == 3) {
	active = true;
      }
    }
    else if (j == 7) {
      if(rootID == 2)
        active = true;
    }
    else if (j == 8) {
      if (jsv(0) and rootID == 1) {
	active = false;
	Vec gd = evalGeneralizedRelativeVelocity();
	gdDir = gd(0)>0?1:-1;
      }
    }
  }

  void GeneralizedTransmission::updatecorr(int j) {
    if (j == 4) {
      if (active)
	corr(0) = 0;
    }
  }

  void GeneralizedTransmission::calccorrSize(int j) {
    DualRigidBodyLink::calccorrSize(j);
    if (j <= 2)
      corrSize = 0;
    else if (j == 4)
      corrSize = active;
  }

  void GeneralizedTransmission::checkRoot() {
    rootID = 0;
    if (jsv(0)) {
      if (iSync) {
	if (fabs((*i)(getTime())) <= gddTol)
	  rootID = 1; // Kein Stoß, wenn neue Übersetzung Null ist
	else
	  rootID = 3;
      }
      else {
	if (active or fabs(i0) <= gddTol) 
	  rootID = 1; // Synchronisation wenn sich Übersetzung ändert oder Null war
	else
	  rootID = 2;
      }
    }
    ds->setRootID(max(ds->getRootID(), rootID));
  }

  void GeneralizedTransmission::updaterFactors() {
    if (active) {

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

  void GeneralizedTransmission::solveConstraintsFixpointSingle() {
    if (active) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      gdd(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdd(0) += a[j] * laMBS(ja[j]);

      la(0) -= rFactor(0) * gdd(0);
    }
  }

  void GeneralizedTransmission::solveImpactsFixpointSingle() {
    if (active) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      gdn(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdn(0) += a[j] * LaMBS(ja[j]);

      La(0) -= rFactor(0) * gdn(0);
    }
  }

  void GeneralizedTransmission::solveImpactsGaussSeidel() {
    if (active) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      gdn(0) = b(laInd);
      for (int j = ia[laInd] + 1; j < ia[laInd + 1]; j++)
        gdn(0) += a[j] * LaMBS(ja[j]);

      La(0) = -gdn(0) / a[ia[laInd]];
    }
  }

  void GeneralizedTransmission::checkConstraintsForTermination() {
    if (active) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &laMBS = ds->getla(false);
      const Vec &b = ds->evalbc();

      gdd(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdd(0) += a[j] * laMBS(ja[j]);

      if (fabs(gdd(0) > gddTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void GeneralizedTransmission::checkImpactsForTermination() {
    if (active) {

      const double *a = ds->evalGs()();
      const int *ia = ds->getGs().Ip();
      const int *ja = ds->getGs().Jp();
      const Vec &LaMBS = ds->getLa(false);
      const Vec &b = ds->evalbi();

      gdn(0) = b(laInd);
      for (int j = ia[laInd]; j < ia[laInd + 1]; j++)
        gdn(0) += a[j] * LaMBS(ja[j]);

      if (fabs(gdn(0) > gdTol)) {
        ds->setTermination(false);
        return;
      }
    }
  }

  void GeneralizedTransmission::initializeUsingXML(DOMElement *element) {
    DualRigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"transmissionFunction");
    setTransmissionFunction(ObjectFactory::createAndInit<Function<double(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedSynchronizationForce");
    setGeneralizedSynchronizationForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"impulsiveSynchronization");
    setImpulsiveSynchronization(E(e)->getText<bool>());
  }

}
