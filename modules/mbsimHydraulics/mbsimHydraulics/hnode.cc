/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/environment.h"
#include "mbsim/utils/eps.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/constitutive_laws/unilateral_constraint.h"
#include "mbsim/constitutive_laws/bilateral_impact.h"
#include "mbsim/constitutive_laws/unilateral_newton_impact.h"
#include "mbsimControl/signal_.h"
#include "mbsim/objectfactory.h"

#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/sphere.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  HNode::HNode(const string &name) : Link(name), QHyd(0), nLines(0), updQHyd(true) {
  }

  void HNode::calcSize() {
    nla = 1;
    updSize = false;
  }

  void HNode::enableOpenMBV(double size, double pMin, double pMax, const Vec3 &WrON_) {
    openMBVSphere=OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
    openMBVSphere->setRadius(size);
    openMBVSphere->setMinimalColorValue(pMin);
    openMBVSphere->setMaximalColorValue(pMax);
    WrON=WrON_;
  }


  void HNode::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"inflow");
    while (e && E(e)->getTagName()==MBSIMHYDRAULICS%"inflow") {
      refInflowString.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"outflow");
    while (e && E(e)->getTagName()==MBSIMHYDRAULICS%"outflow") {
      refOutflowString.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"enableOpenMBVSphere");
    if (e) {
      DOMElement * ee;
      double size=1, pMin=0e5, pMax=10e5;
      Vec3 localWrON;
      ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"size");
      if(ee) size=E(ee)->getText<double>();
      ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"minimalPressure");
      if(ee) pMin=E(ee)->getText<double>();
      ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"maximalPressure");
      if(ee) pMax=E(ee)->getText<double>();
      ee = E(e)->getFirstElementChildNamed(MBSIMHYDRAULICS%"position");
      if (ee) localWrON=E(ee)->getText<Vec>(3);
      enableOpenMBVSphere(size, pMin, pMax, localWrON);
    }
  }

  void HNode::addInFlow(HLine * in) {
    connectedLinesStruct c;
    c.line=in;
    c.inflow=true;
    in->calcuSize(0);
    if (in->getuSize(0))
      connectedLines.push_back(c);
    else
      connected0DOFLines.push_back(c);
    in->setToNode(this);
  }

  void HNode::addOutFlow(HLine * out) {
    connectedLinesStruct c;
    c.line=out;
    c.inflow=false;
    out->calcuSize(0);
    if (out->getuSize(0))
      connectedLines.push_back(c);
    else
      connected0DOFLines.push_back(c);
    out->setFromNode(this);
  }

  void HNode::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      for (unsigned int i=0; i<refInflowString.size(); i++)
        addInFlow(getByPath<HLine>(refInflowString[i]));
      for (unsigned int i=0; i<refOutflowString.size(); i++)
        addOutFlow(getByPath<HLine>(refOutflowString[i]));
    }
    else if (stage==preInit) {
      gd.resize(1);
      la.resize(1);
      nLines=connectedLines.size();
      W[0].resize(nLines);
      W[1].resize(nLines);
      V[1].resize(nLines);
      V[0].resize(nLines);
      h[0].resize(nLines);
      h[1].resize(nLines);
      r[0].resize(nLines);
      r[1].resize(nLines);
      for (unsigned int i=0; i<nLines; i++) {
        connectedLines[i].sign = 
          ((connectedLines[i].inflow) ?
           connectedLines[i].line->getInflowFactor() :
           connectedLines[i].line->getOutflowFactor());
      }
    }
    else if (stage==plotting) {
      if(plotFeature[plotRecursive]) {
        plotColumns.push_back("Node pressure [bar]");
        if(plotFeature[debug]) {
          plotColumns.push_back("Volume flow into and out the node [l/min]");
          plotColumns.push_back("Mass flow into and out the node [kg/min]");
        }
      }
      if(plotFeature[openMBV] and openMBVSphere) {
        if (openMBVGrp) {
          openMBVSphere->setName("Node");
          openMBVGrp->addObject(openMBVSphere);
        }
        else {
          openMBVSphere->setName(name);
          parent->getOpenMBVGrp()->addObject(openMBVSphere);
        }
      }
    }
    else if (stage==unknownStage) {
      gdTol/=6e4;
    }
    Link::init(stage, config);
  }

  void HNode::updateWRef(const Mat &WParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int laI=laInd;
      const int laJ=laInd;
      const int hI=connectedLines[i].line->gethInd(j);
      const int hJ=hI+connectedLines[i].line->getGeneralizedVelocitySize()-1;
      W[j][i].resize()>>WParent(RangeV(hI, hJ), RangeV(laI, laJ));
    }
  }

  void HNode::updateVRef(const Mat &VParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int laI=laInd;
      const int laJ=laInd;
      const int hI=connectedLines[i].line->gethInd(j);
      const int hJ=hI+connectedLines[i].line->getGeneralizedVelocitySize()-1;
      V[j][i].resize()>>VParent(RangeV(hI, hJ), RangeV(laI, laJ));
    }
  }

  void HNode::updatehRef(const Vec& hParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hInd=connectedLines[i].line->gethInd(j);
      const RangeV I(hInd, hInd+connectedLines[i].line->getGeneralizedVelocitySize()-1);
      h[j][i].resize() >> hParent(I);
    }
  }

  void HNode::updaterRef(const Vec& rParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hInd=connectedLines[i].line->gethInd(j);
      const RangeV I(hInd, hInd+connectedLines[i].line->getGeneralizedVelocitySize()-1);
      r[j][i].resize() >> rParent(I);
    }
  }

  void HNode::updatedhdqRef(const Mat& dhdqParent, int j) {
    throw runtime_error("Error in HNode::updatedhdqRef");
  }

  void HNode::updatedhduRef(const SqrMat& dhduParent, int j) {
    throw runtime_error("Error in HNode::updatedhduRef");
  }

  void HNode::updatedhdtRef(const Vec& dhdtParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hInd = connectedLines[i].line->gethInd(j);
      const RangeV I=RangeV(hInd, hInd+connectedLines[i].sign.size()-1);
      dhdt[i].resize()>>dhdtParent(I);
    }
  }

  void HNode::updateQHyd() {
    QHyd=0;
    for (unsigned int i=0; i<nLines; i++)
      QHyd-=((connectedLines[i].inflow) ?
          connectedLines[i].line->evalQOut() :
          connectedLines[i].line->evalQIn())(0);
    for (unsigned int i=0; i<connected0DOFLines.size(); i++)
      QHyd-=((connected0DOFLines[i].inflow) ?
          connected0DOFLines[i].line->evalQOut() :
          connected0DOFLines[i].line->evalQIn())(0);
    updQHyd = false;
  }

  void HNode::updategd() {
    gd(0)=-evalQHyd();
  }

  void HNode::updateh(int j) {
    for (unsigned int i=0; i<nLines; i++)
      h[j][i] += trans(connectedLines[i].line->getJacobian()) * connectedLines[i].sign * evalGeneralizedForce()(0);
  }

  void HNode::plot() {
    if(plotFeature[plotRecursive]) {
      plotVector.push_back(evalGeneralizedForce()(0)*1e-5);
      if(plotFeature[debug]) {
        plotVector.push_back(evalQHyd()*6e4);
        plotVector.push_back(QHyd*HydraulicEnvironment::getInstance()->getSpecificMass()*60.);
      }
      if(plotFeature[openMBV] and openMBVSphere) {
        vector<double> data;
        data.push_back(getTime());
        data.push_back(WrON(0));
        data.push_back(WrON(1));
        data.push_back(WrON(2));
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(evalGeneralizedForce()(0));
        openMBVSphere->append(data);
      }
    }
    Link::plot();
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, ConstrainedNode)

  void ConstrainedNode::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      HNode::init(stage, config);
      addDependency(pFun->getDependency());
    }
    else if (stage==unknownStage) {
      HNode::init(stage, config);
      la.init((*pFun)(0));
    }
    else
      HNode::init(stage, config);
    pFun->init(stage, config);
  }

  void ConstrainedNode::updateGeneralizedForces() {
    lambda(0) = (*pFun)(getTime());
    updla = false;
  }

  void ConstrainedNode::initializeUsingXML(DOMElement *element) {
    HNode::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"function");
    setpFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild())); 
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, EnvironmentNode)

  void EnvironmentNode::init(InitStage stage, const InitConfigSet &config) {
    if (stage==unknownStage) {
      HNode::init(stage, config);
      lambda(0)=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
    }
    else
      HNode::init(stage, config);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, ElasticNode)

  ElasticNode::~ElasticNode() {
    delete bulkModulus;
    bulkModulus=NULL;
  }

  void ElasticNode::init(InitStage stage, const InitConfigSet &config) {
    if (stage==plotting) {
      if(plotFeature[plotRecursive])
        plotColumns.push_back("Node bulk modulus [N/mm^2]");
    }
    else if (stage==unknownStage) {
      double pinf=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
      if (fabs(p0)<epsroot) {
        msg(Warn) << "ElasticNode \"" << getPath() << "\" has no initial pressure. Using EnvironmentPressure instead." << endl;
        p0=pinf;
      }
      lambda(0)=p0;
      x0=Vec(1, INIT, p0);

      double E0=HydraulicEnvironment::getInstance()->getBasicBulkModulus();
      double kappa=HydraulicEnvironment::getInstance()->getKappa();
      bulkModulus = new OilBulkModulus(path, E0, pinf, kappa, fracAir);
    }
    HNode::init(stage, config);
  }

  void ElasticNode::initializeUsingXML(DOMElement * element) {
    HNode::initializeUsingXML(element);
    DOMElement * e;
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"volume");
    V=E(e)->getText<double>();
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"initialPressure");
    p0=E(e)->getText<double>();
    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"fracAir");
    fracAir=E(e)->getText<double>();
  }

  void ElasticNode::updateGeneralizedForces() {
    lambda = x;
    updla = false;
  }

  void ElasticNode::updatexd() {
    xd(0)=(*bulkModulus)(evalGeneralizedForce()(0))/V*evalQHyd();
  }

  void ElasticNode::plot() {
    if(plotFeature[plotRecursive])
      plotVector.push_back((*bulkModulus)(evalGeneralizedForce()(0))*1e-6);
    HNode::plot();
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, RigidNode)

  RigidNode::RigidNode(const string &name) : HNode(name), gdn(0), gdd(0), gfl(new BilateralConstraint), gil(new BilateralImpact) {
    gfl->setParent(this);
    gfl->setName("gfl");
    gil->setParent(this);
    gil->setName("gil");
  }

  RigidNode::~RigidNode() {
    if (gfl) {
      delete gfl;
      gfl=NULL;
    }
    if (gil) {
      delete gil;
      gil=NULL;
    }
  }

  void RigidNode::init(InitStage stage, const InitConfigSet &config) {
    HNode::init(stage, config);
    if(gfl) gfl->init(stage, config);
    if(gil) gil->init(stage, config);
  }

  const double& RigidNode::evalgdn() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdn;
  }

  const double& RigidNode::evalgdd() {
    if(ds->getUpdatela()) ds->updatela();
    return gdd;
  }

  void RigidNode::updateGeneralizedForces() {
    lambda = evalla();
    updla = false;
  }

  void RigidNode::updategd() {
    HNode::updategd();
    if (getTime()<epsroot) {
      if (fabs(QHyd)>epsroot)
        msg(Warn) << "RigidNode \"" << getPath() << "\": has an initial hydraulic flow not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
    }
  }

  void RigidNode::updateW(int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hJ=connectedLines[i].line->getGeneralizedVelocitySize()-1;
      W[j][i](RangeV(0,hJ), RangeV(0, 0))+=trans(connectedLines[i].line->getJacobian()) * connectedLines[i].sign;
    }
  }

  void RigidNode::updaterFactors() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();

    double sum = 0;
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      sum += fabs(a[j]);

    const double ai = a[ia[laInd]];
    if(ai > sum) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1./ai;
    }
    else {
      rFactorUnsure(0) = 1;
      rFactor(0) = 1./ai;
    }
  }

  void RigidNode::solveImpactsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);
    
    La(0) = gil->project(La(0), gdn, gd(0), rFactor(0));
  }

  void RigidNode::solveConstraintsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0));
  }

  void RigidNode::solveImpactsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    La(0) = gil->solve(a[ia[laInd]], gdn, gd(0));
  }

  void RigidNode::solveConstraintsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
  }

  void RigidNode::solveImpactsRootFinding() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    res(0) = La(0) - gil->project(La(0), gdn, gd(0), rFactor(0));
  }

  void RigidNode::solveConstraintsRootFinding() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gfl->project(la(0), gdd, rFactor(0));
  }

  void RigidNode::jacobianImpacts() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->evalG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gil->diff(La(0), gdn, gd(0), rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidNode::jacobianConstraints() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->evalG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gfl->diff(la(0), gdd, rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidNode::checkImpactsForTermination() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &LaMBS = ds->getLa(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*LaMBS(ja[j]);

    if(!gil->isFulfilled(La(0),gdn,gd(0),LaTol,gdTol))
      ds->setTermination(false);
  }

  void RigidNode::checkConstraintsForTermination() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0),gdd,laTol,gddTol))
      ds->setTermination(false);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, RigidCavitationNode)

  RigidCavitationNode::RigidCavitationNode(const string &name) : HNode(name), pCav(0), active(false), active0(false), gdn(0), gdd(0), gfl(new UnilateralConstraint), gil(new UnilateralNewtonImpact) {
  }

  RigidCavitationNode::~RigidCavitationNode() {
    if (gfl) {
      delete gfl;
      gfl=NULL;
    }
    if (gil) {
      delete gil;
      gil=NULL;
    }
  }

  void RigidCavitationNode::init(InitStage stage, const InitConfigSet &config) {
    if (stage==preInit)
      x0=Vec(1, INIT, 0);
    else if (stage==plotting) {
      if(plotFeature[plotRecursive])
        plotColumns.push_back("active");
    }
    else if (stage==unknownStage)
      lambda(0) = pCav;
    HNode::init(stage, config);
    if(gfl) gfl->init(stage, config);
    if(gil) gil->init(stage, config);
  }

  void RigidCavitationNode::plot() {
    if(plotFeature[plotRecursive])
      plotVector.push_back(active);
    HNode::plot();
  }

  void RigidCavitationNode::initializeUsingXML(DOMElement * element) {
    HNode::initializeUsingXML(element);
    DOMElement * e;
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"cavitationPressure");
    setCavitationPressure(E(e)->getText<double>());
  }

  void RigidCavitationNode::checkActive(int j) {
    if(j==1) 
      active=(x(0)<=0);
    else if(j==3) {
      if (active) {
        if (evalgdn() <= gdTol)
          active = true;
        else
          active = false;
      }
    }
    else
      throw runtime_error("Error in RigidCavitationNode::checkActive");
  }

  bool RigidCavitationNode::gActiveChanged() {
    bool changed = false;
    if (active0 != active)
      changed = true;
    active0=active;
    return changed;
  }

  const double& RigidCavitationNode::evalgdn() {
    if(ds->getUpdateLa()) ds->updateLa();
    return gdn;
  }

  const double& RigidCavitationNode::evalgdd() {
    if(ds->getUpdatela()) ds->updatela();
    return gdd;
  }

  void RigidCavitationNode::updateGeneralizedForces() {
    lambda(0) = pCav;
    updla = false;
  }

  void RigidCavitationNode::updateg() {
    g(0)=x(0);
  }

  void RigidCavitationNode::updateW(int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hJ=connectedLines[i].sign.size()-1;
      W[j][i](RangeV(0,hJ), RangeV(0, 0))=connectedLines[i].sign;
    }
  }

  void RigidCavitationNode::updatexd() {
    xd(0) = isActive() ? (fabs(evalgdn())>(gdTol)?gdn:0) : -evalQHyd();
  }

  void RigidCavitationNode::updateStopVector() {
    sv(0) = isActive() ? (evalGeneralizedForce()(0)-pCav)*1e-5 : -x(0)*6e4;
  }

  void RigidCavitationNode::checkRoot() {
    if(jsv(0)) {
      if(active) {
        active = false;
        ds->setRootID(max(ds->getRootID(),1)); 
      }
      else {
        active = true;
        ds->setRootID(max(ds->getRootID(),3)); // Impact
      }
    }
  }

  void RigidCavitationNode::updaterFactors() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();

    double sum = 0;
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      sum += fabs(a[j]);

    const double ai = a[ia[laInd]];
    if(ai > sum) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1./ai;
    }
    else {
      rFactorUnsure(0) = 1;
      rFactor(0) = 1./ai;
    }
  }

  void RigidCavitationNode::solveImpactsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);
    
    la(0) = gil->project(la(0), gdn, gd(0), rFactor(0), pCav*getStepSize());
  }

  void RigidCavitationNode::solveConstraintsFixpointSingle() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0), pCav);
  }

  void RigidCavitationNode::solveImpactsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    const double om = 1.0;
    const double buf = gil->solve(a[ia[laInd]], gdn, gd(0));
    la(0) += om*(buf - la(0));
    if (la(0)<pCav*getStepSize())
      la(0) = pCav*getStepSize();
  }

  void RigidCavitationNode::solveConstraintsGaussSeidel() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
    if (la(0)<pCav)
      la(0) = pCav;
  }

  void RigidCavitationNode::solveImpactsRootFinding() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);
    
    res(0) = la(0)-gil->project(la(0), gdn, gd(0), rFactor(0), pCav*getStepSize());
  }

  void RigidCavitationNode::solveConstraintsRootFinding() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gfl->project(la(0), gdd, rFactor(0), pCav);
  }

  void RigidCavitationNode::jacobianImpacts() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->evalG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gil->diff(la(0), gdn, gd(0), rFactor(0), pCav);

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidCavitationNode::jacobianConstraints() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->evalG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gfl->diff(la(0), gdd, rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidCavitationNode::checkImpactsForTermination() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbi();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    if(!gil->isFulfilled(la(0), gdn, gd(0), LaTol, gdTol, pCav*getStepSize()))
      ds->setTermination(false);
  }

  void RigidCavitationNode::checkConstraintsForTermination() {
    const double *a = ds->evalGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla(false);
    const Vec &b = ds->evalbc();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0), gdd, laTol, gddTol, pCav))
      ds->setTermination(false);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, PressurePump)

  void PressurePump::initializeUsingXML(DOMElement * element) {
    HNode::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"pressureFunction");
    setpFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
  }

  void PressurePump::init(InitStage stage, const InitConfigSet &config) {
    HNode::init(stage, config);
    pFunction->init(stage, config);
  }

  void PressurePump::updateGeneralizedForces() {
    lambda(0)=(*pFunction)(getTime());
    lambda = false;
  }

}
