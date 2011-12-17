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

#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/obsolet_hint.h"
#include "mbsim/utils/eps.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/constitutive_laws.h"
#include "mbsimControl/signal_.h"
#include "mbsimHydraulics/obsolet_hint.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/sphere.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimHydraulics {

  HNode::HNode(const string &name) : Link(name), QHyd(0), nLines(0)
# ifdef HAVE_OPENMBVCPPINTERFACE
                                     , openMBVGrp(NULL), openMBVSphere(NULL), WrON(3)
#endif
                                     {
                                     }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void HNode::enableOpenMBV(double size, double pMin, double pMax, Vec WrON_) {
    if(size>=0) {
      openMBVSphere=new OpenMBV::Sphere;
      openMBVSphere->setRadius(size);
      openMBVSphere->setMinimalColorValue(pMin);
      openMBVSphere->setMaximalColorValue(pMax);
      WrON=WrON_;
    }
    else 
      openMBVSphere=0;
  }
#endif


  void HNode::initializeUsingXML(TiXmlElement *element) {
    Link::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"inflow");
    while (e) {
      refInflowString.push_back(e->Attribute("ref"));
      e=e->NextSiblingElement(MBSIMHYDRAULICSNS"inflow");
    }
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"outflow");
    while (e) {
      refOutflowString.push_back(e->Attribute("ref"));
      e=e->NextSiblingElement(MBSIMHYDRAULICSNS"outflow");
    }
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"enableOpenMBVSphere");
    if (e) {
      TiXmlElement * ee;
      ee = e->FirstChildElement(MBSIMHYDRAULICSNS"size");
      double size=Element::getDouble(ee);
      ee = e->FirstChildElement(MBSIMHYDRAULICSNS"minimalPressure");
      double pMin=Element::getDouble(ee);
      ee = e->FirstChildElement(MBSIMHYDRAULICSNS"maximalPressure");
      double pMax=Element::getDouble(ee);
      ee = e->FirstChildElement(MBSIMHYDRAULICSNS"position");
      Vec localWrON(3, INIT, 0);
      if (ee)
        localWrON=Element::getVec(ee, 3);
      enableOpenMBV(size, pMin, pMax, localWrON);
    }
#endif
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

  void HNode::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      for (unsigned int i=0; i<refInflowString.size(); i++)
        addInFlow(getByPath<HLine>(process_hline_string(refInflowString[i])));
      for (unsigned int i=0; i<refOutflowString.size(); i++)
        addOutFlow(getByPath<HLine>(process_hline_string(refOutflowString[i])));
      Link::init(stage);
    }
    else if (stage==MBSim::resize) {
      Link::init(stage);
      gd.resize(1);
      la.resize(1);
      nLines=connectedLines.size();
      for (unsigned int i=0; i<nLines; i++) {
        connectedLines[i].sign = 
          ((connectedLines[i].inflow) ?
           connectedLines[i].line->getInflowFactor() :
           connectedLines[i].line->getOutflowFactor());
        const int rows=connectedLines[i].sign.size();
        W[0].push_back(Mat(rows, laSize));
        V[0].push_back(Mat(rows, laSize));
        h[0].push_back(Vec(rows));
        W[1].push_back(Mat(rows, laSize));
        V[1].push_back(Mat(rows, laSize));
        h[1].push_back(Vec(rows));
        dhdq.push_back(Mat(rows, 0));
        dhdu.push_back(SqrMat(rows));
        dhdt.push_back(Vec(rows));
        r[0].push_back(Vec(rows));
        r[1].push_back(Vec(rows));
      }
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Node pressure [bar]");
        if(getPlotFeature(debug)==enabled) {
          plotColumns.push_back("Volume flow into and out the node [l/min]");
          plotColumns.push_back("Mass flow into and out the node [kg/min]");
        }
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVSphere) {
          if (openMBVGrp) {
            openMBVSphere->setName("Node");
            openMBVGrp->addObject(openMBVSphere);
          }
          else {
            openMBVSphere->setName(name);
            parent->getOpenMBVGrp()->addObject(openMBVSphere);
          }
        }
#endif
        Link::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      Link::init(stage);
      gdTol/=6e4;
    }
    else
      Link::init(stage);
  }

  void HNode::updateWRef(const Mat &WParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int laI=laInd;
      const int laJ=laInd;
      const int hI=connectedLines[i].line->gethInd(j);
      const int hJ=hI+connectedLines[i].line->getJacobian().cols()-1;
      W[j][i].resize()>>WParent(Index(hI, hJ), Index(laI, laJ));
    }
  }

  void HNode::updateVRef(const Mat &VParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int laI=laInd;
      const int laJ=laInd;
      const int hI=connectedLines[i].line->gethInd(j);
      const int hJ=hI+connectedLines[i].line->getJacobian().cols()-1;
      V[j][i].resize()>>VParent(Index(hI, hJ), Index(laI, laJ));
    }
  }


  void HNode::updatehRef(const Vec& hParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hInd=connectedLines[i].line->gethInd(j);
      const Index I(hInd, hInd+connectedLines[i].line->getJacobian().cols()-1);
      h[j][i].resize() >> hParent(I);
    }
  }

  void HNode::updaterRef(const Vec& rParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hInd=connectedLines[i].line->gethInd(j);
      const Index I(hInd, hInd+connectedLines[i].line->getJacobian().cols()-1);
      r[j][i].resize() >> rParent(I);
    }
  }

  void HNode::updatedhdqRef(const Mat& dhdqParent, int j) {
    throw;
  }

  void HNode::updatedhduRef(const SqrMat& dhduParent, int j) {
    throw;
  }

  void HNode::updatedhdtRef(const Vec& dhdtParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hInd = connectedLines[i].line->gethInd(j);
      const Index I=Index(hInd, hInd+connectedLines[i].sign.size()-1);
      dhdt[i].resize()>>dhdtParent(I);
    }
  }

  void HNode::updategd(double t) {
    QHyd=0;
    for (unsigned int i=0; i<nLines; i++)
      QHyd-=((connectedLines[i].inflow) ?
          connectedLines[i].line->getQOut() :
          connectedLines[i].line->getQIn())(0);
    for (unsigned int i=0; i<connected0DOFLines.size(); i++)
      QHyd-=((connected0DOFLines[i].inflow) ?
          connected0DOFLines[i].line->getQOut() :
          connected0DOFLines[i].line->getQIn())(0);
    gd(0)=-QHyd;
  }

  void HNode::updateh(double t, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      h[j][i] += trans(connectedLines[i].line->getJacobian()) * connectedLines[i].sign * la(0);
    }
  }

  void HNode::updater(double t, int j) {
    cout << "HNode \"" << name << "\": updater()" << endl; 
  }

  void HNode::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(la(0)*1e-5/(isActive()?dt:1.));
      if(getPlotFeature(debug)==enabled) {
        plotVector.push_back(QHyd*6e4);
        plotVector.push_back(QHyd*HydraulicEnvironment::getInstance()->getSpecificMass()*60.);
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVSphere) {
        vector<double> data;
        data.push_back(t);
        data.push_back(WrON(0));
        data.push_back(WrON(1));
        data.push_back(WrON(2));
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(la(0)/(isSetValued()?dt:1.));
        openMBVSphere->append(data);
      }
#endif
      Link::plot(t, dt);
    }
  }


  void ConstrainedNode::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HNode::init(stage);
      la.init((*pFun)(0));
    }
    else
      HNode::init(stage);
  }

  void ConstrainedNode::updateg(double t) {
    HNode::updateg(t);
    la(0)=(*pFun)(t);
  }

  void ConstrainedNode::initializeUsingXML(TiXmlElement *element) {
    HNode::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMHYDRAULICSNS"function");
    pFun=MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()); 
    pFun->initializeUsingXML(e->FirstChildElement());
  }



  void EnvironmentNode::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HNode::init(stage);
      la(0)=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
    }
    else
      HNode::init(stage);
  }


  ElasticNode::~ElasticNode() {
    delete bulkModulus;
    bulkModulus=NULL;
  }

  void ElasticNode::init(InitStage stage) {
    if (stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Node bulk modulus [N/mm^2]");
        HNode::init(stage);
      }
    }
    else if (stage==unknownStage) {
      HNode::init(stage);
      double pinf=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
      if (fabs(p0)<epsroot()) {
        cout << "WARNING ElasticNode \"" << name << "\" has no initial pressure. Using EnvironmentPressure instead." << endl;
        p0=pinf;
      }
      la(0)=p0;
      x0=Vec(1, INIT, p0);

      double E0=HydraulicEnvironment::getInstance()->getBasicBulkModulus();
      double kappa=HydraulicEnvironment::getInstance()->getKappa();
      bulkModulus = new OilBulkModulus(name, E0, pinf, kappa, fracAir);
      E=(*bulkModulus)(la(0));
    }
    else
      HNode::init(stage);
  }

  void ElasticNode::initializeUsingXML(TiXmlElement * element) {
    HNode::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"volume");
    V=getDouble(e);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"initialPressure");
    p0=getDouble(e);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fracAir");
    fracAir=getDouble(e);
  }

  void ElasticNode::updatexRef(const Vec &xParent) {
    HNode::updatexRef(xParent);
    la >> x;
  }

  void ElasticNode::updatexd(double t) {
    E=(*bulkModulus)(la(0));
    xd(0)=E/V*QHyd;
  }

  void ElasticNode::updatedx(double t, double dt) {
    E=(*bulkModulus)(la(0));
    xd(0)=E/V*QHyd*dt;
  }

  void ElasticNode::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(E*1e-6);
      HNode::plot(t, dt);
    }
  }


  RigidNode::RigidNode(const string &name) : HNode(name), gdn(0), gdd(0), gfl(new BilateralConstraint), gil(new BilateralImpact) {
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

  void RigidNode::updategd(double t) {
    HNode::updategd(t);
    if (t<epsroot()) {
      if (fabs(QHyd)>epsroot())
        cout << "WARNING: RigidNode \"" << name << "\": has an initial hydraulic flow not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
    }
  }

  void RigidNode::updateW(double t, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hJ=connectedLines[i].line->getJacobian().cols()-1;
      W[j][i](Index(0,hJ), Index(0, 0))+=trans(connectedLines[i].line->getJacobian()) * connectedLines[i].sign;
    }
  }

  void RigidNode::updaterFactors() {
    const double *a = ds->getGs()();
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

  void RigidNode::solveImpactsFixpointSingle(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);
    
    la(0) = gil->project(la(0), gdn, gd(0), rFactor(0));
  }

  void RigidNode::solveConstraintsFixpointSingle() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0));
  }

  void RigidNode::solveImpactsGaussSeidel(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    la(0) = gil->solve(a[ia[laInd]], gdn, gd(0));
  }

  void RigidNode::solveConstraintsGaussSeidel() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
  }

  void RigidNode::solveImpactsRootFinding(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gil->project(la(0), gdn, gd(0), rFactor(0));
  }

  void RigidNode::solveConstraintsRootFinding() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gfl->project(la(0), gdd, rFactor(0));
  }

  void RigidNode::jacobianImpacts() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->getG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gil->diff(la(0), gdn, gd(0), rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidNode::jacobianConstraints() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->getG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gfl->diff(la(0), gdd, rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidNode::checkImpactsForTermination(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    if(!gil->isFulfilled(la(0),gdn,gd(0),LaTol,gdTol))
      ds->setTermination(false);
  }

  void RigidNode::checkConstraintsForTermination() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0),gdd,laTol,gddTol))
      ds->setTermination(false);
  }


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

  void RigidCavitationNode::init(InitStage stage) {
    if (stage==MBSim::resize) {
      HNode::init(stage);
      g.resize(1, INIT, 0);
      x.resize(1, INIT, 0);
      sv.resize(1, INIT, 0);
      x0=Vec(1, INIT, 0);
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("active");
        HNode::init(stage);
      }
    }
    else
      HNode::init(stage);
  }

  void RigidCavitationNode::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(active);
      HNode::plot(t, dt);
    }
  }

  void RigidCavitationNode::initializeUsingXML(TiXmlElement * element) {
    HNode::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"cavitationPressure");
    setCavitationPressure(getDouble(e));
  }

  void RigidCavitationNode::checkActiveg() {
    active=(g(0)<=0);
  }

  void RigidCavitationNode::checkActivegdn() {
    if (active) {
      if (gdn <= gdTol)
        active = true;
      else
        active = false;
    }
  }

  bool RigidCavitationNode::gActiveChanged() {
    bool changed = false;
    if (active0 != active)
      changed = true;
    active0=active;
    return changed;
  }

  void RigidCavitationNode::updateg(double t) {
    if (g.size())
      g(0)=x(0);
  }

  void RigidCavitationNode::updateh(double t, int j) {
    la(0) = pCav;
    HNode::updateh(t, j);
  }

  void RigidCavitationNode::updateStopVector(double t) {
    sv(0) = isActive() ? (la(0)-pCav)*1e-5 : -x(0)*6e4;
  }

  void RigidCavitationNode::updateW(double t, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      const int hJ=connectedLines[i].sign.size()-1;
      W[j][i](Index(0,hJ), Index(0, 0))=connectedLines[i].sign;
    }
  }

  void RigidCavitationNode::updatexd(double t) {
    xd(0) = isActive() ? (fabs(gdn)>(gdTol)?gdn:0) : -QHyd;
  }

  void RigidCavitationNode::updatedx(double t, double dt) {
    xd(0) = isActive() ? (fabs(gdn)>(gdTol)?gdn:0)*dt : -QHyd*dt;
  }

  void RigidCavitationNode::updateCondition() {
    if(jsv(0)) {
      if(active) {
        active = false;
        return;
      }
      else {
        active = true;
        ds->setImpact(true);
        return;
      }
    }
  }


  void RigidCavitationNode::updaterFactors() {
    const double *a = ds->getGs()();
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

  void RigidCavitationNode::solveImpactsFixpointSingle(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);
    
    la(0) = gil->project(la(0), gdn, gd(0), rFactor(0), pCav*dt);
  }

  void RigidCavitationNode::solveConstraintsFixpointSingle() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->project(la(0), gdd, rFactor(0), pCav);
  }

  void RigidCavitationNode::solveImpactsGaussSeidel(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    const double om = 1.0;
    const double buf = gil->solve(a[ia[laInd]], gdn, gd(0));
    la(0) += om*(buf - la(0));
    if (la(0)<pCav*dt)
      la(0) = pCav*dt;
  }

  void RigidCavitationNode::solveConstraintsGaussSeidel() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    la(0) = gfl->solve(a[ia[laInd]], gdd);
    if (la(0)<pCav)
      la(0) = pCav;
  }

  void RigidCavitationNode::solveImpactsRootFinding(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);
    
    res(0) = la(0)-gil->project(la(0), gdn, gd(0), rFactor(0), pCav*dt);
  }

  void RigidCavitationNode::solveConstraintsRootFinding() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    res(0) = la(0) - gfl->project(la(0), gdd, rFactor(0), pCav);
  }

  void RigidCavitationNode::jacobianImpacts() {
    const SqrMat Jprox = ds->getJprox();
    const SqrMat G = ds->getG();

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
    const SqrMat G = ds->getG();

    RowVec jp1=Jprox.row(laInd);
    RowVec e1(jp1.size());
    e1(laInd) = 1;
    Vec diff = gfl->diff(la(0), gdd, rFactor(0));

    jp1 = e1-diff(0)*e1;
    for(int j=0; j<G.size(); j++) 
      jp1(j) -= diff(1)*G(laInd,j);
  }

  void RigidCavitationNode::checkImpactsForTermination(double dt) {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdn = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    if(!gil->isFulfilled(la(0), gdn, gd(0), LaTol, gdTol, pCav*dt))
      ds->setTermination(false);
  }

  void RigidCavitationNode::checkConstraintsForTermination() {
    const double *a = ds->getGs()();
    const int *ia = ds->getGs().Ip();
    const int *ja = ds->getGs().Jp();
    const Vec &laMBS = ds->getla();
    const Vec &b = ds->getb();

    gdd = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdd += a[j]*laMBS(ja[j]);

    if(!gfl->isFulfilled(la(0), gdd, laTol, gddTol, pCav))
      ds->setTermination(false);
  }


  void PressurePump::initializeUsingXML(TiXmlElement * element) {
    HNode::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"pressureSignal");
    pSignalString=e->Attribute("ref");
  }

  void PressurePump::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      HNode::init(stage);
      if (pSignalString!="")
        setpSignal(getByPath<MBSimControl::Signal>(process_signal_string(pSignalString)));
    }
    else
      HNode::init(stage);
  }

  void PressurePump::updateg(double t) {
    HNode::updateg(t);
    la(0)=(pSignal->getSignal())(0);
  }

}
