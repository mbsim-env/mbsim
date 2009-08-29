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
 * Contact: schneidm@users.berlios.de
 */

#include "mbsimHydraulics/hydnode.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsim/utils/eps.h"
#include "mbsim/dynamic_system_solver.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/sphere.h"
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  HydNode::HydNode(const string &name) : Link(name), QHyd(0), nLines(0)
# ifdef HAVE_OPENMBVCPPINTERFACE
                                         , openMBVGrp(NULL), openMBVSphere(NULL), WrON(3)
#endif
                                         {
                                         }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void HydNode::enableOpenMBV(double size, double pMin, double pMax, Vec WrON_) {
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

  HydLineAbstract * HydNode::getHydLineAbstractByPath(string path) {
    int pos=path.find("HydLine");
    path.erase(pos, 7);
    path.insert(pos, "Object");
    Object * h = parent->getObjectByPath(path);
    if (dynamic_cast<HydLineAbstract *>(h))
      return static_cast<HydLineAbstract *>(h);
    else {
      std::cerr << "ERROR! \"" << path << "\" is not of HydLineAbstract-Type." << std::endl; 
      _exit(1);
    }
  }


  void HydNode::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    e=element->FirstChildElement();
    while (e && (e->ValueStr()==MBSIMHYDRAULICSNS"inflow" || e->ValueStr()==MBSIMHYDRAULICSNS"outflow")) {
      if (e->ValueStr()==MBSIMHYDRAULICSNS"inflow")
        addInFlow(getHydLineAbstractByPath(e->Attribute("ref")));
      else
        addOutFlow(getHydLineAbstractByPath(e->Attribute("ref")));
      e=e->NextSiblingElement();
    }
  }

  void HydNode::addInFlow(HydLineAbstract * in) {
    connectedLinesStruct c;
    c.line=in;
    c.inflow=true;
    connectedLines.push_back(c);
    in->setToNode(this);
  }

  void HydNode::addOutFlow(HydLineAbstract * out) {
    connectedLinesStruct c;
    c.line=out;
    c.inflow=false;
    connectedLines.push_back(c);
    out->setFromNode(this);
  }

  void HydNode::init(InitStage stage) {
    if (stage==MBSim::resize) {
      Link::init(stage);
      gd.resize(1);
      la.resize(1);
      nLines=connectedLines.size();
      for (unsigned int i=0; i<nLines; i++) {
        connectedLines[i].sign = 
          ((connectedLines[i].inflow) ?
           connectedLines[i].line->getInflowFactor() :
           connectedLines[i].line->getOutflowFactor());
        int cols=connectedLines[i].sign.size();
        W.push_back(Mat(cols, laSize));
        V.push_back(Mat(cols, laSize));
        h.push_back(Vec(cols));
        hLink.push_back(Vec(cols));
        dhdq.push_back(Mat(cols, 0));
        dhdu.push_back(SqrMat(cols));
        dhdt.push_back(Vec(cols));
        r.push_back(Vec(cols));
      }
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Node pressure [bar]");
        plotColumns.push_back("Fluidflow into and out the node [l/min]");
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (openMBVSphere) {
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
      gdTol*=1e-6;
    }
    else
      Link::init(stage);
  }

  void HydNode::updateWRef(const Mat &WParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      int laI=laInd;
      int laJ=laInd;
      int hI=connectedLines[i].line->gethInd(parent,j);
      int hJ=hI+connectedLines[i].sign.size()-1;
      W[i].resize()>>WParent(Index(hI, hJ), Index(laI, laJ));
    }
  }

  void HydNode::updateVRef(const Mat &VParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      int laI=laInd;
      int laJ=laInd;
      int hI=connectedLines[i].line->gethInd(parent,j);
      int hJ=hI+connectedLines[i].sign.size()-1;
      V[i].resize()>>VParent(Index(hI, hJ), Index(laI, laJ));
    }
  }

  void HydNode::updatehRef(const Vec& hParent, const Vec& hLinkParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      int hInd=connectedLines[i].line->gethInd(parent, j);
      Index I(hInd, hInd+connectedLines[i].sign.size()-1);
      h[i].resize() >> hParent(I);
      hLink[i].resize() >> hLinkParent(I);
    }
  }

  void HydNode::updatedhdqRef(const Mat& dhdqParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      int hInd = connectedLines[i].line->gethInd(parent, j);
      Index I=Index(hInd, hInd+connectedLines[i].sign.size()-1);
      dhdq[i].resize()>>dhdqParent(I);
    }
  }

  void HydNode::updatedhduRef(const SqrMat& dhduParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      int hInd = connectedLines[i].line->gethInd(parent, j);
      Index I=Index(hInd, hInd+connectedLines[i].sign.size()-1);
      dhdu[i].resize()>>dhduParent(I);
    }
  }

  void HydNode::updatedhdtRef(const Vec& dhdtParent, int j) {
    for (unsigned int i=0; i<nLines; i++) {
      int hInd = connectedLines[i].line->gethInd(parent, j);
      Index I=Index(hInd, hInd+connectedLines[i].sign.size()-1);
      dhdt[i].resize()>>dhdtParent(I);
    }
  }

  void HydNode::updaterRef(const Vec &rParent) {
    for (unsigned int i=0; i<nLines; i++) {
      int rI=connectedLines[i].line->gethInd(parent);
      int rJ=rI+connectedLines[i].sign.size()-1;
      r[i]>>rParent(Index(rI, rJ));
    }
  }

  void HydNode::updategd(double t) {
    QHyd=0;
    for (unsigned int i=0; i<nLines; i++)
      QHyd-=((connectedLines[i].inflow) ?
          connectedLines[i].line->getQOut(t) :
          connectedLines[i].line->getQIn(t))(0);
    gd(0)=-QHyd;
  }

  void HydNode::updateh(double t) {
    for (unsigned int i=0; i<nLines; i++) {
      h[i] += connectedLines[i].sign * la;
      hLink[i] += connectedLines[i].sign * la;
    }
  }

  void HydNode::updatedhdz(double t) {
    vector<Vec> hLink0, h0;

    for(unsigned int i=0; i<nLines; i++) { // save old values
      hLink0.push_back(hLink[i].copy());
      h0.push_back(h[i].copy());
    }
    if(nLines)
      updateh(t); 
    vector<Vec> hLinkEnd, hEnd;
    for(unsigned int i=0; i<nLines; i++) { // save with correct state
      hLinkEnd.push_back(hLink[i].copy());
      hEnd.push_back(h[i].copy());
    }

    for(unsigned int i=0; i<nLines; i++) { 
      /** velocity dependent calculations **/
      for(int j=0; j<connectedLines[i].sign.cols(); j++) {
        hLink[i] = hLink0[i]; // set to old values
        h[i] = h0[i];

        double uParentj = connectedLines[i].line->getu()(j); // save correct position
        connectedLines[i].line->getu()(j) += epsroot(); // update with disturbed positions assuming same active links
        connectedLines[i].line->updateStateDependentVariables(t); 
        updategd(t);
        updateh(t);

        dhdu[i].col(j) += (hLink[i]-hLinkEnd[i])/epsroot();
        connectedLines[i].line->getu()(j) = uParentj;
      }

      /** position dependent calculations **/
      if(connectedLines[i].sign.cols() > 0) {
        for(int j=0; j<connectedLines[i].line->getq().size(); j++) {
          hLink[i] = hLink0[i]; // set to old values
          h[i] = h0[i];

          double qParentj = connectedLines[i].line->getq()(j); // save correct position

          connectedLines[i].line->getq()(j) += epsroot(); // update with disturbed positions assuming same active links
          connectedLines[i].line->updateStateDependentVariables(t); 
          updateg(t);
          updategd(t);
          connectedLines[i].line->updateT(t); 
          updateJacobians(t);
          updateh(t);
          dhdq[i].col(j) += (hLink[i]-hLinkEnd[i])/epsroot();
          connectedLines[i].line->getq()(j) = qParentj;
        }
      }

      /** time dependent calculations **/
      hLink[i] = hLink0[i]; // set to old values
      h[i] = h0[i];

      double t0 = t; // save correct position

      t += epsroot(); // update with disturbed positions assuming same active links
      connectedLines[i].line->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      connectedLines[i].line->updateT(t); 
      updateJacobians(t);
      updateh(t);

      dhdt[i] += (hLink[i]-hLinkEnd[i])/epsroot();
      t = t0;
    }

    /** back to initial state **/
    for(unsigned int i=0; i<nLines; i++) {
      connectedLines[i].line->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      connectedLines[i].line->updateT(t); 
      updateJacobians(t);
      hLink[i] = hLinkEnd[i];
      h[i] = hEnd[i];
    }
  }

  void HydNode::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(la(0)*1e-5/(isSetValued()?dt:1.));
      plotVector.push_back(QHyd*6e4);
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


  void HydNodeConstrained::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HydNode::init(stage);
      la.init((*pFun)(0));
    }
    else
      HydNode::init(stage);
  }

  void HydNodeConstrained::updateg(double t) {
    HydNode::updateg(t);
    la(0)=(*pFun)(t);
  }

  void HydNodeConstrained::initializeUsingXML(TiXmlElement *element) {
    HydNode::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMHYDRAULICSNS"function");
    pFun=ObjectFactory::getInstance()->getInstance()->createFunction1_SS(e->FirstChildElement()); 
    pFun->initializeUsingXML(e->FirstChildElement());
    //    e=element->FirstChildElement("function");
  }



  void HydNodeEnvironment::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HydNode::init(stage);
      la(0)=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
    }
    else
      HydNode::init(stage);
  }


  HydNodeElastic::~HydNodeElastic() {
    delete bulkModulus;
  }

  void HydNodeElastic::init(InitStage stage) {
    if (stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Node bulk modulus [N/mm^2]");
        HydNode::init(stage);
      }
    }
    else if (stage==unknownStage) {
      HydNode::init(stage);
      double pinf=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
      if (fabs(p0)<epsroot()) {
        cout << "WARNING HydNodeElastic \"" << name << "\" has no initial pressure. Using EnvironmentPressure instead." << endl;
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
      HydNode::init(stage);
  }

  void HydNodeElastic::initializeUsingXML(TiXmlElement * element) {
    HydNode::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"volume");
    V=atof(e->GetText());
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"initialPressure");
    p0=atof(e->GetText());
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fracAir");
    fracAir=atof(e->GetText());
  }

  void HydNodeElastic::updatexRef(const Vec &xParent) {
    HydNode::updatexRef(xParent);
    la >> x;
  }

  void HydNodeElastic::updatexd(double t) {
    E=(*bulkModulus)(la(0));
    xd=-E/V*gd;
  }

  void HydNodeElastic::updatedx(double t, double dt) {
    E=(*bulkModulus)(la(0));
    xd=-E/V*gd*dt;
  }

  void HydNodeElastic::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(E*1e-6);
      HydNode::plot(t, dt);
    }
  }


  void HydNodeRigid::init(InitStage stage) {
    if (stage==MBSim::unknownStage) {
      HydNode::init(stage);
      for (unsigned int i=0; i<nLines; i++) {
        Vec u0=connectedLines[i].line->getu0();
        bool zero=true;
        for (int j=0; j<u0.size(); j++)
          if (fabs(u0(j))>epsroot())
            zero=false;
        if (!zero)
          cout << "WARNING in HydNodeRigid \"" << getName() << "\": HydraulicLine \"" << connectedLines[i].line->getName() << "\" has an initialGeneralizedVelocity not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
      }
    }
    else
      HydNode::init(stage);
  }

  void HydNodeRigid::updatewbRef(const Vec &wbParent) {
    Link::updatewbRef(wbParent);
    gd >> wb;
  }

  void HydNodeRigid::updategd(double t) {
    HydNode::updategd(t);
    if (t<epsroot()) {
      if (fabs(QHyd)>epsroot())
        cout << "WARNING: HydNodeRigid \"" << name << "\": has an initial hydraulic flow not equal to zero. Just Time-Stepping Integrators can handle this correctly." << endl;
    }
  }

  void HydNodeRigid::updateW(double t) {
    for (unsigned int i=0; i<nLines; i++) {
      int hJ=connectedLines[i].sign.size()-1;
      W[i](Index(0,hJ), Index(0, 0))=connectedLines[i].sign;
    }
  }

  void HydNodeRigid::solveImpactsFixpointSingle() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    la(0) -= rFactor(0)*gdn;
  }

  void HydNodeRigid::solveImpactsGaussSeidel() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    la(0) -= gdn/a[ia[laIndDS+0]];
  }

  void HydNodeRigid::solveImpactsRootFinding() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    res(0) = rFactor(0)*gdn;
  }

  void HydNodeRigid::jacobianImpacts() {
    SqrMat Jprox = ds->getJprox();
    SqrMat G = ds->getG();
    RowVec jp1=Jprox.row(laIndDS);
    jp1.init(0);
    for(int j=0; j<G.size(); j++) 
      jp1(j) = rFactor(0) * G(laIndDS, j);
  }

  void HydNodeRigid::updaterFactors() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();

    double sum = 0;
    for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
      sum += fabs(a[j]);

    double ai = a[ia[laIndDS]];
    if(ai > sum) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1./ai;
    }
    else {
      rFactorUnsure(0) = 1;
      rFactor(0) = 1./ai;
    }
  }

  void HydNodeRigid::checkImpactsForTermination() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn += a[j]*laMBS(ja[j]);

    if(!(fabs(gdn)<gdTol))
      ds->setTermination(false);
  }
}
