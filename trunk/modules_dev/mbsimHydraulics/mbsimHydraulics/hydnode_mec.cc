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

#include "hydnode_mec.h"
#include "environment.h"
#include "mbsim/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/userfunction.h"
#include "mbsim/dynamic_system_solver.h"
#include "hydline.h"
#include "objectfactory.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include "openmbvcppinterface/arrow.h"
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  HydNodeMec::HydNodeMec(const string &name) : HydNode(name), QMec(1)
#ifdef HAVE_OPENMBVCPPINTERFACE
                                               , openMBVArrowSize(0)
#endif
                                               {
                                               }

  HydNodeMec::~HydNodeMec() {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVArrows.clear();
#endif
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void HydNodeMec::enableOpenMBVArrows(double size) {
    openMBVArrowSize=(size>.0)?size:.0;
  }
#endif

  unsigned int HydNodeMec::addTransMecArea(Frame * f, Vec fN, double area, bool considerVolumeChange) {
    connectedTransFrameStruct transFrame;
    transFrame.frame = f;
    transFrame.normal = fN/nrm2(fN);
    transFrame.area = area;
    transFrame.considerVolumeChange = considerVolumeChange;
    connectedTransFrames.push_back(transFrame);
    return connectedTransFrames.size()-1;
  }

  unsigned int HydNodeMec::addRotMecArea(Frame * f, Vec fN, double area, Frame * frameOfReference, bool considerVolumeChange) {
    connectedRotFrameStruct rotFrame;
    rotFrame.frame = f;
    rotFrame.normal = fN/nrm2(fN);
    rotFrame.area = area;
    rotFrame.fref = frameOfReference;
    rotFrame.considerVolumeChange = considerVolumeChange;
    connectedRotFrames.push_back(rotFrame);
    return connectedTransFrames.size()-1;
  }

  void HydNodeMec::init() {
    HydNode::init();
    nTrans=connectedTransFrames.size();
    for (unsigned int i=0; i<nTrans; i++) {
      int j=connectedTransFrames[i].frame->getJacobianOfTranslation().cols();
      W.push_back(Mat(j, laSize));
      V.push_back(Mat(j, laSize));
      h.push_back(Vec(j));
      hLink.push_back(Vec(j));
      dhdq.push_back(Mat(j, 0));
      dhdu.push_back(SqrMat(j));
      dhdt.push_back(Vec(j));
      r.push_back(Vec(j));
    }
    nRot=connectedRotFrames.size();
    for (unsigned int i=0; i<nRot; i++) {
      int j=connectedRotFrames[i].frame->getJacobianOfRotation().cols();
      W.push_back(Mat(j, laSize));
      V.push_back(Mat(j, laSize));
      h.push_back(Vec(j));
      hLink.push_back(Vec(j));
      dhdq.push_back(Mat(j, 0));
      dhdu.push_back(SqrMat(j));
      dhdt.push_back(Vec(j));
      r.push_back(Vec(j));
    }
    pinf=Vec(1, INIT, HydraulicEnvironment::getInstance()->getEnvironmentPressure());
    x.resize(xSize);
    x0=Vec(1, INIT, V0);
  }

  void HydNodeMec::updateWRef(const Mat &WParent, int j) {
    HydNode::updateWRef(WParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedTransFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols() -1;
      W[nLines+i].resize()>>WParent(Index(hI, hJ), Index(laI, laJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedRotFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedRotFrames[i].frame->getJacobianOfRotation().cols() -1;
      W[nTrans+nLines+i].resize()>>WParent(Index(hI, hJ), Index(laI, laJ));
    }
  }

  void HydNodeMec::updateVRef(const Mat &VParent, int j) {
    HydNode::updateVRef(VParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedTransFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols() -1;
      V[nLines+i].resize()>>VParent(Index(hI, hJ), Index(laI, laJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int laI = laInd;
      int laJ = laInd;
      int hI = connectedRotFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedRotFrames[i].frame->getJacobianOfRotation().cols() -1;
      V[nTrans+nLines+i].resize()>>VParent(Index(hI, hJ), Index(laI, laJ));
    }
  }

  void HydNodeMec::updatehRef(const Vec &hParent, const Vec& hLinkParent, int j) {
    HydNode::updatehRef(hParent, hLinkParent, j);
    for (unsigned int i=0; i<nTrans; i++) {
      int hI = connectedTransFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols()-1;
      h[nLines+i].resize() >> hParent(Index(hI, hJ));
      hLink[nLines+i].resize() >> hLinkParent(Index(hI, hJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int hI = connectedRotFrames[i].frame->getParent()->gethInd(parent, j);
      int hJ = hI + connectedRotFrames[i].frame->getJacobianOfRotation().cols()-1;
      h[nTrans+nLines+i].resize() >> hParent(Index(hI, hJ));
      hLink[nTrans+nLines+i].resize() >> hLinkParent(Index(hI, hJ));
    }
  }

  void HydNodeMec::updatedhdqRef(const Mat& dhdqParent, int j) {
    // for (unsigned int i=0; i<nLines; i++) {
    //   int hInd = connectedLines[i].line->gethInd(parent, j);
    //   Index I=Index(hInd, hInd);
    //   dhdq[i].resize()>>dhdqParent(I);
    // }
  }

  void HydNodeMec::updatedhduRef(const SqrMat& dhduParent, int j) {
    //  for (unsigned int i=0; i<nLines; i++) {
    //    int hInd = connectedLines[i].line->gethInd(parent, j);
    //    Index I=Index(hInd, hInd);
    //    dhdu[i].resize()>>dhduParent(I);
    //  }
  }

  void HydNodeMec::updatedhdtRef(const Vec& dhdtParent, int j) {
    // for (unsigned int i=0; i<nLines; i++) {
    //   int hInd = connectedLines[i].line->gethInd(parent, j);
    //   Index I=Index(hInd, hInd);
    //   dhdt[i].resize()>>dhdtParent(I);
    // }
  }

  void HydNodeMec::updaterRef(const Vec &rParent) {
    HydNode::updaterRef(rParent);
    for (unsigned int i=0; i<nTrans; i++) {
      int rI = connectedTransFrames[i].frame->getParent()->gethInd(parent);
      int rJ = rI + connectedTransFrames[i].frame->getJacobianOfTranslation().cols()-1;
      r[nLines+i] >> rParent(Index(rI, rJ));
    }
    for (unsigned int i=0; i<nRot; i++) {
      int rI = connectedRotFrames[i].frame->getParent()->gethInd(parent);
      int rJ = rI + connectedRotFrames[i].frame->getJacobianOfRotation().cols()-1;
      r[nTrans+nLines+i] >> rParent(Index(rI, rJ));
    }
  }

  void HydNodeMec::updategd(double t) {
    HydNode::updategd(t);

    QMecTrans=0;
    for (unsigned int i=0; i<nTrans; i++)
      if (connectedTransFrames[i].considerVolumeChange)
        QMecTrans += 
          connectedTransFrames[i].area * 
          trans(
              connectedTransFrames[i].frame->getOrientation() * 
              connectedTransFrames[i].normal
              ) * connectedTransFrames[i].frame->getVelocity();

    QMecRot=0;
    for (unsigned int i=0; i<nRot; i++) {
      if (connectedRotFrames[i].considerVolumeChange) {
        Vec WrRefF = 
          -connectedRotFrames[i].fref->getPosition()
          +connectedRotFrames[i].frame->getPosition();
        double distance = nrm2(WrRefF);
        double OmegaRel = 
          trans(
              crossProduct(
                WrRefF / distance, 
                connectedRotFrames[i].frame->getOrientation()*connectedRotFrames[i].normal
                )
              ) * (
                connectedRotFrames[i].frame->getAngularVelocity()
                - connectedRotFrames[i].fref->getAngularVelocity()
                );
        QMecRot += 
          OmegaRel * distance * connectedRotFrames[i].area;
      }
    }
    QMec=QMecTrans+QMecRot;
    gd(0)+=QMec;
  }

  void HydNodeMec::updateh(double t) {
    HydNode::updateh(t);
    for (unsigned int i=0; i<nTrans; i++) {
      h[nLines+i] +=
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedTransFrames[i].frame->getOrientation() * 
         connectedTransFrames[i].normal
        //) * (la - pinf);
        ) * (la);
      hLink[nLines+i] +=
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedTransFrames[i].frame->getOrientation() * 
         connectedTransFrames[i].normal
        //) * (la - pinf);
        ) * (la);
    }
    for (unsigned int i=0; i<nRot; i++) {
      h[nTrans+nLines+i] += 
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedRotFrames[i].frame->getOrientation() * 
         connectedRotFrames[i].normal
        //) * (la - pinf);
        ) * (la);
      hLink[nTrans+nLines+i] += 
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedRotFrames[i].frame->getOrientation() * 
         connectedRotFrames[i].normal
        //) * (la - pinf);
        ) * (la);
    }
  }

  void HydNodeMec::updatedhdz(double t) {
  }

  void HydNodeMec::updater(double t) {
    HydNode::updater(t);
    for (unsigned int i=0; i<nTrans; i++)
      r[nLines+i] += W[nLines+i] * la;
    for (unsigned int i=0; i<nRot; i++)
      r[nTrans+nLines+i] += W[nTrans+nLines+i] * la;
  }

  void HydNodeMec::updatexd(double t) {
    xd(0)=QMec;
  }

  void HydNodeMec::updatedx(double t, double dt) {
    xd(0)=QMec*dt;
  }

  void HydNodeMec::initPlot() {
    plotColumns.push_back("Volume [mm^3]");
    plotColumns.push_back("QTrans [mm^3/s]");
    plotColumns.push_back("QRot [mm^3/s]");
    plotColumns.push_back("Mechanical surface flow into and out the node [mm^3/s]");
    plotColumns.push_back("gd(0)");
    for (unsigned int i=0; i<nTrans; i++)
      plotColumns.push_back("interface force on area " + numtostr(int(i)));
    for (unsigned int i=0; i<nRot; i++)
      plotColumns.push_back("interface force on area " + numtostr(int(i)));
#ifdef HAVE_OPENMBVCPPINTERFACE
    if (openMBVArrowSize>0) {
      for (int i=0; i<nTrans+nRot; i++) {
        openMBVArrows.push_back(new OpenMBV::Arrow);
        openMBVArrows.back()->setArrowHead(openMBVArrowSize/4., openMBVArrowSize/4.);
        openMBVArrows.back()->setDiameter(openMBVArrowSize/10.);
      }
      for (unsigned int i=0; i<nTrans; i++) {
        openMBVArrows[i]->setName(
            name+
            "_ForceOn_"+
            connectedTransFrames[i].frame->getName());
        parent->getOpenMBVGrp()->addObject(openMBVArrows[i]);
      }
      for (unsigned int i=0; i<nRot; i++) {
        openMBVArrows[nTrans+i]->setName(
            getName()+
            "_ForceOn_"+
            connectedRotFrames[i].frame->getName());
        parent->getOpenMBVGrp()->addObject(openMBVArrows[nTrans+i]);
      }
    }
#endif
    HydNode::initPlot();
  }

  void HydNodeMec::plot(double t, double dt) {
    plotVector.push_back(x(0)*1e9);
    plotVector.push_back(QMecTrans*1e9);
    plotVector.push_back(QMecRot*1e9);
    plotVector.push_back(QMec*1e9);
    plotVector.push_back(gd(0));
    for (unsigned int i=0; i<nTrans; i++)
      //plotVector.push_back(connectedTransFrames[i].area*(la(0)-pinf(0)));
      plotVector.push_back(connectedTransFrames[i].area*(la(0)));
    for (unsigned int i=0; i<nRot; i++)
      //plotVector.push_back(connectedRotFrames[i].area*(la(0)-pinf(0)));
      plotVector.push_back(connectedRotFrames[i].area*(la(0)));
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(getPlotFeature(openMBV)==enabled && openMBVSphere) {
      WrON.init(0);
      for (unsigned int i=0; i<nTrans; i++)
        WrON+=connectedTransFrames[i].frame->getPosition();
      for (unsigned int i=0; i<nRot; i++)
        WrON+=connectedRotFrames[i].frame->getPosition();
      WrON/=double(nTrans+nRot);
    }
    if (openMBVArrows.size()) {
      for (unsigned int i=0; i<nTrans; i++) {
        vector<double> data;
        Vec toPoint=connectedTransFrames[i].frame->getPosition();
        Vec dir=(
            connectedTransFrames[i].frame->getOrientation() * 
            connectedTransFrames[i].normal
            ) *
          //openMBVArrowSize*1e-5*(isSetValued()?la(0)/dt:(la-pinf)(0));
          openMBVArrowSize*1e-5*(isSetValued()?la(0)/dt:(la)(0));
        data.push_back(t);
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        data.push_back(dir(0));
        data.push_back(dir(1));
        data.push_back(dir(2));
        data.push_back(1.);
        openMBVArrows[i]->append(data);
      }
      for (unsigned int i=0; i<nRot; i++) {
        vector<double> data;
        Vec toPoint=connectedRotFrames[i].frame->getPosition();
        Vec dir=(
            connectedRotFrames[i].frame->getOrientation() * 
            connectedRotFrames[i].normal
            ) *
          //openMBVArrowSize*1e-5*(isSetValued()?la(0)/dt:(la-pinf)(0));
          openMBVArrowSize*1e-5*(isSetValued()?la(0)/dt:(la)(0));
        data.push_back(t);
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        data.push_back(dir(0));
        data.push_back(dir(1));
        data.push_back(dir(2));
        data.push_back(1.);
        openMBVArrows[nTrans+i]->append(data);
      }
    }
#endif
    HydNode::plot(t, dt);
  }


  HydNodeMecConstrained::HydNodeMecConstrained(const string &name) : HydNodeMec(name) {
  }

  void HydNodeMecConstrained::setpFunction(UserFunction * pFun_) {
    pFun=pFun_;
  }

  void HydNodeMecConstrained::init() {
    HydNodeMec::init();
    la.init((*pFun)(0)(0));
    x0=Vec(1, INIT, V0);
  }

  void HydNodeMecConstrained::updateg(double t) {
    HydNodeMec::updateg(t);
    la=(*pFun)(t);
  }


  HydNodeMecElastic::HydNodeMecElastic(const string &name) : HydNodeMec(name) {
  }

  double HydNodeMecElastic::calcBulkModulus(double p) {
    if(p<=0.1) {
      cout << "HydraulicEnvironment: pressure near zero! Continuing anyway, using p=0.1 Pa" << endl;
      p=0.1;
    }
    // Umdruck zur Vorlesung
    // Grundlagen der Oelhydraulik
    // W.Backe
    // H.Murrenhoff
    // 10. Auflage 1994
    // Formel (3-11), S. 103
    return factor[0]/(1.+factor[1]*pow(p, factor[2]));
  }

  void HydNodeMecElastic::init() {
    HydNodeMec::init();
    la(0)=p0;
    Vec x0Tmp(2);
    x0Tmp(0)=V0;
    x0Tmp(1)=p0;
    x0.resize(2);
    x0=x0Tmp;

    double E0=HydraulicEnvironment::getInstance()->getBasicBulkModulus();
    double kappa=HydraulicEnvironment::getInstance()->getKappa();
    factor[0]=E0*(1.+fracAir);
    factor[1]=pow(pinf(0), 1./kappa) * fracAir * E0 / kappa;
    factor[2]=-(1.+1./kappa);
    E=calcBulkModulus(la(0));
  }

  void HydNodeMecElastic::updatexRef(const Vec &xParent) {
    HydNodeMec::updatexRef(xParent);
    la >> x(Index(1,1));
  }

  void HydNodeMecElastic::updatexd(double t) {
    HydNodeMec::updatexd(t);
    E=calcBulkModulus(la(0));
    xd(1)=-E/x(0)*gd(0);
  }

  void HydNodeMecElastic::updatedx(double t, double dt) {
    HydNodeMec::updatedx(t, dt);
    E=calcBulkModulus(la(0));
    xd(1)=-E/x(0)*gd(0)*dt;
  }

  void HydNodeMecElastic::initPlot() {
    plotColumns.push_back("Node bulk modulus [N/mm^2]");
    plotColumns.push_back("gd(0)");
    HydNodeMec::initPlot();
  }

  void HydNodeMecElastic::plot(double t, double dt) {
    plotVector.push_back(E*1e-6);
    plotVector.push_back(gd(0));
    HydNodeMec::plot(t, dt);
  }

  void HydNodeMecElastic::initializeUsingXML(TiXmlElement *element) {
    TiXmlElement *e;
    HydNodeMec::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fracAir");
    setFracAir(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"p0");
    setp0(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"initialVolume");
    setInitialVolume(atof(e->GetText()));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"transMecArea");
    TiXmlElement *ee;
    ee=e->FirstChildElement(MBSIMNS"frameOfReference");
    Frame *ref=getFrameByPath(ee->Attribute("ref"));
    if(!ref) { cerr<<"ERROR! Cannot find frame: "<<ee->Attribute("ref")<<endl; _exit(1); }
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"normal");
    Vec normal(ee->GetText());
    ee=e->FirstChildElement(MBSIMHYDRAULICSNS"area");
    double area=atof(ee->GetText());
    addTransMecArea(ref, normal, area);
    e=e->NextSiblingElement();
  }


  HydNodeMecRigid::HydNodeMecRigid(const string &name) : HydNodeMec(name) {
  }

  void HydNodeMecRigid::init() {
    HydNodeMec::init();
    x0=Vec(1, INIT, V0);
  }

  void HydNodeMecRigid::updatewbRef(const Vec &wbParent) {
    Link::updatewbRef(wbParent);
    gd >> wb;
  }

  void HydNodeMecRigid::updateW(double t) {
    for (unsigned int i=0; i<nLines; i++) {
      int hJ=connectedLines[i].sign.cols()-1;
      W[i](Index(0,hJ), Index(0, 0)) = connectedLines[i].sign;
    }
    for (unsigned int i=0; i<nTrans; i++) {
      int hJ=connectedTransFrames[i].frame->getJacobianOfTranslation().cols()-1;
      W[nLines+i](Index(0,hJ), Index(0, 0)) =
        connectedTransFrames[i].area * 
        trans(connectedTransFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedTransFrames[i].frame->getOrientation() * 
         connectedTransFrames[i].normal
        );
    }
    for (unsigned int i=0; i<nRot; i++) {
      int hJ=connectedRotFrames[i].frame->getJacobianOfTranslation().cols()-1;
      W[nTrans+nLines+i](Index(0,hJ), Index(0, 0)) = 
        connectedRotFrames[i].area * 
        trans(connectedRotFrames[i].frame->getJacobianOfTranslation()) * 
        (
         connectedRotFrames[i].frame->getOrientation() * 
         connectedRotFrames[i].normal
        );
    }
  }

  void HydNodeMecRigid::solveImpactsFixpointSingle() {
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

  void HydNodeMecRigid::solveImpactsGaussSeidel() {
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

  void HydNodeMecRigid::solveImpactsRootFinding() {
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

  void HydNodeMecRigid::updaterFactors() {
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

  void HydNodeMecRigid::checkImpactsForTermination() {
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
