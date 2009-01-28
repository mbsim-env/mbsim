/* Copyright (C) 2004-2006  Martin Förg

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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include "body_rigid.h"
#include "port.h"
#include "contour.h"
#include "link.h"
#include "multi_body_system.h"

#ifdef HAVE_AMVIS
#include "crigidbody.h"
#include "data_interface_base.h"
using namespace AMVis;
#endif

namespace MBSim {

  BodyRigid::BodyRigid(const string &name) : Body(name), m(0), I(3), Mh(6), WrOK(3), WvK(3), WomegaK(3), KomegaK(3), KrKS(3), AWK(3), AK0K(3), l(6), WF(3), WM(3), WLtmp(6), WFtmp(WLtmp(0,2)), WMtmp(WLtmp(3,5)), rot(cardanAngles), inertiaWithRespectToCOG(false) 
# ifdef HAVE_AMVIS
					     ,
					     bodyAMVis(0), AMVisDataRel(false), AMVisInstance(0)
# endif
  {
    AK0K(0,0)=1.0;
    AK0K(1,1)=1.0;
    AK0K(2,2)=1.0;

  }

  BodyRigid::~BodyRigid() {
#   ifdef HAVE_AMVIS
      if(AMVisInstance==0) delete bodyAMVis; bodyAMVis=NULL;
      delete bodyAMVisUserFunctionColor; bodyAMVisUserFunctionColor=NULL;
#   endif
  }

  void BodyRigid::calcSize() {
    assert(JT.cols()<=3);
    assert(JR.cols()<=3);

    Body::calcSize();

    uSize = JT.cols() + JR.cols();

    if(JR.cols() == 3 && rot == eulerParameters)
      qSize = uSize+1;
    else
      qSize = uSize;
  } 

  void BodyRigid::init() {

    Body::init();

    if(JT.rows() == 0)
      JT.resize(3,0);
    if(JR.rows() == 0)
      JR.resize(3,0);

    Mh(0,0) = m;
    Mh(1,1) = m;
    Mh(2,2) = m;

    Mat tKrKS = tilde(KrKS);
    Mh(Index(3,5)) = I;
    if (inertiaWithRespectToCOG) Mh(Index(3,5)) +=  m*JTJ(tKrKS);
    Mh(Index(0,2),Index(3,5)) = m*trans(tKrKS);

    iT = Index(0,JT.cols()-1);
    iR = Index(JT.cols(),JT.cols()+JR.cols()-1);

    //T.resize(qSize,uSize);
    for(int i=0; i<uSize; i++) 
      T(i,i) = 1;

    for(int i=0; i<JT.cols(); i++)
      JT.col(i) /= nrm2(JT.col(i));
    for(int i=0; i<JR.cols(); i++)
      JR.col(i) /= nrm2(JR.col(i));

    if(JR.cols()== 0) {
      updateAK0K = &BodyRigid::noUpdateAK0K;
      updateT_ = &BodyRigid::noUpdateT;
    } else if(JR.cols() == 1) {
      updateAK0K = &BodyRigid::updateAK0KAxis;
      updateT_ = &BodyRigid::noUpdateT;
    } else if(JR.cols() == 2) {
      //updateAK0K = &BodyRigid::updateAK0Kxz;
      //updateT_ = &BodyRigid::updateTxz;
      cout << "Rotations with two angles not yet implemented!" << endl;
      assert(JR.cols()!=2);
    } else if(rot == cardanAngles) {
      updateAK0K = &BodyRigid::updateAK0KCardanAngles;
      updateT_ = &BodyRigid::updateTCardanAngles;
    } else if(rot == eulerParameters) {
      updateAK0K = &BodyRigid::updateAK0KEulerParameters;
      updateT_ = &BodyRigid::updateTEulerParameters;
      H.resize(3,4);
      TH.resize(3,4);
      if(q0.size() == 0) {
	q0.resize(qSize);
	q0(JT.cols()) = 1;
      }
      assert(fabs(nrm2(q0(JT.cols(),JT.cols()+3))-1)<=1e-16);
    } 
  }

  void BodyRigid::initPlotFiles() {

    Body::initPlotFiles();

#ifdef HAVE_AMVIS
    if(bodyAMVis && AMVisInstance==0)
      bodyAMVis->writeBodyFile();
#endif

    if(plotLevel>0) {
      plotfile <<"# "<< plotNr++ << ": WxOS" << endl;
      plotfile <<"# "<< plotNr++ << ": WyOS" << endl;
      plotfile <<"# "<< plotNr++ << ": WzOS" << endl;
      plotfile <<"# "<< plotNr++ << ": alpha" << endl;
      plotfile <<"# "<< plotNr++ << ": beta" << endl;
      plotfile <<"# "<< plotNr++ << ": gamma" <<endl;

      if(plotLevel>2) {
	plotfile <<"# "<< plotNr++ << ": T" << endl;
	plotfile <<"# "<< plotNr++ << ": V" << endl;
	plotfile <<"# "<< plotNr++ << ": E" << endl;
      }

    }
  }

  void BodyRigid::plotParameters() {
    parafile << "BodyRigid"  << JT.cols() << JR.cols() << endl;
    parafile << "# mass:\n"    << m    << endl;
    parafile << "# inertia:\n" << I    << endl;
    parafile << "# JT:\n"      << JT   << endl;
    parafile << "# JR:\n"      << JR   << endl;
    if(port.size()>0) parafile << "# ports: " <<endl;
    for(unsigned int i=0; i<port.size(); i++) {
      Vec KrKPtemp = KrKP[i];
      parafile << "# KrKP: (port:  name= "<< port[i]->getName()<<",  ID= "<<port[i]->getID()<<") = (" << KrKPtemp(0) <<","<< KrKPtemp(1) <<","<< KrKPtemp(2) << ")" << endl;
    }
  }

  void BodyRigid::plot(double t, double dt) {
    Body::plot(t,dt);
    Vec WrOS = computeWrOS();

#ifdef HAVE_AMVIS
    if(plotLevel>0 || bodyAMVis)
#else
      if(plotLevel>0)
#endif
      {
	double alpha;
	double beta=asin(AWK(0,2));
	double gamma;
	double nenner=cos(beta);
	if (nenner>1e-10) {
	  alpha=atan2(-AWK(1,2),AWK(2,2));
	  gamma=atan2(-AWK(0,1),AWK(0,0));
	} else {
	  alpha=0;
	  gamma=atan2(AWK(1,0),AWK(1,1));
	}

	if(plotLevel>0) {
	  plotfile<<" "<<WrOS(0)<<" "<<WrOS(1)<<" "<<WrOS(2);
	  plotfile<<" "<<alpha<<" "<<beta<<" "<<gamma; 

	  if(plotLevel>2) {
	    double Ttemp = computeKineticEnergy();
	    double Vtemp = computePotentialEnergy();
	    plotfile <<" "<<  Ttemp;
	    plotfile <<" "<<  Vtemp;
	    plotfile <<" "<< Ttemp+Vtemp;
	  }
	}

#ifdef HAVE_AMVIS
	if(bodyAMVis) {
	  if(AMVisDataRel)
	    WrOS >> WrOK;
	  bodyAMVis->setTime(t);
	  bodyAMVis->setTranslation(WrOS(0),WrOS(1),WrOS(2));
	  bodyAMVis->setRotation(alpha,beta,gamma);
	  if (bodyAMVisUserFunctionColor) {
	    double color = (*bodyAMVisUserFunctionColor)(t)(0);
	    if (color>1) color =1;
	    if (color<0) color =0;
	    bodyAMVis->setColor(color);
	  }
	  bodyAMVis->appendDataset(AMVisInstance);
	}
#endif
      }
  }

  void BodyRigid::updateKinematics(double t) {
    updateCenterOfGravity(t);
    updatePorts(t);
    updateContours(t);
  }

  void BodyRigid::updatePorts(double t) {
    for(unsigned int i=0; i<port.size(); i++) {
      WrKP[i] = AWK * KrKP[i];
      port[i]->setWrOP(WrOK + WrKP[i]);
      port[i]->setWvP(WvK + crossProduct(WomegaK, WrKP[i]));
      port[i]->setWomegaP(WomegaK);
      port[i]->setAWP(AWK);
    }
  }

  void BodyRigid::updateContours(double t) {
    for(unsigned int i=0; i<contour.size(); i++) {
      SqrMat AWC = AWK*AKC[i];
      WrKC[i] = AWK * KrKC[i];
      contour[i]->setWrOP(WrOK + WrKC[i]);
      contour[i]->setWvP(WvK + crossProduct(WomegaK, WrKC[i]));
      contour[i]->setWomegaC(WomegaK);
      contour[i]->setAWC(AWC);
    }
  }

  void BodyRigid::sumUpForceElements(double t) {
    WF = m*mbs->getGrav();
    WM = crossProduct(AWK*KrKS,WF);

    for(unsigned int i=0; i<linkSingleValuedPortData.size(); i++) {
      int portID = linkSingleValuedPortData[i].ID;
      int objectID = linkSingleValuedPortData[i].objectID;
      WLtmp = linkSingleValuedPortData[i].link->getLoad(objectID);
      WF += WFtmp;
      WM += WMtmp + crossProduct(WrKP[portID],WFtmp);
    }
    for(unsigned int i=0; i<linkSingleValuedContourData.size(); i++) {
      if(linkSingleValuedContourData[i].link->isActive()) {
	int objectID = linkSingleValuedContourData[i].objectID;
	WLtmp = linkSingleValuedContourData[i].link->getLoad(objectID);
	WF += WFtmp;
	Vec WrKC = linkSingleValuedContourData[i].link->getWrOC(objectID)-WrOK;
	WM += WMtmp + crossProduct(WrKC,WFtmp);
      }
    }
  }

  void BodyRigid::addPort(const string &name, const Vec &KrKP) {
    Port *port = new Port(name);
    addPort(port,KrKP);
  }

  void BodyRigid::addPort(Port* port, const Vec &KrKP_) {
    Object::addPort(port);
    KrKP.push_back(KrKP_.copy()); 
    WrKP.push_back(Vec());
  }

  void BodyRigid::addContour(Contour* contour, const Vec &KrKC_, const SqrMat &AKC_) {
    Object::addContour(contour);
    KrKC.push_back(KrKC_.copy()); 
    WrKC.push_back(Vec());
    AKC.push_back(AKC_.copy());

    // HitSphere anpassen !!!
    contour->adjustParentHitSphere(KrKC_);
  }

  void BodyRigid::updatezd(double t) {
    qd = T*u;
    ud = slvLLFac(LLM, h+r);
  }

  void BodyRigid::updatedu(double t, double dt) {
    ud = slvLLFac(LLM, h*dt +r);
  }

  void BodyRigid::updatedq(double t, double dt) {
    qd = T*u*dt;
  }

  void BodyRigid::setJT(const Mat &JT_) {
    JT = JT_;
    assert(JT.rows()==3);
  }

  void BodyRigid::setJR(const Mat &JR_) {
    JR = JR_;
    assert(JR.rows()==3);
  }

  double BodyRigid::computePotentialEnergy() {
    return - m * ( trans(mbs->getGrav()) * ( WrOK + AWK*KrKS ) ); 
  }

  void BodyRigid::updateAK0KCardanAngles() {
    int iRInd = iR.start();
    double a=q(iRInd);
    double b=q(iRInd+1);
    double g=q(iRInd+2);

    AK0K(0,0) = cos(b)*cos(g);
    AK0K(1,0) = sin(a)*sin(b)*cos(g)+cos(a)*sin(g);
    AK0K(2,0) = -cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
    AK0K(0,1) = -cos(b)*sin(g);
    AK0K(1,1) = -sin(g)*sin(b)*sin(a)+cos(a)*cos(g);
    AK0K(2,1) = cos(a)*sin(b)*sin(g)+sin(a)*cos(g);
    AK0K(0,2) = sin(b);
    AK0K(1,2) = -sin(a)*cos(b);
    AK0K(2,2) = cos(a)*cos(b);
  }

  void BodyRigid::updateTCardanAngles() {
    int iRInd = iR.start();
    double beta = q(iRInd+1);
    double gamma = q(iRInd+2);
    double cos_beta = cos(beta);
    double sin_beta = sin(beta);
    double cos_gamma = cos(gamma);
    double sin_gamma = sin(gamma);
    double tan_beta = sin_beta/cos_beta;

    T(iRInd,iRInd) = cos_gamma/cos_beta;
    T(iRInd,iRInd+1) = -sin_gamma/cos_beta;
    T(iRInd+1,iRInd) = sin_gamma;
    T(iRInd+1,iRInd+1) = cos_gamma;
    T(iRInd+2,iRInd) = -cos_gamma*tan_beta;
    T(iRInd+2,iRInd+1) = sin_gamma*tan_beta;           
  }

  void BodyRigid::updateAK0KEulerParameters() {
    DiagMat E(3,INIT,1.0);
    int iRInd = iR.start();

    Vec e = q(iRInd+1,qSize-1);
    H.col(0) = -e;
    TH.col(0) = -e;
    SqrMat Te = tilde(e);
    H(Index(0,2),Index(1,3)) = Te+q(iRInd)*E;
    TH(Index(0,2),Index(1,3)) = -Te+q(iRInd)*E;
    AK0K = SqrMat(H*trans(TH));
  }

  void BodyRigid::updateTEulerParameters() {
    T(Index(iR.start(),qSize-1),iR) = 0.5 * trans(TH);
  }

  void BodyRigid::updateAK0KAxis() {
    Vec a = JR.col(0);
    double cosq=cos(q(qSize-1));
    double sinq=sin(q(qSize-1));
    double onemcosq=1-cosq;
    double a0a1=a(0)*a(1);
    double a0a2=a(0)*a(2);
    double a1a2=a(1)*a(2);
    AK0K(0,0) = cosq+onemcosq*a(0)*a(0);
    AK0K(1,0) = onemcosq*a0a1+a(2)*sinq;
    AK0K(2,0) = onemcosq*a0a2-a(1)*sinq;
    AK0K(0,1) = onemcosq*a0a1-a(2)*sinq;
    AK0K(1,1) = cosq+onemcosq*a(1)*a(1);
    AK0K(2,1) = onemcosq*a1a2+a(0)*sinq;
    AK0K(0,2) = onemcosq*a0a2+a(1)*sinq;
    AK0K(1,2) = onemcosq*a1a2-a(0)*sinq;
    AK0K(2,2) = cosq+onemcosq*a(2)*a(2);
  }

}
