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

#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimControl/signal_.h"
#include "mbsimHydraulics/rigid_line.h"

using namespace std;

namespace MBSim {

  double ZetaLinePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      double d=((const RigidLine*)(line))->getDiameter();
      double A=M_PI*d*d/4.;
      c*=rho/2./A/A;
      initialized=true;
    }
    return c*Q*abs(Q);
  }

  void ZetaLinePressureLoss::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"zeta");
    setZeta(Element::getDouble(e));
  }


  double LaminarTubeFlowLinePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double nu=HydraulicEnvironment::getInstance()->getDynamicViscosity();
      double d=((const RigidLine*)(line))->getDiameter();
      double l=((const RigidLine*)(line))->getLength();
      double area=M_PI*d*d/4.;
      c=32.*nu*l/d/d/area;
      initialized=true;
    }
    return c*Q;
  }

  double CurveFittedLinePressureLoss::operator()(const double &Q, const void * line) {
    if (!initialized) {
      double eta=HydraulicEnvironment::getInstance()->getKinematicViscosity();
      ReynoldsFactor=dHyd/(M_PI*dRef*dRef/4.)/eta;
      initialized=true;
    }
    Re=ReynoldsFactor*Q; 
    return Re*((Re>0)?aPos+bPos*Re:aNeg-bNeg*Re); 
  }

  void CurveFittedLinePressureLoss::initializeUsingXML(TiXmlElement * element) {
    PressureLoss::initializeUsingXML(element);
    dRef=Element::getDouble(element->FirstChildElement(MBSIMHYDRAULICSNS"referenceDiameter"));
    dHyd=Element::getDouble(element->FirstChildElement(MBSIMHYDRAULICSNS"hydraulicDiameter"));
    aPos=Element::getDouble(element->FirstChildElement(MBSIMHYDRAULICSNS"aPositive"));
    bPos=Element::getDouble(element->FirstChildElement(MBSIMHYDRAULICSNS"bPositive"));
    aNeg=Element::getDouble(element->FirstChildElement(MBSIMHYDRAULICSNS"aNegative"));
    bNeg=Element::getDouble(element->FirstChildElement(MBSIMHYDRAULICSNS"bNegative"));
  }

  const double signalValue(const ClosableRigidLine * l) {
    const double s=l->getSignal()->getSignal()(0);
    const double sMin=l->getMinimalValue();
    return (s<sMin)?sMin:s;
  }

  double RelativeAreaZetaClosablePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      double d=((const RigidLine*)(line))->getDiameter();
      double A=M_PI*d*d/4.;
      c*=rho/2./A/A;
      initialized=true;
    }
    const double areaRel=signalValue((const ClosableRigidLine*)(line));
    return c*Q*abs(Q)/areaRel/areaRel;
  }

  void RelativeAreaZetaClosablePressureLoss::initializeUsingXML(TiXmlElement * element) {
    ClosablePressureLoss::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"zeta");
    setZeta(Element::getDouble(e));
  }


  double RelativeAlphaClosablePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double d=((const RigidLine*)(line))->getDiameter();
      double area=M_PI*d*d/4.;
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      alpha2=alpha*alpha;
      c=rho/2./area/area;
      initialized=true;
    }
    double alphaRel=signalValue((const ClosableRigidLine*)(line));
    if (alphaRel>1.)
      alphaRel=1.;
    return (1./(alphaRel * alphaRel * alpha2) - 1.)*c*Q*fabs(Q);
  }

  void RelativeAlphaClosablePressureLoss::initializeUsingXML(TiXmlElement * element) {
    ClosablePressureLoss::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"alpha");
    setAlpha(Element::getDouble(e));
  }


  void CheckvalveClosablePressureLoss::initializeUsingXML(TiXmlElement * element) {
    ClosablePressureLoss::initializeUsingXML(element);
    TiXmlElement * e = element->FirstChildElement(MBSIMHYDRAULICSNS"ballRadius");
    setBallRadius(Element::getDouble(e));
  }


  double GammaCheckvalveClosablePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      siga=sin(gamma); 
      coga=cos(gamma); 
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      c=1./alpha/alpha*coga*coga*coga*coga*rho/2.;
      initialized=true;
    }
    const double xOpen=signalValue((const ClosableRigidLine*)(line));
    area=M_PI*xOpen*siga*(2.*rBall*coga+xOpen);
    return c*Q*fabs(Q)/area/area; 
  }

  void GammaCheckvalveClosablePressureLoss::initializeUsingXML(TiXmlElement * element) {
    CheckvalveClosablePressureLoss::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"alpha");
    setAlpha(Element::getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"gamma");
    setGamma(Element::getDouble(e));
  }


  double IdelchickCheckvalveClosablePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double d=((const RigidLine*)(line))->getDiameter();
      double area=M_PI*d*d*.25;
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      d0=d;
      c=rho/2./area/area;
      initialized=true;
    }
    const double xOpen=signalValue((const ClosableRigidLine*)(line));
    const double hdivd0=xOpen/d0;
    const double beta2=.8/hdivd0;
    const double beta3=.14/hdivd0/hdivd0; // TODO
    return c*Q*fabs(Q)*(2.7-beta2-beta3); 
  }


  double ConeCheckvalveClosablePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double d=((const RigidLine*)(line))->getDiameter();
      double rOpen=d/2.;
      double rBall2=rBall*rBall;
      double rOpen2=rOpen*rOpen;
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      c=rho/2./alpha/alpha/M_PI/M_PI;
      numer[0]=rBall2;
      numer[1]=2.*sqrt(rBall2-rOpen2);
      denom[0]=4.*(rBall2-rOpen2);
      denom[1]=4.*sqrt(rBall2-rOpen2);
      initialized=true;
    }
    const double xOpen=signalValue((const ClosableRigidLine*)(line));
    const double xOpen2=xOpen*xOpen;
    const double xOpen4=xOpen2*xOpen2;
    return c*(xOpen2+xOpen*numer[1]+numer[0])/(xOpen2+xOpen*denom[1]+denom[0])/xOpen4 * Q * fabs(Q);
  }

  void ConeCheckvalveClosablePressureLoss::initializeUsingXML(TiXmlElement * element) {
    CheckvalveClosablePressureLoss::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"alpha");
    setAlpha(Element::getDouble(e));
  }


//  void LeakagePressureLoss::init(InitStage stage, vector<string> * plotColumns) {
//    if (stage==MBSim::resolveXMLPath) {
//      PressureLoss::init(stage, plotColumns);
//      if (s1vPath!="")  
//        setSurface1Velocity(getSignalByPath(line->getParent(), s1vPath));
////        setSurface1Velocity(getSignalByPath(line, s1vPath));
//      if (s2vPath!="")  
//        setSurface2Velocity(getSignalByPath(line->getParent(), s2vPath));
////        setSurface2Velocity(getSignalByPath(line, s2vPath));
//      if (glPath!="")  
//        setGapLength(getSignalByPath(line->getParent(), glPath));
////        setGapLength(getSignalByPath(line, glPath));
//    }
//    else if (stage==MBSim::resize) {
//      PressureLoss::init(stage, plotColumns);
//      /*
//      if (s1vSignal)
//        assert(s1vSignal->getSignal().size()==1);
//      if (s2vSignal)
//        assert(s2vSignal->getSignal().size()==1);
//      if (glSignal)
//        assert(glSignal->getSignal().size()==1);
//      else
//        */
//        gl=line->getRigidHLine()->getLength();
//    }
//    else if (stage==MBSim::plot) {
//      if (s1vSignal)
//        plotColumns->push_back(name+"_Surface1Velocity [m/s]");
//      if (s2vSignal)
//        plotColumns->push_back(name+"_Surface2Velocity [m/s]");
//      if (glSignal)
//        plotColumns->push_back(name+"_GapLength [mm]");
//      plotColumns->push_back(name+"_dpQ [bar]");
//      if (xdfac!=0)
//        plotColumns->push_back(name+"_dpxd [bar]");
//      PressureLoss::init(stage, plotColumns);
//    }
//    else
//      PressureLoss::init(stage, plotColumns);
//  }
//
//  void LeakagePressureLoss::update(const double &Q) {
//    PressureLoss::update(Q);
//    if (s1vSignal)
//      s1v=s1vSignal->getSignal()(0);
//    if (s2vSignal)
//      s2v=s2vSignal->getSignal()(0);
//    if (glSignal)
//      gl=glSignal->getSignal()(0);
//  }
//
//  void LeakagePressureLoss::plot(vector<double>* plotVector) {
//    if (s1vSignal)
//      plotVector->push_back(s1v);
//    if (s2vSignal)
//      plotVector->push_back(s2v);
//    if (glSignal)
//      plotVector->push_back(gl*1e3);
//    plotVector->push_back(dpQ*1e-5);
//    if (xdfac!=0)
//      plotVector->push_back(dpxd*1e-5);
//    PressureLoss::plot(plotVector);
//  }
//  
//  void LeakagePressureLoss::initializeUsingXML(TiXmlElement * element) {
//    PressureLoss::initializeUsingXML(element);
//    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"firstSurfaceVelocity");
//    if (e)
//      s1vPath=e->Attribute("ref");
//    e=element->FirstChildElement(MBSIMHYDRAULICSNS"secondSurfaceVelocity");
//    if (e)
//      s2vPath=e->Attribute("ref");
//    e=element->FirstChildElement(MBSIMHYDRAULICSNS"gapLength");
//    if (e)
//      glPath=e->Attribute("ref");
//  }
//
//
//  void PlaneLeakagePressureLoss::init(InitStage stage, vector<string>* plotColumns) {
//    if (stage==MBSim::resize) {
//      LeakagePressureLoss::init(stage, plotColumns);
//      double h=static_cast<PlaneLeakage*>(line->getRigidHLine())->getGapHeight();
//      double b=static_cast<PlaneLeakage*>(line->getRigidHLine())->getGapWidth();
//      double eta=HydraulicEnvironment::getInstance()->getDynamicViscosity();
//      // vgl. E. Becker: Technische Strömungslehre, (6. 13)
//      qfac = 12.*eta/b/h/h/h;
//      xdfac = -6.*eta/h/h;
//    }
//    else
//      LeakagePressureLoss::init(stage, plotColumns);
//  }
//
//  double PlaneLeakagePressureLoss::operator()(const double& Q, const void *) {
//    if (abs(Q)<1e-7) {
//      dpQ=0;
//      dpxd=0;
//      pLoss=0;
//    }
//    else {
//    dpQ=qfac*abs(gl)*Q;
//    dpxd=xdfac*abs(gl)*(s1v+s2v);
//    pLoss=dpQ+dpxd;
//    }
//    return pLoss;
//  }
//
//
//  void CircularLeakagePressureLoss::init(InitStage stage, vector<string>* plotColumns) {
//    if (stage==MBSim::resize) {
//      LeakagePressureLoss::init(stage, plotColumns);
//      if (!glSignal)
//        gl=static_cast<CircularLeakage*>(line->getRigidHLine())->getLength();
//      rI=static_cast<CircularLeakage*>(line->getRigidHLine())->getInnerRadius();
//      rO=static_cast<CircularLeakage*>(line->getRigidHLine())->getOuterRadius();
//      hGap=static_cast<CircularLeakage*>(line->getRigidHLine())->getGapHeight();
//    }
//    else
//      LeakagePressureLoss::init(stage, plotColumns);
//  }
//
//
//  void EccentricCircularLeakagePressureLoss::init(InitStage stage, vector<string> * plotColumns) {
//    if (stage==MBSim::resize) {
//      CircularLeakagePressureLoss::init(stage, plotColumns);
//      double area=M_PI*(rO*rO-rI*rI);
//      double alpha=area*hGap*hGap/12./HydraulicEnvironment::getInstance()->getKinematicViscosity()*(1.+1.5*ecc*ecc*ecc);
//      qfac=1./alpha/area;
//      xdfac=-1./2./alpha;
//    }
//    else
//      CircularLeakagePressureLoss::init(stage, plotColumns);
//  }
//
//  double EccentricCircularLeakagePressureLoss::operator()(const double& Q, const void *) {
//    dpQ=qfac*abs(gl)*Q;
//    dpxd=xdfac*abs(gl)*(s1v+s2v);
//    pLoss=dpQ+dpxd;
//    return pLoss;
//  }
//
//  void EccentricCircularLeakagePressureLoss::initializeUsingXML(TiXmlElement * element) {
//    CircularLeakagePressureLoss::initializeUsingXML(element);
//    TiXmlElement * e;
//    e=element->FirstChildElement(MBSIMHYDRAULICSNS"eccentricity");
//    setEccentricity(Element::getDouble(e));
//  }
//
//
//  void RealCircularLeakagePressureLoss::init(InitStage stage, vector<string> * plotColumns) {
//    if (stage==MBSim::resize) {
//      CircularLeakagePressureLoss::init(stage, plotColumns);
//      vIfac=rI*rI*M_PI;
//      vOfac=rO*rO*M_PI;
//      vIOfac=(vOfac-vIfac)/(2.*log(rO/rI));
//      qfac=1./(1./gl*M_PI/8./HydraulicEnvironment::getInstance()->getDynamicViscosity()*(rO*rO*rO*rO-rI*rI*rI*rI-(rO*rO-rI*rI)*(rO*rO-rI*rI)/log(rO/rI)));
//      vIfac*=-qfac;
//      vOfac*=-qfac;
//      vIOfac*=-qfac;
//    }
//    else
//      CircularLeakagePressureLoss::init(stage, plotColumns);
//  }
//
//  double RealCircularLeakagePressureLoss::operator()(const double& Q, const void *) {
//    dpQ=qfac*abs(gl)*Q;
//    double vI=s1v;
//    double vO=s2v;
//    dpxd=vOfac*vO-vIfac*vI-(vO-vI)*vIOfac;
//    pLoss=dpQ+dpxd;
//    return pLoss;
//  }
//
//  //  void PositiveFlowLimittingPressureLoss::update(const double& Q) {
//  //    QLimit=checkSizeSignal->getSignal()(0);
//  //    setClosed(Q>QLimit);
//  //  }
//  //
//  //  double RegularizedPositiveFlowLimittingPressureLoss::operator()(const double& Q, const void *) {
//  //    if (isClosed()) {
//  //      zeta = zeta1 * ((Q>QLimit+offset) ? 1. : (Q-QLimit)/offset * zeta1);
//  //      pLoss = 1.e4;
//  //    }
//  //    else {
//  //      zeta = 0;
//  //      pLoss = 0;
//  //    }
//  //    return pLoss;
//  //  }
//  //
//  //  void NegativeFlowLimittingPressureLoss::update(const double& Q) {
//  //    setClosed(Q<checkSizeSignal->getSignal()(0));
//  //  }

}

