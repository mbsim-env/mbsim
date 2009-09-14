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
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/leakage_line.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"

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

  double RelativeAreaZetaClosablePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      double d=((const RigidLine*)(line))->getDiameter();
      double A=M_PI*d*d/4.;
      c*=rho/2./A/A;
      initialized=true;
    }
    const double areaRel=((const ClosableRigidLine*)(line))->getRegularizedValue();
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
    double alphaRel=((const ClosableRigidLine*)(line))->getRegularizedValue();
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
    const double xOpen=((const ClosableRigidLine*)(line))->getRegularizedValue();
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
    const double xOpen=((const ClosableRigidLine*)(line))->getRegularizedValue();
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
    const double xOpen=((const ClosableRigidLine*)(line))->getRegularizedValue();
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


  double PlaneLeakagePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double h=((const PlaneLeakageLine*)(line))->getGapHeight();
      double w=((const PlaneLeakageLine*)(line))->getGapWidth();
      double eta=HydraulicEnvironment::getInstance()->getDynamicViscosity();
      // vgl. E. Becker: Technische Strömungslehre, (6. 13)
      qfac = 12.*eta/w/h/h/h;
      xdfac = -6.*eta/h/h;
      initialized=true;
    }
    const double gl=((const PlaneLeakageLine*)(line))->getGapLength();
    const double s1v=((const PlaneLeakageLine*)(line))->getSurface1Velocity();
    const double s2v=((const PlaneLeakageLine*)(line))->getSurface2Velocity();
    return qfac*gl*Q + xdfac*gl*(s1v+s2v);
  }


  double EccentricCircularLeakagePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double rI=((const CircularLeakageLine*)(line))->getInnerRadius();
      double rO=((const CircularLeakageLine*)(line))->getOuterRadius();
      double hGap=((const CircularLeakageLine*)(line))->getGapHeight();
      double area=M_PI*(rO*rO-rI*rI);
      double nu=HydraulicEnvironment::getInstance()->getKinematicViscosity();
      double alpha=area*hGap*hGap/12./nu*(1.+1.5*ecc*ecc*ecc);
      qfac=1./alpha/area;
      xdfac=-1./2./alpha;
    }
    const double gl=((const CircularLeakageLine*)(line))->getGapLength();
    const double s1v=((const CircularLeakageLine*)(line))->getSurface1Velocity();
    const double s2v=((const CircularLeakageLine*)(line))->getSurface2Velocity();
    return qfac*gl*Q + xdfac*gl*(s1v+s2v);
  }

  void EccentricCircularLeakagePressureLoss::initializeUsingXML(TiXmlElement * element) {
    CircularLeakagePressureLoss::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"eccentricity");
    setEccentricity(Element::getDouble(e));
  }


  double RealCircularLeakagePressureLoss::operator()(const double& Q, const void * line) {
    if (!initialized) {
      double rI=((const CircularLeakageLine*)(line))->getInnerRadius();
      double rA=((const CircularLeakageLine*)(line))->getOuterRadius();
      double eta=HydraulicEnvironment::getInstance()->getDynamicViscosity();
      vIfac=-(-rI*rI+2.*rI*rI*log(rI)-2.*rI*rI*log(rA)+rA*rA)*M_PI/(log(rI)-log(rA))/2.;
      vOfac=(rA*rA-2.*rA*rA*log(rA)+2.*rA*rA*log(rI)-rI*rI)*M_PI/(log(rI)-log(rA))/2.;
      pVfac=(1.+log(rI)-log(rA))*M_PI/eta/(log(rI)-log(rA))*rA*rA*rA*rA/8.-rA*rA*M_PI/eta/(log(rI)-log(rA))*rI*rI/4.+(log(rA)-log(rI)+1.)*M_PI/eta/(log(rI)-log(rA))*rI*rI*rI*rI/8.;
      initialized=true;
    }
    // vgl. Spurk Stroemungslehre S.162, (6.65)
    const double gl=((const CircularLeakageLine*)(line))->getGapLength();
    const double vI=((const CircularLeakageLine*)(line))->getSurface1Velocity();
    const double vO=((const CircularLeakageLine*)(line))->getSurface2Velocity();
    return (Q-vOfac*vO-vIfac*vI)*gl/pVfac;
  }

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

