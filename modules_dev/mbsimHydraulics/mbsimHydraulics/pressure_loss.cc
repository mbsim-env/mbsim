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
#include "mbsimHydraulics/hydline_pressureloss.h"
#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydleakage.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimControl/signal_.h"

using namespace std;

namespace MBSim {

  void PressureLoss::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::plot) 
      plotColumns->push_back(name+"_pLoss [bar]");
  }

  void PressureLoss::plot(vector<double>* plotVector) {
    plotVector->push_back(pLoss*1e-5);
  }


  void PressureLossZeta::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      PressureLoss::init(stage, plotColumns);
      double d=static_cast<RigidLine*>(line->getRigidHLine())->getDiameter();
      double area=M_PI*d*d/4.;
      lossFactor=zeta*HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
    }
    else
      PressureLoss::init(stage, plotColumns);
  }

  void PressureLossZeta::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"zeta");
    zeta=atof(e->GetText());
  }


  void PressureLossLaminarTubeFlow::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      PressureLoss::init(stage, plotColumns);
      double d=static_cast<RigidLine*>(line->getRigidHLine())->getDiameter();
      double l=static_cast<RigidLine*>(line->getRigidHLine())->getLength();
      double area=M_PI*d*d/4.;
      lossFactor=32.*HydraulicEnvironment::getInstance()->getDynamicViscosity()*l/d/d/area;
    }
    else
      PressureLoss::init(stage, plotColumns);
  }


  void PressureLossCurveFit::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      PressureLoss::init(stage, plotColumns);
      double d=static_cast<RigidLine*>(line->getRigidHLine())->getDiameter();
      ReynoldsFactor=dHyd/(M_PI*dRef*dRef/4.)/HydraulicEnvironment::getInstance()->getKinematicViscosity();
      aNeg = (aNeg>0)?aNeg:aPos;
      bNeg = (bNeg>0)?bNeg:bPos;
      assert(aPos>=0);
      assert(bPos>=0);
      assert(aNeg>=0);
      assert(bNeg>=0);
    }
    if (stage==MBSim::plot) {
      PressureLoss::init(stage, plotColumns);
      plotColumns->push_back(name+"_ReynoldsNumber");
    }
    else
      PressureLoss::init(stage, plotColumns);
  }

  double PressureLossCurveFit::operator()(const double &Q, const void *) {
    Re=ReynoldsFactor*Q; 
    pLoss=Re*((Re>0)?aPos+bPos*Re:aNeg-bNeg*Re); 
    return pLoss; 
  }

  void PressureLossCurveFit::plot(vector<double>* plotVector) {
    PressureLoss::plot(plotVector);
    plotVector->push_back(Re);
  }

  void PressureLossCurveFit::initializeUsingXML(TiXmlElement * element) {
    PressureLoss::initializeUsingXML(element);
    dRef=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"referenceDiameter")->GetText());
    dHyd=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"hydraulicDiameter")->GetText());
    aPos=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"aPositive")->GetText());
    bPos=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"bPositive")->GetText());
    aNeg=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"aNegative")->GetText());
    bNeg=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"bNegative")->GetText());
  }


  void VariablePressureLoss::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resolveXMLPath) {
      PressureLoss::init(stage, plotColumns);
      if (signalPath!="")  
//        checkSizeSignal=getSignalByPath(line, signalPath);
        checkSizeSignal=getSignalByPath(line->getParent(), signalPath);
   }
    else if (stage==MBSim::plot) {
      PressureLoss::init(stage, plotColumns);
      plotColumns->push_back(name+"_checksizeSignal");
      plotColumns->push_back(name+"_closed");
      plotColumns->push_back(name+"_checkValue");
    }
    else
      PressureLoss::init(stage, plotColumns);
  }

  void VariablePressureLoss::update(const double &Q) {
    checkValue = checkSizeSignal->getSignal()(0);
    closed=(checkValue<minValue);
    if (closed)
      checkValue=minValue;
  }

  void VariablePressureLoss::plot(vector<double>* plotVector) {
    PressureLoss::plot(plotVector);
    plotVector->push_back(checkSizeSignal->getSignal()(0));
    plotVector->push_back(closed);
    plotVector->push_back(checkValue);
  }

  void VariablePressureLoss::initializeUsingXML(TiXmlElement * element) {
    PressureLoss::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"checksizeSignal");
    signalPath=e->Attribute("ref");
    minValue=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"minimalChecksizeValue")->GetText());
  }


  void VariablePressureLossAreaZeta::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      VariablePressureLoss::init(stage, plotColumns);
      double d=static_cast<RigidLine*>(line->getRigidHLine())->getDiameter();
      double area=M_PI*d*d/4.;
      zetaFac=zeta*HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
    }
    else
      VariablePressureLoss::init(stage, plotColumns);
  }

  void VariablePressureLossAreaZeta::update(const double& Q) {
    VariablePressureLoss::update(Q);
    checkValue = (checkValue>1.)?1.:checkValue;
  }

  double VariablePressureLossAreaZeta::operator()(const double& Q, const void *) {
    pLoss=zetaFac*Q*fabs(Q)/checkValue/checkValue; 
    return pLoss; 
  }

  void VariablePressureLossAreaZeta::initializeUsingXML(TiXmlElement * element) {
    VariablePressureLoss::initializeUsingXML(element);
    zeta=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"zeta")->GetText());
  }


  void VariablePressureLossControlvalveAreaAlpha::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      VariablePressureLoss::init(stage, plotColumns);
      double d=static_cast<RigidLine*>(line->getRigidHLine())->getDiameter();
      assert(alpha>1e-2);
      assert(alpha<=1.);
      alpha2=alpha*alpha;
      double area=M_PI*d*d/4.;
      factor=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;

    }
    else if (stage==MBSim::plot) {
      VariablePressureLoss::init(stage, plotColumns);
      plotColumns->push_back(name+"_zeta");
    }
    else
      VariablePressureLoss::init(stage, plotColumns);
  }

  void VariablePressureLossControlvalveAreaAlpha::update(const double& Q) {
    VariablePressureLoss::update(Q);
    checkValue = (checkValue>1.)?1.:checkValue;
    zeta=(1./alpha2/checkValue/checkValue - 1.);
  }

  double VariablePressureLossControlvalveAreaAlpha::operator()(const double& Q, const void *) {
    pLoss=factor*Q*fabs(Q)*zeta; 
    return pLoss; 
  }

  void VariablePressureLossControlvalveAreaAlpha::plot(vector<double>* plotVector) {
    VariablePressureLoss::plot(plotVector);
    plotVector->push_back(zeta);
  }

  void VariablePressureLossControlvalveAreaAlpha::initializeUsingXML(TiXmlElement * element) {
    VariablePressureLoss::initializeUsingXML(element);
    alpha=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"alpha")->GetText());
  }

  void VariablePressureLossCheckvalve::initializeUsingXML(TiXmlElement * element) {
    VariablePressureLoss::initializeUsingXML(element);
    rBall=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"ballRadius")->GetText());
  }


  void VariablePressureLossCheckvalveGamma::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      VariablePressureLossCheckvalve::init(stage, plotColumns);
      siga=sin(gamma); 
      coga=cos(gamma); 
      zetaFac=1./alpha/alpha*coga*coga*coga*coga*HydraulicEnvironment::getInstance()->getSpecificMass()/2.;
    }
    else if (stage==MBSim::plot) {
      VariablePressureLossCheckvalve::init(stage, plotColumns);
      plotColumns->push_back(name+"_area");
    }
    else
      VariablePressureLossCheckvalve::init(stage, plotColumns);
  }

  void VariablePressureLossCheckvalveGamma::update(const double& Q) {
    VariablePressureLossCheckvalve::update(Q);
    area=M_PI*checkValue*siga*(2.*rBall*coga+checkValue);
  }

  double VariablePressureLossCheckvalveGamma::operator()(const double& Q, const void *) {
    pLoss=zetaFac*Q*fabs(Q)/area/area; 
    return pLoss; 
  }

  void VariablePressureLossCheckvalveGamma::plot(vector<double>* plotVector) {
    VariablePressureLossCheckvalve::plot(plotVector);
    plotVector->push_back(area);
  }

  void VariablePressureLossCheckvalveGamma::initializeUsingXML(TiXmlElement * element) {
    VariablePressureLossCheckvalve::initializeUsingXML(element);
    alpha=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"alpha")->GetText());
    gamma=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"gamma")->GetText());
  }


  void VariablePressureLossCheckvalveIdelchick::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      VariablePressureLossCheckvalve::init(stage, plotColumns);
      double d=static_cast<RigidLine*>(line->getRigidHLine())->getDiameter();
      double area=M_PI*d*d*.25;
      d0=d;
      zetaFac=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
    }
    else if (stage==MBSim::plot) {
      VariablePressureLossCheckvalve::init(stage, plotColumns);
      plotColumns->push_back(name+"_hdivd0");
      plotColumns->push_back(name+"_beta2");
      plotColumns->push_back(name+"_beta3");
    }
    else
      VariablePressureLossCheckvalve::init(stage, plotColumns);
  }

  void VariablePressureLossCheckvalveIdelchick::update(const double& Q) {
    VariablePressureLossCheckvalve::update(Q);
    hdivd0=checkValue/d0;
    beta2=.8/hdivd0;
    beta3=.14/hdivd0/hdivd0; // TODO
  }

  void VariablePressureLossCheckvalveIdelchick::plot(vector<double>* plotVector) {
    VariablePressureLossCheckvalve::plot(plotVector);
    plotVector->push_back(hdivd0);
    plotVector->push_back(beta2);
    plotVector->push_back(beta3);
  }


  void VariablePressureLossCheckvalveCone::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      VariablePressureLossCheckvalve::init(stage, plotColumns);
      double d=static_cast<RigidLine*>(line->getRigidHLine())->getDiameter();
      double rOpen=d/2.;
      double rBall2=rBall*rBall;
      double rOpen2=rOpen*rOpen;
      zetaFac=HydraulicEnvironment::getInstance()->getSpecificMass()/2./alpha/alpha/M_PI/M_PI;
      numer[0]=rBall2;
      numer[1]=2.*sqrt(rBall2-rOpen2);
      denom[0]=4.*(rBall2-rOpen2);
      denom[1]=4.*sqrt(rBall2-rOpen2);
    }
    else if (stage==MBSim::plot) {
      VariablePressureLossCheckvalve::init(stage, plotColumns);
      plotColumns->push_back(name+"_factor");
    }
    else
      VariablePressureLossCheckvalve::init(stage, plotColumns);
  }

  void VariablePressureLossCheckvalveCone::update(const double& Q) {
    VariablePressureLossCheckvalve::update(Q);
    double xOpen=checkValue;
    double xOpen2=checkValue*checkValue;
    double xOpen4=xOpen2*xOpen2;
    factor=zetaFac*(xOpen2+xOpen*numer[1]+numer[0])/(xOpen2+xOpen*denom[1]+denom[0])/xOpen4;
  }

  void VariablePressureLossCheckvalveCone::plot(vector<double>* plotVector) {
    VariablePressureLossCheckvalve::plot(plotVector);
    plotVector->push_back(factor);
  }

  void VariablePressureLossCheckvalveCone::initializeUsingXML(TiXmlElement * element) {
    VariablePressureLossCheckvalve::initializeUsingXML(element);
    alpha=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"alpha")->GetText());
  }


  void LeakagePressureLoss::init(InitStage stage, vector<string> * plotColumns) {
    if (stage==MBSim::resolveXMLPath) {
      PressureLoss::init(stage, plotColumns);
      if (s1vPath!="")  
        setSurface1Velocity(getSignalByPath(line->getParent(), s1vPath));
//        setSurface1Velocity(getSignalByPath(line, s1vPath));
      if (s2vPath!="")  
        setSurface2Velocity(getSignalByPath(line->getParent(), s2vPath));
//        setSurface2Velocity(getSignalByPath(line, s2vPath));
      if (glPath!="")  
        setGapLength(getSignalByPath(line->getParent(), glPath));
//        setGapLength(getSignalByPath(line, glPath));
    }
    else if (stage==MBSim::resize) {
      PressureLoss::init(stage, plotColumns);
      /*
      if (s1vSignal)
        assert(s1vSignal->getSignal().size()==1);
      if (s2vSignal)
        assert(s2vSignal->getSignal().size()==1);
      if (glSignal)
        assert(glSignal->getSignal().size()==1);
      else
        */
        gl=line->getRigidHLine()->getLength();
    }
    else if (stage==MBSim::plot) {
      if (s1vSignal)
        plotColumns->push_back(name+"_Surface1Velocity [m/s]");
      if (s2vSignal)
        plotColumns->push_back(name+"_Surface2Velocity [m/s]");
      if (glSignal)
        plotColumns->push_back(name+"_GapLength [mm]");
      plotColumns->push_back(name+"_dpQ [bar]");
      if (xdfac!=0)
        plotColumns->push_back(name+"_dpxd [bar]");
      PressureLoss::init(stage, plotColumns);
    }
    else
      PressureLoss::init(stage, plotColumns);
  }

  void LeakagePressureLoss::update(const double &Q) {
    PressureLoss::update(Q);
    if (s1vSignal)
      s1v=s1vSignal->getSignal()(0);
    if (s2vSignal)
      s2v=s2vSignal->getSignal()(0);
    if (glSignal)
      gl=glSignal->getSignal()(0);
  }

  void LeakagePressureLoss::plot(vector<double>* plotVector) {
    if (s1vSignal)
      plotVector->push_back(s1v);
    if (s2vSignal)
      plotVector->push_back(s2v);
    if (glSignal)
      plotVector->push_back(gl*1e3);
    plotVector->push_back(dpQ*1e-5);
    if (xdfac!=0)
      plotVector->push_back(dpxd*1e-5);
    PressureLoss::plot(plotVector);
  }
  
  void LeakagePressureLoss::initializeUsingXML(TiXmlElement * element) {
    PressureLoss::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"firstSurfaceVelocity");
    if (e)
      s1vPath=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"secondSurfaceVelocity");
    if (e)
      s2vPath=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"gapLength");
    if (e)
      glPath=e->Attribute("ref");
  }


  void PlaneLeakagePressureLoss::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      LeakagePressureLoss::init(stage, plotColumns);
      double h=static_cast<PlaneLeakage*>(line->getRigidHLine())->getGapHeight();
      double b=static_cast<PlaneLeakage*>(line->getRigidHLine())->getGapWidth();
      double eta=HydraulicEnvironment::getInstance()->getDynamicViscosity();
      // vgl. E. Becker: Technische Strömungslehre, (6. 13)
      qfac = 12.*eta/b/h/h/h;
      xdfac = -6.*eta/h/h;
    }
    else
      LeakagePressureLoss::init(stage, plotColumns);
  }

  double PlaneLeakagePressureLoss::operator()(const double& Q, const void *) {
    if (abs(Q)<1e-7) {
      dpQ=0;
      dpxd=0;
      pLoss=0;
    }
    else {
    dpQ=qfac*abs(gl)*Q;
    dpxd=xdfac*abs(gl)*(s1v+s2v);
    pLoss=dpQ+dpxd;
    }
    return pLoss;
  }


  void CircularLeakagePressureLoss::init(InitStage stage, vector<string>* plotColumns) {
    if (stage==MBSim::resize) {
      LeakagePressureLoss::init(stage, plotColumns);
      if (!glSignal)
        gl=static_cast<CircularLeakage*>(line->getRigidHLine())->getLength();
      rI=static_cast<CircularLeakage*>(line->getRigidHLine())->getInnerRadius();
      rO=static_cast<CircularLeakage*>(line->getRigidHLine())->getOuterRadius();
      hGap=static_cast<CircularLeakage*>(line->getRigidHLine())->getGapHeight();
    }
    else
      LeakagePressureLoss::init(stage, plotColumns);
  }


  void EccentricCircularLeakagePressureLoss::init(InitStage stage, vector<string> * plotColumns) {
    if (stage==MBSim::resize) {
      CircularLeakagePressureLoss::init(stage, plotColumns);
      double area=M_PI*(rO*rO-rI*rI);
      double alpha=area*hGap*hGap/12./HydraulicEnvironment::getInstance()->getKinematicViscosity()*(1.+1.5*ecc*ecc*ecc);
      qfac=1./alpha/area;
      xdfac=-1./2./alpha;
    }
    else
      CircularLeakagePressureLoss::init(stage, plotColumns);
  }

  double EccentricCircularLeakagePressureLoss::operator()(const double& Q, const void *) {
    dpQ=qfac*abs(gl)*Q;
    dpxd=xdfac*abs(gl)*(s1v+s2v);
    pLoss=dpQ+dpxd;
    return pLoss;
  }

  void EccentricCircularLeakagePressureLoss::initializeUsingXML(TiXmlElement * element) {
    CircularLeakagePressureLoss::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"eccentricity");
    setEccentricity(atof(e->GetText()));
  }


  void RealCircularLeakagePressureLoss::init(InitStage stage, vector<string> * plotColumns) {
    if (stage==MBSim::resize) {
      CircularLeakagePressureLoss::init(stage, plotColumns);
      vIfac=rI*rI*M_PI;
      vOfac=rO*rO*M_PI;
      vIOfac=(vOfac-vIfac)/(2.*log(rO/rI));
      qfac=1./(1./gl*M_PI/8./HydraulicEnvironment::getInstance()->getDynamicViscosity()*(rO*rO*rO*rO-rI*rI*rI*rI-(rO*rO-rI*rI)*(rO*rO-rI*rI)/log(rO/rI)));
      vIfac*=-qfac;
      vOfac*=-qfac;
      vIOfac*=-qfac;
    }
    else
      CircularLeakagePressureLoss::init(stage, plotColumns);
  }

  double RealCircularLeakagePressureLoss::operator()(const double& Q, const void *) {
    dpQ=qfac*abs(gl)*Q;
    double vI=s1v;
    double vO=s2v;
    dpxd=vOfac*vO-vIfac*vI-(vO-vI)*vIOfac;
    pLoss=dpQ+dpxd;
    return pLoss;
  }

  //  void PositiveFlowLimittingPressureLoss::update(const double& Q) {
  //    QLimit=checkSizeSignal->getSignal()(0);
  //    setClosed(Q>QLimit);
  //  }
  //
  //  double RegularizedPositiveFlowLimittingPressureLoss::operator()(const double& Q, const void *) {
  //    if (isClosed()) {
  //      zeta = zeta1 * ((Q>QLimit+offset) ? 1. : (Q-QLimit)/offset * zeta1);
  //      pLoss = 1.e4;
  //    }
  //    else {
  //      zeta = 0;
  //      pLoss = 0;
  //    }
  //    return pLoss;
  //  }
  //
  //  void NegativeFlowLimittingPressureLoss::update(const double& Q) {
  //    setClosed(Q<checkSizeSignal->getSignal()(0));
  //  }

}

