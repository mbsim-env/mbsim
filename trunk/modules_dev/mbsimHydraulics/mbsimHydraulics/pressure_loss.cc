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
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimControl/signal_.h"

using namespace std;

namespace MBSim {

  //  nach signal_.h verschoben...
  //  Signal * getSignalByPath(DynamicSystem * ds,  const string& cpath) {
  //    string path=cpath;
  //    int pos=path.find("Signal");
  //    path.erase(pos, 6);
  //    path.insert(pos, "Link");
  //    Link * s = ds->getLinkByPath(path);
  //    if (dynamic_cast<Signal *>(s))
  //      return static_cast<Signal *>(s);
  //    else {
  //      std::cerr << "ERROR! \"" << path << "\" is not of Signal-Type." << std::endl; 
  //      _exit(1);
  //    }
  //  }


  void PressureLoss::initPlot(vector<string>* plotColumns) {
    plotColumns->push_back(name+"_pLoss [bar]");
  }

  void PressureLoss::plot(vector<double>* plotVector) {
    plotVector->push_back(pLoss*1e-5);
  }


  void PressureLossZeta::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    lossFactor=zeta*HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void PressureLossZeta::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"zeta");
    zeta=atof(e->GetText());
  }


  void PressureLossLaminarTubeFlow::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    lossFactor=32.*HydraulicEnvironment::getInstance()->getDynamicViscosity()*l/d/d/area;
  }


  void PressureLossCurveFit::transferLineData(double d, double l) {
    ReynoldsFactor=dHyd/(M_PI*dRef*dRef/4.)/HydraulicEnvironment::getInstance()->getKinematicViscosity();
    aNeg = (aNeg>0)?aNeg:aPos;
    bNeg = (bNeg>0)?bNeg:bPos;
    assert(aPos>=0);
    assert(bPos>=0);
    assert(aNeg>=0);
    assert(bNeg>=0);
  }

  void PressureLossCurveFit::initPlot(vector<string>* plotColumns) {
    PressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_ReynoldsNumber");
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


  void VariablePressureLoss::initPlot(vector<string>* plotColumns) {
    PressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_checksizeSignal");
    plotColumns->push_back(name+"_closed");
    plotColumns->push_back(name+"_checkValue");
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
    checkSizeSignal=getSignalByPath(line->getParent(), e->Attribute("ref"));
    minValue=atof(element->FirstChildElement(MBSIMHYDRAULICSNS"minimalChecksizeValue")->GetText());
  }


  void VariablePressureLossAreaZeta::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    zetaFac=zeta*HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
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


  void VariablePressureLossControlvalveAreaAlpha::transferLineData(double d, double l) {
    assert(alpha>1e-2);
    assert(alpha<=1.);
    alpha2=alpha*alpha;
    double area=M_PI*d*d/4.;
    factor=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void VariablePressureLossControlvalveAreaAlpha::initPlot(vector<string>* plotColumns) {
    VariablePressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_zeta");
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


  void VariablePressureLossCheckvalveGamma::transferLineData(double d, double l) {
    siga=sin(gamma); 
    coga=cos(gamma); 
    zetaFac=1./alpha/alpha*coga*coga*coga*coga*HydraulicEnvironment::getInstance()->getSpecificMass()/2.;
  }

  void VariablePressureLossCheckvalveGamma::initPlot(vector<string>* plotColumns) {
    VariablePressureLossCheckvalve::initPlot(plotColumns);
    plotColumns->push_back(name+"_area");
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


  void VariablePressureLossCheckvalveIdelchick::transferLineData(double d, double l) {
    double area=M_PI*d*d*.25;
    d0=d;
    zetaFac=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void VariablePressureLossCheckvalveIdelchick::initPlot(vector<string>* plotColumns) {
    VariablePressureLossCheckvalve::initPlot(plotColumns);
    plotColumns->push_back(name+"_hdivd0");
    plotColumns->push_back(name+"_beta2");
    plotColumns->push_back(name+"_beta3");
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

