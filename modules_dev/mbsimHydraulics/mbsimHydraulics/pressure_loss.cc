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

#include "pressure_loss.h"
#include "environment.h"
#include "mbsimControl/signal_.h"

using namespace std;

namespace MBSim {

  void PressureLoss::initPlot(vector<string>* plotColumns) {
    plotColumns->push_back(name+"_pLoss [bar]");
  }

  void PressureLoss::plot(vector<double>* plotVector) {
    plotVector->push_back(pLoss*1e-5);
  }

  void PressureLossZeta::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    lossFactor*=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void PressureLossLaminarTubeFlow::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    lossFactor=32.*HydraulicEnvironment::getInstance()->getDynamicViscosity()*l/d/d/area;
  }

  PressureLossCurveFit::PressureLossCurveFit(const string &name, double dRef, double dHyd, double aPos_, double bPos_, double aNeg_, double bNeg_) : PressureLoss(name) {
    ReynoldsFactor=dHyd/(M_PI*dRef*dRef/4.);
    aPos=aPos_;
    bPos=bPos_;
    aNeg = (aNeg_>0)?aNeg_:aPos_;
    bNeg = (bNeg_>0)?bNeg_:bPos_;
    assert(aPos>=0);
    assert(bPos>=0);
    assert(aNeg>=0);
    assert(bNeg>=0);
  }

  void PressureLossCurveFit::transferLineData(double d, double l) {
    ReynoldsFactor/=HydraulicEnvironment::getInstance()->getKinematicViscosity();
  }

  double PressureLossCurveFit::operator()(const double &Q, const void *) {
    Re=ReynoldsFactor*Q; 
    pLoss=Re*((Re>0)?aPos+bPos*Re:aNeg-bNeg*Re); 
    return pLoss; 
  }

  void PressureLossCurveFit::initPlot(vector<string>* plotColumns) {
    PressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_ReynoldsNumber");
  }

  void PressureLossCurveFit::plot(vector<double>* plotVector) {
    PressureLoss::plot(plotVector);
    plotVector->push_back(Re);
  }

  void VariablePressureLoss::initPlot(vector<string>* plotColumns) {
    PressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_closed");
  }

  void VariablePressureLoss::plot(vector<double>* plotVector) {
    PressureLoss::plot(plotVector);
    plotVector->push_back(closed);
  }

  VariablePressureLossAreaZeta::VariablePressureLossAreaZeta(const string &name, double zeta, double minRelArea_, Signal * checkSizeSignal) : VariablePressureLoss(name, checkSizeSignal) {
    zetaFac=zeta;
    minRelArea=minRelArea_;
  }

  void VariablePressureLossAreaZeta::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    zetaFac*=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void VariablePressureLossAreaZeta::update(const double& Q) {
    relArea = checkSizeSignal->getSignal()(0);
    setClosed(relArea<minRelArea);
    relArea = (isClosed() ? minRelArea : relArea);
  }

  double VariablePressureLossAreaZeta::operator()(const double& Q, const void *) {
    pLoss=zetaFac*Q*fabs(Q)/relArea/relArea; 
    return pLoss; 
  }

  void VariablePressureLossAreaZeta::initPlot(vector<string>* plotColumns) {
    VariablePressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_relArea");
  }

  void VariablePressureLossAreaZeta::plot(vector<double>* plotVector) {
    VariablePressureLoss::plot(plotVector);
    plotVector->push_back(relArea);
  }
  
  VariablePressureLossControlvalveAreaAlpha::VariablePressureLossControlvalveAreaAlpha(const string &name, double alpha, double minRelArea_, Signal * checkSizeSignal) : VariablePressureLoss(name, checkSizeSignal), minRelArea(minRelArea_) {
    assert(alpha>1e-2);
    assert(alpha<=1.);
    alpha2=alpha*alpha;
  }

  void VariablePressureLossControlvalveAreaAlpha::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    factor=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void VariablePressureLossControlvalveAreaAlpha::update(const double& Q) {
    relArea = checkSizeSignal->getSignal()(0);
    setClosed(relArea<minRelArea);
    relArea = (isClosed() ? minRelArea : relArea);
    if (relArea>1.) {
      cout << "WARNING! VariablePressureLossControlvalveAreaAlpha \"" << name << "\": relArea > 1., setting back to 1." << endl;
      relArea = 1.;
    }
  }

  double VariablePressureLossControlvalveAreaAlpha::operator()(const double& Q, const void *) {
    zeta=(1./alpha2/relArea/relArea - 1.);
    pLoss=factor*Q*fabs(Q)*zeta; 
    return pLoss; 
  }

  void VariablePressureLossControlvalveAreaAlpha::initPlot(vector<string>* plotColumns) {
    VariablePressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_relArea");
    plotColumns->push_back(name+"_zeta");
  }

  void VariablePressureLossControlvalveAreaAlpha::plot(vector<double>* plotVector) {
    VariablePressureLoss::plot(plotVector);
    plotVector->push_back(relArea);
    plotVector->push_back(zeta);
  }

  void VariablePressureLossCheckvalve::update(const double& Q) {
    xOpen=checkSizeSignal->getSignal()(0);
    setClosed(xOpen<minimalXOpen);
    if (isClosed())
      xOpen=minimalXOpen;
  }

  void VariablePressureLossCheckvalve::initPlot(vector<string>* plotColumns) {
    VariablePressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_xOpen");
  }

  void VariablePressureLossCheckvalve::plot(vector<double>* plotVector) {
    VariablePressureLoss::plot(plotVector);
    plotVector->push_back(xOpen);
  }


  void VariablePressureLossCheckvalveGamma::transferLineData(double d, double l) {
    zetaFac*=HydraulicEnvironment::getInstance()->getSpecificMass()/2.;
  }

  void VariablePressureLossCheckvalveGamma::update(const double& Q) {
    VariablePressureLossCheckvalve::update(Q);
    area=M_PI*xOpen*siga*(2.*rBall*coga+xOpen);
  }

  double VariablePressureLossCheckvalveGamma::operator()(const double& Q, const void *) {
    pLoss=zetaFac*Q*fabs(Q)/area/area; 
    return pLoss; 
  }

  void VariablePressureLossCheckvalveGamma::initPlot(vector<string>* plotColumns) {
    VariablePressureLossCheckvalve::initPlot(plotColumns);
    plotColumns->push_back(name+"_area");
  }

  void VariablePressureLossCheckvalveGamma::plot(vector<double>* plotVector) {
    VariablePressureLossCheckvalve::plot(plotVector);
    plotVector->push_back(area);
  }


  void VariablePressureLossCheckvalveIdelchick::transferLineData(double d, double l) {
    double area=M_PI*d*d*.25;
    d0=d;
    zetaFac=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void VariablePressureLossCheckvalveIdelchick::update(const double& Q) {
    VariablePressureLossCheckvalve::update(Q);
    hdivd0=xOpen/d0;
    beta2=.8/hdivd0;
    beta3=.14/hdivd0/hdivd0; // TODO
  }

  void VariablePressureLossCheckvalveIdelchick::initPlot(vector<string>* plotColumns) {
    VariablePressureLossCheckvalve::initPlot(plotColumns);
    plotColumns->push_back(name+"_hdivd0");
    plotColumns->push_back(name+"_beta2");
    plotColumns->push_back(name+"_beta3");
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

