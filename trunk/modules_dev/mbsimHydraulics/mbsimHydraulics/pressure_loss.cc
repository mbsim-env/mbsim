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
#include "mbsim/userfunction.h"
#include "environment.h"

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
    assert(aPos>0);
    assert(bPos>0);
    assert(aNeg>0);
    assert(bNeg>0);
  }

  void PressureLossCurveFit::transferLineData(double d, double l) {
    ReynoldsFactor/=HydraulicEnvironment::getInstance()->getKinematicViscosity();
  }

  void PressureLossCurveFit::initPlot(vector<string>* plotColumns) {
    PressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_ReynoldsNumber");
  }

  void PressureLossCurveFit::plot(vector<double>* plotVector) {
    PressureLoss::plot(plotVector);
    plotVector->push_back(Re);
  }
  
  void PressureLossVar::initPlot(vector<string>* plotColumns) {
    PressureLoss::initPlot(plotColumns);
    plotColumns->push_back(name+"_closed");
  }

  void PressureLossVar::plot(vector<double>* plotVector) {
    PressureLoss::plot(plotVector);
    plotVector->push_back(closed);
  }
  
  PressureLossVarAreaZeta::PressureLossVarAreaZeta(const string &name, double zeta, UserFunction * relAreaFun_, double minRelArea_) : PressureLossVar(name) {
    zetaFac=zeta;
    relAreaFun=relAreaFun_;
    minRelArea=minRelArea_;
  }

  void PressureLossVarAreaZeta::transferLineData(double d, double l) {
    double area=M_PI*d*d/4.;
    zetaFac*=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void PressureLossVarAreaZeta::updateg(double t) {
    relArea = (*relAreaFun)(t)(0);
    setClosed(relArea<minRelArea);
    relArea = (isClosed() ? minRelArea : relArea);
  }

  void PressureLossVarAreaZeta::initPlot(vector<string>* plotColumns) {
    PressureLossVar::initPlot(plotColumns);
    plotColumns->push_back(name+"_relArea");
  }

  void PressureLossVarAreaZeta::plot(vector<double>* plotVector) {
    PressureLossVar::plot(plotVector);
    plotVector->push_back(relArea);
  }

  void PressureLossVarCheckvalve::initPlot(vector<string>* plotColumns) {
    PressureLossVar::initPlot(plotColumns);
    plotColumns->push_back(name+"_xOpen");
  }

  void PressureLossVarCheckvalve::plot(vector<double>* plotVector) {
    PressureLossVar::plot(plotVector);
    plotVector->push_back(xOpen);
  }

  
  void PressureLossVarCheckvalveGamma::transferLineData(double d, double l) {
    zetaFac*=HydraulicEnvironment::getInstance()->getSpecificMass()/2.;
  }

  void PressureLossVarCheckvalveGamma::setXOpen(double xOpen_) {
    PressureLossVarCheckvalve::setXOpen(xOpen_);
    area=M_PI*xOpen*siga*(2.*rBall*coga+xOpen);
  }

  void PressureLossVarCheckvalveGamma::initPlot(vector<string>* plotColumns) {
    PressureLossVarCheckvalve::initPlot(plotColumns);
    plotColumns->push_back(name+"_area");
  }

  void PressureLossVarCheckvalveGamma::plot(vector<double>* plotVector) {
    PressureLossVarCheckvalve::plot(plotVector);
    plotVector->push_back(area);
  }

  
  void PressureLossVarCheckvalveIdelchick::transferLineData(double d, double l) {
    double area=M_PI*d*d*.25;
    d0=d;
    zetaFac=HydraulicEnvironment::getInstance()->getSpecificMass()/2./area/area;
  }

  void PressureLossVarCheckvalveIdelchick::setXOpen(double xOpen_) {
    PressureLossVarCheckvalve::setXOpen(xOpen_);
    hdivd0=xOpen/d0;
    beta2=.8/hdivd0;
    beta3=.14/hdivd0/hdivd0; // TODO
  }

  void PressureLossVarCheckvalveIdelchick::initPlot(vector<string>* plotColumns) {
    PressureLossVarCheckvalve::initPlot(plotColumns);
    plotColumns->push_back(name+"_hdivd0");
    plotColumns->push_back(name+"_beta2");
    plotColumns->push_back(name+"_beta3");
  }

  void PressureLossVarCheckvalveIdelchick::plot(vector<double>* plotVector) {
    PressureLossVarCheckvalve::plot(plotVector);
    plotVector->push_back(hdivd0);
    plotVector->push_back(beta2);
    plotVector->push_back(beta3);
  }

}

