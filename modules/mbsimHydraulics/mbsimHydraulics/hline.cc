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

#include <config.h>
#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimControl/signal_.h"
#include "mbsim/frames/frame.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimHydraulics {

  void HLine::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Object::init(stage);
    }
    else if (stage==preInit) {
      Object::init(stage);
      if (!nFrom && !nFromRelative) 
        THROW_MBSIMERROR("No fromNode!");
      if (!nTo && !nToRelative) 
        THROW_MBSIMERROR("No toNode!");
      if (nFrom && nFrom==nTo) 
        THROW_MBSIMERROR("fromNode and toNode are the same!");
    }
    else
      Object::init(stage);
  }
      
  void HLine::setFrameOfReference(Frame *frame) {
    frameOfReference = frame; 
  }

  void HLine::initializeUsingXML(DOMElement * element) {
    Object::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"frameOfReference");
    if (e) {
      saved_frameOfReference=E(e)->getAttribute("ref");
      e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"direction");
      setDirection(getVec(e,3));
    }
  }

  void RigidHLine::addInflowDependencyOnOutflow(RigidHLine * line) {
    setInflowRelative(true);
    dependencyOnOutflow.push_back(line);
    line->setOutflowRelative(true);
  }

  void RigidHLine::addInflowDependencyOnInflow(RigidHLine * line) {
    setInflowRelative(true);
    dependencyOnInflow.push_back(line);
    line->setInflowRelative(true);
  }

  Mat RigidHLine::calculateJacobian(vector<RigidHLine*> dep_check) {
    for (unsigned int i=0; i<dep_check.size()-1; i++)
      if (this==dep_check[i])
        THROW_MBSIMERROR("Kinematic Loop in hydraulic system. Check model!");

    // TODO efficient calculation (not every loop is necessary)
    Mat JLocal;
    if(dependency.size()==0) {
      if(getuRelSize()==1)
        JLocal=Mat(1,1,INIT,1);
      else {
        JLocal=Mat(1,getuRelSize(),INIT,0);
        JLocal(0,uInd[0])=1;
      }
    }
    else {
      JLocal=Mat(1,getuRelSize(),INIT,0);
      dep_check.push_back(this);
      for (unsigned int i=0; i<dependencyOnOutflow.size(); i++) {
        const Mat Jdep=((RigidHLine*)dependencyOnOutflow[i])->calculateJacobian(dep_check);
        JLocal(0,RangeV(0,Jdep.cols()-1))+=Jdep;
      }
      for (unsigned int i=0; i<dependencyOnInflow.size(); i++) {
        const Mat Jdep=((RigidHLine*)dependencyOnInflow[i])->calculateJacobian(dep_check);
        JLocal(0,RangeV(0,Jdep.cols()-1))-=Jdep;
      }
    }
    return JLocal;
  }

  void RigidHLine::updateQ() {
    if(dependency.size()==0)
      QIn(0)=u(0);
    else {
      QIn.init(0);
      for (unsigned int i=0; i<dependencyOnOutflow.size(); i++)
        QIn+=(dependencyOnOutflow[i])->evalQIn();
      for (unsigned int i=0; i<dependencyOnInflow.size(); i++)
        QIn-=(dependencyOnInflow[i])->evalQIn();
    }
    QOut = -QIn;
    updQ = false;
  }

  void RigidHLine::updatePressureLossGravity() {
    if (frameOfReference)
      pressureLossGravity=-trans(frameOfReference->evalOrientation()*MBSimEnvironment::getInstance()->getAccelerationOfGravity())*direction*HydraulicEnvironment::getInstance()->getSpecificMass()*length;
    updPLG = false;
  }

  void RigidHLine::updateh(int j) {
    h[j]-=trans(Jacobian.row(0))*evalPressureLossGravity();
  }
      
  void RigidHLine::updateM() {
    M+=Mlocal(0,0)*JTJ(Jacobian);
  }

  void RigidHLine::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      for (unsigned int i=0; i<refDependencyOnInflowString.size(); i++)
        addInflowDependencyOnInflow(getByPath<RigidHLine>(refDependencyOnInflowString[i]));
      for (unsigned int i=0; i<refDependencyOnOutflowString.size(); i++)
        addInflowDependencyOnOutflow(getByPath<RigidHLine>(refDependencyOnOutflowString[i]));
      HLine::init(stage);
    }
    else if (stage==preInit) {
      HLine::init(stage);
      for (unsigned int i=0; i<dependencyOnInflow.size(); i++)
        dependency.push_back(dependencyOnInflow[i]);
      for (unsigned int i=0; i<dependencyOnOutflow.size(); i++)
        dependency.push_back(dependencyOnOutflow[i]);
      uRel.resize((dependency.size()?0:1));

      vector<RigidHLine *> dep_check;
      dep_check.push_back(this);
      Jacobian=calculateJacobian(dep_check);
    }
    else if(stage==plotting) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Volume flow [l/min]");
        plotColumns.push_back("Mass flow [kg/min]");
        if (frameOfReference)
          plotColumns.push_back("pressureLoss due to gravity [bar]");
        HLine::init(stage);
      }
    }
    else
      HLine::init(stage);
  }
  
  void RigidHLine::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(evalQIn()(0)*6e4);
      plotVector.push_back(getQIn()(0)*HydraulicEnvironment::getInstance()->getSpecificMass()*60.);
      if (frameOfReference)
        plotVector.push_back(evalPressureLossGravity()*1e-5);
      HLine::plot();
    }
  }

  void RigidHLine::initializeUsingXML(DOMElement * element) {
    HLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"inflowDependencyOnInflow");
    if (!e)
      e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"inflowDependencyOnOutflow");
    else {
      if (e->getPreviousElementSibling()) {
        if (E(e->getPreviousElementSibling())->getTagName()==MBSIMHYDRAULICS%"inflowDependencyOnOutflow")
          e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"inflowDependencyOnOutflow");
      }
    }
    while (e && (E(e)->getTagName()==MBSIMHYDRAULICS%"inflowDependencyOnInflow" || E(e)->getTagName()==MBSIMHYDRAULICS%"inflowDependencyOnOutflow")) {
      if (E(e)->getTagName()==MBSIMHYDRAULICS%"inflowDependencyOnInflow")
        refDependencyOnInflowString.push_back(E(e)->getAttribute("ref"));
      else
        refDependencyOnOutflowString.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"length");
    setLength(getDouble(e));
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, ConstrainedLine)

  void ConstrainedLine::updateQ() {
    QIn(0)=(*QFunction)(getTime());
    QOut = -QIn;
    updQ = false;
  }

  void ConstrainedLine::initializeUsingXML(DOMElement * element) {
    HLine::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"function");
    setQFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
  }

  void ConstrainedLine::init(InitStage stage) {
    if (stage==preInit) {
      Object::init(stage); // no check of connected lines
      if (!nFrom && !nTo) 
        THROW_MBSIMERROR("needs at least one connected node!");
      if (nFrom==nTo) 
        THROW_MBSIMERROR("fromNode and toNode are the same!");
    }
    else
      HLine::init(stage);
    QFunction->init(stage);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, FluidPump)

  void FluidPump::updateQ() {
    QIn(0) = (*QFunction)(getTime());
    QOut = -QIn;
    updQ = false;
  }

  void FluidPump::initializeUsingXML(DOMElement * element) {
    HLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"volumeflowFunction");
    setQFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
  }

  void FluidPump::init(InitStage stage) {
    if (stage==preInit) {
      Object::init(stage); // no check of connected lines
      if (!nFrom && !nTo) 
        THROW_MBSIMERROR("needs at least one connected node!");
      if (nFrom==nTo)
        THROW_MBSIMERROR("fromNode and toNode are the same!");
    }
    else
      HLine::init(stage);
    QFunction->init(stage);
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMHYDRAULICS, StatelessOrifice)

  void StatelessOrifice::initializeUsingXML(DOMElement * element) {
    HLine::initializeUsingXML(element);
    DOMElement * e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"inflowPressureFunction");
    setInflowFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"outflowPressureFunction");
    setOutflowFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"openingFunction");
    setOpeningFunction(MBSim::ObjectFactory::createAndInit<MBSim::Function<double(double)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"diameter");
    setDiameter(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"alpha");
    setAlpha(getDouble(e));
    e=E(element)->getFirstElementChildNamed(MBSIMHYDRAULICS%"areaModus");
    setCalcAreaModus(getInt(e));
  }

  void StatelessOrifice::init(InitStage stage) {
    if (stage==preInit) {
      Object::init(stage); // no check of connected lines
      if (!nFrom && !nTo) 
        THROW_MBSIMERROR("needs at least one connected node!");
      if (nFrom==nTo)
        THROW_MBSIMERROR("fromNode and toNode are the same!");
    }
    else if (stage==plotting) {
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("pInflow [bar]");
        plotColumns.push_back("pOutflow [bar]");
        plotColumns.push_back("dp [bar]");
        plotColumns.push_back("sign [-]");
        plotColumns.push_back("opening [mm]");
        plotColumns.push_back("area [mm^2]");
        plotColumns.push_back("sqrt_dp [sqrt(bar)]");
        plotColumns.push_back("Q [l/min]");
        HLine::init(stage);
      }
    }
    else if (stage==unknownStage) {
      const double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      alpha=alpha*sqrt(2./rho);
      HLine::init(stage);
    }
    else
      HLine::init(stage);
    inflowFunction->init(stage);
    outflowFunction->init(stage);
    openingFunction->init(stage);
  }

  void StatelessOrifice::plot() {
    if (getPlotFeature(plotRecursive)==enabled) {
      double Q = evalQIn()(0);
      plotVector.push_back(pIn*1e-5);
      plotVector.push_back(pOut*1e-5);
      plotVector.push_back(dp*1e-5);
      plotVector.push_back(sign);
      plotVector.push_back(opening*1e3);
      plotVector.push_back(area*1e6);
      plotVector.push_back(sqrt_dp*sqrt(1e-5));
      plotVector.push_back(Q*6e4);
      HLine::plot();
    }
  }

  void StatelessOrifice::updateQ() {
    pIn=(*inflowFunction)(getTime());
    pOut=(*outflowFunction)(getTime());
    dp=fabs(pIn-pOut);
    sign=((pIn-pOut)<0)?-1.:1.;
    opening=(*openingFunction)(getTime());
    if (opening<0)
      opening=0;
    if (calcAreaModus==0) { // wie <GammaCheckvalveClosablePressureLoss>
      const double gamma = M_PI/4.;
      const double sg = sin(gamma);
      const double cg = cos(gamma);
      area= M_PI * opening * sg/cg * (diameter + opening/cg);
    }
    else if (calcAreaModus==1) { // Kolben wird verschoben
      area= M_PI * diameter * opening;
    }
    else
      throw runtime_error("Error in StatelessOrifice::calculateQ");

    sqrt_dp=0;
    const double dpReg=.1e5;
    if (dp>dpReg)
      sqrt_dp=sqrt(dp);
    else {
      const double f=sqrt(dpReg);
      const double fS=.5/f;
      const double a0=0;
      const double a1=(-dpReg*fS+2.*f)/dpReg;
      const double a2=(-f+dpReg)/dpReg/dpReg;
      sqrt_dp=a2*dp*dp+a1*dp+a0;
    }

    QIn(0) = sign*area*alpha*sqrt_dp;
    QOut = -QIn;
    updQ = false;
  }

}
