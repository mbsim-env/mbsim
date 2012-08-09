/* Copyright (C) 2011 Markus Schneider

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
 *
 * Contact:
 *   schneidm@amm.mw.tu-muenchen.de
 *
 */ 

#include "mbsimControl/massless_spring_damper.h"
#include "mbsimControl/objectfactory.h"
#include "mbsimControl/signal_.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  MasslessSpringDamper::MasslessSpringDamper(const string& name) : SignalProcessingSystem(name), c(0), F0(0), dPos(0), dNeg(0), FFricPos(0), FFricNeg(1./epsroot()), xMin(-1./epsroot()), xMax(1./epsroot()) {
  }

  void MasslessSpringDamper::initializeUsingXML(TiXmlElement * element) {
    SignalProcessingSystem::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMCONTROLNS"springStiffness");
    setSpringStiffness(getDouble(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"basicSpringForce");
    setBasicSpringForce(getDouble(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"dampingCoefficient");
    setDampingCoefficient(getDouble(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"negativeDampingCoefficient");
    if (e)
      setNegativeDampingCoefficient(getDouble(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"frictionForce");
    if (e)
      setFrictionForce(getDouble(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"negativeFrictionForce");
    if (e)
      setNegativeFrictionForce(getDouble(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"minimalPositionValue");
    if (e)
      setMinimumPositionValue(getDouble(e));
    e=element->FirstChildElement(MBSIMCONTROLNS"maximalPositionValue");
    if (e)
      setMaximumPositionValue(getDouble(e));
  }

  void MasslessSpringDamper::updatedx(double t, double dt) {
    const double F=(inputSignal->getSignal())(0);
    double h=F+F0-c*x(0);
    if (h<0) {
      if (x(0)<xMin)
        xd(0)=0;
      else {
        h+=FFricNeg;
        if (h>0)
          h=0;
        xd(0)=h/dNeg;
      }
    }
    else {
      if (x(0)>xMax)
        xd(0)=0;
      else {
        h-=FFricPos;
        if (h<0)
          h=0;
        xd(0)=h/dPos;
      }
    }
    xdLocal=xd(0);
    xd(0)*=dt;
  }

  void MasslessSpringDamper::updatexd(double t) {
    const double F=(inputSignal->getSignal())(0);
    double h=F+F0-c*x(0);
    if (h<0) {
      if (x(0)<xMin)
        xd(0)=0;
      else {
        h+=FFricNeg;
        if (h>0)
          h=0;
        xd(0)=h/dNeg;
      }
    }
    else {
      if (x(0)>xMax)
        xd(0)=0;
      else {
        h-=FFricPos;
        if (h<0)
          h=0;
        xd(0)=h/dPos;
      }
    }
    xdLocal=xd(0);
    /*
     *const double F=(inputSignal->getSignal())(0);
     *const double h=F+F0-c*x(0);
     *if (h<0) {
     *  if (x(0)<xMin)
     *    xd(0)=0;
     *  else
     *    xd(0)=h/dNeg;
     *}
     *else {
     *  if (x(0)>xMax)
     *    xd(0)=0;
     *  else
     *    xd(0)=h/dPos;
     *}
     *xdLocal=xd(0);
     */
  }

  void MasslessSpringDamper::init(MBSim::InitStage stage) {
    if (stage==MBSim::resize) {
      SignalProcessingSystem::init(stage);
      x.resize(xSize, INIT, 0);
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        if (getPlotFeature(globalPosition)==enabled) {
          plotColumns.push_back("Position");
          plotColumns.push_back("Velocity");
        }
        SignalProcessingSystem::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      assert(dPos>epsroot());
      if (fabs(dNeg)<epsroot())
        dNeg=dPos;
      assert(dNeg>epsroot());

      assert(!(FFricPos<0));
      if (fabs(FFricNeg) > (1./epsroot()-1))
        FFricNeg=FFricPos;
      assert(!(FFricNeg<0));

      SignalProcessingSystem::init(stage);
    }
    else
      SignalProcessingSystem::init(stage);
  }

  void MasslessSpringDamper::plot(double t, double dt) {
    Vec y=calculateOutput();
    if(getPlotFeature(plotRecursive)==enabled) {
      if (getPlotFeature(globalPosition)==enabled) {
        plotVector.push_back(y(0));
        plotVector.push_back(xdLocal);
      }
    }
    SignalProcessingSystem::plot(t,dt);
  }

}
