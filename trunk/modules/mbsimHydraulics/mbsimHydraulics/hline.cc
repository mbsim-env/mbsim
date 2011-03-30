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

#include "mbsimHydraulics/hline.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/rigid_line_pressureloss.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimControl/signal_.h"
#include "mbsim/frame.h"
#include "mbsimHydraulics/obsolet_hint.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

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
        throw MBSimError("ERROR! HLine \""+name+"\" has no fromNode!");
      if (!nTo && !nToRelative) 
        throw MBSimError("ERROR! HLine \""+name+"\" has no toNode!");
      if (nFrom && nFrom==nTo) 
        throw MBSimError("ERROR! HLine \""+name+"\": fromNode and toNode are the same!");
    }
    else
      Object::init(stage);
  }
      
  void HLine::setFrameOfReference(Frame *frame) {
    frameOfReference = frame; 
  }

  void HLine::initializeUsingXML(TiXmlElement * element) {
    Object::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"frameOfReference");
    if (e) {
      saved_frameOfReference=e->Attribute("ref");
      e=element->FirstChildElement(MBSIMHYDRAULICSNS"direction");
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
        throw MBSimError("Kinematic Loop in hydraulic system. Check model!");

    // TODO efficient calculation (not every loop is necessary)
    Mat JLocal;
    if(dependency.size()==0) {
      if(M.size()==1)
        JLocal=Mat(1,1,INIT,1);
      else {
        JLocal=Mat(1,M.size(),INIT,0);
        JLocal(0,uInd[0])=1;
      }
    }
    else {
      JLocal=Mat(1,M.size(),INIT,0);
      dep_check.push_back(this);
      for (unsigned int i=0; i<dependencyOnOutflow.size(); i++) {
        const Mat Jdep=((RigidHLine*)dependencyOnOutflow[i])->calculateJacobian(dep_check);
        JLocal(0,Index(0,Jdep.cols()-1))+=Jdep;
      }
      for (unsigned int i=0; i<dependencyOnInflow.size(); i++) {
        const Mat Jdep=((RigidHLine*)dependencyOnInflow[i])->calculateJacobian(dep_check);
        JLocal(0,Index(0,Jdep.cols()-1))-=Jdep;
      }
    }
    return JLocal;
  }

  void RigidHLine::updateStateDependentVariables(double t) {
    if(dependency.size()==0)
      Q(0)=u(0);
    else {
      Q.init(0);
      for (unsigned int i=0; i<dependencyOnOutflow.size(); i++)
        Q+=(dependencyOnOutflow[i])->getQIn();
      for (unsigned int i=0; i<dependencyOnInflow.size(); i++)
        Q-=(dependencyOnInflow[i])->getQIn();
    }
  }

  void RigidHLine::updateh(double t) {
    if (frameOfReference)
      pressureLossGravity=-trans(frameOfReference->getOrientation()*MBSimEnvironment::getInstance()->getAccelerationOfGravity())*direction*HydraulicEnvironment::getInstance()->getSpecificMass()*length;
    h-=trans(Jacobian.row(0))*pressureLossGravity;
  }
      
  void RigidHLine::updateM(double t) {
    M+=Mlocal(0,0)*JTJ(Jacobian); 
  }

  void RigidHLine::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      for (unsigned int i=0; i<refDependencyOnInflowString.size(); i++)
        addInflowDependencyOnInflow(getByPath<RigidHLine>(process_hline_string(refDependencyOnInflowString[i])));
      for (unsigned int i=0; i<refDependencyOnOutflowString.size(); i++)
        addInflowDependencyOnOutflow(getByPath<RigidHLine>(process_hline_string(refDependencyOnOutflowString[i])));
      HLine::init(stage);
    }
    else if (stage==MBSim::preInit) {
      HLine::init(stage);
      for (unsigned int i=0; i<dependencyOnInflow.size(); i++)
        dependency.push_back(dependencyOnInflow[i]);
      for (unsigned int i=0; i<dependencyOnOutflow.size(); i++)
        dependency.push_back(dependencyOnOutflow[i]);
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Volume flow [l/min]");
        plotColumns.push_back("Mass flow [kg/min]");
        if (frameOfReference)
          plotColumns.push_back("pressureLoss due to gravity [bar]");
        HLine::init(stage);
      }
    }
    else if(stage==MBSim::unknownStage) {
      HLine::init(stage);

      vector<RigidHLine *> dep_check;
      dep_check.push_back(this);
      Jacobian=calculateJacobian(dep_check);
    }
    else
      HLine::init(stage);
  }
  
  void RigidHLine::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(Q(0)*6e4);
      plotVector.push_back(Q(0)*HydraulicEnvironment::getInstance()->getSpecificMass()*60.);
      if (frameOfReference)
        plotVector.push_back(pressureLossGravity*1e-5);
      HLine::plot(t, dt);
    }
  }

  void RigidHLine::initializeUsingXML(TiXmlElement * element) {
    HLine::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"inflowDependencyOnInflow");
    if (!e)
      e=element->FirstChildElement(MBSIMHYDRAULICSNS"inflowDependencyOnOutflow");
    else {
      if (e->PreviousSibling()) {
        if (e->PreviousSibling()->ValueStr()==MBSIMHYDRAULICSNS"inflowDependencyOnOutflow")
          e=element->FirstChildElement(MBSIMHYDRAULICSNS"inflowDependencyOnOutflow");
      }
    }
    while (e && (e->ValueStr()==MBSIMHYDRAULICSNS"inflowDependencyOnInflow" || e->ValueStr()==MBSIMHYDRAULICSNS"inflowDependencyOnOutflow")) {
      if (e->ValueStr()==MBSIMHYDRAULICSNS"inflowDependencyOnInflow")
        refDependencyOnInflowString.push_back(e->Attribute("ref"));
      else
        refDependencyOnOutflowString.push_back(e->Attribute("ref"));
      e=e->NextSiblingElement();
    }
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLength(getDouble(e));
  }


  void ConstrainedLine::updateStateDependentVariables(double t) {
    Q(0)=(*QFun)(t);
  }

  void ConstrainedLine::initializeUsingXML(TiXmlElement * element) {
    HLine::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMHYDRAULICSNS"function");
    Function1<double, double> * qf=MBSim::ObjectFactory::getInstance()->createFunction1_SS(e->FirstChildElement()); 
    setQFunction(qf);
    qf->initializeUsingXML(e->FirstChildElement());
  }

  void ConstrainedLine::init(MBSim::InitStage stage) {
    if (stage==preInit) {
      Object::init(stage); // no check of connected lines
      if (!nFrom && !nTo) 
        throw MBSimError("ERROR! ConstrainedLine \""+name+"\" needs at least one connected node!");
      if (nFrom==nTo) 
        throw MBSimError("ERROR! ConstrainedLine \""+name+"\": fromNode and toNode are the same!");
    }
    else
      HLine::init(stage);
  }


  Vec FluidPump::getQIn() {return QSignal->getSignal(); }
  Vec FluidPump::getQOut() {return -1.*QSignal->getSignal(); }

  void FluidPump::initializeUsingXML(TiXmlElement * element) {
    HLine::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"volumeflowSignal");
    QSignalString=e->Attribute("ref");
  }

  void FluidPump::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      HLine::init(stage);
      if (QSignalString!="")
        setQSignal(getByPath<Signal>(process_signal_string(QSignalString)));
    }
    else if (stage==preInit) {
      Object::init(stage); // no check of connected lines
      if (!nFrom && !nTo) 
        throw MBSimError("ERROR! FluidPump \""+name+"\" needs at least one connected node!");
      if (nFrom==nTo)
        throw MBSimError("ERROR! FluidPump \""+name+"\": fromNode and toNode are the same!");
    }
    else
      HLine::init(stage);
  }

}
