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

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimControl;

namespace MBSimHydraulics {

  void HLine::init(InitStage stage) {
    if (stage==preInit) {
      Object::init(stage);
      if (!nFrom) { cerr<<"ERROR! HLine \""<<name<<"\" has no fromNode!"<<endl; _exit(1); }
      if (!nTo) { cerr<<"ERROR! HLine \""<<name<<"\" has no toNode!"<<endl; _exit(1); }
      if (nFrom==nTo) { cerr<<"ERROR! HLine \""<<name<<"\": fromNode and toNode are the same!"<<endl; _exit(1); }
      
      dependency.clear(); // no hydraulic-objects in tree structure
    }
    else
      Object::init(stage);
  }

  void HLine::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    Object::initializeUsingXML(element);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"direction");
    setDirection(getVec(e,3));
  }

  Signal * HLine::getSignalByPath(string path) {
    int pos=path.find("Signal");
    path.erase(pos, 6);
    path.insert(pos, "Link");
    Link * s = getLinkByPath(path);
    if (dynamic_cast<Signal *>(s))
      return static_cast<Signal *>(s);
    else {
      std::cerr << "ERROR! \"" << path << "\" is not of Signal-Type." << std::endl; 
      _exit(1);
    }
  }
 

  void RigidHLine::updateh(double t) {
    pressureLossGravity=-trans(frameOfReference->getOrientation()*MBSimEnvironment::getInstance()->getAccelerationOfGravity())*direction*HydraulicEnvironment::getInstance()->getSpecificMass()*length;
    h(0)=-pressureLossGravity;
  }

  void RigidHLine::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("Fluidflow [l/min]");
        plotColumns.push_back("Massflow [kg/min]");
        plotColumns.push_back("pressureLoss due to gravity [bar]");
        HLine::init(stage);
      }
    }
    else
      HLine::init(stage);
  }
  
  void RigidHLine::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(u(0)*6e4);
      plotVector.push_back(u(0)*HydraulicEnvironment::getInstance()->getSpecificMass()*60.);
      plotVector.push_back(pressureLossGravity*1e-5);
      HLine::plot(t, dt);
    }
  }

  void RigidHLine::initializeUsingXML(TiXmlElement * element) {
    TiXmlElement * e;
    HLine::initializeUsingXML(element);
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
      if (!nFrom && !nTo) { cerr<<"ERROR! ConstrainedLine \""<<name<<"\" needs at least one connected node!"<<endl; _exit(1); }
      if (nFrom==nTo) { cerr<<"ERROR! ConstrainedLine \""<<name<<"\": fromNode and toNode are the same!"<<endl; _exit(1); }
    }
    else
      HLine::init(stage);
  }


  Vec FluidPump::getQIn(double t) {return QSignal->getSignal(); }
  Vec FluidPump::getQOut(double t) {return -1.*QSignal->getSignal(); }

  void FluidPump::initializeUsingXML(TiXmlElement * element) {
    HLine::initializeUsingXML(element);
    TiXmlElement * e=element->FirstChildElement(MBSIMHYDRAULICSNS"volumeflowSignal");
    QSignalString=e->Attribute("ref");
  }

  void FluidPump::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      HLine::init(stage);
      if (QSignalString!="")
        setQSignal(getSignalByPath(QSignalString));
    }
    else if (stage==preInit) {
      Object::init(stage); // no check of connected lines
      if (!nFrom && !nTo) { cerr<<"ERROR! ConstrainedLine \""<<name<<"\" needs at least one connected node!"<<endl; _exit(1); }
      if (nFrom==nTo) { cerr<<"ERROR! ConstrainedLine \""<<name<<"\": fromNode and toNode are the same!"<<endl; _exit(1); }
    }
    else
      HLine::init(stage);
  }

}
