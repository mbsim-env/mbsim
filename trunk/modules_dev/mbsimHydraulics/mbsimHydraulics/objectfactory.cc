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

#include "config.h"
#include "mbsimHydraulics/objectfactory.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsimHydraulics/hnode_mec.h"
#include "mbsimHydraulics/environment.h"
#include "mbsimHydraulics/controlvalve43.h"
#include "mbsimHydraulics/checkvalve.h"
#include "mbsimHydraulics/leakage_line.h"

using namespace std;

namespace MBSim {

  HydraulicsObjectFactory *HydraulicsObjectFactory::instance=NULL;


  void HydraulicsObjectFactory::initialize() {
    if(instance==0) {
      instance=new HydraulicsObjectFactory;
      ObjectFactory::getInstance()->registerObjectFactory(instance);
    }
  }


  Function1<double, double> * HydraulicsObjectFactory::createFunction1_SS(TiXmlElement * element) {
    if (element==0) return 0;
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PressureLossZeta")
      return new PressureLossZeta(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PressureLossLaminarTubeFlow")
      return new PressureLossLaminarTubeFlow(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PressureLossCurveFit")
      return new PressureLossCurveFit(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossAreaZeta")
      return new VariablePressureLossAreaZeta(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossAreaZeta")
      return new RegularizedVariablePressureLossAreaZeta(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossControlvalveAreaAlpha")
      return new VariablePressureLossControlvalveAreaAlpha(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossControlvalveAreaAlpha")
      return new RegularizedVariablePressureLossControlvalveAreaAlpha(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossCheckvalveGamma")
      return new VariablePressureLossCheckvalveGamma(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossCheckvalveGamma")
      return new RegularizedVariablePressureLossCheckvalveGamma(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossCheckvalveIdelchick")
      return new VariablePressureLossCheckvalveIdelchick(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossCheckvalveIdelchick")
      return new RegularizedVariablePressureLossCheckvalveIdelchick(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"VariablePressureLossCheckvalveCone")
      return new VariablePressureLossCheckvalveCone(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedVariablePressureLossCheckvalveCone")
      return new RegularizedVariablePressureLossCheckvalveCone(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PlaneLeakagePressureLoss")
      return new PlaneLeakagePressureLoss(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"EccentricCircularLeakagePressureLoss")
      return new EccentricCircularLeakagePressureLoss(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RealCircularLeakagePressureLoss")
      return new RealCircularLeakagePressureLoss(element->Attribute("name"));
  }


  Object * HydraulicsObjectFactory::createObject(TiXmlElement * element) {
    if (element==0) return 0;
    if (element->ValueStr()==MBSIMHYDRAULICSNS"RigidLine")
      return new RigidLine(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"PlaneLeakage")
      return new PlaneLeakage(element->Attribute("name"));
    if (element->ValueStr()==MBSIMHYDRAULICSNS"CircularLeakage")
      return new CircularLeakage(element->Attribute("name"));
    return 0;
  }


  Link* HydraulicsObjectFactory::createLink(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"ConstrainedNode")
      return new ConstrainedNode(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"EnvironmentNode")
      return new EnvironmentNode(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"ElasticNode")
      return new ElasticNode(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"RigidNode")
      return new RigidNode(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"ConstrainedNodeMec")
      return new ConstrainedNodeMec(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"EnvironmentNodeMec")
      return new EnvironmentNodeMec(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"ElasticNodeMec")
      return new ElasticNodeMec(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"RigidNodeMec")
      return new RigidNodeMec(element->Attribute("name"));
    return 0;
  }


  Environment* HydraulicsObjectFactory::getEnvironment(TiXmlElement *element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"HydraulicEnvironment")
      return HydraulicEnvironment::getInstance();
    return 0;
  }


  Group * HydraulicsObjectFactory::createGroup(TiXmlElement * element) {
    if(element==0) return 0;
    if(element->ValueStr()==MBSIMHYDRAULICSNS"Controlvalve43")
      return new Controlvalve43(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"RegularizedControlvalve43")
      return new RegularizedControlvalve43(element->Attribute("name"));
    if(element->ValueStr()==MBSIMHYDRAULICSNS"BallCheckvalve")
      return new Checkvalve(element->Attribute("name"));
    return 0;
  }

}
