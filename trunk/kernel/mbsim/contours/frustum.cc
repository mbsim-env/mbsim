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
 * Contact: mfoerg@users.berlios.de
 */

#include<config.h>
#include "mbsim/contours/frustum.h"
#include <mbsim/utils/utils.h>


#include <fmatvec.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif


using namespace std;
using namespace fmatvec;

namespace MBSim {

  void Frustum::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
  
      if(getPlotFeature(plotRecursive)==enabled) {
  #ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          ((OpenMBV::Frustum*)openMBVRigidBody)->setInitialTranslation(0.,h,0.);
          ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(r(0));
          ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(r(1));
          ((OpenMBV::Frustum*)openMBVRigidBody)->setHeight(h);
        }
  #endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  //TODO: Same function as in flexible_body_2s_13_mfr_mindlin (transformCW) --> this is just the transformation into cylindrical coordinates --> get it into utils?
  Vec Frustum::computeLagrangeParameter(const Vec &WrPoint) {
    Vec CrPoint = WrPoint.copy();

    CrPoint -= R.getPosition();
    CrPoint = R.getOrientation().T() * CrPoint; // position in moving frame of reference

    const double xt = CrPoint(0);
    const double yt = CrPoint(2);

    CrPoint.resize(2);

    CrPoint(0) = sqrt(xt * xt + yt * yt);
    CrPoint(1) = ArcTan(yt, xt);

    return CrPoint;
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void Frustum::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Frustum;
      ((OpenMBV::Frustum*)openMBVRigidBody)->setStaticColor(0.5);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setInitialRotation(-M_PI*0.5,0.,0.);
    }
    else openMBVRigidBody=0;
  }
#endif

  void Frustum::initializeUsingXML(TiXmlElement *element) {
    RigidContour::initializeUsingXML(element);
    TiXmlElement* e;
    e=element->FirstChildElement(MBSIMNS"baseRadius");
    r(0)=getDouble(e);
    e=element->FirstChildElement(MBSIMNS"topRadius");
    r(1)=getDouble(e);
    e=element->FirstChildElement(MBSIMNS"height");
    h=getDouble(e);
    if (element->FirstChildElement(MBSIMNS"solid"))
      outCont=true;
    else
      outCont=false;
#ifdef HAVE_OPENMBVCPPINTERFACE
    if(element->FirstChildElement(MBSIMNS"enableOpenMBV"))
      enableOpenMBV();
#endif
  }
}
