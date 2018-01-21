/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/frames/frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/mbsim_event.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  const PlotFeatureEnum position;
  const PlotFeatureEnum angle;
  const PlotFeatureEnum velocity;
  const PlotFeatureEnum angularVelocity;
  const PlotFeatureEnum acceleration;
  const PlotFeatureEnum angularAcceleration;

  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, position)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, angle)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, velocity)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, angularVelocity)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, acceleration)
  MBSIM_OBJECTFACTORY_REGISTERENUM(PlotFeatureEnum, MBSIM, angularAcceleration)

  Frame::Frame(const string &name) : Element(name), AWP(EYE) {

    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;
    updJac[0] = true;
    updJac[1] = true;
    updJac[2] = true;
  }

  void Frame::plot() {
    if(plotFeature[plotRecursive]) {
      if(plotFeature[position]) {
        for(int i=0; i<evalPosition().size(); i++)
          plotVector.push_back(getPosition()(i));
      }
      if(plotFeature[angle]) {
        Vec3 cardan=AIK2Cardan(evalOrientation());
        for(int i=0; i<cardan.size(); i++)
          plotVector.push_back(cardan(i));
      }
      if(plotFeature[velocity]) {
        for(int i=0; i<evalVelocity().size(); i++)
          plotVector.push_back(getVelocity()(i));
      }
      if(plotFeature[angularVelocity]) {
        for(int i=0; i<evalAngularVelocity().size(); i++)
          plotVector.push_back(getAngularVelocity()(i));
      }
      if(plotFeature[acceleration]) {
        for(int i=0; i<evalAcceleration().size(); i++)
          plotVector.push_back(getAcceleration()(i));
      }
      if(plotFeature[angularAcceleration]) {
        for(int i=0; i<evalAngularAcceleration().size(); i++)
          plotVector.push_back(getAngularAcceleration()(i));
      }
    }
    if(plotFeature[openMBV] and openMBVFrame && !openMBVFrame->isHDF5Link()) {
      vector<double> data;
      data.push_back(getTime());
      data.push_back(evalPosition()(0));
      data.push_back(getPosition()(1));
      data.push_back(getPosition()(2));
      Vec3 cardan=AIK2Cardan(getOrientation());
      data.push_back(cardan(0));
      data.push_back(cardan(1));
      data.push_back(cardan(2));
      data.push_back(0);
      openMBVFrame->append(data);
    }
    Element::plot();
  }

  void Frame::init(InitStage stage, const InitConfigSet &config) {
    if(stage==unknownStage) {
      WJP[0].resize(hSize[0]);
      WJR[0].resize(hSize[0]);
      WJP[1].resize(hSize[1]);
      WJR[1].resize(hSize[1]);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[position]) {
          plotColumns.emplace_back("position (x)");
          plotColumns.emplace_back("position (y)");
          plotColumns.emplace_back("position (z)");
        }
        if(plotFeature[angle]) {
          plotColumns.emplace_back("angle (alpha)");
          plotColumns.emplace_back("angle (beta)");
          plotColumns.emplace_back("angle (gamma)");
        }
        if(plotFeature[velocity]) {
          plotColumns.emplace_back("velocity (x)");
          plotColumns.emplace_back("velocity (y)");
          plotColumns.emplace_back("velocity (z)");
        }
        if(plotFeature[angularVelocity]) {
          plotColumns.emplace_back("angular velocity (x)");
          plotColumns.emplace_back("angular velocity (y)");
          plotColumns.emplace_back("angular velocity (z)");
        }
        if(plotFeature[acceleration]) {
          plotColumns.emplace_back("acceleration (x)");
          plotColumns.emplace_back("acceleration (y)");
          plotColumns.emplace_back("acceleration (z)");
        }
        if(plotFeature[angularAcceleration]) {
          plotColumns.emplace_back("angular acceleration (x)");
          plotColumns.emplace_back("angular acceleration (y)");
          plotColumns.emplace_back("angular acceleration (z)");
        }
      }
      if(plotFeature[openMBV] and openMBVFrame) {
          openMBVFrame->setName(name);
          parent->getOpenMBVGrp()->addObject(openMBVFrame);
      }
    }
    Element::init(stage, config);
  }

  void Frame::initializeUsingXML(DOMElement *element) {
    Element::initializeUsingXML(element);

    DOMElement *ee=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(ee) {
      OpenMBVFrame ombv;
      ombv.initializeUsingXML(ee);
      openMBVFrame=ombv.createOpenMBV(); 
    }
  }

  void Frame::resetUpToDate() { 
    updJac[0] = true;
    updJac[1] = true;
    updJac[2] = true;
    updGA = true;
    updPos = true;
    updVel = true;
    updAcc = true;
  }

  void Frame::resetPositionsUpToDate() {
    updPos = true;
  }

  void Frame::resetVelocitiesUpToDate() {
    updVel = true;
  }

  void Frame::resetJacobiansUpToDate() {
    updJac[0] = true;
    updJac[1] = true;
    updJac[2] = true;
  }

  void Frame::resetGyroscopicAccelerationsUpToDate() {
    updGA = true;
  }

}
