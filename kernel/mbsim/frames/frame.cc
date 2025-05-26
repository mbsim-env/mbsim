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
#include "mbsim/utils/rotarymatrices.h"
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/group.h>
#include <hdf5serie/simpleattribute.h>

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
      if(plotFeature[position])
	Element::plot(evalPosition());
      if(plotFeature[angle])
	Element::plot(AIK2Cardan(evalOrientation()));
      if(plotFeature[velocity])
	Element::plot(evalVelocity());
      if(plotFeature[angularVelocity])
	Element::plot(evalAngularVelocity());
      if(plotFeature[acceleration])
        Element::plot(evalAcceleration());
      if(plotFeature[angularAcceleration])
	Element::plot(evalAngularAcceleration());
    }
    if(plotFeature[openMBV] and openMBVFrame) {
      array<double,8> data;
      data[0] = getTime();
      data[1] = evalPosition()(0);
      data[2] = getPosition()(1);
      data[3] = getPosition()(2);
      Vec3 cardan=AIK2Cardan(getOrientation());
      data[4] = cardan(0);
      data[5] = cardan(1);
      data[6] = cardan(2);
      data[7] = 0;
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
        if(plotFeature[position])
	  addToPlot("position",{"x","y","z"});
        if(plotFeature[angle])
	  addToPlot("angle",{"alpha","beta","gamma"});
        if(plotFeature[velocity])
	  addToPlot("velocity",{"x","y","z"});
        if(plotFeature[angularVelocity])
	  addToPlot("angular velocity",{"x","y","z"});
        if(plotFeature[acceleration])
	  addToPlot("acceleration",{"x","y","z"});
        if(plotFeature[angularAcceleration])
	  addToPlot("angular acceleration",{"x","y","z"});
      }
      if(plotFeature[openMBV] and openMBVFrame) {
	openMBVFrame->setName(name);
	parent->getFramesOpenMBVGrp()->addObject(openMBVFrame);
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

  void Frame::createPlotGroup() {
    plotGroup=parent->getFramesPlotGroup()->createChildObject<H5::Group>(name)();
    plotGroup->createChildAttribute<H5::SimpleAttribute<string>>("Description")()->write("Object of class: "+boost::core::demangle(typeid(*this).name()));
    plotColumns.insert(plotColumns.begin(), "time");
  }

}
