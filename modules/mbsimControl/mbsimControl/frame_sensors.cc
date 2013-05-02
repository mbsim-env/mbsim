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
#include "mbsimControl/frame_sensors.h"
#include "mbsimControl/objectfactory.h"

#include "mbsim/frame.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  void AbsolutCoordinateSensor::initializeUsingXML(TiXmlElement * element) {
    Sensor::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"frame");
    frameString=e->Attribute("ref");
    e=element->FirstChildElement(MBSIMCONTROLNS"direction");
    direction=getMat(e,3,0);
    for (int i=0; i<direction.cols(); i++)
      direction.col(i)=direction.col(i)/nrm2(direction.col(i));
  }

  void AbsolutCoordinateSensor::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (frameString!="")
        setFrame(getByPath<Frame>(frameString));
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }

  Vec AbsolutePositionSensor::getSignal() {
    return direction.T()*frame->getPosition();
  }


  Vec AbsoluteVelocitySensor::getSignal() {
    return direction.T()*frame->getVelocity();
  }


  void AbsoluteAngularPositionSensor::updategd(double t) {
    gd=direction.T()*frame->getAngularVelocity();
  }

  Vec AbsoluteAngularPositionSensor::getSignal() {
    return g;
  }


  Vec AbsoluteAngularVelocitySensor::getSignal() {
    return direction.T()*frame->getAngularVelocity();
  }
  

  void RelativeCoordinateSensor::initializeUsingXML(TiXmlElement * element) {
    Sensor::initializeUsingXML(element);
    TiXmlElement *e;
    e=element->FirstChildElement(MBSIMCONTROLNS"frame");
    refFrameString=e->Attribute("ref");
    relFrameString=e->Attribute("rel");
    e=element->FirstChildElement(MBSIMCONTROLNS"direction");
    direction=getMat(e,3,0);
    for (int i=0; i<direction.cols(); i++)
      direction.col(i)=direction.col(i)/nrm2(direction.col(i));
  }

  void RelativeCoordinateSensor::init(InitStage stage) {
    if (stage==MBSim::resolveXMLPath) {
      if (refFrameString!="")
        setReferenceFrame(getByPath<Frame>(refFrameString));
      if (relFrameString!="")
        setRelativeFrame(getByPath<Frame>(relFrameString));
      Sensor::init(stage);
    }
    else
      Sensor::init(stage);
  }


  Vec RelativePositionSensor::getSignal() {
    Vec WrRefRel=relFrame->getPosition()-refFrame->getPosition();
    return (refFrame->getOrientation()*direction).T()*WrRefRel;
  }


  Vec RelativeVelocitySensor::getSignal() {
    Vec WvRefRel=relFrame->getVelocity()-refFrame->getVelocity();
    return (refFrame->getOrientation()*direction).T()*WvRefRel;
  }


  void RelativeAngularPositionSensor::updategd(double t) {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    gd=(refFrame->getOrientation()*direction).T()*WomegaRefRel;
  }

  Vec RelativeAngularPositionSensor::getSignal() {
    return g;
  }


  Vec RelativeAngularVelocitySensor::getSignal() {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    return (refFrame->getOrientation()*direction).T()*WomegaRefRel;
  }

}
