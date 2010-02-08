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

#include "mbsimControl/frame_sensors.h"
#include "mbsimControl/objectfactory.h"

#include "mbsim/frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimControl {

  void AbsolutCoordinateSensor::initializeUsingXML(TiXmlElement * element) {
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

  Vec AbsolutPositionSensor::getSignal() {
    return trans(direction)*frame->getPosition();
  }


  Vec AbsolutVelocitySensor::getSignal() {
    return trans(direction)*frame->getVelocity();
  }


  void AbsolutAngularPositionSensor::updategd(double t) {
    gd=trans(direction)*frame->getAngularVelocity();
  }

  Vec AbsolutAngularPositionSensor::getSignal() {
    return g;
  }


  Vec AbsolutAngularVelocitySensor::getSignal() {
    return trans(direction)*frame->getAngularVelocity();
  }
  

  void RelativeCoordinateSensor::initializeUsingXML(TiXmlElement * element) {
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
    return trans(refFrame->getOrientation()*direction)*WrRefRel;
  }


  Vec RelativeVelocitySensor::getSignal() {
    Vec WvRefRel=relFrame->getVelocity()-refFrame->getVelocity();
    return trans(refFrame->getOrientation()*direction)*WvRefRel;
  }


  void RelativeAngularPositionSensor::updategd(double t) {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    gd=trans(refFrame->getOrientation()*direction)*WomegaRefRel;
  }

  Vec RelativeAngularPositionSensor::getSignal() {
    return g;
  }


  Vec RelativeAngularVelocitySensor::getSignal() {
    Vec WomegaRefRel=relFrame->getAngularVelocity()-refFrame->getAngularVelocity();
    return trans(refFrame->getOrientation()*direction)*WomegaRefRel;
  }

}
