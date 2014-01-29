/* Copyright (C) 2004-2013 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _KINEMATICS_OBSERVER_H__
#define _KINEMATICS_OBSERVER_H__
#include "mbsim/observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <mbsim/utils/openmbv_utils.h>
#include <openmbvcppinterface/arrow.h>
#endif

namespace MBSim {
  class Frame;

  class KinematicsObserver : public Observer {
    protected:
      Frame* frame;
      std::string saved_frame;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group *openMBVPosGrp, *openMBVVelGrp, *openMBVAngVelGrp, *openMBVAccGrp, *openMBVAngAccGrp;
      OpenMBV::Arrow *openMBVPositionArrow, *openMBVVelocityArrow, *openMBVAngularVelocityArrow, *openMBVAccelerationArrow, *openMBVAngularAccelerationArrow;
#endif

    public:
      KinematicsObserver(const std::string &name="");
      void setFrame(Frame *frame_) { frame = frame_; } 
      void init(InitStage stage);
      virtual void plot(double t, double dt);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVPosition, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(diameter,(double),0.5)(headDiameter,(double),1)(headLength,(double),1)(type,(OpenMBV::Arrow::Type),OpenMBV::Arrow::toHead)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(scaleLength,(double),1))) { enableOpenMBVArrow(openMBVPositionArrow,diffuseColor,transparency,diameter,headDiameter,headLength,type,referencePoint,scaleLength); }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVVelocity, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(diameter,(double),0.5)(headDiameter,(double),1)(headLength,(double),1)(type,(OpenMBV::Arrow::Type),OpenMBV::Arrow::toHead)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(scaleLength,(double),1))) { enableOpenMBVArrow(openMBVVelocityArrow,diffuseColor,transparency,diameter,headDiameter,headLength,type,referencePoint,scaleLength); }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAngularVelocity, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(diameter,(double),0.5)(headDiameter,(double),1)(headLength,(double),1)(type,(OpenMBV::Arrow::Type),OpenMBV::Arrow::toDoubleHead)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(scaleLength,(double),1))) { enableOpenMBVArrow(openMBVAngularVelocityArrow,diffuseColor,transparency,diameter,headDiameter,headLength,type,referencePoint,scaleLength); }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAcceleration, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(diameter,(double),0.5)(headDiameter,(double),1)(headLength,(double),1)(type,(OpenMBV::Arrow::Type),OpenMBV::Arrow::toHead)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(scaleLength,(double),1))) { enableOpenMBVArrow(openMBVAccelerationArrow,diffuseColor,transparency,diameter,headDiameter,headLength,type,referencePoint,scaleLength); }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAngularAcceleration, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(diameter,(double),0.5)(headDiameter,(double),1)(headLength,(double),1)(type,(OpenMBV::Arrow::Type),OpenMBV::Arrow::toDoubleHead)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(scaleLength,(double),1))) { enableOpenMBVArrow(openMBVAngularAccelerationArrow,diffuseColor,transparency,diameter,headDiameter,headLength,type,referencePoint,scaleLength); }
#endif
  };

  class AbsoluteKinematicsObserver : public KinematicsObserver {
    public:
      AbsoluteKinematicsObserver(const std::string &name="") : KinematicsObserver(name) {}
  };

  class RelativeKinematicsObserver : public KinematicsObserver {
    private:
      Frame* refFrame;
      std::string saved_frameOfReference;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVrTrans, *openMBVrRel;
      OpenMBV::Arrow *openMBVvTrans, *openMBVvRot, *openMBVvRel, *openMBVvF;
      OpenMBV::Arrow *openMBVaTrans, *openMBVaRot, *openMBVaZp, *openMBVaCor, *openMBVaRel, *openMBVaF;
      OpenMBV::Arrow *openMBVomTrans, *openMBVomRel;
      OpenMBV::Arrow *openMBVpsiTrans, *openMBVpsiRot, *openMBVpsiRel;
#endif

    public:
      RelativeKinematicsObserver(const std::string &name="");
      void setFrames(Frame *frame0, Frame *frame1) { frame = frame0; refFrame = frame1; } 
      void setFrameOfReference(Frame *frame_) { refFrame = frame_; }

      void init(InitStage stage);
      virtual void plot(double t, double dt);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
  };

}  

#endif

