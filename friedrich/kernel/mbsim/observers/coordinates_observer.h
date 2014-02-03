/* Copyright (C) 2004-2011 MBSim Development Team
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

#ifndef _COORDINATES_OBSERVER_H__
#define _COORDINATES_OBSERVER_H__
#include "mbsim/observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>
#endif

namespace MBSim {
  class Frame;

  class CoordinatesObserver : public Observer {
    protected:
      Frame* frame;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group *openMBVPosGrp, *openMBVVelGrp, *openMBVAccGrp;
      OpenMBV::Arrow *openMBVPosition, *openMBVVelocity, *openMBVAcceleration;
      OpenMBV::Frame* openMBVFrame;
#endif

    public:
      CoordinatesObserver(const std::string &name="");
      void setFrame(Frame *frame_) { frame = frame_; } 

      void init(InitStage stage);
      virtual void plot(double t, double dt);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVPosition, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVPosition=ombv.createOpenMBV(); 
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVVelocity, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVVelocity=ombv.createOpenMBV(); 
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAcceleration, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVAcceleration=ombv.createOpenMBV(); 
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVFrame, tag, (optional (size,(double),1)(offset,(double),1)(transparency,(double),0))) { 
        OpenMBVFrame ombv(size,offset,"[-1;1;1]",transparency);
        openMBVFrame=ombv.createOpenMBV(); 
      }
#endif

    private:
      std::string saved_frame;
  };

  class CartesianCoordinatesObserver : public CoordinatesObserver {
    private:
      fmatvec::Vec3 ex, ey, ez;
      fmatvec::SqrMat3 A;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVXPosition, *openMBVYPosition, *openMBVZPosition, *openMBVXVelocity, *openMBVYVelocity, *openMBVZVelocity, *openMBVXAcceleration, *openMBVYAcceleration, *openMBVZAcceleration; 
#endif

    public:
      CartesianCoordinatesObserver(const std::string &name="");

      void init(InitStage stage);
      virtual void plot(double t, double dt);
  };

  class CylinderCoordinatesObserver : public CoordinatesObserver {
    private:
      fmatvec::Vec3 ez;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVRadialPosition, *openMBVZPosition, *openMBVRadialVelocity, *openMBVCircularVelocity, *openMBVZVelocity, *openMBVRadialAcceleration, *openMBVCircularAcceleration, *openMBVZAcceleration; 
#endif

    public:
      CylinderCoordinatesObserver(const std::string &name="");

      void init(InitStage stage);
      virtual void plot(double t, double dt);
  };

  class NaturalCoordinatesObserver : public CoordinatesObserver {
    private:
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *openMBVTangentialAcceleration, *openMBVNormalAcceleration;
#endif

    public:
      NaturalCoordinatesObserver(const std::string &name="");

      void init(InitStage stage);
      virtual void plot(double t, double dt);
  };  

}

#endif

