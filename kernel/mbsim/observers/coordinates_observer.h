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
#include <openmbvcppinterface/arrow.h>
namespace OpenMBV {
  class Frame;
}
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
      void setOpenMBVPositionArrow(OpenMBV::Arrow *arrow) { openMBVPosition = arrow; }
      void setOpenMBVVelocityArrow(OpenMBV::Arrow *arrow) { openMBVVelocity = arrow; }
      void setOpenMBVAccelerationArrow(OpenMBV::Arrow *arrow) { openMBVAcceleration = arrow; }

      virtual void enableOpenMBVPosition(double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVVelocity(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVAcceleration(double scale=1, OpenMBV::Arrow::ReferencePoint refPoint=OpenMBV::Arrow::fromPoint, double diameter=0.5, double headDiameter=1, double headLength=1, double color=0.5);
      virtual void enableOpenMBVFrame(double size=1, double offset=1);

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

