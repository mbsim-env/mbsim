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
#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>
#endif

namespace MBSim {
  class Frame;

  class KinematicsObserver : public Observer {
    protected:
      Frame* frame;
      std::string saved_frame;
#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Group> openMBVPosGrp, openMBVVelGrp, openMBVAngVelGrp, openMBVAccGrp, openMBVAngAccGrp;
      boost::shared_ptr<OpenMBV::Arrow> openMBVPosition, openMBVVelocity, openMBVAngularVelocity, openMBVAcceleration, openMBVAngularAcceleration;
#endif

    public:
      KinematicsObserver(const std::string &name="");
      void setFrame(Frame *frame_) { frame = frame_; } 
      void init(InitStage stage);
      virtual void plot(double t, double dt);
      virtual void initializeUsingXML(xercesc::DOMElement *element);

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVPosition, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVPosition=ombv.createOpenMBV(); 
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVVelocity, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVVelocity=ombv.createOpenMBV(); 
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAngularVelocity, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        openMBVAngularVelocity=ombv.createOpenMBV(); 
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAcceleration, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        openMBVAcceleration=ombv.createOpenMBV(); 
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVAngularAcceleration, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::fromPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        openMBVAngularAcceleration=ombv.createOpenMBV(); 
      }
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
      boost::shared_ptr<OpenMBV::Arrow> openMBVrTrans, openMBVrRel;
      boost::shared_ptr<OpenMBV::Arrow> openMBVvTrans, openMBVvRot, openMBVvRel, openMBVvF;
      boost::shared_ptr<OpenMBV::Arrow> openMBVaTrans, openMBVaRot, openMBVaZp, openMBVaCor, openMBVaRel, openMBVaF;
      boost::shared_ptr<OpenMBV::Arrow> openMBVomTrans, openMBVomRel;
      boost::shared_ptr<OpenMBV::Arrow> openMBVpsiTrans, openMBVpsiRot, openMBVpsiRel;
#endif

    public:
      RelativeKinematicsObserver(const std::string &name="");
      void setFrameOfReference(Frame *frame_) { refFrame = frame_; }

      void init(InitStage stage);
      virtual void plot(double t, double dt);
      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}  

#endif

