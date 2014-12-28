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

#ifndef _KINEMATIC_EXCITATION_H_
#define _KINEMATIC_EXCITATION_H_

#include "mbsim/mechanical_link.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {

  class KinematicExcitation : public MechanicalLink {
    protected:
      Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func;
      RigidBody* body;
      Frame C;
    public:
      KinematicExcitation(const std::string &name);
      void updateStateDependentVariables(double t);
      void updateh(double t, int i=0);
      void updateW(double t, int i=0);
      void updateJacobians(double t, int j=0);
      void updatehRef(const fmatvec::Vec &hParent, int j=0);
      void updateWRef(const fmatvec::Mat &WParent, int j=0);
      void setDependentBody(RigidBody* body_) {body = body_;}

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      void init(InitStage stage);
      bool isSetValued() const;
      void calclaSize(int j);
      void calcgSize(int j);
      void calcgdSize(int j);

      void setForceFunction(Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("Force");
      }

      void plot(double t, double dt=1);

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        FArrow=ombv.createOpenMBV(); 
      }
      void setOpenMBVForce(OpenMBV::Arrow *arrow) { FArrow=arrow; }

      /** \brief Visualize a moment arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        MArrow=ombv.createOpenMBV(); 
      }
      void setOpenMBVMoment(OpenMBV::Arrow *arrow) { MArrow=arrow; }
#endif

    protected:
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *FArrow, *MArrow;
#endif

  };

  class GeneralizedPositionExcitation : public KinematicExcitation {
    protected:
      Function<fmatvec::VecV(double)> *f;
    public:
      GeneralizedPositionExcitation(const std::string &name) : KinematicExcitation(name) {}

      void calcxSize();
      void init(InitStage stage);

      void updateStateDependentVariables(double t);
      void updatexd(double t) { }
      void updateg(double t) { }
      void updategd(double t) { }
      void updatewb(double t, int i=0);

      std::string getType() const { return "GeneralizedPositionExcitation"; }

      void setExcitationFunction(Function<fmatvec::VecV(double)>* f_) {
        f = f_;
        f->setParent(this);
        f->setName("Excitation");
      }
  };

  class GeneralizedVelocityExcitation : public KinematicExcitation {
    protected:
      Function<fmatvec::VecV(fmatvec::VecV,double)> *f;
    public:
      GeneralizedVelocityExcitation(const std::string &name) : KinematicExcitation(name) {}

      void calcxSize();
      void init(InitStage stage);

      void updateStateDependentVariables(double t);
      void updatexd(double t);
      void updateg(double t) { }
      void updategd(double t) { }
      void updatewb(double t, int i=0);

      std::string getType() const { return "GeneralizedVelocityExcitation"; }

      void setExcitationFunction(Function<fmatvec::VecV(fmatvec::VecV,double)>* f_) { 
        f = f_;
        f->setParent(this);
        f->setName("Excitation");
      }
      void setExcitationFunction(Function<fmatvec::VecV(fmatvec::VecV)>* f_) {
        setExcitationFunction(new StateDependentFunction<fmatvec::VecV>(f_));
      }
      void setExcitationFunction(Function<fmatvec::VecV(double)>* f_) { 
        setExcitationFunction(new TimeDependentFunction<fmatvec::VecV>(f_));
      }
  };

  class GeneralizedAccelerationExcitation : public KinematicExcitation {
    protected:
      Function<fmatvec::VecV(fmatvec::VecV,double)> *f;
    public:
      GeneralizedAccelerationExcitation(const std::string &name) : KinematicExcitation(name) {}

      void calcxSize();
      void init(InitStage stage);

      void updateStateDependentVariables(double t);
      void updatexd(double t);
      void updateg(double t) { }
      void updategd(double t) { }
      void updatewb(double t, int i=0);

      std::string getType() const { return "GeneralizedAccelerationExcitation"; }

      void setExcitationFunction(Function<fmatvec::VecV(fmatvec::VecV,double)>* f_) { 
        f = f_;
        f->setParent(this);
        f->setName("Excitation");
      }
      void setExcitationFunction(Function<fmatvec::VecV(fmatvec::VecV)>* f_) { 
        setExcitationFunction(new StateDependentFunction<fmatvec::VecV>(f_));
      }
      void setExcitationFunction(Function<fmatvec::VecV(double)>* f_) { 
        setExcitationFunction(new TimeDependentFunction<fmatvec::VecV>(f_));
      }
  };

}

#endif
