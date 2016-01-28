/* Copyright (C) 2004-2015 MBSim Development Team
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

#include "mbsim/rigid_body_link.h"
#include "mbsim/functions/auxiliary_functions.h"

namespace MBSim {

  class RigidBody;

  class KinematicExcitation : public RigidBodyLink {
    protected:
      Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func;
      std::string saved_DependentBody;
    public:
      KinematicExcitation(const std::string &name);
      void updateGeneralizedForce(double t);
      void setDependentBody(RigidBody* body_) { body[0] = body_; }

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      void init(InitStage stage);
      bool isSetValued() const { return func?false:true; }
      void calclaSize(int j);
      void calcgSize(int j);
      void calcgdSize(int j);

      void setForceFunction(Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *func_) {
        func=func_;
        func->setParent(this);
        func->setName("Force");
      }
  };

  class GeneralizedPositionExcitation : public KinematicExcitation {
    protected:
      Function<fmatvec::VecV(double)> *f;
    public:
      GeneralizedPositionExcitation(const std::string &name) : KinematicExcitation(name) {}

      void calcxSize();
      void init(InitStage stage);

      void updateGeneralizedPositions(double t);
      void updateGeneralizedVelocities(double t);
      void updatexd(double t) { }
      void updatewb(double t);

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

      void updateGeneralizedPositions(double t);
      void updateGeneralizedVelocities(double t);
      void updatexd(double t);
      void updatewb(double t);

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

      void updateGeneralizedPositions(double t);
      void updateGeneralizedVelocities(double t);
      void updatexd(double t);
      void updatewb(double t);

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
