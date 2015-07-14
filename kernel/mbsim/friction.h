/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef _FRICTION_H_
#define _FRICTION_H_

#include "mbsim/mechanical_link.h"
#include <mbsim/frame.h>
#include "mbsim/functions/function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {

  class RigidBody;
  class FrictionForceLaw;

  class GeneralizedFriction : public MechanicalLink {
    protected:
      //Function<double(double,double)> *func;
      FrictionForceLaw *func;
      Function<double(double)> *laN;
      std::vector<RigidBody*> body;
    public:
      GeneralizedFriction(const std::string &name="");
      ~GeneralizedFriction();
      void updateGeneralizedVelocities(double t);
      void updateForceDirections(double t);
      void updateGeneralizedSingleValuedForces(double t);
      void updateg(double t) { }
      void updategd(double t) { gd = getGeneralizedRelativeVelocity(t); }
      void updateh(double t, int i=0);

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      virtual bool isSingleValued() const { return true; }
      std::string getType() const { return "GeneralizedFriction"; }
      void init(InitStage stage);

      /** \brief Set the function for the torque calculation. */
      //void setGeneralizedForceFunction(Function<double(double,double)> *func_) { func=func_; }
      void setGeneralizedFrictionForceLaw(FrictionForceLaw *func_);
      void setGeneralizedNormalForceFunction(Function<double(double)> *laN_) { 
        laN = laN_; 
        laN->setParent(this);
      }

      void setRigidBodyFirstSide(RigidBody* body_) { body[0] = body_; }
      void setRigidBodySecondSide(RigidBody* body_) { body[1] = body_; }

      void initializeUsingXML(xercesc::DOMElement *element);

      void updatehRef(const fmatvec::Vec &hParent, int j=0);

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow acting on each of both connected frames */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        MechanicalLink::setOpenMBVForce(ombv.createOpenMBV());
      }

      /** \brief Visualize a torque arrow acting on each of both connected frames */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        MechanicalLink::setOpenMBVMoment(ombv.createOpenMBV());
      }
#endif
    private:
      std::string saved_body1, saved_body2;
  };

}

#endif 

