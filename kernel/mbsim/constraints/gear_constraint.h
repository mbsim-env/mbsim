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

#ifndef _GEAR_CONSTRAINT_H
#define _GEAR_CONSTRAINT_H

#include "mbsim/constraints/constraint.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

namespace MBSim {

  class RigidBody;
  class Frame;
  class Transmission;

  class GearConstraint : public Constraint {

    public:
      GearConstraint(const std::string &name="");

      void addTransmission(const Transmission &transmission);

      void init(InitStage stage);

      void setDependentRigidBody(RigidBody* body_) {bd=body_; }

      void updateGeneralizedCoordinates();
      void updateGeneralizedJacobians(int j=0);
      void setUpInverseKinetics();

      void initializeUsingXML(xercesc::DOMElement * element);

      virtual std::string getType() const { return "GearConstraint"; }
    
#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow acting on frame2 */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        FArrow=ombv.createOpenMBV();
      }

      /** \brief Visualize a moment arrow */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toDoubleHead,referencePoint,scaleLength,scaleSize);
        MArrow=ombv.createOpenMBV();
      }
#endif

    private:
      std::vector<RigidBody*> bi;
      RigidBody *bd;
      std::vector<double> ratio;

      std::string saved_DependentBody;
      std::vector<std::string> saved_IndependentBody;

#ifdef HAVE_OPENMBVCPPINTERFACE
      std::shared_ptr<OpenMBV::Arrow> FArrow, MArrow;
#endif
  };

}

#endif
