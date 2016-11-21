/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _GENERALIZED_CONSTRAINT_H
#define _GENERALIZED_CONSTRAINT_H

#include "constraint.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {

  /** 
   * \brief Class for generalized constraints
   * \author Martin Foerg
   */
  class GeneralizedConstraint : public Constraint {
    protected:
      Frame *support;

      std::shared_ptr<OpenMBV::Arrow> FArrow, MArrow;

    public:
      GeneralizedConstraint(const std::string &name) : Constraint(name), support(NULL) { }

      void init(InitStage stage);

      virtual void setSupportFrame(Frame *frame) { support = frame; }

      void initializeUsingXML(xercesc::DOMElement * element);

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

    private:
      std::string saved_DependentBody, saved_supportFrame;
  };

}

#endif
