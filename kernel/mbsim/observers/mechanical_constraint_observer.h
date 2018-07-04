/* Copyright (C) 2004-2017 MBSim Development Team
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

#ifndef _MECHANICAL_CONSTRAINT_OBSERVER_H__
#define _MECHANICAL_CONSTRAINT_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  BOOST_PARAMETER_NAME(sideOfInteraction)

  class MechanicalConstraint;

  class MechanicalConstraintObserver : public Observer {
    protected:
      MechanicalConstraint* constraint;
      std::string saved_constraint;
      std::shared_ptr<OpenMBVInteractionArrow> ombvForce, ombvMoment;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce, openMBVMoment;
#ifndef SWIG
      double (MechanicalConstraintObserver::*evalOMBVForceColorRepresentation[2])();
      double (MechanicalConstraintObserver::*evalOMBVMomentColorRepresentation[2])();
#endif
      double evalNone() { return 1; }
      double evalAbsoluteForce();
      double evalAbsoluteMoment();

    public:
      MechanicalConstraintObserver(const std::string &name="");
      void setMechanicalConstraint(MechanicalConstraint *constraint_) { constraint = constraint_; } 

      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvForce = std::shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        ombvMoment = std::shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBV::Arrow::toDoubleHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
  };

}  

#endif
