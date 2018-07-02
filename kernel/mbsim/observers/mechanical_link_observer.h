/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _MECHANICAL_LINK_OBSERVER_H__
#define _MECHANICAL_LINK_OBSERVER_H__

#include "mbsim/observers/observer.h"

#include <mbsim/utils/boost_parameters.h>
#include <mbsim/utils/openmbv_utils.h>
#include <mbsim/utils/index.h>

namespace MBSim {

  BOOST_PARAMETER_NAME(sideOfInteraction)

  class MechanicalLink;

  class MechanicalLinkObserver : public Observer {
    public:
      enum SideOfInteraction {
        action=0,
        reaction,
        both,
        unknown
      };

    protected:
      MechanicalLink* link;
      SideOfInteraction sideOfForceInteraction{action}, sideOfMomentInteraction{action};
      std::string saved_link;
      std::shared_ptr<OpenMBVArrow> ombvForce, ombvMoment;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce, openMBVMoment;

    public:
      MechanicalLinkObserver(const std::string &name="");
      void setMechanicalLink(MechanicalLink *link_) { link = link_; } 

      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(sideOfInteraction,(SideOfInteraction),action))) {
        sideOfForceInteraction = sideOfInteraction;
        ombvForce = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBV::Arrow::toHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
      void setOMBVForce(const std::shared_ptr<OpenMBVArrow> &arrow) { ombvForce=arrow; }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(sideOfInteraction,(SideOfInteraction),action))) {
        sideOfMomentInteraction = sideOfInteraction;
        ombvMoment = std::shared_ptr<OpenMBVArrow>(new OpenMBVArrow(scaleLength,scaleSize,OpenMBV::Arrow::toDoubleHead,referencePoint,minimalColorValue,maximalColorValue,diffuseColor,transparency));
      }
      void setOMBVMoment(const std::shared_ptr<OpenMBVArrow> &arrow) { ombvMoment=arrow; }
  };

}  

#endif
