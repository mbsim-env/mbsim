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

  class MechanicalLink;

  class MechanicalLinkObserver : public Observer {
    protected:
      MechanicalLink* link;
      std::string saved_link;
      std::shared_ptr<OpenMBVInteractionArrow> ombvForce, ombvMoment;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce, openMBVMoment;
#ifndef SWIG
      double (MechanicalLinkObserver::*evalOMBVForceColorRepresentation[2])();
      double (MechanicalLinkObserver::*evalOMBVMomentColorRepresentation[2])();
#endif
      double evalNone() { return 1; }
      double evalAbsoluteForce();
      double evalAbsoluteMoment();
      Frame *outputFrame { nullptr };

    public:
      MechanicalLinkObserver(const std::string &name="");
      void setMechanicalLink(MechanicalLink *link_) { link = link_; } 
      void setOutputFrame(Frame *outputFrame_) { outputFrame = outputFrame_; }

      void init(InitStage stage, const InitConfigSet &config) override;
      void plot() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVForce, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvForce = std::shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
      void setOMBVForce(const std::shared_ptr<OpenMBVInteractionArrow> &arrow) { ombvForce=arrow; }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVMoment, tag, (optional (sideOfInteraction,(OpenMBVInteractionArrow::SideOfInteraction),OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBVArrow::ReferencePoint),OpenMBVArrow::toPoint)(colorRepresentation,(OpenMBVArrow::ColorRepresentation),OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvMoment = std::shared_ptr<OpenMBVInteractionArrow>(new OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,OpenMBVArrow::toDoubleHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
      void setOMBVMoment(const std::shared_ptr<OpenMBVInteractionArrow> &arrow) { ombvMoment=arrow; }
    private:
      std::string saved_outputFrame;
  };

}  

#endif
