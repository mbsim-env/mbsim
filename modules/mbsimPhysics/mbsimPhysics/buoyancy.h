/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef _BUOYANCY_H
#define _BUOYANCY_H

#include "mbsim/links/floating_frame_link.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {
  class Frame;
  template <class Sig> class Function;
}

namespace MBSimPhysics {

  class Buoyancy : public MBSim::FloatingFrameLink {
    public:
      Buoyancy(const std::string &name="");
      ~Buoyancy() override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void plot() override;

      using FloatingFrameLink::connect;

      void connect(MBSim::Frame *frame_) { frame[1] = frame_; }

      void setDisplacedVolume(double V_) { V = V_; }
      void setDensityFunction(MBSim::Function<double(double)> *func);
      void setGravityFunction(MBSim::Function<double(double)> *func);

      void updateGeneralizedPositions() override { updrrel = false; }
      void updateGeneralizedVelocities() override { updvrel = false; }
      void updatelaF() override;

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      bool isSingleValued() const override { return true; }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (sideOfInteraction,(MBSim::OpenMBVInteractionArrow::SideOfInteraction),MBSim::OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(MBSim::OpenMBVArrow::ReferencePoint),MBSim::OpenMBVArrow::toPoint)(colorRepresentation,(MBSim::OpenMBVArrow::ColorRepresentation),MBSim::OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvArrow = std::shared_ptr<MBSim::OpenMBVInteractionArrow>(new MBSim::OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,MBSim::OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

    protected:
      double V{0};
      MBSim::Function<double(double)> *frho{nullptr};
      MBSim::Function<double(double)> *fg{nullptr};

      std::shared_ptr<MBSim::OpenMBVInteractionArrow> ombvArrow;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce;
  };

}

#endif
