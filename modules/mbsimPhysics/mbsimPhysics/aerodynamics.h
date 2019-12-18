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

#ifndef _AERODYNAMICS_H
#define _AERODYNAMICS_H

#include "mbsim/links/floating_frame_link.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {
  class Frame;
  template <class Sig> class Function;
}

namespace MBSimPhysics {

  class Aerodynamics : public MBSim::FloatingFrameLink {
    public:
      Aerodynamics(const std::string &name="");
      ~Aerodynamics() override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void plot() override;

      using FloatingFrameLink::connect;

      void connect(MBSim::Frame *frame_) { frame[1] = frame_; }

      void setDensityFunction(MBSim::Function<double(double)> *func);
      void setCoefficientFunction(MBSim::Function<fmatvec::Vec3(fmatvec::Vec2)> *func);
      void setReferenceSurface(double A_) { A = A_; }
      void setWindSpeed(const fmatvec::Vec3 &vW_) { vW.assign(vW_); }

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
      MBSim::Function<double(double)> * frho{nullptr};
      MBSim::Function<fmatvec::Vec3(fmatvec::Vec2)> * fc{nullptr};
      double A{1};
      fmatvec::Vec3 vW;

      std::shared_ptr<MBSim::OpenMBVInteractionArrow> ombvArrow;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce;
  };

}

#endif
