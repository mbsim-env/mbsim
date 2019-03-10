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

#ifndef _WEIGHT_H
#define _WEIGHT_H

#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/floating_relative_frame.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {
  class RigidBody;
  class Frame;
  template <class Sig> class Function;
}

namespace MBSimPhysics {

  class Weight : public MBSim::MechanicalLink {
    public:
      Weight(const std::string &name="");
      ~Weight() override;
      void resetUpToDate() override;
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void connect(MBSim::RigidBody *body_) { body = body_; }
      void connect(MBSim::Frame* frame_, MBSim::RigidBody *body_);

      void plot() override;

      void setGravityFunction(MBSim::Function<double(double)> *func);

      void updatePositions(MBSim::Frame *frame) override;
      void updateGeneralizedPositions() override;
      void updatePositions() override;
      void updatelaF();
      void updateForce() override;
      void updateh(int i=0) override;

      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(); return WrP0P1; }

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      bool isSingleValued() const override { return true; }

      void updateWRef(const fmatvec::Mat& ref, int i=0) override { }
      void updateVRef(const fmatvec::Mat& ref, int i=0) override { }
      void updatehRef(const fmatvec::Vec &hRef, int i=0) override;
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0);
      void updaterRef(const fmatvec::Vec &ref, int i=0) override { }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (sideOfInteraction,(MBSim::OpenMBVInteractionArrow::SideOfInteraction),MBSim::OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(MBSim::OpenMBVArrow::ReferencePoint),MBSim::OpenMBVArrow::toPoint)(colorRepresentation,(MBSim::OpenMBVArrow::ColorRepresentation),MBSim::OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvArrow = std::shared_ptr<MBSim::OpenMBVInteractionArrow>(new MBSim::OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,MBSim::OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

    protected:
      MBSim::RigidBody* body{nullptr};
      MBSim::Frame* frame{nullptr};
      MBSim::Function<double(double)> *fg{nullptr};
      fmatvec::Vec3 WrP0P1;
      bool updPos;

      MBSim::FloatingRelativeFrame C;

      std::shared_ptr<MBSim::OpenMBVInteractionArrow> ombvArrow;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce;

    private:
      std::string saved_ref1, saved_ref2;
  };

}

#endif
