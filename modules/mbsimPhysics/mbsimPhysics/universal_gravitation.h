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

#ifndef _UNIVERSAL_GRAVITATION_H
#define _UNIVERSAL_GRAVITATION_H

#include "mbsim/links/mechanical_link.h"

#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"

namespace MBSim {
  class RigidBody;
}

namespace MBSimPhysics {

  class UniversalGravitation : public MBSim::MechanicalLink {
    public:
      UniversalGravitation(const std::string &name="");
      void resetUpToDate();
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void connect(MBSim::RigidBody *body0, MBSim::RigidBody* body1);

      void plot() override;

      void setGravitationalConstant(double ga_) { ga = ga_; }

      void updateGeneralizedPositions() override;
      void updatePositions() override;
      void updatelaF();
      void updateForce() override;
      void updateh(int i=0) override;

      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(); return WrP0P1; }

      bool isActive() const override { return true; }
      bool gActiveChanged() override { return false; }
      bool isSingleValued() const override { return true; }

      void updateWRef(fmatvec::Mat& ref, int i=0) override { }
      void updateVRef(fmatvec::Mat& ref, int i=0) override { }
      void updatehRef(fmatvec::Vec &hRef, int i=0) override;
      virtual void updatedhdqRef(fmatvec::Mat& ref, int i=0);
      virtual void updatedhduRef(fmatvec::SqrMat& ref, int i=0);
      virtual void updatedhdtRef(fmatvec::Vec& ref, int i=0);
      void updaterRef(fmatvec::Vec &ref, int i=0) override { }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (sideOfInteraction,(MBSim::OpenMBVInteractionArrow::SideOfInteraction),MBSim::OpenMBVInteractionArrow::action)(scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(MBSim::OpenMBVArrow::ReferencePoint),MBSim::OpenMBVArrow::toPoint)(colorRepresentation,(MBSim::OpenMBVArrow::ColorRepresentation),MBSim::OpenMBVArrow::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvArrow = std::shared_ptr<MBSim::OpenMBVInteractionArrow>(new MBSim::OpenMBVInteractionArrow(sideOfInteraction,scaleLength,scaleSize,MBSim::OpenMBVArrow::toHead,referencePoint,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

    protected:
      std::vector<MBSim::RigidBody*> body;
      double ga{6.67408e-11};
      fmatvec::Vec3 WrP0P1;
      bool updPos;

      std::shared_ptr<MBSim::OpenMBVInteractionArrow> ombvArrow;
      std::vector<std::shared_ptr<OpenMBV::Arrow>> openMBVForce;

    private:
      std::string saved_ref1, saved_ref2;
 };

}

#endif
