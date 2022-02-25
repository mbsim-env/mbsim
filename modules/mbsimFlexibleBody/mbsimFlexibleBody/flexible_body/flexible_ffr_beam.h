/* Copyright (C) 2004-2020 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FLEXIBLE_BEAM_H_
#define _FLEXIBLE_BEAM_H_

#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"
#include "mbsim/utils/boost_parameters.h"

namespace MBSim {

  BOOST_PARAMETER_NAME(visualization)
}

namespace MBSimFlexibleBody {

  /*!
   *  \brief Flexible beam using a floating frame of reference formulation
   *
   * */
  class FlexibleFfrBeam : public GenericFlexibleFfrBody {

    public:
      FlexibleFfrBeam(const std::string &name="") : GenericFlexibleFfrBody(name) { }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void setNumberOfNodes(int nN_) { nN = nN_; }
      void setLength(double l_) { l = l_; }
      void setCrossSectionArea(double A_) { A = A_; }
      void setMomentOfInertia(const fmatvec::Vec3 &I) { Iy=I(0); Iz=I(1); Iyz=I(2); }
      void setYoungsModulus(double E_) { E = E_; }
      void setDensity(double rho_) { rho = rho_; }
      void setBoundaryConditions(const fmatvec::MatVx3 &bc_) { bc <<= bc_; }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (visualization,(OpenMBVFlexibleFfrBeam::Visualization),OpenMBVFlexibleFfrBeam::points)(colorRepresentation,(OpenMBVFlexibleBody::ColorRepresentation),OpenMBVFlexibleBody::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvBody = std::shared_ptr<OpenMBVFlexibleFfrBeam>(new OpenMBVFlexibleFfrBeam(visualization,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
      void setPlotNodeNumbers(const fmatvec::VecVI &plotNodes_) { plotNodes <<= plotNodes_; }
      void setTension(bool ten_) { ten = ten_; }
      void setBendingAboutZAxis(bool benz_) { benz = benz_; }
      void setBendingAboutYAxis(bool beny_) { beny = beny_; }
      void setTorsion(bool tor_) { tor = tor_; }
      void setInterfaceNodeNumbers(const fmatvec::VecVI &inodes_) { inodes <<= inodes_; }
      void setNormalModeNumbers(const fmatvec::VecVI &nmodes_) { nmodes <<= nmodes_; }
      void setFixedBoundaryNormalModes(bool fixedBoundaryNormalModes_) { fixedBoundaryNormalModes = fixedBoundaryNormalModes_; }

    private:
      int nN{3};
      double l{1};
      double A{1e-4};
      double Iy{1e-10};
      double Iz{1e-10};
      double Iyz{0};
      double E{2e11};
      double rho{7870};
      fmatvec::MatVx3 bc;
      std::shared_ptr<OpenMBVFlexibleFfrBeam> ombvBody;
      bool ten{false};
      bool benz{true};
      bool beny{true};
      bool tor{false};
      fmatvec::VecVI inodes;
      fmatvec::VecVI nmodes;
      bool fixedBoundaryNormalModes{false};
  };

}

#endif
