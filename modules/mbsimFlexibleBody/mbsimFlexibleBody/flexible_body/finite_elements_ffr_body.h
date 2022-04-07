/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _FINITE_ELEMENTS_FFR_BODY_H_
#define _FINITE_ELEMENTS_FFR_BODY_H_

#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"
#include "mbsim/utils/boost_parameters.h"

namespace MBSim {

  BOOST_PARAMETER_NAME(visualization)
}

namespace MBSimFlexibleBody {

  /*!
   *  \brief Flexible body consisting of finite elements using a floating frame of reference formulation
   *
   * */
  class FiniteElementsFfrBody : public GenericFlexibleFfrBody {

    public:
      enum ElementType {
        C3D20=0,
        unknownElementType
      };

      FiniteElementsFfrBody(const std::string &name="") : GenericFlexibleFfrBody(name), xi(fmatvec::NONINIT), wi(fmatvec::NONINIT) { }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void setYoungsModulus(double E_) { E = E_; }
      void setPoissonsRatio(double nu_) { nu = nu_; }
      void setDensity(double rho_) { rho = rho_; }
      void setNodes(const fmatvec::MatV &nodes_) { nodes <<= nodes_; }
      void setElements(const fmatvec::MatVI &elements_) { elements <<= elements_; }
      void setElementType(ElementType type_) { type = type_; }
      void setBoundaryConditions(const fmatvec::MatVx3 &bc_) { bc <<= bc_; }
      void setInterfaceNodeNumbers(const fmatvec::VecVI &inodes_) { inodes <<= inodes_; }
      void setNormalModeNumbers(const fmatvec::VecVI &nmodes_) { nmodes <<= nmodes_; }
      void setFixedBoundaryNormalModes(bool fixedBoundaryNormalModes_) { fixedBoundaryNormalModes = fixedBoundaryNormalModes_; }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (visualization,(OpenMBVFiniteElementsBody::Visualization),OpenMBVFiniteElementsBody::faces)(colorRepresentation,(OpenMBVFlexibleBody::ColorRepresentation),OpenMBVFlexibleBody::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvBody = std::shared_ptr<OpenMBVFiniteElementsBody>(new OpenMBVFiniteElementsBody(visualization,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
      void setPlotNodeNumbers(const fmatvec::VecVI &plotNodes_) { plotNodes <<= plotNodes_; }
      std::map<int,double> getWeightingFactors(const fmatvec::VecVI &elesel, int faceNum);

    private:
      double N1(double x, double y, double z, int i);
      double N2(double x, double y, double z, int i);
      double N3(double x, double y, double z, int i);
      double N4(double x, double y, double z, int i);
      double dN1dxq(double x, double y, double z, int i);
      double dN1dyq(double x, double y, double z, int i);
      double dN1dzq(double x, double y, double z, int i);
      double dN2dxq(double x, double y, double z, int i);
      double dN2dyq(double x, double y, double z, int i);
      double dN2dzq(double x, double y, double z, int i);
      double dN3dxq(double x, double y, double z, int i);
      double dN3dyq(double x, double y, double z, int i);
      double dN3dzq(double x, double y, double z, int i);
      double dN4dxq(double x, double y, double z, int i);
      double dN4dyq(double x, double y, double z, int i);
      double dN4dzq(double x, double y, double z, int i);
#ifndef SWIG
      double (FiniteElementsFfrBody::*Ni[20])(double x, double y, double z, int i);
      double (FiniteElementsFfrBody::*dNidq[20][3])(double x, double y, double z, int i);
#endif

      double E{2e11};
      double rho{7870};
      double nu{0.3};
      ElementType type{C3D20};
      fmatvec::MatV nodes;
      fmatvec::MatVI elements;
      std::map<int,fmatvec::Vec3> nodalPos;
      std::map<int,fmatvec::VecVI> ele;
      std::vector<fmatvec::Vec3> rN;
      fmatvec::Vec3 xi;
      fmatvec::Vec3 wi;
      fmatvec::MatVx3 bc;
      std::shared_ptr<OpenMBVFiniteElementsBody> ombvBody;
      fmatvec::VecVI inodes;
      fmatvec::VecVI nmodes;
      bool fixedBoundaryNormalModes{false};
  };

}

#endif
