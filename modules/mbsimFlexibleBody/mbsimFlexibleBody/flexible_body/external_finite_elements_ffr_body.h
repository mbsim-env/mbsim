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

#ifndef _EXTERNAL_FINITE_ELEMENTS_BODY_H_
#define _EXTERNAL_FINITE_ELEMENTS_BODY_H_

#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"
#include "mbsim/utils/boost_parameters.h"

namespace MBSim {

  BOOST_PARAMETER_NAME(visualization)
}

namespace MBSimFlexibleBody {

  /*!
   *  \brief Flexible body consisting of finite elements from an external FE software
   *
   * */
  class ExternalFiniteElementsFfrBody : public GenericFlexibleFfrBody {

    public:
      enum Formalism {
        consistentMass=0,
        lumpedMass,
        unknown
      };

      ExternalFiniteElementsFfrBody(const std::string &name="") : GenericFlexibleFfrBody(name) { }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void setNodes(const fmatvec::MatV &nodes_) { nodes <<= nodes_; }
      void setNodeNumbers(const fmatvec::VecVI &nodeNum);
      void setMassMatrix(const fmatvec::MatVx3 &M_) { M <<= M_; }
      void setStiffnessMatrix(const fmatvec::MatVx3 &K_) { K <<= K_; }
      void setNumberOfNodalTranslationalDegreesOfFreedom(int net_) { net = net_; }
      void setNumberOfNodalRotationalDegreesOfFreedom(int ner_) { ner = ner_; }
      void setFormalism(Formalism formalism_) { formalism = formalism_; }
      void addBoundaryNodes(const fmatvec::VecVI &bnodes_) { bnodes.emplace_back(bnodes_); }
      void addDegreesOfFreedom(const fmatvec::VecVI &dof_) { dof.emplace_back(dof_); }
      void setInterfaceNodeNumbers(const fmatvec::VecVI &inodes_) { inodes <<= inodes_; }
      void setNormalModeNumbers(const fmatvec::VecVI &nmodes_) { nmodes <<= nmodes_; }
      void setFixedBoundaryNormalModes(bool fixedBoundaryNormalModes_) { fixedBoundaryNormalModes = fixedBoundaryNormalModes_; }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (visualization,(OpenMBVFlexibleFfrBody::Visualization),OpenMBVFlexibleFfrBody::points)(colorRepresentation,(OpenMBVFlexibleBody::ColorRepresentation),OpenMBVFlexibleBody::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvBody = std::shared_ptr<OpenMBVFlexibleFfrBody>(new OpenMBVFlexibleFfrBody(visualization,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
      void setPlotNodeNumbers(const fmatvec::VecVI &plotNodes_) { plotNodes <<= plotNodes_; }

    private:
      void importData();

      fmatvec::MatVx3 M, K;
      fmatvec::MatV nodes;
      std::map<int,fmatvec::Vec3> nodalPos;
      Formalism formalism{lumpedMass};
      std::map<int,fmatvec::VecVI> bc;
      std::vector<fmatvec::VecVI> bnodes;
      std::vector<fmatvec::VecVI> dof;
      std::shared_ptr<OpenMBVFlexibleFfrBody> ombvBody;
      int net{3};
      int ner{0};
      fmatvec::VecVI inodes;
      fmatvec::VecVI nmodes;
      bool fixedBoundaryNormalModes{false};
  };

}

#endif
