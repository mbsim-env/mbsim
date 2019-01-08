/* Copyright (C) 2004-2019 MBSim Development Team
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

#ifndef _CALCULIX_BODY_H_
#define _CALCULIX_BODY_H_

#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"
#include "mbsim/utils/boost_parameters.h"

namespace MBSimFlexibleBody {

  /*!
   *  \brief Flexible body using a floating frame of reference formulation
   *
   * */
  class CalculixBody : public GenericFlexibleFfrBody {

    public:
      CalculixBody(const std::string &name="") : GenericFlexibleFfrBody(name) { }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void setResultFileName(const std::string &resultFileName_) { resultFileName = resultFileName_; }
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (colorRepresentation,(OpenMBVFlexibleBody::ColorRepresentation),OpenMBVFlexibleBody::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombvBody = std::shared_ptr<OpenMBVCalculixBody>(new OpenMBVCalculixBody(colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }

    protected:
      std::string resultFileName;

    private:
      void readDisplacements();
      void readStresses();
      void readNodes();
      void readElements();
      void readModes();
      void readDOF();
      void readStiffMatrix();
      void readMassMatrix();
      void importData();
      std::ifstream isRes, isStiff, isMass, isDOF;
      size_t nn{0}, ne{0}, nm{1000};
      fmatvec::VecV u0;
      fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Var,int> eles;
      fmatvec::SymMatV M, K;
      std::vector<std::pair<size_t,size_t>> dof;
      std::vector<fmatvec::VecV> disp;
      std::vector<fmatvec::VecV> stress;
      std::vector<MBSim::Index> ombvIndices;
      std::shared_ptr<OpenMBVCalculixBody> ombvBody;
  };

}

#endif
