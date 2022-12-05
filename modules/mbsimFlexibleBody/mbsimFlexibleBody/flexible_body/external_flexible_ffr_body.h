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

#ifndef _EXTERNAL_FLEXIBLE_FFR_BODY_H_
#define _EXTERNAL_FLEXIBLE_FFR_BODY_H_

#include "mbsimFlexibleBody/flexible_body/generic_flexible_ffr_body.h"
#include "mbsim/utils/boost_parameters.h"

namespace MBSim {

  BOOST_PARAMETER_NAME(visualization)
}

namespace MBSimFlexibleBody {

  /*!
   *  \brief Flexible body using a floating frame of reference formulation. The model is created externally and saved in a H5-file.
   *
   * */
  class ExternalFlexibleFfrBody : public GenericFlexibleFfrBody {

    public:
      ExternalFlexibleFfrBody(const std::string &name="") : GenericFlexibleFfrBody(name) { }

      void setInputDataFile(const std::string& inputDataFile_) { inputDataFile = inputDataFile_; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (visualization,(OpenMBVExternalFlexibleFfrBody::Visualization),OpenMBVExternalFlexibleFfrBody::points)(colorRepresentation,(OpenMBVFlexibleBody::ColorRepresentation),OpenMBVFlexibleBody::none)(minimalColorValue,(double),0)(maximalColorValue,(double),1)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
	ombvBody = std::shared_ptr<OpenMBVExternalFlexibleFfrBody>(new OpenMBVExternalFlexibleFfrBody(visualization,colorRepresentation,minimalColorValue,maximalColorValue,diffuseColor,transparency,pointSize,lineWidth));
      }
      void setOpenMBVNodeNumbers(const std::vector<int> &visuNodes_) { visuNodes = visuNodes_; }
      void setPlotNodeNumbers(const std::vector<int> &plotNodes_) { plotNodes = plotNodes_; }

    private:
      std::string inputDataFile;
      std::shared_ptr<OpenMBVExternalFlexibleFfrBody> ombvBody;
      void importData();
      std::vector<int> ombvIndices;
  };

}

#endif
