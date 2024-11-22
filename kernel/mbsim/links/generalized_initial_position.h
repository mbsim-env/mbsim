/* Copyright (C) 2004-2024 MBSim Development Team
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

#ifndef _GENERALIZED_INITIAL_POSITION_H_
#define _GENERALIZED_INITIAL_POSITION_H_

#include "mbsim/links/initial_condition.h"

namespace MBSim {

  using Index = int;
  class Object;

  class GeneralizedInitialPosition : public InitialCondition {
    protected:
      Object *object;
      std::string objectString;
      std::vector<Index> indices;
      fmatvec::VecV q0;
      fmatvec::MatV JRel;
    public:
      GeneralizedInitialPosition(const std::string &name="");
      ~GeneralizedInitialPosition() override { }

      void setObject(Object *object_) { object = object_; }
      void setIndices(const std::vector<Index> &indices_) { indices = indices_; }
      void setValues(const fmatvec::Vec &q0_) { q0 <<= q0_; }

      void calcSize() override;
      void calclaSize(int j) override;
      void calccorrSize(int j) override;

      void updateGeneralizedPositions() override;

      void updateg() override;
      void updateW(int i=0) override;

      void updateWRef(fmatvec::Mat &WParent, int j=0) override;

      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement * element) override;
  };

}

#endif
