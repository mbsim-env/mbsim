/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _LINEAR_ELASTIC_FUNCTION_H_
#define _LINEAR_ELASTIC_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /**
   * \brief tbd
   * \author Martin Foerg
   */
  class LinearElasticFunction : public Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> {
    public:
      /** 
       * \brief standard constructor
       */
      LinearElasticFunction() = default;

      /** 
       * \brief constructor
       * \param stiffness matrix
       * \param damping matrix
       */
      LinearElasticFunction(const fmatvec::SymMatV &K_, const fmatvec::SymMatV &D_) : K(K_), D(D_) { }

      std::pair<int, int> getRetSize() const override { return std::make_pair(K.size(),1); }

      void init(InitStage stage, const InitConfigSet &config) override;

      fmatvec::VecV operator()(const fmatvec::VecV& q, const fmatvec::VecV& u) override { return K*q + D*u; }

      void setStiffnessMatrix(const fmatvec::SymMatV &K_) { K = K_; }
      void setDampingMatrix(const fmatvec::SymMatV &D_) { D = D_; }

      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      fmatvec::SymMatV K, D;
  };

}

#endif
