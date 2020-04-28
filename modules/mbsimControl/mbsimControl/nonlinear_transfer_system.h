/* Copyright (C) 204-2020 MBSim Development Team
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

#ifndef _NONLINEAR_TRANSFER_SYSTEM_
#define _NONLINEAR_TRANSFER_SYSTEM_

#include "mbsimControl/signal_.h"
#include "mbsim/functions/function.h"

namespace MBSimControl {

  /*!
   * \brief Nonlinear tansfer system
   * \author Martin Foerg
   */
  class NonlinearTransferSystem : public Signal {

    public:   
      NonlinearTransferSystem(const std::string& name="") : Signal(name) { }

      void initializeUsingXML(xercesc::DOMElement * element) override;
      
      void calcxSize() override { xSize = F->getRetSize().first; }
      
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;

      void updateSignal() override;
      void updatexd() override;
      
      void setSystemFunction(MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *F_) {
        F=F_;
        F->setParent(this);
        F->setName("SystemFunction");
      };
      void setOutputFunction(MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *H_) {
        H=H_;
        H->setParent(this);
        H->setName("OutputFunction");
      };

      void setInputSignal(Signal * inputSignal_) { inputSignal = inputSignal_; }
      int getSignalSize() const override { return H->getRetSize().first; }

    protected:
      Signal* inputSignal{nullptr};
      std::string inputSignalString;
      MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *F{nullptr};
      MBSim::Function<fmatvec::VecV(fmatvec::VecV,fmatvec::VecV)> *H{nullptr};
  };

}

#endif
