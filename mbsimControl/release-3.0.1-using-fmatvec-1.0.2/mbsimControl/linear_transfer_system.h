/* Copyright (C) 2006  Mathias Bachmayer

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
 *
 * Contact:
 *   mbachmayer@gmx.de
 *
 */ 

#ifndef _LINEAR_TRANSFER_SYSTEM_
#define _LINEAR_TRANSFER_SYSTEM_

#include "mbsimControl/signal_processing_system.h"

namespace MBSimControl {

  /*!
   * \brief LinearTransferSystem
   * \author Markus Schneider
   */
  class LinearTransferSystem : public SignalProcessingSystem {

    public:   
      LinearTransferSystem(const std::string& name);
      virtual std::string getType() const {return "LinearTransferSystem"; }
      void initializeUsingXML(TiXmlElement * element);
      
      void calcxSize() {xSize=A.rows(); }
      
      void init(MBSim::InitStage stage);

      void updatedx(double t, double dt);
      void updatexd(double t);
      
      void plot(double t,double dt);
     
      void setPID(double P_, double I_, double D_);
      void setABCD(fmatvec::Mat A_,fmatvec::Mat B_,fmatvec::Mat C_,fmatvec::Mat D_);
      void setBandwidth(double Hz_fg);
      void setIntegrator(double OutputGain);
      void setI2(double OutputGain);
      void setPT1(double P, double T);
      void setGain(double P);
      void showABCD();

    protected:
      fmatvec::Mat A,B,C,D;
      double R1,R2,c;
      fmatvec::Vec calculateOutput();
      fmatvec::Vec (LinearTransferSystem::*calculateOutputMethod)();
      fmatvec::Vec outputMethodC();
      fmatvec::Vec outputMethodD();
      fmatvec::Vec outputMethodCD();
  };
}

#endif
