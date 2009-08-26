/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _HYDLEAKAGE_H_
#define  _HYDLEAKAGE_H_

#include "hydline.h"
#include "pressure_loss.h"

namespace MBSim {

  class HydLeakage : public HydLine {
    public:
      HydLeakage(const std::string &name);
      virtual std::string getType() const { return "HydLeakage"; }
      
      void init(InitStage stage);
      
      void setGeometry(double lGap, double hGap, double wGap);
    
    private:
      double lGap, hGap, wGap;
  };


  class LeakagePressureLoss : public PressureLoss {
    public:
      LeakagePressureLoss(const std::string &name);
      virtual void transferLeakageGapData(double lGap, double hGap, double wGap) {};
  };

  class LeakagePressureLossHagenPoiseuille : public LeakagePressureLoss {
    public:
      LeakagePressureLossHagenPoiseuille(const std::string &name);
      virtual void transferLeakageGapData(double lGap, double hGap, double wGap);

      double operator()(double Q);
    private:
      double lossFactor;
  };

}

#endif   /* ----- #ifndef _HYDLEAKAGE_H_  ----- */

