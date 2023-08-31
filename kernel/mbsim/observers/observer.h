/* Copyright (C) 2004-2013 MBSim Development Team
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

#ifndef _OBSERVER_H__
#define _OBSERVER_H__
#include "mbsim/element.h"

namespace MBSim {

  class Observer : public Element {
    private:
      std::shared_ptr<OpenMBV::Group> openMBVGrp;

    public:
      Observer(const std::string &name);

      void init(InitStage stage, const InitConfigSet &config) override;

      std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() override { return openMBVGrp; }

      void createPlotGroup() override;

      int getisInd() { return isInd; }
      int getisSize() { return isSize; }
      virtual void calcisSize() { isSize = 0; }

      /**
       * This function is called just before the internal state vector gets updated.
       * This means that the internal state is still at its old value when this function is called but get updated immediately afterwards.
       * You can use this function to do actions which should only be done at valid integrator states (but not at integrates states which may be rejected later on).
       */
      virtual void aboutToUpdateInternalState() {}

      virtual void postprocessing() {}

      virtual void setisInd(int isInd_) { isInd = isInd_; }
      virtual void updateInternalStateRef(fmatvec::Vec& cur, fmatvec::Vec& next);

    protected:
      fmatvec::Vec curis, nextis;
      int isSize { 0 }, isInd { 0 };
  };

}  

#endif
