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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef  _CONTROLVALVE43_H_
#define  _CONTROLVALVE43_H_

#include "mbsim/group.h"
#include <mbsim/functions/function.h>

namespace MBSimHydraulics {

  class ClosableRigidLine;
  class RigidNode;
  class HLine;

  /*! Controlvalve43 */
  class Controlvalve43 : public MBSim::Group {
    public:
      Controlvalve43(const std::string& name="");
      
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement * element) override;

      void setLength(double l_);
      void setDiameter(double d_);
      void setAlpha(double alpha_, double alphaBack_=0);
      void setPARelativeAlphaFunction(MBSim::Function<double(double)> * relAlphaPA_) {
        relAlphaPA=std::shared_ptr<MBSim::Function<double(double)>>(relAlphaPA_);
        relAlphaPA->setParent(this);
        relAlphaPA->setName("PARealtvieAlpha");
      } 
      void setMinimalRelativeAlpha(double minRelAlpha_);
      void setOffset(double off) {offset=off; }
      void setRelativePositionFunction(MBSim::Function<double(double)> * s) {
        position = std::shared_ptr<MBSim::Function<double(double)>>(s);
        position->setParent(this);
        position->setName("Position");
      }
      void setSetValued(bool setValued=true);
      void setPInflow(HLine * hl);
      void setAOutflow(HLine * hl);
      void setBOutflow(HLine * hl);
      void setTOutflow(HLine * hl);
      void printRelativeAlphaCharacteristikCurve(bool print=true) {pRACC=print; }

    protected:
      ClosableRigidLine * lPA, * lPB, * lAT, * lBT;
      RigidNode * nP, * nA, * nB, * nT;
      double offset;
      std::shared_ptr<MBSim::Function<double(double)>> relAlphaPA, position;
      MBSim::Function<double(double)> * checkSizeFunctionPA, * checkSizeFunctionPB, * checkSizeFunctionAT, * checkSizeFunctionBT;
    
    private:
      std::string nPInflowString, nAOutflowString, nBOutflowString, nTOutflowString;
      bool pRACC;
  };
}

#endif   /* ----- #ifndef _CONTROLVALVE43_H_  ----- */

