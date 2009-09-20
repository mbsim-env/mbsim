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

#ifndef  _CONTROLVALVE43_H_
#define  _CONTROLVALVE43_H_

#include "mbsim/group.h"
#include "mbsim/utils/function.h"

namespace MBSim {

  class ClosableRigidLine;
  class RigidNode;
  class Signal;
  class HLine;

  /*! Controlvalve43 */
  class Controlvalve43 : public Group {
    public:
      Controlvalve43(const std::string& name);
      HLine * getHLineByPath(std::string path);
      Signal * getSignalByPath(std::string path);
      void init(InitStage stage);
      void initializeUsingXML(TiXmlElement * element);

      void setLength(double l_);
      void setDiameter(double d_);
      void setAlpha(double alpha_);
      void setPARelativeAlphaFunction(Function1<double, double> * relAlphaPA_) {relAlphaPA=relAlphaPA_; } 
      void setMinimalRelativeAlpha(double minRelAlpha_);
      void setOffset(double off) {offset=off; }
      void setRelativePositionSignal(Signal * s) {position = s; }
      void setSetValued(bool setValued=true);
      void setPInflow(HLine * hl);
      void setAOutflow(HLine * hl);
      void setBOutflow(HLine * hl);
      void setTOutflow(HLine * hl);

    protected:
      ClosableRigidLine * lPA, * lPB, * lAT, * lBT;
      RigidNode * nP, * nA, * nB, * nT;
      double offset;
      Function1<double, double> * relAlphaPA;
      Signal * position;
      Signal * checkSizeSignalPA, * checkSizeSignalPB, * checkSizeSignalAT, * checkSizeSignalBT;
    
    private:
      std::string positionString;
      std::string nPInflowString, nAOutflowString, nBOutflowString, nTOutflowString;
  };
}

#endif   /* ----- #ifndef _CONTROLVALVE43_H_  ----- */

