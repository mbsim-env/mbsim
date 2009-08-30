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

  class RigidLine;
  class Signal;

  class Controlvalve43 : public Group {
    public:
      Controlvalve43(const std::string& name);

      void setLength(double l_) {l=l_; }
      void setDiameter(double d_) {d=d_; }
      void setLineLength(double l_) {ll=l_; }
      void setLineDiameter(double d_) {ld=d_; }
      void setAlpha(double alpha_) {alpha=alpha_; }
      void setMinimalRelativeArea(double minRelArea_) {minRelArea=minRelArea_; }
      void setOffset(double off) {offset=off; }
      void setPARelativeAreaFunction(Function1<double, double> * relAreaPA_) {relAreaPA=relAreaPA_; } 
      void setPositionSignal(Signal * s) {position = s; }

      RigidLine * getLineP() {return lP; }
      RigidLine * getLineA() {return lA; }
      RigidLine * getLineB() {return lB; }
      RigidLine * getLineT() {return lT; }

      void init(InitStage stage);
      void initializeUsingXML(TiXmlElement * element);
    protected:
      RigidLine * lPA, * lPB, * lAT, * lBT, * lP, * lA, * lB, *lT;
      bool regularized;
      double l, ll, d, ld, alpha, minRelArea, offset;
      Function1<double, double> * relAreaPA;
      Signal * position;
      Signal * checkSizeSignalPA, * checkSizeSignalPB, * checkSizeSignalAT, * checkSizeSignalBT;
  };

  class RegularizedControlvalve43 : public Controlvalve43 {
    public:
      RegularizedControlvalve43(const std::string& name) : Controlvalve43(name) {regularized=true; }
  };

}

#endif   /* ----- #ifndef _CONTROLVALVE43_H_  ----- */

