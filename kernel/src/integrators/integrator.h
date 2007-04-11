/* Copyright (C) 2004-2006  Martin Förg
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _INTEGRATOR_H_
#define _INTEGRATOR_H_

#include <fmatvec.h>
#include <string>

namespace MBSim {

  class MultiBodySystem;


  /*! \brief Integrator-interface for multi-body systems 
  */
  class Integrator {

    protected:

      static MultiBodySystem* system;
      double tStart, tEnd, dtPlot;
      fmatvec::Vec z0;
      int warnLevel;
      bool output;
      string name;

    public:

      Integrator();
      virtual ~Integrator() {};

      void settEnd(double tEnd_) {tEnd = tEnd_;}
      void setdtPlot(double dtPlot_) {dtPlot = dtPlot_;}
      void setz0(const fmatvec::Vec &z0_) {z0 = z0_;}
      void setWarnLevel(int level) {warnLevel = level;}
      void setOutput(bool flag) {output = flag;}
      void settStart(double tStart_){tStart=tStart_;}
      /** Start the integration. */
      virtual void integrate(MultiBodySystem& system) = 0;
  };

}

#endif
