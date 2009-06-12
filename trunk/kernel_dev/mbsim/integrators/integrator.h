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

#define MBSIMINTNS "{http://mbsim.berlios.de/MBSimIntegrator}"

#include<fmatvec.h>
#include<string>
#include"mbsimtinyxml/tinyxml-src/tinyxml.h"

namespace MBSim {

  class DynamicSystemSolver;


  /*! \brief Integrator-interface for dynamic systems */
  class Integrator {

    protected:
      static DynamicSystemSolver* system;
      double tStart, tEnd, dtPlot;
      fmatvec::Vec z0;
      int warnLevel;
      bool output;
      std::string name;

    public:
      /*! Constructor with \default tStart(0.), \default tEnd(1.), \default dtPlot(1e-4), \default warnLevel(0), \default output(true), \default name("Integrator") */
      Integrator();
      /*! Destructor */
      virtual ~Integrator() {};
	  /*! Set integration end time \param tEnd_ */	
      void settEnd(double tEnd_) {tEnd = tEnd_;}
      /*! Set plot step size \param dtPlot_ */
      void setdtPlot(double dtPlot_) {dtPlot = dtPlot_;}
      /*! Set initial state \param z0_ */
      void setz0(const fmatvec::Vec &z0_) {z0 = z0_;}
      /*! Set warn level \param level TODO DOC */
      void setWarnLevel(int level) {warnLevel = level;}
      /*! Set output \param flag (true/false) */
      void setOutput(bool flag) {output = flag;}
      /*! Set integration start time \param tStart_ */
      void settStart(double tStart_){tStart=tStart_;}
      /*! Start the integration for \param system */
      virtual void integrate(DynamicSystemSolver& system) = 0;

      virtual void initializeUsingXML(TiXmlElement *element);
  };

}

#endif
