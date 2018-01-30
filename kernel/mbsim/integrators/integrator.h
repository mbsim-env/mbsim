/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _INTEGRATOR_H_
#define _INTEGRATOR_H_

#include <fmatvec/fmatvec.h>
#include <fmatvec/atom.h>
#include <mbxmlutilshelper/dom.h>
#include <mbsim/mbsim_event.h>
#include <mbsim/solver.h>
#include <string>
#include <iostream>

namespace MBSimIntegrator {

  const MBXMLUtils::NamespaceURI MBSIMINT("http://www.mbsim-env.de/MBSimIntegrator");

  /**
   * \brief integrator-interface for dynamic systems
   * \author Martin Foerg
   * \date 2009-07-13 some comments (Thorsten Schindler) 
   */
  class Integrator : public MBSim::Solver {
    public:
      /**
       * \brief constructor 
       */
      Integrator();
      
      /**
       * \brief destructor
       */
      ~Integrator() override = default;
      
      /* GETTER / SETTER */
      void setStartTime(double tStart_) { tStart=tStart_; }
      void setEndTime(double tEnd_) { tEnd = tEnd_; }
      void setPlotStepSize(double dtPlot_) { dtPlot = dtPlot_; }
      void setInitialState(const fmatvec::Vec &z0_) { z0 = z0_; }
      void setWarnLevel(int level) { warnLevel = level; }
      void setOutput(bool flag) { output = flag; }
      double getStartTime() const { return tStart; }
      double getEndTime() const { return tEnd; }
      double getPlotStepSize() const { return dtPlot; }
      const fmatvec::Vec& getInitialState() const override { return z0; }
      int getWarnLevel() const { return warnLevel; }
      bool getOutput() const { return output; }
      /***************************************************/
      
      void execute() override { integrate(); }

      /* INTERFACE FOR DERIVED CLASSES */
      /*! 
       * \brief start the integration of the system set by setSystem.
       * Each class implemeting this function should call debugInit first.
       */
      virtual void integrate() = 0;

      //! Convinent function: call setSystem(&sys) and integrate()
      void integrate(MBSim::DynamicSystemSolver& sys) { setSystem(&sys); integrate(); }

      /*! Each class implemeting the Integrator::integrate function should call this function first.
       * This function does currently only some minor modification of the integrator data (like
       * end time) dependent on environment variables. This is used mainly for debugging purposes like
       * automatic valgrind runs with a very small tEnd time.
       */
      void debugInit();

      virtual void preIntegrate() { throw MBSim::MBSimError("preIntegrate is not defined"); }
      virtual void subIntegrate(double tStop) { throw MBSim::MBSimError("subIntegrate is not defined"); }
      virtual void postIntegrate() { throw MBSim::MBSimError("postIntegrate is not defined"); }

      /*! 
       * \brief initialize integrator
       * \param XML description
       */
      void initializeUsingXML(xercesc::DOMElement *element) override;

      static Integrator* readXMLFile(const std::string &filename);

      void setPlotIntegrationData(bool plotIntegrationData_) { plotIntegrationData = plotIntegrationData_; }

      void setWriteIntegrationSummary(bool writeIntegrationSummary_) { writeIntegrationSummary = writeIntegrationSummary_; }

    protected:
      /**
       * \brief start, end, plot time
       */
      double tStart{0.}, tEnd{1.}, dtPlot{1e-4};

      /**
       * \brief initial state
       */
      fmatvec::Vec z0;

      /**
       * \brief warn level
       */
      int warnLevel{0};

      /**
       * \brief flag for ouput printing
       */
      bool output{true};

      /**
       * \brief name of integrator
       */
      std::string name;

      /**
       * \brief plot integration data
       */
      bool plotIntegrationData{false};

      /**
       * \brief write integration summary
       */
      bool writeIntegrationSummary{false};
  };

}

#endif /* _INTEGRATOR_H_ */
