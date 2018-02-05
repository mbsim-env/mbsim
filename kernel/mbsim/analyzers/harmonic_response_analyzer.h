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

#ifndef _HARMONIC_RESPONSE_ANALYSER_H_
#define _HARMONIC_RESPONSE_ANALYSER_H_

#include "fmatvec/fmatvec.h"
#include "mbsim/functions/function.h"
#include "mbsim/solver.h"

namespace MBSimAnalyzer {

  const MBXMLUtils::NamespaceURI MBSIMANALYZER("http://www.mbsim-env.de/MBSimAnalyzer");

  /**
   * \brief HarmonicResponseAnalyzer for dynamic systems
   * \author Martin Foerg
   */
  class HarmonicResponseAnalyzer : public MBSim::Solver {

    class Residuum : public MBSim::Function<fmatvec::Vec(fmatvec::Vec)> {
      public:
        Residuum(MBSim::DynamicSystemSolver *sys_) : sys(sys_) { }
        fmatvec::Vec operator()(const fmatvec::Vec &z);
      private:
        MBSim::DynamicSystemSolver *sys;
    };

    public:

      enum Task { frequencyResponse };

      /**
       * \brief Standard constructor 
       */
      HarmonicResponseAnalyzer() : tStart(0), dtPlot(1e-2), compEq(false), task(frequencyResponse) { }
      
      /**
       * \brief Perform a harmonic response analysis of the system
       */
      void execute();

      /**
       * \brief Set the start time for the analysis
       * \param tStart_ The start time
       */
      void setStartTime(double tStart_) { tStart=tStart_; }

      void setExcitationFrequencies(const fmatvec::VecV& fE_) { fE = fE_; }

      void setSystemFrequencies(const fmatvec::VecV& fS_) { fS = fS_; }

      /**
       * \brief Set the plot step size for the analysis
       * \param dtPlot_ The plot step size
       */
      void setPlotStepSize(double dtPlot_) { dtPlot = dtPlot_; }

      /**
       * \brief Set the initital state for the analysis
       * \param z0 The initital state
       */
      void setInitialState(const fmatvec::Vec &z0) { zEq = z0; }
//      void setEquilibriumState(const fmatvec::Vec &zEq_) { zEq = zEq_; }

      /**
       * \brief Determine the equilibrium state for the analysis
       * \param eq True, if the equilibrium state should be determined
       */
      void setDetermineEquilibriumState(bool eq) { compEq = eq; }

      void setTask(Task task_) { task = task_; }

      const fmatvec::Vec& getInitialState() const { return zEq; }

      /**
       * \brief Set the name of the output file
       * \param fileName_ The output file name
       */
      void setOutputFileName(const std::string &fileName_) { fileName = fileName_; }

      void initializeUsingXML(xercesc::DOMElement *element);

    protected:

      fmatvec::Vec zEq, zh;
      fmatvec::VecV fE, fS;
      fmatvec::Mat Zh;
      double tStart, dtPlot;
      bool compEq;
      Task task;

      std::string fileName;

      void computeFrequencyResponse();
  };

}

#endif
