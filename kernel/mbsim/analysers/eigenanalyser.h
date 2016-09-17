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

#ifndef _EIGENANALYSER_H_
#define _EIGENANALYSER_H_

#include "fmatvec/fmatvec.h"
#include "mbsim/functions/function.h"
#include "mbsim/solver.h"

namespace MBSimAnalyser {

  const MBXMLUtils::NamespaceURI MBSIMANALYSER("http://www.mbsim-env.de/MBSimAnalyser");

  /**
   * \brief Eigenanalyser for dynamic systems
   * \author Martin Foerg
   */
  class Eigenanalyser : public MBSim::Solver {

    class Residuum : public MBSim::Function<fmatvec::Vec(fmatvec::Vec)> {
      public:
        Residuum(MBSim::DynamicSystemSolver *sys_, double t_);
        fmatvec::Vec operator()(const fmatvec::Vec &z);
      private:
        MBSim::DynamicSystemSolver *sys;
        double t;
    };

    public:

      enum Task { eigenfrequencies, eigenmodes, eigenmode, eigenmotion };

      /**
       * \brief Standard constructor 
       */
      Eigenanalyser() : tStart(0), tEnd(1), dtPlot(1e-2), A(1), n(1), compEq(false), task(eigenmode) {}
      
      /**
       * \brief Destructor
       */
      ~Eigenanalyser() {}

      void execute(MBSim::DynamicSystemSolver& system) { analyse(system); }

      /**
       * \brief Perform the eigenanalysis
       * \param system The dynamic system to be analysed
       */
      void analyse(MBSim::DynamicSystemSolver& system);

      /**
       * \brief Set the initial deviation of the equilibrium
       * \param deltaz_ The deviation
       */
      void setInitialDeviation(const fmatvec::Vec &deltaz0_) { deltaz0 = deltaz0_; }

      /**
       * \brief Set the amplitude for the eigemode analysis
       * \param A_ The amplitude
       */
      void setAmplitude(double A_) { A = A_; }

      /**
       * \brief Set the mode for the eigemode analysis
       * \param n_ The mode number
       */
      void setMode(int n_) { n = n_; }

      /**
       * \brief Set the start time for the analysis
       * \param tStart_ The start time
       */
      void setStartTime(double tStart_) { tStart=tStart_; }

      /**
       * \brief Set the end time for the analysis
       * \param tEnd_ The end time
       */
      void setEndTime(double tEnd_) { tEnd = tEnd_; }

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
       * \brief Get the eigenvalues
       * \return A vector containing the eigenvalues of the system
       */
      const fmatvec::Vector<fmatvec::Ref, std::complex<double> >& getEigenvalues() const { return w; }

      /**
       * \brief Get the eigenvectors
       * \return A matrix containing the eigenvectors of the system
       */
      const fmatvec::SquareMatrix<fmatvec::Ref, std::complex<double> >& getEigenvectors() const { return V; }

      /**
       * \brief Get the eigenfrequencies
       * \return A vector containing the eigenfrequencies of the system
       */
      const fmatvec::Vec& getEigenfrequencies() const { return freq; }

      /**
       * \brief Set the name of the output file
       * \param fileName_ The output file name
       */
      void setOutputFileName(const std::string &fileName_) { fileName = fileName_; }

      void initializeUsingXML(xercesc::DOMElement *element);

    protected:

      fmatvec::Vec zEq, deltaz0;
      double tStart, tEnd, dtPlot, A;
      int n;
      bool compEq;
      Task task;

      fmatvec::SquareMatrix<fmatvec::Ref, std::complex<double> > V;
      fmatvec::Vector<fmatvec::Ref, std::complex<double> > w;
      fmatvec::Vec freq;
      std::vector<std::pair<double,int> > f;

      std::string fileName;

      bool saveEigenanalyis(const std::string &fileName);
      void computeEigenfrequencies();
      void computeEigenvalues();
      void computeEigenmodes();
      void computeEigenmode();
      void computeEigenmotion();
 };

}

#endif
