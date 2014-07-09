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

#ifndef _EIGENANALYSIS_H_
#define _EIGENANALYSIS_H_

#include "fmatvec/fmatvec.h"
#include "fmatvec/function.h"

namespace MBSim {

  class DynamicSystemSolver;

  /**
   * \brief Eigenanalysis for dynamic systems
   * \author Martin Foerg
   */
  class Eigenanalysis {

    class Residuum : public fmatvec::Function<fmatvec::Vec(fmatvec::Vec)> {
      public:
        Residuum(DynamicSystemSolver *sys_, double t_);
        fmatvec::Vec operator()(const fmatvec::Vec &z);
      private:
        DynamicSystemSolver *sys;
        double t;
    };

    public:
      /**
       * \brief Standard constructor 
       */
      Eigenanalysis() : tStart(0), tEnd(1), dtPlot(1e-2) {}
      
      /**
       * \brief Destructor
       */
      ~Eigenanalysis() {}

      /**
       * \brief Perform the eigenanalysis
       * \param system The dynamic system to be analysed
       */
      void analyse(DynamicSystemSolver& system);

      /**
       * \brief Set the initial guess for the computation of the equilibrium position
       * \param z0_ The initial guess
       */
      void setInitialGuess(const fmatvec::Vec &z0_) { z0 = z0_; }

      /**
       * \brief Set the initial deviation of the equilibrium position
       * \param deltaz_ The deviation
       */
      void setInitialDeviation(const fmatvec::Vec &deltaz0_) { deltaz0 = deltaz0_; }

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
       * \brief Set the equilibrium state for the analysis
       * \param zEq_ The equilibrium state
       */
      void setEquilibriumState(const fmatvec::Vec &zEq_) { zEq = zEq_; }

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

      /**
       * \brief Compute and plot the i-th eigenmode
       * \param system The dynamic system to be analysed
       */
      void eigenmode(int i, DynamicSystemSolver& system);

      /**
       * \brief Compute and plot all eigenmodes
       * \param system The dynamic system to be analysed
       */
      void eigenmodes(DynamicSystemSolver& system);

      /**
       * \brief Compute and plot the eigenmotion
       * \param system The dynamic system to be analysed
       */
      void eigenmotion(DynamicSystemSolver& system);

    protected:

      static DynamicSystemSolver* system;

      fmatvec::Vec z0, deltaz0, zEq;
      double tStart, tEnd, dtPlot;

      fmatvec::SquareMatrix<fmatvec::Ref, std::complex<double> > V;
      fmatvec::Vector<fmatvec::Ref, std::complex<double> > w;
      fmatvec::Vec freq;
      std::vector<std::pair<double,int> > f;

      std::string fileName;

      bool saveEigenanalyis(const std::string &fileName);
      bool loadEigenanalyis(const std::string &fileName);
      void computeEigenfrequencies();
 };

}

#endif
