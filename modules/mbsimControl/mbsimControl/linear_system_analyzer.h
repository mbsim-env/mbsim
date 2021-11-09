/* Copyright (C) 2004-2021 MBSim Development Team
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

#ifndef _LINEAR_SYSTEM_ANALYZER_H_
#define _LINEAR_SYSTEM_ANALYZER_H_

#include "mbsim/solver.h"
#include "mbsim/functions/function.h"
#include "mbsim/utils/index.h"
#include <mbsim/utils/boost_parameters.h>

namespace MBSim {

  BOOST_PARAMETER_NAME(modeRange)
  BOOST_PARAMETER_NAME(frequencyRange)
  BOOST_PARAMETER_NAME(inputNumber)

}

namespace MBSimControl {

  /*!
   * \brief Linear system analyzer
   * \author Martin Foerg
   */
  class LinearSystemAnalyzer : public MBSim::Solver {
    public:
      LinearSystemAnalyzer() : mRange("[0;9]"), fRange("[0;1e4]") { }
      ~LinearSystemAnalyzer() override;
      void execute() override;
      void setInitialTime(double t0_) { t0 = t0_; }
      void setInitialState(const fmatvec::Vec &z0_) { z0 <<= z0_; }
      void setInitialInput(const fmatvec::Vec &u0_) { u0 <<= u0_; }
      void setMinimumNaturalFrequency(double fmin_) { fmin = fmin_; }
      void setMaximumNaturalFrequency(double fmax_) { fmax = fmax_; }
      void setNaturalModeScaleFactor(double modeScaleFactor_) { modeScaleFactor = modeScaleFactor_; }
      void setNaturalModeScale(const fmatvec::VecV &modeScale_) { modeScale <<= modeScale_; }
      void setExcitationFrequencies(const fmatvec::VecV &fex_) { fex <<= fex_; }
      void setExcitationAmplitudeFunction(MBSim::Function<fmatvec::VecV(double)> *Amp_) { Amp = Amp_; }
      void setPlotStepSize(double dtPlot_) { dtPlot = dtPlot_; }
      void setLoops(double loops_) { loops = loops_; }
      const fmatvec::Vec& getInitialState() const override { return z0; }
      void initializeUsingXML(xercesc::DOMElement *element) override;

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), visualizeNaturalModeShapes, MBSim::tag, (optional (modeRange,(fmatvec::Vec2),"[0;9]"))) {
	msv = true;
	mRange = modeRange;
      }

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), visualizeFrequencyResponse, MBSim::tag, (optional (frequencyRange,(fmatvec::Vec2),"[0;1e4]")(inputNumber,(double),0))) {
	frv = true;
	fRange = frequencyRange;
	inum = inputNumber;
      }

    protected:
      double t0{0};
      double fmin{1e-2};
      double fmax{1e5};
      double modeScaleFactor{1};
      fmatvec::VecV modeScale;
      MBSim::Function<fmatvec::VecV(double)> *Amp{nullptr};
      fmatvec::Vec z0, zEq, u0, fex;
      bool msv{false};
      bool frv{false};
      fmatvec::Vec2 mRange, fRange;
      MBSim::Index inum{0};
      int loops{5};
      double dtPlot{1e-2};
  };

}

#endif
