/* Copyright (C) 2004-2025  Martin Förg
 
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
 *   martin.o.foerg@googlemail.com
 *
 */

#ifndef _DAE_INTEGRATOR_H_
#define _DAE_INTEGRATOR_H_

#include "implicit_integrator.h"
#include <optional>

namespace MBSim {

  /** \brief Base class for all DAE integrators
  */
  class DAEIntegrator : public ImplicitIntegrator {

    public:
      enum Formalism {
        ODE=0,
        DAE1,
        DAE2,
        DAE3,
        GGL,
        unknown
      };

    protected:
      // DAE1
      void par_ud_xd_gdd_par_q_u(fmatvec::Mat &J, const fmatvec::Vec &ud_=fmatvec::Vec());
      void par_zd_gdd_par_q_u(fmatvec::Mat &J, const fmatvec::Vec &ud_=fmatvec::Vec());
      void par_ud_xd_par_x(fmatvec::Mat &J);
      // DAE2
      void par_ud_xd_gd_par_q(fmatvec::Mat &J);
      void par_zd_gd_par_q(fmatvec::Mat &J);
      // DAE3
      void par_ud_xd_g_par_q(fmatvec::Mat &J);
      void par_zd_g_par_q(fmatvec::Mat &J);
      // GGL
      void par_ud_xd_gd_g_par_q(fmatvec::Mat &J);
      void par_zd_gd_g_par_q(fmatvec::Mat &J);

      void calcSize() override;
      void init() override;
      virtual void reinit();

      /** formalism **/
      Formalism formalism{ODE};

      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      std::optional<double> aTolScalar;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      std::optional<double> rTolScalar;

      std::optional<double> aTolPos, aTolVel, aTol1st, aTolForce, rTolPos, rTolVel, rTol1st, rTolForce;

      int laInd, lInd;
      fmatvec::Vec gd0, g0; // saved constraints
      fmatvec::RangeV Rla, Rl; // ranges for la and GGL alg.-states
      fmatvec::RangeV RlaMove, RlMove; // ranges for la and GGL alg.-states reduced by q

    public:
      ~DAEIntegrator() override = default;

      void initializeUsingXML(xercesc::DOMElement *element) override;

      void setFormalism(Formalism formalism_) { formalism = formalism_; }

      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol <<= aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTolScalar = aTol_; }
      void setRelativeTolerance(const fmatvec::Vec &rTol_) { rTol <<= rTol_; }
      void setRelativeTolerance(double rTol_) { rTolScalar = rTol_; }

      void setAbsolutePositionTolerance(double aTolPos_) { aTolPos = aTolPos_; }
      void setAbsoluteVelocityTolerance(double aTolVel_) { aTolVel = aTolVel_; }
      void setAbsoluteFirstOrderTolerance(double aTol1st_) { aTol1st = aTol1st_; }
      void setAbsoluteForceTolerance(double aTolForce_) { aTolForce = aTolForce_; }
      void setRelativePositionTolerance(double rTolPos_) { rTolPos = rTolPos_; }
      void setRelativeVelocityTolerance(double rTolVel_) { rTolVel = rTolVel_; }
      void setRelativeFirstOrderTolerance(double rTol1st_) { rTol1st = rTol1st_; }
      void setRelativeForceTolerance(double rTolForce_) { rTolForce = rTolForce_; }
  };

}

#endif
