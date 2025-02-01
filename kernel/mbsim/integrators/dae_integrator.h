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
      virtual void reinit();

      /** formalism **/
      Formalism formalism{ODE};

      fmatvec::Vec gd0, g0; // residual work arrays for jacobian evaluation
      fmatvec::RangeV Rla, Rl; // ranges in y and jacobimatrix for la and GGL alg.-states
      fmatvec::RangeV RlaMove, RlMove; // ranges in y and jacobimatrix for la and GGL alg.-states reduced by q

    public:
      ~DAEIntegrator() override = default;

      void setFormalism(Formalism formalism_) { formalism = formalism_; }
  };

}

#endif
