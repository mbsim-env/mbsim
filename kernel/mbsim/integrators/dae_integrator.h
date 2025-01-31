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

#include "root_finding_integrator.h"

namespace MBSim {

  void setZero(fmatvec::Mat &A, const fmatvec::RangeV &rows, const fmatvec::RangeV &cols);

  /** \brief Base class for all DAE-Integrators
  */
  class DAEIntegrator : public RootFindingIntegrator {

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
      // ODE
      void par_ud_xd_par_q(fmatvec::Mat &J);
      void par_zd_par_q(fmatvec::Mat &J);
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
      // ODE, DAE2, DAE3 and GGL
      void par_ud_xd_par_u_x(fmatvec::Mat &J, bool updla);

      void calcSize();
      void initConstantMagnitudes();
      void initVariableMagnitudes();

      /** formalism **/
      Formalism formalism{ODE};
      /** reduced form **/
      bool reduced{false};
      /** numerical jacobian */
      bool numericalJacobian{false};

      int neq;

      fmatvec::Vec res0, qd0, ud0, xd0, gd0, g0; // residual work arrays for jacobian evaluation
      fmatvec::RangeV Rq, Ru, Rx, Rz, Rla, Rl; // ranges in y and jacobimatrix for q, u, x, z, la and GGL alg.-states
      fmatvec::RangeV RuMove, RxMove, RlaMove, RlMove; // ranges in y and jacobimatrix for u, x, z, la and GGL alg.-states reduced by q
      int rowMove;

    public:
      ~DAEIntegrator() override = default;

      void setFormalism(Formalism formalism_) { formalism = formalism_; }
      void setReducedForm(bool reduced_) { reduced = reduced_; }
      void setNumericalJacobian(bool numericalJacobian_) { numericalJacobian = numericalJacobian_; }
  };

}

#endif
