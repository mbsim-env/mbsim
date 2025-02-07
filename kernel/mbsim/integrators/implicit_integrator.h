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

#ifndef _IMPLICIT_INTEGRATOR_H_
#define _IMPLICIT_INTEGRATOR_H_

#include "root_finding_integrator.h"

namespace MBSim {

  void setZero(fmatvec::Mat &A, const fmatvec::RangeV &rows, const fmatvec::RangeV &cols);

  /** \brief Base class for all implicit integrators
  */
  class ImplicitIntegrator : public RootFindingIntegrator {

    protected:
      virtual double delta(int i, double z) const;
      void par_ud_xd_par_q(fmatvec::Mat &J);
      void par_zd_par_q(fmatvec::Mat &J);
      void par_ud_xd_par_u_x(fmatvec::Mat &J, bool updla);
      void par_zd_par_z(fmatvec::Mat &J, bool updla);

      virtual void calcSize();
      void init();

      /** reduced form **/
      bool reduced{false};
      /** numerical jacobian */
      bool numericalJacobian{false};

      int neq, qMove;
      fmatvec::Vec zd0; // saved derivatives of the solution components
      fmatvec::RangeV Rq, Ru, Rx, Rz; // ranges for q, u, x and z
      fmatvec::RangeV RuMove, RxMove; // ranges for u and x reduced by q

    public:
      ~ImplicitIntegrator() override = default;

      void setReducedForm(bool reduced_) { reduced = reduced_; }
      void setNumericalJacobian(bool numericalJacobian_) { numericalJacobian = numericalJacobian_; }
  };

}

#endif
