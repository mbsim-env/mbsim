/* Copyright (C) 2004-2012 MBSim Development Team
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

#ifndef NUMERICS_DAMPINGFUNCITONS_H_
#define NUMERICS_DAMPINGFUNCITONS_H_

#include <mbsim/numerics/functions/criteria_functions.h>

namespace MBSim {

  template<class VecType, class AT>
  class DampingFunction : public Function2<double, fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > {
    public:
    /**
     * \brief constructor
     */
    DampingFunction();

    /*
     * \brief destructor
     */
    virtual ~DampingFunction() {
    }

    /* GETTER / SETTER*/
    void setFunction(Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > * function_) {
      function = function_;
    }
    void setCriteriaFunction(CriteriaFunction<VecType, AT> * criteria_) {
      criteria = criteria_;
    }
    /******************/

    virtual double operator ()(const fmatvec::Vector<VecType, AT> & x, const fmatvec::Vector<VecType, AT> & dx, const void * = NULL) = 0;

    protected:
    /**
     * \brief function that computes the values
     */
    Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > *function;

    /**
     * \brief criteria that defines if a solution gets better
     */
    CriteriaFunction<VecType, AT> * criteria;

  };

  template<class VecType, class AT>
  class StandardDampingFunction : public DampingFunction<VecType, AT> {

    public:
      /**
       * \brief constructor
       */
      StandardDampingFunction(unsigned int kmax_ = 300);

      /*
       * \brief destructor
       */
      virtual ~StandardDampingFunction() {
      }

      virtual double operator ()(const fmatvec::Vector<VecType, AT> & x, const fmatvec::Vector<VecType, AT> & dx, const void * = NULL);

    protected:
      /**
       * \brief maximal damping steps
       */
      unsigned int kmax;
  };

  template<class VecType, class AT>
  DampingFunction<VecType, AT>::DampingFunction() :
    function(0), criteria(0){

  }

  template<class VecType, class AT>
  StandardDampingFunction<VecType, AT>::StandardDampingFunction(unsigned int kmax_ /* = 300*/) :
      DampingFunction<VecType, AT>(), kmax(kmax_) {
  }

  template<class VecType, class AT>
  double StandardDampingFunction<VecType, AT>::operator ()(const fmatvec::Vector<VecType, AT> & x, const fmatvec::Vector<VecType, AT> & dx, const void *) {
    double alpha = 1;
    fmatvec::Vector<VecType, AT> xnew(x.size(), fmatvec::NONINIT);

    for (unsigned int k = 0; k < kmax; k++) {
      xnew = x - alpha * dx;
      fmatvec::Vector<VecType, AT> f = (*this->function)(xnew);
      if(this->criteria->isBetter(xnew)) {
        return alpha;
      }
      alpha *= 0.5;
    }

    return 1;
  }
}
#endif //NUMERICS_DAMPINGFUNCITONS_H_
