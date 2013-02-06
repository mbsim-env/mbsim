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

#ifndef NUMERICS_NUMERICSMULTIDIMENSIONALNEWTONMETHOD_H_
#define NUMERICS_NUMERICSMULTIDIMENSIONALNEWTONMETHOD_H_

#include <fmatvec.h>

#include <map>

#include <mbsim/numerics/functions/newton_method_jacobian_functions.h>
#include <mbsim/numerics/functions/criteria_functions.h>
#include <mbsim/numerics/functions/damping_functions.h>

namespace fmatvec {
  bool operator<(const fmatvec::Index & i1, const fmatvec::Index & i2);
}

namespace MBSim {

  /**
   * \brief Newton method for multidimensional root finding
   * \author Kilian Grundl (3.2.2012)
   */
  template<class VecType, class AT>
  class MultiDimensionalNewtonMethod {
    public:
      /*!
       * \brief plain constructor
       */
      MultiDimensionalNewtonMethod();

      virtual ~MultiDimensionalNewtonMethod() {
      }

      /* GETTER / SETTER */
      int getNumberOfIterations() const {
        return iter;
      }
      int getNumberOfMaximalIterations() const {
        return itermax;
      }
      int getInfo() const {
        return info;
      }
      void setMaximumNumberOfIterations(int itmax_) {
        itermax = itmax_;
      }
      void setFunction(Function1<fmatvec::Vector<VecType, AT>, fmatvec::Vector<VecType, AT> > *function_) {
        function = function_;
      }
      void setJacobianFunction(NewtonJacobianFunction<VecType, AT> * jacobian_) {
        jacobian = jacobian_;
      }
      void setDampingFunction(DampingFunction<VecType, AT> * damping_) {
        this->damping = damping_;
      }
      void setCriteriaFunction(CriteriaFunction<VecType, AT> * criteria_) {
        this->criteria = criteria_;
      }
      CriteriaFunction<VecType, AT> * getCriteriaFunction() {
        return this->criteria;
      }
      /***************************************************/

      /**
       * \brief solve nonlinear root function
       * \param initialValue initial value
       */
      fmatvec::Vector<VecType, AT> solve(const fmatvec::Vector<VecType, AT> & initialValue);

    private:

      /**
       * \brief root function
       */
      Function1<fmatvec::Vec, fmatvec::Vec> *function;

      /**
       * \brief Jacobian matrix
       */
      NewtonJacobianFunction<VecType, AT> *jacobian;

      /*
       * \brief damping function
       */
      DampingFunction<VecType, AT> *damping;

      /*
       * \brief criteria function
       *
       * This function defines the criteria when to stop the Newton algorithm
       */
      CriteriaFunction<VecType, AT> *criteria;

      /**
       * \brief maximum number of iterations, actual number of iterations, maximum number of damping steps, information about success
       */
      int itermax;

      /*
       * \brief number of iterations
       */
      int iter;

      /*
       * \brief information about the result of the method
       */
      int info;

  };

  template<class VecType, class AT>
  MultiDimensionalNewtonMethod<VecType, AT>::MultiDimensionalNewtonMethod() :
    function(0), jacobian(0), damping(0), criteria(0), itermax(300), iter(0), info(1){
  }

  template<class VecType, class AT>
  fmatvec::Vector<VecType, AT> MultiDimensionalNewtonMethod<VecType, AT>::solve(const fmatvec::Vector<VecType, AT> & initialValue) {
    /*Reset for comparing*/
    criteria->clear();
    criteria->setFunction(function);
    info = (*criteria)(initialValue);
    if(info == 0)
      return initialValue;

    if(damping) {
      damping->setCriteriaFunction(criteria);
      damping->setFunction(function);
    }

    jacobian->setFunction(function);

    info = 1;
    iter = 0;
    /*End - Reset*/

    //current position in function
    fmatvec::Vector<VecType, AT> x = initialValue;

    //current value of function
    fmatvec::Vector<VecType, AT> f = (*function)(x);

    //direction of function (derivative)
    fmatvec::SquareMatrix<VecType, AT> J = (*jacobian)(x);

    //step to next position
    fmatvec::Vector<VecType, AT> dx = slvLU(J, f, info);

    //Damp the solution
    if(damping)
      x -=  (*damping)(x, dx) * dx;
    else
      x -= dx;

    f = (*function)(x);

    for (iter = 1; iter < itermax; iter++) {

      //Get the information about the criteria
      info = (*criteria)(x);

      //Criteria with info = 1 means: go on  (else there might be a solution found (=0) or something else)
      if (info != 1) {
        if(info == -1) //new solution is worse than solution before --> return solution before
          return x + dx;
        else
          return x;
      }

      //compute current value
      f = (*function)(x);

      //Compute Jacobian
      J = (*jacobian)(x);

      //get step
      dx = slvLU(J, f, info);

      //cout << "dxn[" << iter << "] = " << dx << endl;

      //Damp the solution
      if(damping)
        x -=  (*damping)(x, dx) * dx;
      else
        x -= dx;

    }

    info = (*criteria)(x);
    return x;
  }

}
#endif /* NUMERICSMULTIDIMENSIONALNEWTONMETHOD_H_ */
