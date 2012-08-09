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

#ifndef NUMERICS_LEMKE_ALGORITHM_H_
#define NUMERICS_LEMKE_ALGORITHM_H_

#include <fmatvec.h>

namespace MBSim {

  class LemkeAlgorithm {
    public:
      LemkeAlgorithm(const bool & DEBUGLEVEL_ = 0): DEBUGLEVEL(DEBUGLEVEL_) {
      }

      LemkeAlgorithm(const fmatvec::SqrMat & M_, const fmatvec::Vec & q_, const bool & DEBUGLEVEL_ = 0) :
          M(M_), q(q_), DEBUGLEVEL(DEBUGLEVEL_) {
        assert(M_.rows() == q.size());
        assert(M_.cols() == q.size());
      }

      LemkeAlgorithm(const fmatvec::SymMat & M_, const fmatvec::Vec & q_, const bool & DEBUGLEVEL_ = 0) :
          DEBUGLEVEL(DEBUGLEVEL_) {
        setSystem(M_, q_);
      }

      /* GETTER / SETTER */
      /**
       * \brief return info of solution process
       */
      int getInfo() {
        return info;
      }

      /**
       * \brief get the number of steps until the solution was found
       */
      int getSteps(void) {
        return steps;
      }

      /**
       * \brief set system with Matrix M and vector q
       */
      void setSystem(const fmatvec::SqrMat & M_, const fmatvec::Vec & q_) {
        assert(M_.rows() == q_.size());
        assert(M_.cols() == q_.size());
        M = M_;
        q = q_;
      }

      /**
       * \brief set system with Matrix M and vector q
       */
      void setSystem(const fmatvec::SymMat & M_, const fmatvec::Vec & q_) {
        M = fmatvec::SqrMat(M_.size(), M_.size(), fmatvec::NONINIT);
        for(int i = 0; i < M.size(); i++)
          for(int j = 0; j < M.size(); j++)
            M(i, j) = M_(i, j);
        q = q_;
      }
      /***************************************************/

      /**
       * \brief solve algorithm adapted from : Fast Implementation of Lemke’s Algorithm for Rigid Body Contact Simulation (John E. Lloyd)
       */
      fmatvec::Vec solve(unsigned int maxloops = 0);

      virtual ~LemkeAlgorithm() {
      }

    protected:
      int findLexicographicMinimum(const fmatvec::Mat &A, const int & pivotColIndex);
      bool LexicographicPositive(const fmatvec::Vec & v);
      void GaussJordanEliminationStep(fmatvec::Mat &A, int pivotRowIndex, int pivotColumnIndex, const std::vector<size_t> & basis);
      bool greaterZero(const fmatvec::Vec & vector);
      bool validBasis(const std::vector<size_t> & basis);

      fmatvec::SqrMat M;
      fmatvec::Vec q;

      /**
       * \brief number of steps until the Lemke algorithm found a solution
       */
      unsigned int steps;

      /**
       * \brief define level of debug output
       */
      bool DEBUGLEVEL;

      /**
       * \brief did the algorithm find a solution
       *
       * -1 : not successful
       *  0 : successful
       */
      int info;
  };

} /* namespace MBSim */
#endif /* NUMERICS_LEMKE_ALGORITHM_H_ */
