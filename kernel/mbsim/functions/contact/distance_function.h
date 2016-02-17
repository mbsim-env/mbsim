/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef _DISTANCE_FUNCTION_H_
#define _DISTANCE_FUNCTION_H_

#include <mbsim/functions/function.h>

namespace MBSim {

  template <typename Sig> class DistanceFunction;

  /*! 
   * \brief class for distances and root functions of contact problems
   * \author Roland Zander
   * \date 2009-04-21 some comments (Thorsten Schindler)
   */
  template <typename Ret, typename Arg>
  class DistanceFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    protected:
      double t;
    public:
      virtual ~DistanceFunction() { }

      void setTime(double t_) { t = t_; }

      /* INTERFACE FOR DERIVED CLASSES */
      /*!
       * \param contour parameter
       * \return root function evaluation at contour parameter
       */
      virtual Ret operator()(const Arg &x) = 0;

      /*!
       * \param contour parameter
       * \return possible contact-distance at contour parameter
       */
      virtual double operator[](const Arg& x) { return nrm2(getWrD(x)); }

      /*!
       * \param contour parameter
       * \return helping distance vector at contour parameter
       */
      virtual fmatvec::Vec3 getWrD(const Arg& x) = 0;
      /*************************************************/
  };

}

#endif 

