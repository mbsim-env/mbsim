/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include "fmatvec/fmatvec.h"
#include "mbsim/mbsim_event.h"
#include <mbsim/numerics/csparse.h>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <limits>
#include <vector>
#include <set>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace MBSim {

  double sign(double x);

  /*!
   * \brief calculates planar angle in [0,2\pi] with respect to Cartesian coordinates of: Arc Tangent (y/x)
   * \param Cartesian x-coordinate
   * \param Cartesian y-coordinate
   * \return angle
   */
  double ArcTan(double x,double y);

  /*!
   * \brief calculate a fmatvec::Mat out of a sparse matrix
   */
  fmatvec::Mat cs2Mat(cs* sparseMat);

  template <class T>
    inline std::string toStr(const T &val) {
      std::stringstream s;
      s << std::setprecision(std::numeric_limits<double>::digits10+1) << val;
      return s.str();
    }

  template <class Arg>
    class ToDouble {
    };

  template <>
    class ToDouble<double> {
      public:
        static double cast(const double &x) {
          return x;
        }
    };

  template <class Col>
    class ToDouble<fmatvec::Vector<Col,double> > {
      public:
        static double cast(const fmatvec::Vector<Col,double> &x) {
          return x.e(0); 
        }
    };

  template <class Row>
    class ToDouble<fmatvec::RowVector<Row,double> > {
      public:
        static double cast(const fmatvec::RowVector<Row,double> &x) {
          return x.e(0); 
        }
    };

  template <class Ret>
  class FromMatStr {
    public:
      static Ret cast(const char *x) {
        return Ret(x);
      }
  };

  template <>
  class FromMatStr<double> {
    public:
      static double cast(const std::string &x) {
        return boost::lexical_cast<double>(boost::algorithm::trim_copy(x));
      }
  };

  template <class Ret>
  class FromDouble {
    public:
      static Ret cast(double x) {
        throw std::runtime_error("FromDouble::cast not implemented for current type.");
      }
  };

  template <class Col>
  class FromDouble<fmatvec::Vector<Col,double> > {
    public:
      static fmatvec::Vector<Col,double> cast(double x) {
        return fmatvec::Vector<Col,double>(1,fmatvec::INIT,x);
      }
  };

  template <class Col>
  class FromDouble<fmatvec::RowVector<Col,double> > {
    public:
      static fmatvec::RowVector<Col,double> cast(double x) {
        return fmatvec::RowVector<Col,double>(1,fmatvec::INIT,x);
      }
  };

  template <class Row, class Col>
  class FromDouble<fmatvec::Matrix<fmatvec::General,Row,Col,double> > {
    public:
      static fmatvec::Matrix<fmatvec::General,Row,Col,double> cast(double x) {
        return fmatvec::Matrix<fmatvec::General,Row,Col,double>(1,1,fmatvec::INIT,x);
      }
  };

  template <>
  class FromDouble<double> {
    public:
      static double cast(double x) {
        return x;
      }
  };

  template <class Ret>
  class FromVecV {
    public:
      static Ret cast(const fmatvec::VecV &x) {
        return x;
      }
  };

  template <>
  class FromVecV<double> {
    public:
      static double cast(const fmatvec::VecV &x) {
        return x(0);
      }
  };

  /**
   * \return perpendicular vector
   * \param input vector
   */
  fmatvec::Vec3 computeTangential(const fmatvec::Vec3 &n);

}

#endif
