/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Created on: Jul 30, 2013
 * Contact: kilian.grundl@gmail.com
 */

#include <fmatvec/fmatvec.h>
#include <mbsim/mbsim_event.h>

namespace MBSimEHD {

  class LagrShapeFct1D {
// Lagrange shape functions and parametric derivatives for line elements
// Michael Hofer, 21.03.2015

// Node numbering and parametric coordinate system:
//
//       line2                 line3
//
//    1---------2 --> xi    1----3----2 --> xi

    private:
      static int getDim(const std::string & shape) {
        if (shape == "line2") {
          return 2;
        }
        else if (shape == "line3") {
          return 3;
        }
        else {
          throw MBSim::MBSimError("Unknwon shape type.");
        }
        return 0;
      }

    public:

// Shape functions evaluated at parametric point xi
//
// Input:
//   xi:     Parametric coordinate
//   shape:  Shape of element
//
// Output:
//   N:      Row vector with shape functions evaluated at xi
//           [N^1  N^2  N^3  ...]
      static fmatvec::RowVecV Fct(const fmatvec::Vec2 & xi, const std::string & shape) {

        fmatvec::RowVecV N(getDim(shape));

        if (shape == "line2") {
          N(0) = 1. / 2. * (1 - xi(0));
          N(1) = 1. / 2. * (1 + xi(0));
        }
        else if (shape == "line3") {
          N(0) = -1. / 2. * xi(0) * (1. - xi(0));
          N(1) = 1. / 2. * xi(0) * (1. + xi(0));
          N(2) = 1. - pow(xi(0), 2.);
        }

        return N;
      }

      // First parametric derivative of shape functions evaluated
      // at parametric point xi
      //
      // Input:
      //   xi:     Parametric coordinate
      //   shape:  Shape of element
      //
      // Output:
      //   Ndxi:   Row vector with first parametric derivative of
      //           shape functions evaluated at xi
      //           [N^1,xi  N^2,xi  N^3,xi  ...]
      static fmatvec::Mat2xV Deriv1(const fmatvec::Vec2 & xi, const std::string & shape) {
        fmatvec::Mat2xV Ndxi(getDim(shape));

        if (shape == "line2") {
          Ndxi(0, 0) = -1. / 2.;
          Ndxi(0, 1) = 1. / 2.;
        }
        else if (shape == "line3") {
          Ndxi(0, 0) = -1. / 2. + xi(0);
          Ndxi(0, 1) = 1. / 2. + xi(0);
          Ndxi(0, 2) = -2. * xi(0);
        }

        return Ndxi;
      }

      // Second parametric derivative of shape functions evaluated
      // at parametric point xi
      //
      // Input:
      //   xi:         Parametric coordinate
      //   shape:      Shape of element
      //
      // Output:
      //   Ndxidxi:    Row vector with second parametric derivative of
      //               shape functions evaluated at xi
      //               [N^1,xixi  N^2,xixi  N^3,xixi  ...]
      static fmatvec::Mat3xV Deriv2(const fmatvec::Vec2 & xi, const std::string & shape) {
        fmatvec::Mat3xV Ndxidxi(getDim(shape));

        if (shape == "line2") {
          // nothing to do here
        }
        else if (shape == "line3") {
          Ndxidxi(0, 0) = 1.;
          Ndxidxi(0, 1) = 1.;
          Ndxidxi(0, 2) = -2.;
        }

        return Ndxidxi;

      }
  };

  class LagrShapeFct2D {
      // Lagrange shape functions and parametric derivatives for
      // quadrilateral elements
      // Michael Hofer, 20.12.2014

      // Node numbering and parametric coordinate system:
      //
      //       quad4                  quad8                  quad9
      //
      //        xi2                    xi2                    xi2
      //         ^                      ^                      ^
      //         |                      |                      |
      //    4---------3            4----7----3            4----7----3
      //    |    |    |            |    |    |            |    |    |
      //    |    + -- | --> xi1    8    + -- 6 --> xi1    8    9 -- 6 --> xi1
      //    |         |            |         |            |         |
      //    1---------2            1----5----2            1----5----2

      // Other node numbering:
      //
      //                             quad8on
      //
      //                               xi2
      //                                ^
      //                                |
      //                           7----6----5
      //                           |    |    |
      //                           8    + -- 4 --> xi1
      //                           |         |
      //                           1----2----3

    private:
      static int getDim(const std::string & shape) {
        if (shape == "quad4") {
          return 4;
        }
        else if (shape == "quad8") {
          return 8;
        }
        else if (shape == "quad8on") {
          return 8;
        }
        else if (shape == "quad9") {
          return 9;
        }
        else {
          throw MBSim::MBSimError("Unknwon shape type.");
        }
        return 0;
      }

    public:
      // Shape functions evaluated at parametric point xi
      //
      // Input:
      //   xi:     Vector with parametric coordinates [xi1; xi2]
      //   shape:  Shape of element
      //
      // Output:
      //   N:      Row vector with shape functions evaluated at xi
      //           [N^1  N^2  N^3  ...]
      static fmatvec::RowVecV Fct(const fmatvec::Vec2 & xi, const std::string & shape) {
        fmatvec::RowVecV N(getDim(shape));

        double xi1 = xi(0);
        double xi2 = xi(1);

        if (shape == "quad4") {
          N(0) = 1. / 4. * (1 - xi1) * (1 - xi2);
          N(1) = 1. / 4. * (1 + xi1) * (1 - xi2);
          N(2) = 1. / 4. * (1 + xi1) * (1 + xi2);
          N(3) = 1. / 4. * (1 - xi1) * (1 + xi2);
        }
        else if (shape == "quad8") {
          N(0) = 1. / 4. * (1 - xi1) * (1 - xi2) * (-xi1 - xi2 - 1);
          N(1) = 1. / 4. * (1 + xi1) * (1 - xi2) * (xi1 - xi2 - 1);
          N(2) = 1. / 4. * (1 + xi1) * (1 + xi2) * (xi1 + xi2 - 1);
          N(3) = 1. / 4. * (1 - xi1) * (1 + xi2) * (-xi1 + xi2 - 1);
          N(4) = 1. / 2. * (1 - pow(xi1, 2)) * (1 - xi2);
          N(5) = 1. / 2. * (1 + xi1) * (1 - pow(xi2, 2));
          N(6) = 1. / 2. * (1 - pow(xi1, 2)) * (1 + xi2);
          N(7) = 1. / 2. * (1 - xi1) * (1 - pow(xi2, 2));
        }
        else if (shape == "quad8on") {
          // Just other node numbering
          fmatvec::RowVecV Nquad8 = Fct(xi, "quad8");
          N(0) = Nquad8(0);
          N(1) = Nquad8(4);
          N(2) = Nquad8(1);
          N(3) = Nquad8(5);
          N(4) = Nquad8(2);
          N(5) = Nquad8(6);
          N(6) = Nquad8(3);
          N(7) = Nquad8(7);
        }
        else if (shape == "quad9") {
          N(0) = 1. / 4. * xi1 * xi2 * (1 - xi1) * (1 - xi2);
          N(1) = -1. / 4. * xi1 * xi2 * (1 + xi1) * (1 - xi2);
          N(2) = 1. / 4. * xi1 * xi2 * (1 + xi1) * (1 + xi2);
          N(3) = -1. / 4. * xi1 * xi2 * (1 - xi1) * (1 + xi2);
          N(4) = -1. / 2. * xi2 * (1 - pow(xi1, 2)) * (1 - xi2);
          N(5) = 1. / 2. * xi1 * (1 + xi1) * (1 - pow(xi2, 2));
          N(6) = 1. / 2. * xi2 * (1 - pow(xi1, 2)) * (1 + xi2);
          N(7) = -1. / 2. * xi1 * (1 - xi1) * (1 - pow(xi2, 2));
          N(8) = (1. - pow(xi1, 2)) * (1 - pow(xi2, 2));
        }

        return N;

      }

      // First parametric derivatives of shape functions evaluated
      // at parametric point xi
      //
      // Input:
      //   xi:     Vector with parametric coordinates [xi1; xi2]
      //   shape:  Shape of element
      //
      // Output:
      //   Ndxi:   Matrix with first parametric derivatives of
      //           shape functions evaluated at xi
      //           [N^1,xi_1  N^2,xi_1  N^3,xi_1  ...;
      //            N^1,xi_2  N^2,xi_2  N^3,xi_2  ...]
      static fmatvec::Mat2xV Deriv1(const fmatvec::Vec2 & xi, const std::string & shape) {
        fmatvec::Mat2xV Ndxi(getDim(shape));

        double xi1 = xi(0);
        double xi2 = xi(1);

        if (shape == "quad4") {
          Ndxi(0, 0) = -1. / 4. * (1 - xi2);
          Ndxi(1, 0) = -1. / 4. * (1 - xi1);

          Ndxi(0, 1) = 1. / 4. * (1 - xi2);
          Ndxi(1, 1) = -1. / 4. * (1 + xi1);

          Ndxi(0, 2) = 1. / 4. * (1 + xi2);
          Ndxi(1, 2) = 1. / 4. * (1 + xi1);

          Ndxi(0, 3) = -1. / 4. * (1 + xi2);
          Ndxi(1, 3) = 1. / 4. * (1 - xi1);
        }
        else if (shape == "quad8") {
          Ndxi(0, 0) = -1. / 4. * (-2 * xi1 - xi2) * (1 - xi2);
          Ndxi(1, 0) = -1. / 4. * (1 - xi1) * (-xi1 - 2 * xi2);

          Ndxi(0, 1) = 1. / 4. * (2 * xi1 - xi2) * (1 - xi2);
          Ndxi(1, 1) = -1. / 4. * (1 + xi1) * (xi1 - 2 * xi2);

          Ndxi(0, 2) = 1. / 4. * (2 * xi1 + xi2) * (1 + xi2);
          Ndxi(1, 2) = 1. / 4. * (1 + xi1) * (xi1 + 2 * xi2);

          Ndxi(0, 3) = -1. / 4. * (-2 * xi1 + xi2) * (1 + xi2);
          Ndxi(1, 3) = 1. / 4. * (1 - xi1) * (-xi1 + 2 * xi2);

          Ndxi(0, 4) = -xi1 * (1 - xi2);
          Ndxi(1, 4) = -1. / 2. * (1 - pow(xi1, 2));

          Ndxi(0, 5) = 1. / 2. * (1 - pow(xi2, 2));
          Ndxi(1, 5) = (1 + xi1) * (-xi2);

          Ndxi(0, 6) = -xi1 * (1 + xi2);
          Ndxi(1, 6) = 1. / 2. * (1 - pow(xi1, 2));

          Ndxi(0, 7) = -1. / 2. * (1 - pow(xi2, 2));
          Ndxi(1, 7) = (1 - xi1) * (-xi2);
        }
        else if (shape == "quad8on") {
          // Just other node numbering
          fmatvec::Mat2xV Nquad8 = Deriv1(xi, "quad8");
          for (int i = 0; i < 2; i++) {
            Ndxi(i, 0) = Nquad8(i, 0);
            Ndxi(i, 1) = Nquad8(i, 4);
            Ndxi(i, 2) = Nquad8(i, 1);
            Ndxi(i, 3) = Nquad8(i, 5);
            Ndxi(i, 4) = Nquad8(i, 2);
            Ndxi(i, 5) = Nquad8(i, 6);
            Ndxi(i, 6) = Nquad8(i, 3);
            Ndxi(i, 7) = Nquad8(i, 7);
          }
        }
        else if (shape == "quad9") {
          Ndxi(0, 0) = 1. / 4. * xi2 * (1 - 2 * xi1) * (1 - xi2);
          Ndxi(1, 0) = 1. / 4. * xi1 * (1 - xi1) * (1 - 2 * xi2);

          Ndxi(0, 1) = -1. / 4. * xi2 * (1 + 2 * xi1) * (1 - xi2);
          Ndxi(1, 1) = -1. / 4. * xi1 * (1 + xi1) * (1 - 2 * xi2);

          Ndxi(0, 2) = 1. / 4. * xi2 * (1 + 2 * xi1) * (1 + xi2);
          Ndxi(1, 2) = 1. / 4. * xi1 * (1 + xi1) * (1 + 2 * xi2);

          Ndxi(0, 3) = -1. / 4. * xi2 * (1 - 2 * xi1) * (1 + xi2);
          Ndxi(1, 3) = -1. / 4. * xi1 * (1 - xi1) * (1 + 2 * xi2);

          Ndxi(0, 4) = xi1 * xi2 * (1 - xi2);
          Ndxi(1, 4) = -1. / 2. * (1 - pow(xi1, 2)) * (1 - 2 * xi2);

          Ndxi(0, 5) = 1. / 2. * (1 + 2 * xi1) * (1 - pow(xi2, 2));
          Ndxi(1, 5) = -xi1 * xi2 * (1 + xi1);

          Ndxi(0, 6) = -xi1 * xi2 * (1 + xi2);
          Ndxi(1, 6) = 1. / 2. * (1 - pow(xi1, 2)) * (1 + 2 * xi2);

          Ndxi(0, 7) = -1. / 2. * (1 - 2 * xi1) * (1 - pow(xi2, 2));
          Ndxi(1, 7) = xi1 * xi2 * (1 - xi1);

          Ndxi(0, 8) = -2 * xi1 * (1 - pow(xi2, 2));
          Ndxi(1, 8) = -2 * (1 - pow(xi1, 2)) * xi2;
        }

        return Ndxi;
      }

      // Second parametric derivatives of shape functions evaluated
      // at parametric point xi
      //
      // Input:
      //   xi:         Vector with parametric coordinates [xi1; xi2]
      //   shape:      Shape of element
      //
      // Output:
      //   Ndxidxi:    Matrix with second parametric derivatives of
      //               shape functions evaluated at xi
      //               [N^1,xi1xi1  N^2,xi1xi1  N^3,xi1xi1  ...;
      //                N^1,xi2xi2  N^2,xi2xi2  N^3,xi2xi2  ...;
      //                N^1,xi1xi2  N^2,xi1xi2  N^3,xi1xi2  ...]

      static fmatvec::Mat3xV Deriv2(const fmatvec::Vec2 & xi, const std::string & shape) {
        fmatvec::Mat3xV Ndxidxi(getDim(shape));

        double xi1 = xi(0);
        double xi2 = xi(1);

        if (shape == "quad4") {
          Ndxidxi(0, 0) = 0;
          Ndxidxi(1, 0) = 0;
          Ndxidxi(2, 0) = 1. / 4.;

          Ndxidxi(0, 1) = 0;
          Ndxidxi(1, 1) = 0;
          Ndxidxi(2, 1) = -1. / 4.;

          Ndxidxi(0, 2) = 0;
          Ndxidxi(1, 2) = 0;
          Ndxidxi(2, 2) = 1. / 4.;

          Ndxidxi(0, 3) = 0;
          Ndxidxi(1, 3) = 0;
          Ndxidxi(2, 3) = -1. / 4.;
        }
        else if (shape == "quad8") {
          Ndxidxi(0, 0) = 1. / 2. * (1 - xi2);
          Ndxidxi(1, 0) = 1. / 2. * (1 - xi1);
          Ndxidxi(2, 0) = 1. / 4. * (1 - 2 * xi1 - 2 * xi2);

          Ndxidxi(0, 1) = 1. / 2. * (1 - xi2);
          Ndxidxi(1, 1) = 1. / 2. * (1 + xi1);
          Ndxidxi(2, 1) = -1. / 4. * (1 + 2 * xi1 - 2 * xi2);

          Ndxidxi(0, 2) = 1. / 2. * (1 + xi2);
          Ndxidxi(1, 2) = 1. / 2. * (1 + xi1);
          Ndxidxi(2, 2) = 1. / 4. * (1 + 2 * xi1 + 2 * xi2);

          Ndxidxi(0, 3) = 1. / 2. * (1 + xi2);
          Ndxidxi(1, 3) = 1. / 2. * (1 - xi1);
          Ndxidxi(2, 3) = -1. / 4. * (1 - 2 * xi1 + 2 * xi2);

          Ndxidxi(0, 4) = -(1. - xi2);
          Ndxidxi(1, 4) = 0;
          Ndxidxi(2, 4) = xi1;

          Ndxidxi(0, 5) = 0;
          Ndxidxi(1, 5) = -(1. + xi1);
          Ndxidxi(2, 5) = -xi2;

          Ndxidxi(0, 6) = -(1. + xi2);
          Ndxidxi(1, 6) = 0;
          Ndxidxi(2, 6) = -xi1;

          Ndxidxi(0, 8) = 0;
          Ndxidxi(1, 8) = -(1. - xi1);
          Ndxidxi(2, 8) = xi2;
        }
        else if (shape == "quad8on") {
          // Just other node numbering
          fmatvec::Mat3xV Nquad8 = Deriv2(xi, "quad8");
          for (int i = 0; i < 3; i++) {
            Ndxidxi(i, 0) = Nquad8(i, 0);
            Ndxidxi(i, 1) = Nquad8(i, 4);
            Ndxidxi(i, 2) = Nquad8(i, 1);
            Ndxidxi(i, 3) = Nquad8(i, 5);
            Ndxidxi(i, 4) = Nquad8(i, 2);
            Ndxidxi(i, 5) = Nquad8(i, 6);
            Ndxidxi(i, 6) = Nquad8(i, 3);
            Ndxidxi(i, 7) = Nquad8(i, 7);
          }
        }
        else if (shape == "quad9") {
          Ndxidxi(0, 0) = -1. / 2. * xi2 * (1 - xi2);
          Ndxidxi(1, 0) = -1. / 2. * xi1 * (1 - xi1);
          Ndxidxi(2, 0) = 1. / 4. * (1 - 2 * xi1) * (1 - 2 * xi2);

          Ndxidxi(0, 1) = -1. / 2. * xi2 * (1 - xi2);
          Ndxidxi(1, 1) = 1. / 2. * xi1 * (1 + xi1);
          Ndxidxi(2, 1) = -1. / 4. * (1 + 2 * xi1) * (1 - 2 * xi2);

          Ndxidxi(0, 2) = 1. / 2. * xi2 * (1 + xi2);
          Ndxidxi(1, 2) = 1. / 2. * xi1 * (1 + xi1);
          Ndxidxi(2, 2) = 1. / 4. * (1 + 2 * xi1) * (1 + 2 * xi2);

          Ndxidxi(0, 3) = 1. / 2. * xi2 * (1 + xi2);
          Ndxidxi(1, 3) = -1. / 2. * xi1 * (1 - xi1);
          Ndxidxi(2, 3) = -1. / 4. * (1 - 2 * xi1) * (1 + 2 * xi2);

          Ndxidxi(0, 4) = xi2 * (1 - xi2);
          Ndxidxi(1, 4) = 1 - pow(xi1, 2);
          Ndxidxi(2, 4) = xi1 * (1 - 2 * xi2);

          Ndxidxi(0, 5) = 1 - pow(xi2, 2);
          Ndxidxi(1, 5) = -xi1 * (1 + xi1);
          Ndxidxi(2, 5) = -xi2 * (1 + 2 * xi1);

          Ndxidxi(0, 6) = -xi2 * (1 + xi2);
          Ndxidxi(1, 6) = 1 - pow(xi1, 2);
          Ndxidxi(2, 6) = -xi1 * (1 + 2 * xi2);

          Ndxidxi(0, 7) = 1 - pow(xi2, 2);
          Ndxidxi(1, 7) = xi1 * (1 - xi1);
          Ndxidxi(2, 7) = xi2 * (1 - 2 * xi1);

          Ndxidxi(0, 8) = -2 * (1 - pow(xi2, 2));
          Ndxidxi(1, 8) = -2 * (1 - pow(xi1, 2));
          Ndxidxi(2, 8) = 4 * xi1 * xi2;
        }

        return Ndxidxi;
      }
  };
}
