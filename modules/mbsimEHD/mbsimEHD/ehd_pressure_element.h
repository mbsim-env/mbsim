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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _EHD_PRESSURE_ELEMENT_H_
#define _EHD_PRESSURE_ELEMENT_H_

#include "lubricant.h"
#include "cylindersolid_cylinderhollow_ehd.h"

#include <fmatvec/fmatvec.h>

#include <string>

namespace MBSimEHD {

  // Build shape function matrix
  // Michael Hofer, 20.03.2015
  //
  // Input:
  //   N:          Shape functions N or derivatives Ndxi, Ndxidxi
  //   nnodval:    Number of nodal values
  //
  // Output:
  //   Nm:         Shape function matrix
  //
  // Note:
  //   For the case nnodval = 1 this function is not needed, i.e. Nm = N
  fmatvec::MatV ShapeFctMat(const fmatvec::RowVecV N, const int & nnodval);

  //   shape:      Struct with element shape properties
  //               ndim:   Spatial dimension
  //               nnod:   Number of nodes
  //               nnodd:  Number of nodes in spatial directions
  //               ser:    Number of additional nodes located at
  //                       element boundary (serendipity elements)
  struct ElementShape {
      int ndim;
      int nnod;
      int nnodd[2];
      int ser;
      std::string name;
  };

  // Element shape properties
  // Michael Hofer, 24.03.2015
  //
  // Input:
  //   shapeName:  Name of element shape
  //
  // Output:
  // ElementShape (see above)
  ElementShape ElementShapes(const std::string & shapeName);

  class EHDPressureElement {
      // Pressure-based element for solving Reynolds equation
      // on one- or two-dimensional domain
      // Michael Hofer, 22.03.2015

      friend class EHDMesh;

    public:

      //Standard constructor
      EHDPressureElement();

      // Constructor for element
      //
      // Input:
      //   shapeName:  Name of element shape, e.g. 'quad4'
      //   ngp:        Number of Gauss points, e.g. 4 for 2x2
      //
      // Output:
      //   ele:        Object of element
      EHDPressureElement(const std::string & shapeName, const int & ngp);

      /*!
       * \brief initialize following the MBSim-init stages
       */
      void init(MBSim::Element::InitStage stage);

      // Evaluate element
      //
      // Input:
      //   e:      Element number in spatial discretization
      //   pose:   Spatial positions of element nodes
      //   de:     Vector with nodal values for pressure on element
      //   sys:    Object of system (TODO: see class JournalBearing -> it is the geometry description which finally should (probably) be covered by the contactKinematics!!!)
      //
      // Output:
      //   re:     Element residuum
      //   kTe:    Element tangential matrix
      void EvaluateElement(const int & e, const fmatvec::VecV & pose, const fmatvec::VecV & de, fmatvec::VecV & re, fmatvec::SqrMatV & kTe) const;

      // Evaluate element
      //
      // Input:
      //   e:      Element number in spatial discretization
      //   pose:   Spatial positions of element nodes
      //   sys:    Object of system (TODO: see class JournalBearing -> it is the geometry description which finally should (probably) be covered by the contactKinematics!!!)
      //
      // Output:
      //   cffe:   Element of force matrix
      void CalculateForceMatrixElement(const int & e, const fmatvec::VecV & pose, fmatvec::SqrMatV & cffe) const;

      // Return shape function matrices at parametric point xi
      //
      // Input:
      //   pose:   Spatial positions of element nodes
      //   xi:     Parametric point
      //
      // Output:
      //   Np:     Matrix of shape functions for pressure (p. 16, eq. 3.5 ff)
      //   Npdx:   Spatial gradient of Np (Np = N), see
      //   Ndxdx:  Second spatial derivatives of N
      //   Nx:     Matrix of shape functions for geometry
      //   detJ:   Element Jacobi determinant
      void GetShapeFunctions(const fmatvec::VecV & pose, const fmatvec::VecV &xi, fmatvec::RowVecV & Np, fmatvec::Mat2xV & Npdx, fmatvec::Mat3xV & Ndxdx, fmatvec::Mat2xV & Nx, double & detJ) const;

      // Evaluate linear penalty law
      //
      // Input:
      //   ele:    Object of element
      //   p:      Pressure
      //
      // Output:
      //   fP:     Penalty term
      //   fPdp:   Derivative of fP with respect to p
      void EvaluatePenaltyLaw(const double & p, double & fP, double & fPdp) const;

      int getndof() {
        return ndof;
      }

      void setLubricant(const Lubricant & lub_) {
        lub = lub_;
      }

      void setContactKinematics(ContactKinematicsEHDInterface * cK_) {
        ck = cK_;
      }

    protected:
      ElementShape shape;              // Properties of element shape
      int ndofpernod = 1;     // Number of DOFs per node
      int ndof;               // Number of DOFs (= ndofpernod * nnod)
      int ngp;                // Number of Gauss points for integration

      bool squeeze = true;     // Flag for consideration of squeeze terms
      double pp = 0;                 // Penalty parameter (positive?!)
      bool SUPG = false;  // Flag for SUPG method

      /*!
       * \brief lubricant of the Finite-Element
       */
      Lubricant lub;

      /*!
       * \brief contact kinematics for calls inside Evaluate Element
       */
      ContactKinematicsEHDInterface * ck;

      /*!
       * \brief spatial positions of the gauss-points of the element
       */
      fmatvec::MatVx2 xigp;

      /*!
       * \brief weights of the gauss-points
       */
      fmatvec::VecV wgp;
  };

//
//function fe2 = ComputeResElementForce(ele, pose, de, sys)
//            // Compute resultant force on element
//            //
//            // Input:
//            //   ele:    Object of element
//            //   pose:   Spatial positions of element nodes
//            //   de:     Vector with nodal values for pressure on element
//            //   sys:    Object of system
//            //
//            // Output:
//            //   fe2:    Element force on body 2
//
//            // Define abbreviation
//ndime = ele.shape.ndim;
//
//            // Initialize three-dimensional resultant force vector
//fe2 = zeros(3, 1);
//
//            // Retrieve Gauss points and weights
//switch ndime
//case 1
//[xigp, wgp] = GaussPoints1D(ele.ngp);
//case 2
//[xigp, wgp] = GaussPoints2D(ele.ngp);
//}
//
//            // Loop through all Gauss points to integrate resultant force
//            // vector
//for g = 1:1:ele.ngp
//            // Extract current Gauss point and weight
//xi = xigp(g, :)';
//w = wgp(g);
//
//// Get shape function matrices Np and Nx and element
//// Jacobi determinant at xi
//[Np, ~, ~, Nx, detJ] = GetShapeFunctions(ele, pose, xi);
//
//// Compute position x = [y; z] at xi
//x = Nx * pose;
//
//// Compute pressure p at xi
//p = Np * de;
//
//// Retrieve surface normal
//n2 = SurfaceNormal(sys, x);
//
//// Add contribution of current Gauss point to resultant
//// force vector
//fe2 = fe2 - p * n2 * detJ * w;
//}
//}
//
//function [Pepegpe, xgpe] = ComputePecletGaussPoints(ele, e, pose, de, sys, lub)
//            // Compute element Peclet number at Gauss points
//            //
//            // Input:
//            //   ele:        Object of element
//            //   e:          Element number in spatial discretization
//            //   pose:       Spatial positions of element nodes
//            //   de:         Vector with nodal values for pressure on element
//            //   sys:        Object of system
//            //   lub:        Object of lubricant
//            //
//            // Output:
//            //   Pepegpe:    Column vector with element Peclet number at
//            //               Gauss points of element
//            //   xgpe:       Matrix with spatial Gauss point positions in
//            //               its rows
//
//            // Define abbreviation
//ndime = ele.shape.ndim;
//
//            // Initialize vector for element Peclet number at Gauss points
//            // and vector with spatial Gauss point position
//Pepegpe = zeros(ele.ngp, 1);
//xgpe = zeros(ele.ngp, ndime);
//
//            // Define reference values for dimensionless description
//dimLess = sys.dimLess;
//if dimLess
//hr = sys.hrF;
//pr = lub.pr;
//eta0 = lub.eta0;
//xr = sys.xrF;
//lambda = hr^2 * pr / (xr * eta0);
//}
//
//                // Retrieve Gauss points
//switch ndime
//case 1
//[xigp, ~] = GaussPoints1D(ele.ngp);
//case 2
//[xigp, ~] = GaussPoints2D(ele.ngp);
//}
//
//                // Loop through all Gauss points to compute element Peclet
//                // number at Gauss points
//for g = 1:1:ele.ngp
//                // Extract current Gauss point
//xi = xigp(g, :)';
//
//// Get shape function matrices Np and Nx at xi
//[Np, ~, ~, Nx, ~] = GetShapeFunctions(ele, pose, xi);
//
//// Compute position x = [y; z] at xi
//x = Nx * pose;
//
//// Compute pressure p at xi
//p = Np * de;
//
//// Retrieve film thickness at x
//[h1, h2, ~, ~] = Thickness(sys, x, e, g);
//h = h2 - h1;
//
//// Retrieve surface velocities at x
//[~, ~, v1, v2, ~, ~] = Velocities(sys, x, e, g);
//
//// Retrieve fluid parameters and their derivatives at p
//[eta, ~, ~] = DynViscosity(lub, p);
//[rho, rhodp, ~] = Density(lub, p);
//
//// Compute abbreviation in Poiseuille terms (diffusion
//// coefficient) and in density wedge terms (convection tensor)
//C = rho * h^3 / (12 * eta);
//beta = rhodp * h / 2 * [v1 + v2; 0];
//if ndime == 1
//beta = beta(1);
//}
//if dimLess
//beta = beta / lambda;
//}
//
//            // Determine characteristic element length in x-direction
//            // TODO: Check for another definition of he if required
//he = abs(max(pose(1:ndime:})) - min(pose(1:ndime:})));
//
//                // Determine order of interpolation polynomial on element
//                // TODO: Save pe in element shape
//pe = ele.shape.nnodd(1) - 1;
//
//                // Store element Peclet number at Gauss point and position
//                // of Gauss point
//Pepegpe (g) = norm(beta) * he / (2 * C * pe);
//xgpe(g, :) = x';
//}
//}
//
//function pmaxe = FindMaximalPressure(ele, pose, de)
//                // Find maximal pressure on element
//                //
//                // Input:
//                //   ele:    Object of element
//                //   pose:   Spatial positions of element nodes
//                //   de:   Vector with nodal values for pressure on element
//                //
//                // Output:
//                //   pmaxe:  Maximal pressure on element
//
//                // Define abbreviation
//ndime = ele.shape.ndim;
//
//                // Define one or two dimensional grid to find maximal pressure
//xigrid1D = -1:0.25:1;
//ngridd = ones(2, 1);
//ngridd(1:1:ndime) = length(xigrid1D);
//ngrid = 1;
//for i = 1:1:ndime
//ngrid = ngrid * ngridd(i);
//}
//xigrid = zeros(ngrid, ndime);
//k = 1;
//for j = 1:1:ngridd(2)
//for i = 1:1:ngridd(1)
//xigrid(k, :) = [xigrid1D(i), xigrid1D(j)];
//k = k + 1;
//}
//}
//
//                // Loop through all grid points to find maximal pressure
//pmaxe = 0;
//for gr = 1:1:ngrid
//                // Extract current grid point
//xi = xigrid(gr, :)';
//
//// Get shape function matrix Np at xi
//[Np, ~, ~, ~, ~] = GetShapeFunctions(ele, pose, xi);
//
//// Compute pressure p at xi
//p = Np * de;
//
//// Check for maximal pressure
//if p > pmaxe
//pmaxe = p;
//}
//}
//}
//}
//}
}
#endif
