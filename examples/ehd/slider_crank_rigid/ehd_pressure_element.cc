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

#include "ehd_pressure_element.h"

#include "lagrange_shape_functions.h"

#include <mbsim/mbsim_event.h>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

namespace MBSimEHD {

  void GaussPoints1D(int ngp, fmatvec::MatVx2 & xigp, fmatvec::VecV & wgp) {

    if (ngp == 1) {
      xigp(0, 0) = 0;
      wgp(0) = 2;
    }
    else if (ngp == 2) {
      xigp(0, 0) = -1 / sqrt(3);
      xigp(1, 0) = 1 / sqrt(3);
      wgp(0) = 1;
      wgp(1) = 1;
    }
    else if (ngp == 3) {
      xigp(0, 0) = -sqrt(3 / 5);
      xigp(1, 0) = 0;
      xigp(2, 0) = sqrt(3 / 5);
      wgp(0) = 5 / 9;
      wgp(1) = 8 / 9;
      wgp(2) = 5 / 9;
    }
    else if (ngp == 4) {
      xigp(0, 0) = -sqrt((15 + sqrt(120)) / 35);
      xigp(1, 0) = -sqrt((15 - sqrt(120)) / 35);
      xigp(2, 0) = sqrt((15 - sqrt(120)) / 35);
      xigp(3, 0) = sqrt((15 + sqrt(120)) / 35);
      wgp(0) = (18 - sqrt(30)) / 36;
      wgp(1) = (18 + sqrt(30)) / 36;
      wgp(2) = (18 + sqrt(30)) / 36;
      wgp(3) = (18 - sqrt(30)) / 36;
    }
    else
      throw MBSimError("Only up to four Gauss points implemented.");

  }

  void GaussPoints2D(int ngp, fmatvec::MatVx2 & xigp, fmatvec::VecV & wgp) {

    int ngp1D = sqrt(ngp);
    if (ngp1D % 1 != 0)
      throw MBSimError("No quadratic number of Gauss points.");

    MatVx2 xigp1D(ngp1D);
    VecV wgp1D(ngp1D);
    GaussPoints1D(ngp1D, xigp1D, wgp1D);

    int k = 0;
    for (int j = 0; j < ngp1D; j++) {
      for (int i = 0; i < ngp1D; i++) {
        xigp(k, 0) = xigp1D(i, 0);
        xigp(k, 1) = xigp1D(j, 0);
        wgp(k) = wgp1D(i) * wgp1D(j);
        k = k + 1;
      }
    }
  }

  fmatvec::MatV ShapeFctMat(const fmatvec::RowVecV S, const int & nnodval) {
    // Get number of element nodes
    int nnode = S.cols();

    // Build shape function matrix
    MatV Nm(nnodval, nnodval * nnode);
    for (int i = 0; i < nnode; i++) {
      for (int j = 0; j < nnodval; j++) {
        Nm(j, nnodval * i + j) = S(i);
        /*TODO: the matlab code is not fully understood here.
         * It says that instead of the RowVecV-input of "N" it could also handle the derivatives "Ndxi" or "Ndxidxi".
         * Yet the operator "S(i)" (i.e. N(i) in matlab) is not clear as it gets strange elements.
         * One has to check the code in case the function is called for the derivatives
         */
      }
    }
    return Nm;
  }

  ElementShape ElementShapes(const std::string & shapeName) {
    ElementShape shape;

    shape.name = shapeName;
    // default values
    shape.ndim = 1;
    shape.nnod = 1;
    shape.nnodd[0] = 1;
    shape.nnodd[1] = 1;
    shape.ser = 0;

    if (shapeName.compare("line2") == 0) {
      shape.ndim = 1;
      shape.nnod = 2;
      shape.nnodd[0] = shape.nnod;
    }
    else if (shapeName.compare("line3") == 0) {
      shape.ndim = 1;
      shape.nnod = 3;
      shape.nnodd[0] = shape.nnod;
    }
    else if (shapeName.compare("quad4") == 0) {
      shape.ndim = 2;
      shape.nnod = 4;
      shape.nnodd[0] = 2;
      shape.nnodd[1] = 2;
      shape.ser = 0;
    }
    else if (shapeName.compare("quad8") == 0 or shapeName.compare("quad8on") == 0) {
      shape.ndim = 2;
      shape.nnod = 8;
      shape.nnodd[0] = 3;
      shape.nnodd[1] = 3;
      shape.ser = 1;
    }
    else if (shapeName.compare("quad9") == 0) {
      shape.ndim = 2;
      shape.nnod = 9;
      shape.nnodd[0] = 3;
      shape.nnodd[1] = 3;
      shape.ser = 0;
    }
    else {
      throw MBSimError("Unknwon shape type.");
    }

    return shape;
  }

  EHDPressureElement::EHDPressureElement(const std::string & shapeName, const int & ngp) {
    //  // Define ent properties
    shape = ElementShapes(shapeName);
    ndof = shape.nnod * ndofpernod;
    this->ngp = ngp;
  }

  void EHDPressureElement::EvaluateElement(const int & e, const VecV & pose, const VecV & de, const JournalBearing & sys, const Lubricant & lub, fmatvec::VecV & re, SqrMatV & kTe) const {

    // Define abbreviations
    int ndime = shape.ndim;
    int ndofe = ndof;

    // Initialize ent matrices and ent vectors
    SqrMatV ke(ndofe);
    SqrMatV ce(ndofe);
    VecV s0e(ndofe); //TODO: Vec?
    VecV sPe(ndofe); //TODO: Vec?
    VecV sSUPGe(ndofe); //TODO: Vec?

    // Initialize nodal derivatives of ent vectors
    SqrMatV skedd(ndofe);
    SqrMatV scedd(ndofe);
    SqrMatV s0edd(ndofe);
    SqrMatV sPedd(ndofe);
    SqrMatV sSUPGedd(ndofe);

    // Define reference values for dimensionless description
    bool dimLess = sys.dimLess;
    double hr = 0; //TODO: find out what happens in the non dimLess case as hr is not defined then (looking at the matlab code)
    double xr = 0; //TODO: find out what happens in the non dimLess case as xr is not defined then (looking at the matlab code)
    double lambda = 0; //TODO: find out what happens in the non dimLess case as lambda is not defined then (looking at the matlab code)

    if (dimLess) {
      double hr = sys.hrF;
      double pr = lub.pr;
      double eta0 = lub.eta0;
      double xr = sys.xrF;
      double lambda = pow(hr, 2) * pr / (xr * eta0);
    }
    else
      cout << "WARNING: Now in the non-dimLess case and hr, xr and lambda are used later but are not properly initialized..." << endl;

    // Retrieve Gauss points and weights
    MatVx2 xigp(ngp);
    VecV wgp(ngp);
    if (ndime == 1) {
      GaussPoints1D(ngp, xigp, wgp);
    }
    else if (ndime == 2) {
      GaussPoints2D(ngp, xigp, wgp);
    }
    else
      throw MBSimError("No valid dimension");

// Loop through all Gauss points to integrate ent matrices
// and ent vectors and their nodal derivatives
    for (int g = 0; g < ngp; g++) {
// Extract current Gauss point and weight
      VecV xi = xigp.row(g).T();
      double w = wgp(g);

// Get shape function matrices Np and Nx, spatial gradient
// Npdx, second spatial derivatives Ndxdx of shape functions
// (only for SUPG) and element Jacobi determinant at xi
//TODO: sizes of matrices!!
      RowVecV Np;
      Mat2xV Npdx;
      Mat3xV Ndxdx;
      Mat2xV Nx;
      double detJ;
      GetShapeFunctions(pose, xi, Np, Npdx, Ndxdx, Nx, detJ);

// Compute position x = [y; z] at xi
      VecV x = Nx * pose;

// Compute pressure p and its spatial gradient at xi
      double p = Np * de;
      VecV pdx = Npdx * de;

// Retrieve film thickness and spatial derivatives at x
      double h1, h2, h1dy, h2dy; //TODO: correct type?
      sys.Thickness(x, e, g, h1, h2, h1dy, h2dy);
      double h = h2 - h1;
      double hdy = h2dy - h1dy;

// Retrieve surface velocities and spatial derivatives at x
      double u1, u2, v1, v2, v1dy, v2dy; //TODO: correct type?
      sys.Velocities(x, e, g, u1, u2, v1, v2, v1dy, v2dy);
      //TODO: for interface later make Vec3 for all velocities

// Retrieve fluid parameters and their derivatives at p
      double eta, etadp, etadpdp;
      lub.DynViscosity(p, eta, etadp, etadpdp);
      double rho, rhodp, rhodpdp;
      lub.Density(p, rho, rhodp, rhodpdp);

// --------------------------------------------------------
// ent matrices and vectors
// --------------------------------------------------------

// Compute abbreviation in Poiseuille terms (diffusion
// coefficient), eq. 2.14
      double C = rho * pow(h, 3) / (12 * eta);

// Compute abbreviation in density wedge terms (convection
// tensor), eq. 2.16
      Vec2 beta;
      beta(0) = v1 + v2;
      beta = rhodp * h / 2 * beta;

//TODO: the following is not possible in c++ --> must conceptionally adpated!
//      if (ndime == 1) {
//        beta = beta(1);
//      }
      if (dimLess) {
        beta = beta / lambda;
      }

// Compute abbreviation of physical wedge, stretch and
// (optional) squeeze terms (source terms)
// TODO: Add terms with velocity w1 and w2 if required
      double f;
      if (!dimLess) {
        f = -rho * (hdy * (v1 + v2) / 2 + h * (v1dy + v2dy) / 2);
        if (squeeze) {
          f = f - rho * (u2 - u1 - v2 * h2dy + v1 * h1dy);
        }
      }
      else {
        f = -rho * (hdy * (v1 + v2) / 2 + h * (v1dy + v2dy) / 2) / lambda;
        if (squeeze) {
          f = f - rho * ((u2 - u1) * xr / hr - v2 * h2dy + v1 * h1dy) / lambda;
        }
      }

// Add contribution of current Gauss point to element
// matrices and element vector
      ke = ke + Npdx.T() * C * Npdx * detJ * w;
      ce = ce + Np.T() * beta.T() * Npdx * detJ * w;
      s0e = s0e - Np.T() * f * detJ * w;

// --------------------------------------------------------
// Linearization of element matrices and element vector
// --------------------------------------------------------

// Compute nodal derivative of C
      double Cdp = pow(h, 3) / (12 * eta) * rhodp - rho * pow(h, 3) / (12 * pow(eta, 2)) * etadp;
      RowVecV Cdd = Cdp * Np;

// Compute nodal derivative of beta
      Vec2 betadp;
      betadp(0) = v1 + v2;
      betadp = rhodpdp * h / 2 * betadp;
      //TODO: the following is not possible in c++ --> must conceptionally adpated!
//      if (ndime == 1) {
//        betadp = betadp(1);
//      }
      if (dimLess) {
        betadp = betadp / lambda;
      }

      Mat2xV betadd(Np.size());
      betadd.set(0, betadp(0) * Np);
      betadd.set(1, betadp(1) * Np);

      // Compute nodal derivative of f
      // TODO: Add terms with velocity w1 and w2 if required
      double fdp;
      if (not dimLess) {
        fdp = -rhodp * (hdy * (v1 + v2) / 2 + h * (v1dy + v2dy) / 2);
        if (squeeze) {
          fdp = fdp - rhodp * (u2 - u1 - v2 * h2dy + v1 * h1dy);
        }
      }
      else {
        fdp = -rhodp * (hdy * (v1 + v2) / 2 + h * (v1dy + v2dy) / 2) / lambda;
        if (squeeze) {
          fdp = fdp - rhodp * ((u2 - u1) * xr / hr - v2 * h2dy + v1 * h1dy) / lambda;
        }
      }

      RowVecV fdd = fdp * Np;

      // Add contribution of current Gauss point to nodal
      // derivatives of element vectors
      // Note: ske = ke * de, sce = ce * de
      // Note: pdx = Npdx * de (see above)
      skedd = skedd + Npdx.T() * pdx * Cdd * detJ * w;
      scedd = scedd + Np.T() * pdx.T() * betadd * detJ * w;
      s0edd = s0edd - Np.T() * fdd * detJ * w;


      // Note: At this point skedd and scedd are not completed
      //       (matrices ke and ce are missing). The missing
      //       parts are added after the integration loop.

      // --------------------------------------------------------
      // Penalty regularization
      // --------------------------------------------------------

      // Use penalty regularization only if penalty parameter > 0

      if (pp > 0) {
        // Evaluate penalty law
        double fP, fPdp;
        EvaluatePenaltyLaw(p, fP, fPdp);

        // Add contribution of current Gauss point to penalty
        // element vector
        sPe = sPe - Np.T() * fP * detJ * w;

        // Compute nodal derivative of penalty term
        RowVecV fPdd = fPdp * Np;

        // Add contribution of current Gauss point to nodal
        // derivative of penalty element vector
        sPedd = sPedd - Np.T() * fPdd * detJ * w;
      }

      // --------------------------------------------------------
      // Streamline Upwind Petrov-Galerkin (SUPG) method
      // --------------------------------------------------------

      if (SUPG) {
        throw MBSimError("SUPG-Stabilization not implemented!");
//        // Compute Laplacian of pressure
//        Lp = 0;
//        for(i = 1:1:ndime) {
//          Lp = Lp + Ndxdx(i, :) * de;
//        }
//
//        // Compute derivative of C with respect to film thickness
//        Cdh = rho * h ^ 2 / (4 * eta);
//
//// Build spatial gradient of film thickness
//        hdx = [hdy; 0
//        ];
//        if (ndime == 1) {
//          hdx = hdx(1);
//        }
//
//// Compute spatial gradient of C
//        Cdx = Cdp * pdx + Cdh * hdx;
//
//// Compute residuum of Reynolds equation
//        R = -Cdx.T() * pdx - C * Lp + beta.T() * pdx - f;
//
//// Add penalty term to residuum R
//// Note: This term will be zero in the high pressure
////       region in which the SUPG method operates
//        if (pp > 0) {
//          R = R - fP;
//        }
//
//// Determine characteristic element length in x-direction
//// TODO: Check for another definition of he if required
//        he = abs(max(pose(1:ndime:end)
//            )) - min(pose(1:ndime:end)
//        )));
//
//        // Determine order of interpolation polynomial of element
//        // TODO: Save pe in elementshape
//        pe = shape.nnodd(1) - 1;
//
//        // Compute element Peclet number
//        Pepe = norm(beta) * he / (2 * C * pe);
//
//        // Compute parameter zeta
//        zeta = coth(Pepe) - 1 / Pepe;
//
//        // Compute parameter tau
//        tau = he / (2 * norm(beta) * pe) * zeta;
//
//        // Add contribution of current Gauss point to SUPG
//        // element vector
//        sSUPGe = sSUPGe + R * tau * Npdx.T() * beta * detJ * w;
//
//        // Compute nodal derivative of gradient of pressure
//        pdxdd = Npdx;
//
//        // Compute nodal derivative of Laplacian of pressure
//        Lpdd = zeros(1, shape.nnod);
//        for (int i = 0; i < ndime; i++) {
//        Lpdd = Lpdd + Ndxdx(i, :);
//      }
//
//// Compute additional needed derivatives of C
//      Cdpdp = h ^ 3 / (12 * eta) * (rhodpdp - rho / eta * etadpdp + 2 * rho / (eta ^ 2) * etadp ^ 2 - 2 / eta * etadp * rhodp);
//      Cdhdp = h ^ 2 / (4 * eta) * (rhodp - rho / eta * etadp);
//
//// Compute nodal derivative of gradient of C
//      Cdxdd = (Cdpdp * pdx + Cdhdp * hdx) * Np + Cdp * pdxdd;
//
//// Compute nodal derivative of residuum
//      fDdd = -pdx.T() * Cdxdd - Cdx.T() * pdxdd - Lp * Cdd - C * Lpdd;
//      fKdd = pdx.T() * betadd + beta.T() * pdxdd;
//      fQdd = -fdd;
//      Rdd = fDdd + fKdd + fQdd;
//
//// Add nodal derivative of penalty term to Rdd
//      if (pp > 0) {
//        Rdd = Rdd - fPdd;
//      }
//
//// Compute nodal derivative of norm of beta
//      normbetadd = 1 / norm(beta) * beta.T() * betadd;
//
//// Compute nodal derivative of element Peclet number
//      Pepedd = he / (2 * pe) * (1 / C * normbetadd - norm(beta) / (C ^ 2) * Cdd);
//
//// Compute nodal derivative of parameter zeta
//      zetadd = (1 - coth(Pepe) ^ 2 + 1 / (Pepe ^ 2)) * Pepedd;
//
//// Compute nodal derivative of parameter tau
//      taudd = he / (2 * pe) * (-1 / (norm(beta) ^ 2) * zeta * normbetadd + 1 / norm(beta) * zetadd);
//
//// Add contribution of current Gauss point to nodal
//// derivative of SUPG element vector
//      sSUPGedd = sSUPGedd + (tau * Npdx.T() * beta * Rdd + R * Npdx.T() * beta * taudd + R * tau * Npdx.T() * betadd) * detJ * w;
//    }
      }
    }

    // Finish computation of nodal derivatives of element vectors
    skedd = ke + skedd;
    scedd = ce + scedd;

    // Compute element residuum and element tangential matrix
    re = (ke + ce) * de + s0e + sPe + sSUPGe;
    kTe = skedd + scedd + s0edd + sPedd + sSUPGedd;

  }

  void EHDPressureElement::GetShapeFunctions(const fmatvec::VecV & pose, const fmatvec::VecV &xi, fmatvec::RowVecV & Np, fmatvec::Mat2xV & Npdx, fmatvec::Mat3xV & Ndxdx, fmatvec::Mat2xV & Nx, double & detJ) const {

// Define abbreviation
    int ndime = shape.ndim;

// Evaluate Lagrange shape functions and parametric derivatives
//TODO: dimensions?!
    RowVecV N;
    Mat2xV Ndxi;
    Mat3xV Ndxidxi;
    if (ndime == 1) {
      N = LagrShapeFct1D::Fct(xi, shape.name);
      Ndxi = LagrShapeFct1D::Deriv1(xi, shape.name);
      if (SUPG)
        Ndxidxi = LagrShapeFct1D::Deriv2(xi, shape.name);
    }
    else if (ndime == 2) {
      N = LagrShapeFct2D::Fct(xi, shape.name);
      Ndxi = LagrShapeFct2D::Deriv1(xi, shape.name);
      if (SUPG)
        Ndxidxi = LagrShapeFct2D::Deriv2(xi, shape.name);
    }
    else
      throw MBSimError("No valid dimension given!");

// Build shape function matrices for pressure and geometry
    Np = N;
    Mat2xV Npdxi = Ndxi;
    Nx = ShapeFctMat(N, ndime);

// Build isoparametric element Jacobian matrix J = (dx/dxi)^T
    SqrMatV J(ndime);      //TODO: SqrMat?
    for (int i = 0; i < ndime; i++) {
      MatV Nxdxii = ShapeFctMat(Ndxi.row(i), ndime);
      J.set(i, (Nxdxii * pose).T());
    }

    // Compute element Jacobi determinant
    //detJ = J;
    //TODO: check if the determinant is really correct here!
    VecVI ipiv(J.size());

    SqrMatV JLU = facLU(J, ipiv);

    detJ = 0;
    for (int i = 0; i < JLU.cols(); i++)
      detJ *= JLU(i, i);

    //TODO: determinant is working for 2D element:
    detJ = J(0,0) * J(1,1) - J(0,1)*J(1,0);
    SqrMatV Jinv(2);
    Jinv(0,0) = J(1,1);
    Jinv(0,1) = -J(0,1);
    Jinv(1,0) = -J(1,0);
    Jinv(1,1) = J(0,0);
    Jinv = Jinv / detJ;

// Determine spatial gradient of Np (Npdx = Ndx)
    // Npdx = slvLUFac(JLU, Npdxi, ipiv); // TODO does not work

    Npdx = Jinv * Npdxi;

    //Npdx = J  \ Npdxi;

    MatV G;
    if (SUPG) {
      throw MBSimError("SUPG-stabilization no implemented (yet).");
// Compute matrix G
//      if (ndime == 1) {
//        G = J * J;
//      }
//      else if (ndime == 2) {
//      G = [J(1, 1)^2, J(1, 2)^2, 2 * J(1, 1) * J(1, 2); ...
//      J(2, 1)^2, J(2, 2)^2, 2 * J(2, 1) * J(2, 2); ...
//      J(1, 1) * J(2, 1), J(1, 2) * J(2, 2), ...
//      J(1, 1) * J(2, 2) + J(1, 2) * J(2, 1)];
//    }
//    else {
//      throw MBSimError("Wrong dimension!");
//    }
//
//// Compute matrix C1
//    nderiv = (ndime * (ndime + 1) / 2);
//    H = zeros(nderiv, ndime);
//    for (int i = 1; i < nderiv; i++) {
//    Nxdxidxii = ShapeFctMat(Ndxidxi(i, :), ndime);
//    H(i, :) = (Nxdxidxii * pose)';
//  }
//
//// Determine second spatial derivatives of N
//  Ndxdx = G\ (Ndxidxi - H * Npdx);
    }
    else {
      // No stabilization with SUPG
//    Ndxdx =;
    }
  }

  void EHDPressureElement::EvaluatePenaltyLaw(const double & p, double & fP, double & fPdp) const {
    //Compute penalty term and its derivative with respect to p
    if (p < 0) {
      fP = -pp * p;
      fPdp = -pp;
    }
    else {
      fP = 0;
      fPdp = 0;
    }
  }
}
