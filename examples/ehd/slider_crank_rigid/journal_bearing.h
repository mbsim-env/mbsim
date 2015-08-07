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

#ifndef _JOURNAL_BEARING_H_
#define _JOURNAL_BEARING_H_

#include <fmatvec/fmatvec.h>

namespace MBSimEHD {
// Journal bearing with rigid journal and rigid bearing shell
// Michael Hofer, 09.01.2015
  class JournalBearing {

      friend class EHDPressureElement;

    public:
      // Constructor
      //
      // Input:
      //   dimLess:    Flag for dimensionless description
      //
      // Output:
      //   sys:        Object of system
      JournalBearing(bool dimLess = false);

      // Film thickness and derivatives
      //
      // Input:
      //   sys:    Object of system
      //   x:      Point inside fluid domain y or [y; z]
      //   e(~):   Element number in spatial discretization
      //   g(~):   Gauss point number in element corresponding to x
      //
      // Output:
      //   h1:     Distance from bearing shell center point to point
      //           on journal surface
      //   h2:     Distance from bearing shell center point to point
      //           on inner bearing shell surface
      //   h1dy:   Derivative of h1 with respect to y
      //   h2dy:   Derivative of h2 with respect to y
      void Thickness(const fmatvec::VecV & x, const int & e, const int & g, double & h1, double & h2, double & h1dy, double & h2dy) const;

      // Velocities on journal and inner bearing shell surface and
      // derivatives
      //
      // Input:
      //   sys:    Object of system
      //   x:      Point inside fluid domain y or [y; z]
      //   e(~):   Element number in spatial discretization
      //   g(~):   Gauss point number in element corresponding to x
      //
      // Output:
      //   u1, v1: Velocities on journal surface
      //   u2, v2: Velocities on inner bearing shell surface
      //   v1dy:   Derivative of v1 with respect to y
      //   v2dy:   Derivative of v2 with respect to y
      void Velocities(const fmatvec::VecV & x, const int & e, const int & g, double & u1, double & u2, double & v1, double & v2, double & v1dy, double & v2dy) const;

      // Radial and tangential eccentricity in coordinate system F
      // and auxiliary length variable
      //
      // Input:
      //   sys:    Object of system
      //   y:      Coordinate of fluid domain
      //
      // Output:
      //   er:     Radial eccentricity in coordinate system F
      //   er:     Tangential eccentricity in coordinate system F
      //   r1:     Auxiliary length variable
      void Eccentricity(const double & y, double & er, double & et, double & r1) const ;

      // Mapping between y-coordinate of fluid domain (unwrapped
      // inner bearing shell surface) and angle phi of rotated
      // coordinate system F
      //
      // Input:
      //   sys:    Object of system
      //   y:      Coordinate of fluid domain
      //
      // Output:
      //   phi:    Angle of rotated coordinate system F
      //   AFK:    Rotation matrix
      //TODO: fit to MBSim SqrMat3 etc. --> use the mbsim functions!!
      void AngleCoordSys(const double & y, double & phi, fmatvec::SqrMat2 & AFK) const;

      void setomega1(double om1) {
        omega1 = om1;
      }

      void setFr(double Fr_) {
        Fr = Fr_;
      }

      fmatvec::SqrMat3 getM1() {
        return M1;
      }

      double getR2() {
        return R2;
      }

      double getL() {
        return L;
      }

    private:
// Geometry
      double R1;      // Radius of journal
      double R2 = 0.0225;            // Radius of bearing (inner radius)
      double h0 = 1.7e-5;            // Radial bearing clearance
      double L = 0.022;              // Length of bearing

// Inertia
      double m = 10;
      double J = 1;
      fmatvec::SqrMat3 M1;
      fmatvec::SqrMat3 M2;

// External forces
      double g = 9.81;               // Gravity
      double Fr;                     // Radial force revolving with journal

// State variables
      fmatvec::Vec2 IxS1 = "[0; 0]";
// Position of journal center point
      fmatvec::Vec2 IxS2 = "[0; 0]";          // Position of bearing shell center point
      fmatvec::Vec2 IuS1 = "[0; 0]";          // Velocity of journal center point
      fmatvec::Vec2 IuS2 = "[0; 0]";          // Velocity of bearing shell center point
      double phi1 = 0;          // Angle of journal
      double phi2 = 0;          // Angle of bearing shell
      double omega1 = 0;          // Angular velocity of journal
      double omega2 = 0;          // Angular velocity of bearing shell

// Dimensionless description
      bool dimLess;          // Flag for dimensionless description
      double xrF;          // Characteristic size for fluid domain
      double hrF;          // Characteristic size for film thickness
  };

//        function [h, D] = ForceVector(sys, msh, lub, ~, z, D0)
//            // Force vector of multi body system with fixed bearing shell
//            //
//            // Input:
//            //   sys:    Object of system
//            //   msh:    Object of mesh
//            //   lub:    Object of lubricant
//            //   t(~):   Current time
//            //   z:      Current state [xS1; yS1; phi; uS1; vS1; omega1]
//            //   D0:     Start point for solvers
//            //
//            // Output:
//            //   h:      Force vector of multi body system
//            //   D:      Vector with nodal solution for pressure
//
//            // Update time dependent state variables
//            sys.IxS1 = z(1:1:2);
//            sys.phi1 = z(3);
//            sys.IuS1 = z(4:1:5);
//            sys.omega1 = z(6);
//
//            // Compute pressure
//            D = NewtonJosephy(D0, sys, msh, lub, [], [], false, false);
////             D = ProxFormulation(D0, sys, msh, lub, [], [], [], [], false, false);
//
//            // Compute resultant fluid force on journal
//            // Note: IF1 = KF1, because of fixed bearing shell (phi2 = 0)
//            [KF1, ~] = ResultantFluidForce(D, sys, msh);
//            if sys.dimLess
//                detJr = sys.xrF^2;
//                KF1 = KF1 * lub.pr * detJr;
//            end
//
//            // Build force vector of hydrodynamic contact taking into
//            // account symmetry of problem
//            hF1 = 2 * [KF1(1:1:2); 0];
//
//            // Build force vector of gravitational and radial force
//            hS1 = sys.m * [0; -sys.g; 0] ...
//                + sys.Fr * [cos(sys.phi1); sin(sys.phi1); 0];
//
//            // Test equilibrium
//            // hS1 = [0; -sys.Fr; 0];
//
//            // Finally compute force vector of multi body system with
//            // hydrodynamic contact
//            h = hF1 + hS1;
//        end
//
//        function In2 = SurfaceNormal(sys, x)
//            // Surface normal vector of inner bearing shell surface
//            //
//            // Input:
//            //   sys:    Object of system
//            //   x:      Point inside fluid domain y or [y; z]
//            //
//            // Output:
//            //   In2:    Normal vector of inner bearing shell surface
//
//            y = x(1);
//            if sys.dimLess
//                y = y * sys.xrF;
//            end
//
//            phi = AngleCoordSys(sys, y);
//            In2 = -[cos(phi); sin(phi); 0];
//        end
//

//

//

}
#endif
