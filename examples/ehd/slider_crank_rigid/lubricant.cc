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

#include "lubricant.h"

namespace MBsimEHD {
  methods
  function lub = Lubricant(eta0, rho0, alpha, Z, viscModel, densModel, pr)
  // Constructur for lubricant
  //
  // Input:
  //   Fluid properties (for description see above)
  //
  // Output:
  //   lub:    Object of properties

  lub.eta0 = eta0;lub.rho0 = rho0;

  // Check if both parameters alpha and Z were defined
  if ~isempty(alpha) && ~isempty(Z)
  error(['Both parameters alpha and Z were defined. ', ...
      'Define only one of them.']);end

  // Compute alpha out of Z or vice versa
  if isempty(alpha)
  lub.Z = Z;lub.alpha = lub.Z / lub.cp * log(lub.eta0 / lub.etaInf);
  elseif isempty(Z) lub.alpha = alpha;lub.Z = lub.alpha * lub.cp / log(lub.eta0 / lub.etaInf);end

  // Save used models for pressure dependence
  lub.viscModel = viscModel;lub.densModel = densModel;

  // Save reference pressure if a dimensionless description is
  // used
  if nargin == 7
  lub.pr = pr;elseif ~isempty(strfind(viscModel, 'DimLess')) || ...
      ~isempty(strfind(densModel, 'DimLess'))
          error(['No reference pressure for dimensionless ', ...
              'description defined.']);
          end end

          function [eta, etadp, etadpdp] = DynViscosity(lub, p)

          p = max(p, 0);

          switch lub.viscModel
          case 'Barus'
      // Barus formula, see Hamrock eq. (4.7)
          eta = lub.eta0 * exp(lub.alpha * p);etadp = lub.eta0 * lub.alpha * exp(lub.alpha * p);etadpdp = lub.eta0 * lub.alpha^2 * exp(lub.alpha * p);

          case 'Roelands'
   // Roelands formula, see Hamrock eq. (4.10)
                             // Note: log = loge = ln (natural logarithm)
          eta = lub.eta0 * (lub.etaInf / lub.eta0) ...
          ^ (1 - (1 + p / lub.cp)^lub.Z);etadp = -eta * lub.Z / lub.cp ...
          * log(lub.etaInf / lub.eta0) ...
          * (1 + p / lub.cp)^(lub.Z - 1);etadpdp = -etadp / lub.cp / (1 + p / lub.cp) ...
          * (lub.Z * log(lub.etaInf / lub.eta0) ...
              * (1 + p / lub.cp)^lub.Z - (lub.Z - 1));

          case 'constant'
// Constant viscosity
          eta = lub.eta0;etadp = 0;etadpdp = 0;

          case 'BarusDimLess'
// Dimensionless Barus formula
          alphadl = lub.alpha * lub.pr;eta = exp(alphadl * p);etadp = alphadl * exp(alphadl * p);etadpdp = alphadl^2 * exp(alphadl * p);

          case 'RoelandsDimLess'
// Dimensionless Roelands formula
          cpdl = lub.cp / lub.pr;eta = (lub.etaInf / lub.eta0) ...
          ^ (1 - (1 + p / cpdl)^lub.Z);etadp = -eta * lub.Z / cpdl ...
          * log(lub.etaInf / lub.eta0) ...
          * (1 + p / cpdl)^(lub.Z - 1);etadpdp = -etadp / cpdl / (1 + p / cpdl) ...
          * (lub.Z * log(lub.etaInf / lub.eta0) ...
              * (1 + p / cpdl)^lub.Z - (lub.Z - 1));

          case 'constantDimLess'
// Constant viscosity
          eta = 1;etadp = 0;etadpdp = 0;
          end end

          function [rho, rhodp, rhodpdp] = Density(lub, p)

          p = max(p, 0);

          switch lub.densModel
          case 'DowsonHigginson'
        // Dowson and Higginson formula, see Hamrock eq. (4.19)
          rho = lub.rho0 * (1 + lub.a * p / (1 + lub.b * p));rhodp = lub.rho0 * lub.a / (1 + lub.b * p)^2;rhodpdp = -2 * lub.rho0 * lub.a * lub.b ...
          / (1 + lub.b * p)^3;

          case 'constant'
               // Constant density
          rho = lub.rho0;rhodp = 0;rhodpdp = 0;

          case 'DowsonHigginsonDimLess'
 // Dimensionless Dowson and Higginson formula
          adl = lub.a * lub.pr;bdl = lub.b * lub.pr;rho = 1 + adl * p / (1 + bdl * p);rhodp = adl / (1 + bdl * p)^2;rhodpdp = -2 * adl * bdl / (1 + bdl * p)^3;

          case 'constantDimLess'
        // Constant density
          rho = 1;rhodp = 0;rhodpdp = 0;
          end endend
        }
      }
