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

/*!
 * \brief Lubricant with pressure dependent properties
 * \author Michael Hofer, 09.01.2015
 * \author Kilian Grundl (copied to c++ 19.06.2015)
 */

namespace MBSimEHD {

  class Lubricant {
    public:
      enum ViscosityPressureModel {
        constVisc, constViscDimLess, Roelands, RoelandsDimLess, Barus, BarusDimLess
      };
      enum DensityPressureModel {
        constDen, constDenDimLess, DowsonHigginson, DowsonHigginsonDimLess
      };
      /*!
       * \brief standard constructor
       */
      Lubricant();

      /*!
       * \brief constructor to initialize all values
       */
      Lubricant(const double & eta0, const double & rho0, const double & alphaOrZ, const bool & isAlpha, const ViscosityPressureModel & viscModel, const DensityPressureModel & densModel, const double & pr = 0);

      /* Isothermal viscosity-pressure dependence
       *
       * Input:
       *   lub:        Object of lubricant
       *   p:          Pressure
       *
       * Output:
       *   eta:        Dynamic viscosity evaluated at p
       *   etadp:      First derivative of eta evaluated at p
       *   etadpdp:    Second derivative of eta evaluated at p
       *
       * Note:
       *   For dimensionless models the values for p, eta, etadp and
       *   etadpdp are dimensionless numbers
       */
      void DynViscosity(const double & p, double & eta, double & etadp, double & etadpdp);

      /*!
       * Isothermal density-pressure dependence
       *
       * Input:
       *   lub:        Object of lubricant
       *   p:          Pressure
       *
       * Output:
       *   rho:        Density evaluated at p
       *   rhodp:      First derivative of rho evaluated at p
       *   rhodpdp:    Second derivative of rho evaluated at p
       *
       * Note:
       *   For dimensionless models the values for p, eta, etadp and
       *   etadpdp are dimensionless numbers
       */
      void Density(const double & p, double & rho, double & rhodp, double & rhodpdp);

    private:
      double eta0;           // Dynamic viscosity at p = 0 and at a constant
      // temperature, in Ns/m^2

      double rho0;           // Density at p = 0, in kg/m^3

      ViscosityPressureModel viscModel;      // Model for viscosity-pressure dependence

      DensityPressureModel densModel;      // Model for density-pressure dependence

      double alpha;          // Pressure-viscosity coefficient for Barus formula
      // dependent on temperature, in m^2/N

      double Z;              // Viscosity-pressure index for Roelands formula,
      // dimensionless constant

      double pr;             // Reference pressure for dimensionless description

      // Constants for Roelands formula, in Ns/m^2 and N/m^2
      const double etaInf = 6.31e-5;
      const double cp = 1.96e8;

      // Constants for Dowson Higginson formula, in m^2/N
      const double a = 0.6e-9;
      const double b = 1.7e-9;

  };
}
