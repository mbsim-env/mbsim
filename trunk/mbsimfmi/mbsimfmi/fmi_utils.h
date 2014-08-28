/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

/**
 * \author Fabien Pean
 */

#ifndef FMIUTILS_H_
#define FMIUTILS_H_

#include <boost/filesystem.hpp>
#include <sstream>
#include <string>

namespace MBSim {
  class DynamicSystemSolver;
}
namespace MBSimIntegrator {
  class Integrator;
}
namespace fmi {
boost::filesystem::path getSharedLibDir();

void initDssWithXml(std::string xmlpath, MBSim::DynamicSystemSolver **system, MBSimIntegrator::Integrator **integrator=NULL);
void initIntegratorWithXml(std::string xmlpath, MBSimIntegrator::Integrator **integrator);

const std::string getCurrentDateTime();

template <typename T>
std::string NumberToString ( T Number ) {
   std::ostringstream ss;
   ss << Number;
   return ss.str();
}

}


#endif /* FMIUTILS_H_ */
