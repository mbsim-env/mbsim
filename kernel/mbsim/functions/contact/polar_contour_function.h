/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef POLAR_CONTOUR_FUNCTION_H
#define POLAR_CONTOUR_FUNCTION_H

#include <mbsim/functions/function.h>

namespace MBSim {

  class PolarContourFunction : public MBSim::Function<fmatvec::Vec3(double)> {
    public:
      PolarContourFunction();
      ~PolarContourFunction();

      void setRadiusFunction(Function<double(double)> *f_);
      void init(Element::InitStage stage);
      void initializeUsingXML(xercesc::DOMElement *element);

      fmatvec::Vec3 operator()(const double& alpha);
      fmatvec::Vec3 parDer(const double& alpha);
      fmatvec::Vec3 parDerParDer(const double& alpha);

    private:
      MBSim::Function<double(double)> *r;

      double alphaSave, salphaSave, calphaSave, rSave, drdalphaSave, d2rdalpha2Save;

      void updateData(const double& alpha);
  };

}

#endif
