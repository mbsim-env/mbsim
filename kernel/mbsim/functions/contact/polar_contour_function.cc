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

#include <config.h>
#include "polar_contour_function.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(PolarContourFunction, MBSIM%"PolarContourFunction")

  PolarContourFunction::PolarContourFunction() : r(0), alphaSave(-1e12), salphaSave(0.), calphaSave(0.), rSave(0.), drdalphaSave(0.), d2rdalpha2Save(0.) {
  }

  PolarContourFunction::~PolarContourFunction() {
    delete r;
  }

  void PolarContourFunction::setRadiusFunction(Function<double(double)> *r_) {
    r = r_;
    r->setParent(this);
    r->setName("r");
  }

  void PolarContourFunction::init(Element::InitStage stage) {
    Function<Vec3(double)>::init(stage);
    r->init(stage);
  }

  void PolarContourFunction::initializeUsingXML(xercesc::DOMElement *element) {
    xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"radiusFunction");
    setRadiusFunction(ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild()));
  }

  Vec3 PolarContourFunction::operator()(const double& alpha) {
    updateData(alpha);
    Vec3 f(NONINIT);
    f(2) = 0;
    f(0) = rSave*calphaSave;
    f(1) = rSave*salphaSave;
    return f;
  } 

  Vec3 PolarContourFunction::parDer(const double& alpha) {
    updateData(alpha);
    Vec3 f(NONINIT);
    f(2) = 0;
    f(0) = drdalphaSave*calphaSave-rSave*salphaSave;
    f(1) = drdalphaSave*salphaSave+rSave*calphaSave;
    return f;
  }

  Vec3 PolarContourFunction::parDerParDer(const double& alpha) {
    updateData(alpha);
    const double s1=-rSave+d2rdalpha2Save;
    const double s2=2.*drdalphaSave;
    Vec3 f(NONINIT);
    f(2) = 0;
    f(0) = s1*calphaSave-s2*salphaSave;
    f(1) = s1*salphaSave+s2*calphaSave;
    return f;
  }

  void PolarContourFunction::updateData(const double& alpha) {
    if (fabs(alpha-alphaSave)>macheps()) {
      alphaSave = alpha;
      salphaSave = sin(alphaSave);
      calphaSave = cos(alphaSave);
      rSave = (*r)(alphaSave);
      drdalphaSave = r->parDer(alphaSave);
      d2rdalpha2Save = r->parDerParDer(alphaSave);
    }

  }

}
