/* Copyright (C) 2004-2012 MBSim Development Team
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

#include <config.h>
#include "isotropic_rotational_spring_damper.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/eps.h"
#include "mbsim/objectfactory.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, IsotropicRotationalSpringDamper)

  IsotropicRotationalSpringDamper::IsotropicRotationalSpringDamper(const string &name) : FixedFrameLink(name), funcE(NULL), funcD(NULL) {
    nM = 2;
  }

  IsotropicRotationalSpringDamper::~IsotropicRotationalSpringDamper() {
    delete funcE;
    delete funcD;
  }

  void IsotropicRotationalSpringDamper::updateGeneralizedPositions() {
    const SqrMat3 &A = evalGlobalRelativeOrientation();
    rrel(0) = acos(std::max(-1.,std::min(1.,(A(0,0)+A(1,1)+A(2,2)-1)/2.)));
    updrrel = false;
  }

  void IsotropicRotationalSpringDamper::updateGeneralizedVelocities() {
    vrel(0) = evalGlobalMomentDirection().col(1).T() * evalGlobalRelativeAngularVelocity();
    updvrel = false;
  }

  void IsotropicRotationalSpringDamper::updateForceDirections() {
    double al = evalGeneralizedRelativePosition()(0);
    const SqrMat3 &A = getGlobalRelativeOrientation();
    if(fabs(al)<=1e-13)
      n.init(0);
    else {
      n(0) = (A(2,1)-A(1,2))/2./sin(al);
      n(1) = (A(0,2)-A(2,0))/2./sin(al);
      n(2) = (A(1,0)-A(0,1))/2./sin(al);
    }
    DM.set(0, frame[0]->getOrientation() * n);
    n = evalGlobalRelativeAngularVelocity();
    double nrmn = nrm2(n);
    if(nrmn<=1e-13)
      n.init(0);
    else
      n /= nrmn;
    DM.set(1, n);
    updDF = false;
  }

  void IsotropicRotationalSpringDamper::updatelaM() {
    lambdaM(0) = -(*funcE)(evalGeneralizedRelativePosition()(0));
    lambdaM(1) = -(*funcD)(evalGeneralizedRelativeVelocity()(0));
    updlaM = false;
  }

  void IsotropicRotationalSpringDamper::init(InitStage stage, const InitConfigSet &config) {
    FixedFrameLink::init(stage, config);
    funcE->init(stage, config);
    funcD->init(stage, config);
  }

  void IsotropicRotationalSpringDamper::initializeUsingXML(DOMElement *element) {
    FixedFrameLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"elasticMomentFunction");
    Function<double(double)> *f=ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild());
    setElasticMomentFunction(f);
    e=E(element)->getFirstElementChildNamed(MBSIM%"dissipativeMomentFunction");
    f=ObjectFactory::createAndInit<Function<double(double)> >(e->getFirstElementChild());
    setDissipativeMomentFunction(f);
  }

}
