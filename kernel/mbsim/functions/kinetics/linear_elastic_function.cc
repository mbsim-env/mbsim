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
#include "mbsim/functions/kinetics/linear_elastic_function.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, LinearElasticFunction)

  void LinearElasticFunction::init(InitStage stage, const InitConfigSet &config) {
    if(stage==unknownStage) {
      if(not D.size())
        D.resize(K.size());
      else if(D.size() != K.size())
        throwError("Size of damping matrix does not match!");
    }
    Function<VecV(VecV,VecV)>::init(stage, config);
  }

  void LinearElasticFunction::initializeUsingXML(DOMElement *element) {
    Function<VecV(VecV,VecV)>::initializeUsingXML(element);
    DOMElement *e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"stiffnessMatrix");
    K = E(e)->getText<SymMat>();
    e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"dampingMatrix");
    if(e) D = E(e)->getText<SymMat>();
  }

}
