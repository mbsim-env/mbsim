/* Copyright (C) 2004-2009 MBSim Development Team
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
#include <iostream>
#include "mbsimControl/linear_transfer_system.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMCONTROL, LinearTransferSystem)

  void LinearTransferSystem::initializeUsingXML(DOMElement * element) {
    Signal::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    inputSignalString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"systemMatrix");
    setSystemMatrix(E(e)->getText<SqrMatV>());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputMatrix");
    setInputMatrix(E(e)->getText<MatV>(A.rows(), 0));
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"outputMatrix");
    if(e) setOutputMatrix(E(e)->getText<MatV>(0, A.cols()));
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"feedthroughMatrix");
    if(e) setFeedthroughMatrix(E(e)->getText<MatV>());
  }

  void LinearTransferSystem::updateSignal() {
    s = C*x + D*inputSignal->evalSignal();
    upds = false;
  }

  void LinearTransferSystem::updatexd() {
    xd = A*x + B*inputSignal->evalSignal();
  }

  void LinearTransferSystem::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not inputSignalString.empty())
        setInputSignal(getByPath<Signal>(inputSignalString));
      if(not inputSignal)
        throwError("(LinearTransferSystem::init): input signal must be given");
    }
    else if(stage==preInit) {
      if(not A.size())
        throwError("Size of system matrix must be at least 1");
      if(not C())
        C.resize(inputSignal->getSignalSize(),A.size(),Eye());
      if(not D())
        D.resize(C.rows(),inputSignal->getSignalSize());
      if(A.size() != C.cols())
        throwError("Size of system matrix must be equal to number of columns of output matrix");
      if(B.rows() != C.cols())
        throwError("Number of rows of input matrix must be equal to number of columns of output matrix");
      if(B.cols() != inputSignal->getSignalSize())
        throwError("Number of columns of input matrix must be equal to input signal size");
      if(D.rows() != C.rows())
        throwError("Number of rows of feedthrough matrix must be equal to number of rows of output matrix");
      if(D.cols() != inputSignal->getSignalSize())
        throwError("Number of columns of feedthrough matrix must be equal to input signal size");
    }
    Signal::init(stage, config);
  }

}
