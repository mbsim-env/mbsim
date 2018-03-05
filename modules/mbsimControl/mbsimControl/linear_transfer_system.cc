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
    if(e) A = E(e)->getText<SqrMat>();
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputMatrix");
    if(e) B = E(e)->getText<Mat>(A.rows(), 0);
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"outputMatrix");
    if(e) C = E(e)->getText<Mat>(0, A.cols());
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"feedthroughMatrix");
    if(e) D = E(e)->getText<SqrMat>();
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
      if(inputSignalString!="")
        setInputSignal(getByPath<Signal>(inputSignalString));
      if(not inputSignal)
        throwError("(LinearTransferSystem::init): input signal must be given");
    }
    else if(stage==preInit) {
      if(not A.size()) {
        B.resize(0,getSignalSize());
        C.resize(B.cols(),0);
      }
      else
      {
        if(B.rows() != A.size())
          throwError("Number of rows of input matrix must be equal to size of state matrix");
        if(B.cols() != getSignalSize())
          throwError("Number of columns of input matrix must be equal to signal size");
        if(C.rows() != B.cols())
          throwError("Number of rows of output matrix must be equal to signal size");
        if(C.cols() != A.size())
          throwError("Number of columns of output matrix must be equal size of state matrix");
      }
      if(not D.size())
        D.resize(getSignalSize());
      else {
        if(D.size() != C.rows())
          throwError("Size of feedthrough matrix must be equal to signal size");
      }
    }
    Signal::init(stage, config);
  }

}
