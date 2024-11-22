/* Copyright (C) 2004-2024 MBSim Development Team
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
#include "mbsim/links/generalized_initial_position.h"
#include "mbsim/objects/object.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedInitialPosition)

  GeneralizedInitialPosition::GeneralizedInitialPosition(const string &name) : InitialCondition(name) {
    W[0].resize(1);
    W[1].resize(1);
  }

  void GeneralizedInitialPosition::calcSize() {
    ng = indices.size()?indices.size():object->getGeneralizedPositionSize();
    ngd = 0;
    nla = ng;
    updSize = false;
  }

  void GeneralizedInitialPosition::calclaSize(int j) {
    if(j==3)
      laSize = 0;
    else
      laSize = active*nla;
  }

  void GeneralizedInitialPosition::calccorrSize(int j) {
    if(j==4)
      corrSize = 0;
    else
      corrSize = active*nla;
  }

  void GeneralizedInitialPosition::updateGeneralizedPositions() {
    for(int i=0; i<ng; i++)
      rrel(i)=object->evalGeneralizedPosition()(indices[i])-q0(i);
    updrrel = false;
  } 

  void GeneralizedInitialPosition::updateg() {
    g = evalGeneralizedRelativePosition();
  }

  void GeneralizedInitialPosition::updateW(int j) {
    if(laSize) W[j][0] += JRel.T();
  } 
 
  void GeneralizedInitialPosition::updateWRef(Mat &WParent, int j) {
    RangeV I = RangeV(object->gethInd(j),object->gethInd(j)+object->gethSize(j)-1);
    RangeV J = RangeV(laInd,laInd+laSize-1);
    W[j][0].ref(WParent,I,J);
  } 

  void GeneralizedInitialPosition::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if(not objectString.empty())
	setObject(getByPath<Object>(objectString));
      if(not object)
	throwError("(GeneralizedInitialPosition::init): object is not given.");
    }
    else if(stage==unknownStage) {
      int qSize = object->getGeneralizedPositionSize();
      if(indices.size()==0) {
	indices.resize(qSize);
	for(size_t i=0; i<indices.size(); i++)
	  indices[i] = i;
      }
      else if(int(indices.size())>qSize)
	throwError("(GeneralizedInitialPosition::init): size of indices does not match, must be " + to_string(qSize) + ", but is " + to_string(indices.size()) + ".");
      if(not q0())
	q0.resize(indices.size());
      else if(q0.size()!=int(indices.size()))
	throwError("(GeneralizedInitialPosition::init): size of values does not match, must be " + to_string(qSize) + ", but is " + to_string(q0.size()) + ".");
      JRel.resize(indices.size(),object->gethSize(0));
      for(size_t i=0; i<indices.size(); i++)
	JRel(i,object->gethSize(0)-qSize+indices[i]) = 1;
    }
    InitialCondition::init(stage, config);
  }

  void GeneralizedInitialPosition::initializeUsingXML(xercesc::DOMElement * element) {
    InitialCondition::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"object");
    objectString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"indices");
    if(e) {
      auto indices_ = E(e)->getText<VecVI>();
      indices.resize(indices_.size());
      for(size_t i=0; i<indices.size(); i++)
	indices[i] = static_cast<Index>(indices_(i))-1;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"values");
    setValues(E(e)->getText<VecV>());
  }

}
