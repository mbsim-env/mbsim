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
#include "mbsim/links/initial_condition.h"
#include "mbsim/objects/object.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, InitialCondition)

  InitialCondition::InitialCondition(const string &name) : Link(name) {
    W[0].resize(1);
    W[1].resize(1);
  }

  void InitialCondition::calcSize() {
    ng = indices.size()?indices.size():object->getGeneralizedPositionSize();
    ngd = ng;
    nla = ngd;
    updSize = false;
  }

  void InitialCondition::calclaSize(int j) {
    laSize = active*nla;
  }

  void InitialCondition::calcgSize(int j) {
    gSize = active*ng;
  }

  void InitialCondition::calccorrSize(int j) {
    corrSize = active*nla;
  }

  void InitialCondition::calcgdSize(int j) {
    gdSize = active*ngd;
  }

  void InitialCondition::updateGeneralizedPositions() {
    for(int i=0; i<ng; i++)
      rrel(i)=object->evalGeneralizedPosition()(indices[i])-q0(i);
    updrrel = false;
  } 

  void InitialCondition::updateGeneralizedVelocities() {
    for(int i=0; i<ng; i++)
      vrel(i)=object->evalGeneralizedVelocity()(indices[i])-u0(i);
    updvrel = false;
  }

  void InitialCondition::updateg() {
    g = evalGeneralizedRelativePosition();
  }

  void InitialCondition::updategd() {
    gd = evalGeneralizedRelativeVelocity();
  }

  void InitialCondition::updateW(int j) {
    W[j][0] += JRel.T();
  } 
 
  void InitialCondition::updateWRef(Mat &WParent, int j) {
    RangeV I = RangeV(object->gethInd(j),object->gethInd(j)+object->gethSize(j)-1);
    RangeV J = RangeV(laInd,laInd+laSize-1);
    W[j][0].ref(WParent,I,J);
  } 

  void InitialCondition::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if(not objectString.empty())
	setObject(getByPath<Object>(objectString));
      if(not object)
	throwError("(InitialCondition::init): object is not given.");
    }
    else if(stage==unknownStage) {
      int qSize = object->getGeneralizedPositionSize();
      int uSize = object->getGeneralizedVelocitySize();
      if(indices.size()==0) {
	indices.resize(qSize);
	for(size_t i=0; i<indices.size(); i++)
	  indices[i] = i;
      }
      else if(int(indices.size())>qSize)
	throwError("(InitialCondition::init): size of indices does not match, must be " + to_string(qSize) + ", but is " + to_string(indices.size()) + ".");
      if(not q0())
	q0.resize(indices.size());
      else if(q0.size()!=int(indices.size()))
	throwError("(InitialCondition::init): size of q0 does not match, must be " + to_string(qSize) + ", but is " + to_string(q0.size()) + ".");
      if(not u0())
	u0.resize(indices.size());
      else if(u0.size()!=int(indices.size()))
	throwError("(InitialCondition::init): size of u0 does not match, must be " + to_string(uSize) + ", but is " + to_string(u0.size()) + ".");
      JRel.resize(indices.size(),object->gethSize(0));
      for(size_t i=0; i<indices.size(); i++)
	JRel(i,object->gethSize(0)-uSize+indices[i]) = 1;
      for(size_t i=0; i<indices.size(); i++)
    }
    Link::init(stage, config);
  }

  void InitialCondition::initializeUsingXML(xercesc::DOMElement * element) {
    Link::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"object");
    objectString=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIM%"constrainedDegreesOfFreedom");
    if(e) {
      auto indices_ = E(e)->getText<VecVI>();
      indices.resize(indices_.size());
      for(size_t i=0; i<indices.size(); i++)
	indices[i] = static_cast<Index>(indices_(i))-1;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedInitialPosition");
    if(e) setGeneralizedInitialPosition(E(e)->getText<VecV>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedInitialVelocity");
    if(e) setGeneralizedInitialVelocity(E(e)->getText<VecV>());
  }

}
