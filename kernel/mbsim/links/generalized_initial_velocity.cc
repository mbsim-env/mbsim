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
#include "mbsim/links/generalized_initial_velocity.h"
#include "mbsim/objects/object.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedInitialVelocity)

  GeneralizedInitialVelocity::GeneralizedInitialVelocity(const string &name) : InitialCondition(name) {
    W[0].resize(1);
    W[1].resize(1);
  }

  void GeneralizedInitialVelocity::calcSize() {
    ng = 0;
    ngd = indices.size()?indices.size():object->getGeneralizedPositionSize();
    nla = ngd;
    updSize = false;
  }

  void GeneralizedInitialVelocity::calclaSize(int j) {
    if(j==5)
      laSize = 0;
    else
      laSize = active*nla;
  }

  void GeneralizedInitialVelocity::calccorrSize(int j) {
    if(j==2)
      corrSize = 0;
    else
      corrSize = active*nla;
  }

  void GeneralizedInitialVelocity::updateGeneralizedVelocities() {
    for(int i=0; i<ngd; i++)
      vrel(i)=object->evalGeneralizedVelocity()(indices[i])-u0(i);
    updvrel = false;
  }

  void GeneralizedInitialVelocity::updategd() {
    gd = evalGeneralizedRelativeVelocity();
  }

  void GeneralizedInitialVelocity::updateW(int j) {
    if(laSize) W[j][0] += JRel.T();
  } 
 
  void GeneralizedInitialVelocity::updateWRef(Mat &WParent, int j) {
    RangeV I = RangeV(object->gethInd(j),object->gethInd(j)+object->gethSize(j)-1);
    RangeV J = RangeV(laInd,laInd+laSize-1);
    W[j][0].ref(WParent,I,J);
  } 

  void GeneralizedInitialVelocity::init(InitStage stage, const InitConfigSet &config) {
    if (stage==resolveStringRef) {
      if(not objectString.empty())
	setObject(getByPath<Object>(objectString));
      if(not object)
	throwError("(GeneralizedInitialVelocity::init): object is not given.");
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
	throwError("(GeneralizedInitialVelocity::init): size of indices does not match, must be " + to_string(qSize) + ", but is " + to_string(indices.size()) + ".");
      if(not u0())
	u0.resize(indices.size());
      else if(u0.size()!=int(indices.size()))
	throwError("(GeneralizedInitialVelocity::init): size of values does not match, must be " + to_string(uSize) + ", but is " + to_string(u0.size()) + ".");
      JRel.resize(indices.size(),object->gethSize(0));
      for(size_t i=0; i<indices.size(); i++)
	JRel(i,object->gethSize(0)-uSize+indices[i]) = 1;
    }
    InitialCondition::init(stage, config);
  }

  void GeneralizedInitialVelocity::initializeUsingXML(xercesc::DOMElement * element) {
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
