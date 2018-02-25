/* Copyright (C) 2004-2017 MBSim Development Team
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
#include "mbsim/links/generalized_elastic_structure.h"
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, GeneralizedElasticStructure)

  GeneralizedElasticStructure::~GeneralizedElasticStructure() {
    delete func;
  }

  void GeneralizedElasticStructure::updateGeneralizedPositions() {
    for(unsigned i=0; i<body.size(); i++)
      rrel.set(ila[i],body[i]->evalGeneralizedPosition());
    updrrel = false;
  }

  void GeneralizedElasticStructure::updateGeneralizedVelocities() {
    for(unsigned i=0; i<body.size(); i++)
      vrel.set(ila[i],body[i]->evalGeneralizedVelocity());
    updvrel = false;
  }

  void GeneralizedElasticStructure::updateGeneralizedForces() {
    lambda = -(*func)(evalGeneralizedRelativePosition(),evalGeneralizedRelativeVelocity());
    updla = false;
  }
  
  void GeneralizedElasticStructure::updateForce() {
    for(unsigned i=0; i<body.size(); i++)
      F[i] = evalGlobalForceDirection(i)*evalGeneralizedForce()(ila[i]);
    updF = false;
  }

  void GeneralizedElasticStructure::updateMoment() {
    for(unsigned i=0; i<body.size(); i++)
      M[i] = evalGlobalMomentDirection(i)*evalGeneralizedForce()(ila[i]);
    updM = false;
  }

  void GeneralizedElasticStructure::updateR() {
    for(unsigned i=0; i<body.size(); i++) {
      RF[i].set(RangeV(0,2), ila[i], evalGlobalForceDirection(i));
      RM[i].set(RangeV(0,2), ila[i], evalGlobalMomentDirection(i));
    }
    updRMV = false;
  }

  void GeneralizedElasticStructure::updateh(int j) {
    if(j==0) {
      for(unsigned i=0; i<body.size(); i++)
        h[j][i]+=body[i]->evalJRel(j).T()*evalGeneralizedForce()(ila[i]);
    } else {
      for(unsigned i=0; i<body.size(); i++) {
        h[j][i]+=body[i]->getFrameForKinematics()->evalJacobianOfTranslation(j).T()*evalForce(i)  + body[i]->getFrameForKinematics()->evalJacobianOfRotation(j).T()*evalMoment(i);
        h[j][body.size()+i]-=C[i].evalJacobianOfTranslation(j).T()*evalForce(i) + C[i].evalJacobianOfRotation(j).T()*evalMoment(i);
      }
    }
  }

  void GeneralizedElasticStructure::calcSize() {
    ng = 0;
    ngd = 0;
    for(size_t i=0; i<body.size(); i++) {
      ng += body[i]->getGeneralizedPositionSize();
      ngd += body[i]->getGeneralizedVelocitySize();
    }
    nla = ngd;
    updSize = false;
  }

  void GeneralizedElasticStructure::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      RigidBodyLink::init(stage, config);
      for(const auto & i : saved_body)
        body.push_back(getByPath<RigidBody>(i));
      if(body.size()==0)
        throwError("No rigid bodies given!");
    }
    else if(stage==preInit) {
      RigidBodyLink::init(stage, config);
      ila.resize(body.size());
      DF.resize(body.size());
      DM.resize(body.size());
      ratio.resize(body.size(),1);
      int k = 0;
      for(size_t i=0; i<body.size(); i++) {
        ila[i] = RangeV(k, k+body[i]->getGeneralizedVelocitySize()-1);
        DF[i].resize(body[i]->getGeneralizedVelocitySize());
        DM[i].resize(body[i]->getGeneralizedVelocitySize());
        k += body[i]->getGeneralizedVelocitySize();
      }
    }
    else
      RigidBodyLink::init(stage, config);
    func->init(stage, config);
  }

  void GeneralizedElasticStructure::initializeUsingXML(DOMElement* element) {
    RigidBodyLink::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"rigidBody");
    while(e && E(e)->getTagName()==MBSIM%"rigidBody") {
      saved_body.push_back(E(e)->getAttribute("ref"));
      e=e->getNextElementSibling();
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceFunction");
    auto *f=ObjectFactory::createAndInit<Function<VecV(VecV,VecV)> >(e->getFirstElementChild());
    setGeneralizedForceFunction(f);
  }

}
