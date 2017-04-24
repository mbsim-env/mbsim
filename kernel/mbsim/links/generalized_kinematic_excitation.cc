/* Copyright (C) 2004-2015 MBSim Development Team
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
#include "mbsim/links/generalized_kinematic_excitation.h"
#include <mbsim/constitutive_laws/generalized_force_law.h>
#include <mbsim/constitutive_laws/bilateral_impact.h>
#include "mbsim/objects/rigid_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  GeneralizedKinematicExcitation::GeneralizedKinematicExcitation(const string &name) : DualRigidBodyLink(name), fl(NULL), il(NULL) {
    body.resize(1);
    ratio.resize(1);
    ratio[0] = 1;
  }

  GeneralizedKinematicExcitation::~GeneralizedKinematicExcitation() {
    delete fl;
    if(il) delete il;
  }

  bool GeneralizedKinematicExcitation::isSetValued() const {
    return fl->isSetValued();
  }

  void GeneralizedKinematicExcitation::setGeneralizedForceLaw(GeneralizedForceLaw * fl_) {
    fl=fl_;
    fl->setParent(this);
  }

  void GeneralizedKinematicExcitation::calclaSize(int j) {
    laSize = body[0]->getGeneralizedVelocitySize();
  }
  void GeneralizedKinematicExcitation::calcgSize(int j) {
    gSize = body[0]->getGeneralizedVelocitySize();
  }
  void GeneralizedKinematicExcitation::calcgdSize(int j) {
    gdSize = body[0]->getGeneralizedVelocitySize();
  }

  void GeneralizedKinematicExcitation::updateGeneralizedForces() {
    if(isSetValued())
      lambda = la;
    else
      for(int i=0; i<lambda.size(); i++)
        lambda(i) = (*fl)(evalGeneralizedRelativePosition()(i),evalGeneralizedRelativeVelocity()(i));
    updla = false;
  }

  void GeneralizedKinematicExcitation::init(InitStage stage) {
    if(stage==unknownStage) {
      if(fl->isSetValued()) {
        il = new BilateralImpact;
        il->setParent(this);
      }
    }
    RigidBodyLink::init(stage);
    if(fl) fl->init(stage);
    if(il) il->init(stage);
  }

}
