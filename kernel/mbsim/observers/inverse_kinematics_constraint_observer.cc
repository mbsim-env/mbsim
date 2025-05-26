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
#include "mbsim/observers/inverse_kinematics_constraint_observer.h"
#include "mbsim/constraints/inverse_kinematics_constraint.h"
#include "mbsim/links/mechanical_link.h"
#include "mbsim/frames/frame.h"
#include "mbsim/dynamic_system_solver.h"
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, InverseKinematicsConstraintObserver)

  InverseKinematicsConstraintObserver::InverseKinematicsConstraintObserver(const std::string &name) : Observer(name), constraint(nullptr) {
    evalOMBVForceColorRepresentation[0] = &InverseKinematicsConstraintObserver::evalNone;
    evalOMBVForceColorRepresentation[1] = &InverseKinematicsConstraintObserver::evalAbsoluteForce;
    evalOMBVMomentColorRepresentation[0] = &InverseKinematicsConstraintObserver::evalNone;
    evalOMBVMomentColorRepresentation[1] = &InverseKinematicsConstraintObserver::evalAbsoluteMoment;
  }

  void InverseKinematicsConstraintObserver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(not saved_constraint.empty())
        setInverseKinematicsConstraint(getByPath<InverseKinematicsConstraint>(saved_constraint));
      if(not constraint)
        throwError("Inverse kinematics constraint is not given!");
      if(not saved_outputFrame.empty())
        setOutputFrame(getByPath<Frame>(saved_outputFrame));
      if(not outputFrame)
        setOutputFrame(ds->getFrameI());
      Observer::init(stage, config);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive]) {
        if(plotFeature[force]) {
          for(int i=0; i<constraint->getNumberOfMechanicalLinks(); i++)
            addToPlot("force "+to_string(i),{"x","y","z"});
        }
        if(plotFeature[moment]) {
          for(int i=0; i<constraint->getNumberOfMechanicalLinks(); i++)
            addToPlot("moment "+to_string(i),{"x","y","z"});
        }
      }
      Observer::init(stage, config);
      if(plotFeature[openMBV]) {
        if(ombv) {
          openMBVForce.resize(constraint->getNumberOfMechanicalLinks());
          for(size_t i=0; i<openMBVForce.size(); i++) {
            openMBVForce[i]=ombv->createOpenMBV();
            openMBVForce[i]->setName(string("Force")+(openMBVForce.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(openMBVForce[i]);
          }
          openMBVMoment.resize(constraint->getNumberOfMechanicalLinks());
          for(size_t i=0; i<openMBVMoment.size(); i++) {
            openMBVMoment[i]=ombv->createOpenMBV();
            openMBVMoment[i]->setName(string("Moment")+(openMBVMoment.size()>1?to_string(i):string("")));
            getOpenMBVGrp()->addObject(openMBVMoment[i]);
          }
        }
      }
    }
    else if(stage==unknownStage) {
      Observer::init(stage, config);
      if((openMBVForce.size() or openMBVMoment.size()) and not getDynamicSystemSolver()->getInverseKinetics())
        throwError("(InverseKinematicsConstraintObserver::init()): inverse kinetics not enabled");
    }
    else
      Observer::init(stage, config);
  }

  void InverseKinematicsConstraintObserver::plot() {
    if(plotFeature[plotRecursive]) {
      auto TWOut = outputFrame->evalOrientation();
      if(plotFeature[force]) {
        for(int i=0; i<constraint->getNumberOfMechanicalLinks(); i++)
          Element::plot(TWOut.T()*constraint->getMechanicalLink(i)->evalForce());
      }
      if(plotFeature[moment]) {
        for(int i=0; i<constraint->getNumberOfMechanicalLinks(); i++)
          Element::plot(TWOut.T()*constraint->getMechanicalLink(i)->evalMoment());
      }
    }
    if(plotFeature[openMBV]) {
      if(ombv) {
        for(size_t i=0; i<openMBVForce.size(); i++) {
          array<double,8> data;
          data[0] = getTime();
          Vec3 toPoint=constraint->getMechanicalLink(i)->getPointOfApplication(0)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 WF = constraint->getMechanicalLink(i)->evalForce(0);
          data[4] = WF(0);
          data[5] = WF(1);
          data[6] = WF(2);
          data[7] = (this->*evalOMBVForceColorRepresentation[ombv->getColorRepresentation()])(i);
          openMBVForce[i]->append(data);
        }
        for(size_t i=0; i<openMBVMoment.size(); i++) {
          array<double,8> data;
          data[0] = getTime();
          Vec3 toPoint=constraint->getMechanicalLink(i)->getPointOfApplication(0)->evalPosition();
          data[1] = toPoint(0);
          data[2] = toPoint(1);
          data[3] = toPoint(2);
          Vec3 WM = constraint->getMechanicalLink(i)->evalMoment(0);
          data[4] = WM(0);
          data[5] = WM(1);
          data[6] = WM(2);
          data[7] = (this->*evalOMBVMomentColorRepresentation[ombv->getColorRepresentation()])(i);
          openMBVMoment[i]->append(data);
        }
      }
    }

    Observer::plot();
  }

  void InverseKinematicsConstraintObserver::initializeUsingXML(DOMElement *element) {
    Observer::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"inverseKinematicsConstraint");
    saved_constraint=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"outputFrame");
    if(e) saved_outputFrame=E(e)->getAttribute("ref");

    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      ombv = shared_ptr<OpenMBVArrow>(new OpenMBVArrow(1,1,OpenMBVArrow::toHead,OpenMBVArrow::toPoint));
      ombv->initializeUsingXML(e);
    }
  }

  double InverseKinematicsConstraintObserver::evalAbsoluteForce(int i) {
    return nrm2(constraint->getMechanicalLink(i)->evalForce());
  }

  double InverseKinematicsConstraintObserver::evalAbsoluteMoment(int i) {
    return nrm2(constraint->getMechanicalLink(i)->evalMoment());
  }

}
