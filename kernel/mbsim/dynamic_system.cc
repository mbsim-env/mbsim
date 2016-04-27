/* Copyright (C) 2004-2014 MBSim Development Team
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
#include "mbsim/dynamic_system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/modelling_interface.h"
#include "mbsim/links/link.h"
#include "mbsim/objects/object.h"
#include "mbsim/constraints/constraint.h"
#include "mbsim/observers/observer.h"
#include "mbsim/dynamic_system_solver.h"
#include "hdf5serie/file.h"
#include "hdf5serie/simpleattribute.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#include <openmbvcppinterface/frame.h>
#endif

//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace boost;

namespace MBSim {

  DynamicSystem::DynamicSystem(const string &name) :
      Element(name), R(0), PrPF(Vec3()), APF(SqrMat3(EYE)), q0(0), u0(0), x0(0), qSize(0), qInd(0), xSize(0), xInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), rFactorSize(0), rFactorInd(0), svSize(0), svInd(0), LinkStatusSize(0), LinkStatusInd(0), LinkStatusRegSize(0), LinkStatusRegInd(0), corrInd(0)
  {
    uSize[0] = 0;
    uSize[1] = 0;
    uInd[0] = 0;
    uInd[1] = 0;
    hSize[0] = 0;
    hSize[1] = 0;
    hInd[0] = 0;
    hInd[1] = 0;

    I = new Frame("I");
    addFrame(I);
  }

  DynamicSystem::~DynamicSystem() {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      delete *i;
    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for (vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
      delete *i;
    for (vector<Contour*>::iterator i = contour.begin(); i != contour.end(); ++i)
      delete *i;
    for (vector<Observer*>::iterator i = observer.begin(); i != observer.end(); ++i)
      delete *i;
    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      delete *i;
  }

  void DynamicSystem::updateT() {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateT();

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateT();
  }

  void DynamicSystem::updateh(int k) {
    for (int i = 0; i < (int) dynamicsystem.size(); i++)
      dynamicsystem[i]->updateh(k);

    for (int i = 0; i < (int) object.size(); i++)
      object[i]->updateh(k);

    for(unsigned int i=0; i<linkOrdered.size(); i++)
      for(unsigned int j=0; j<linkOrdered[i].size(); j++)
	linkOrdered[i][j]->updateh(k);
  }

  void DynamicSystem::updateM() {
    for (int i = 0; i < (int) dynamicsystem.size(); i++)
      dynamicsystem[i]->updateM();

    for (int i = 0; i < (int) object.size(); i++)
      object[i]->updateM();
  }

  void DynamicSystem::updatedq() {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedq();

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatedq();
  }

  void DynamicSystem::sethSize(int hSize_, int j) {
    hSize[j] = hSize_;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j), j);
      //(*i)->sethInd((*i)->getuInd(j),j);
    }

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j), j);
      //(*i)->sethInd((*i)->getuInd(j),j);
    }
  }

  void DynamicSystem::sethInd(int hInd_, int j) {
    hInd[j] = hInd_;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->sethInd((*i)->getuInd(j), j);
    }

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethInd((*i)->getuInd(j), j);
    }
  }

  void DynamicSystem::calcqSize() {
    qSize = 0;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcqSize();
      //(*i)->setqInd(qSize);
      qSize += (*i)->getqSize();
    }
    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcqSize();
      //(*i)->setqInd(qSize);
      qSize += (*i)->getqSize();
    }
  }

  void DynamicSystem::setsvInd(int svInd_) {
    svInd = svInd_;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->setsvInd(svInd_);
      svInd_ += (*i)->getsvSize();
    }
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->setsvInd(svInd_);
      svInd_ += (*i)->getsvSize();
    }
  }

  void DynamicSystem::setqInd(int qInd_) {
    qInd = qInd_;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->setqInd(qInd_);
      qInd_ += (*i)->getqSize();
    }
    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->setqInd(qInd_);
      qInd_ += (*i)->getqSize();
    }
  }

  void DynamicSystem::calcuSize(int j) {
    uSize[j] = 0;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcuSize(j);
      //     (*i)->setuInd(uSize[j],j);
      uSize[j] += (*i)->getuSize(j);
    }
    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcuSize(j);
      //    (*i)->setuInd(uSize[j],j);
      uSize[j] += (*i)->getuSize(j);
    }
  }

  void DynamicSystem::setuInd(int uInd_, int j) {
    uInd[j] = uInd_;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->setuInd(uInd_, j);
      uInd_ += (*i)->getuSize(j);
    }
    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->setuInd(uInd_, j);
      uInd_ += (*i)->getuSize(j);
    }
  }

//  int DynamicSystem::gethInd(DynamicSystem* sys, int i) {
//    return (this == sys) ? 0 : hInd[i] + parent->gethInd(sys,i);
//  }
//
//  int DynamicSystem::getuInd(DynamicSystem* sys, int i) {
//    return (this == sys) ? 0 : uInd[i] + parent->getuInd(sys,i);
//  }
//
//  int DynamicSystem::getqInd(DynamicSystem* sys) {
//    return (this == sys) ? 0 : qInd + parent->getqInd(sys);
//  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  shared_ptr<OpenMBV::Group> DynamicSystem::getOpenMBVGrp() {
    return openMBVGrp;
  }
#endif

  void DynamicSystem::updatewb() {

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updatewb();
  }

  void DynamicSystem::updateW(int j) {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).updateW(j);
  }

  void DynamicSystem::updateWInverseKinetics() {
    WInverseKinetics.init(0);

    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updateW(1);
  }

  void DynamicSystem::updatebInverseKinetics() {
    bInverseKinetics.init(0);

    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updateb();
  }

  void DynamicSystem::updateV(int j) {

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateV(j);
  }

  void DynamicSystem::updateg() {

    for (int i = 0; i < (int) linkSetValued.size(); i++)
      linkSetValued[i]->updateg();
  }

  void DynamicSystem::updategd() {

    for (int i = 0; i < (int) linkSetValued.size(); i++)
      linkSetValued[i]->updategd();
  }

  void DynamicSystem::updatedx() {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedx();

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatedx();

    for (vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      (**i).updatedx();
  }

  void DynamicSystem::updateStopVector() {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateStopVector();
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->updateStopVector();
  }

  void DynamicSystem::updateLinkStatus() {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateLinkStatus();
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->updateLinkStatus();
  }

  void DynamicSystem::updateLinkStatusReg() {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateLinkStatusReg();
    for (vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (*i)->updateLinkStatusReg();
  }

  void DynamicSystem::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Element::setDynamicSystemSolver(sys);
    for (unsigned i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->setDynamicSystemSolver(sys);

    for (unsigned i = 0; i < object.size(); i++)
      object[i]->setDynamicSystemSolver(sys);

    for (unsigned i = 0; i < link.size(); i++)
      link[i]->setDynamicSystemSolver(sys);

    for (unsigned i = 0; i < constraint.size(); i++)
      constraint[i]->setDynamicSystemSolver(sys);

    for (unsigned i = 0; i < frame.size(); i++)
      frame[i]->setDynamicSystemSolver(sys);

    for (unsigned i = 0; i < contour.size(); i++)
      contour[i]->setDynamicSystemSolver(sys);

    for (unsigned i = 0; i < inverseKineticsLink.size(); i++)
      inverseKineticsLink[i]->setDynamicSystemSolver(sys);

    for (unsigned i = 0; i < observer.size(); i++)
      observer[i]->setDynamicSystemSolver(sys);
  }

  void DynamicSystem::plot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      for (unsigned i = 0; i < dynamicsystem.size(); i++)
        dynamicsystem[i]->plot();
      for (unsigned i = 0; i < object.size(); i++)
        object[i]->plot();
      for (unsigned i = 0; i < link.size(); i++)
        link[i]->plot();
      for (unsigned i = 0; i < constraint.size(); i++)
        constraint[i]->plot();
      for (unsigned i = 0; i < frame.size(); i++)
        frame[i]->plot();
      for (unsigned i = 0; i < contour.size(); i++)
        contour[i]->plot();
      for (unsigned i = 0; i < inverseKineticsLink.size(); i++)
        inverseKineticsLink[i]->plot();
      for (unsigned i = 0; i < observer.size(); i++)
        observer[i]->plot();
    }
  }
  
  void DynamicSystem::plotAtSpecialEvent() {
    if (getPlotFeature(plotRecursive) == enabled) {
      for (unsigned i = 0; i < dynamicsystem.size(); i++)
        dynamicsystem[i]->plotAtSpecialEvent();
      for (unsigned i = 0; i < object.size(); i++)
        object[i]->plotAtSpecialEvent();
      for (unsigned i = 0; i < link.size(); i++)
        link[i]->plotAtSpecialEvent();
      for (unsigned i = 0; i < constraint.size(); i++)
        constraint[i]->plotAtSpecialEvent();
      for (unsigned i = 0; i < frame.size(); i++)
        frame[i]->plotAtSpecialEvent();
      for (unsigned i = 0; i < contour.size(); i++)
        contour[i]->plotAtSpecialEvent();
      for (unsigned i = 0; i < inverseKineticsLink.size(); i++)
        inverseKineticsLink[i]->plotAtSpecialEvent();
      for (unsigned i = 0; i < observer.size(); i++)
        observer[i]->plotAtSpecialEvent();
    }
  }

  void DynamicSystem::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      for (unsigned i = 0; i < dynamicsystem.size(); i++)
        dynamicsystem[i]->closePlot();
      for (unsigned i = 0; i < object.size(); i++)
        object[i]->closePlot();
      for (unsigned i = 0; i < link.size(); i++)
        link[i]->closePlot();
      for (unsigned i = 0; i < frame.size(); i++)
        frame[i]->closePlot();

      //if (not parent) //TODO: why was that active once?
      //  delete plotGroup;
    }
  }

  void DynamicSystem::init(InitStage stage) {
    if (stage == resolveXMLPath) {
      if (saved_frameOfReference != "")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
    }
    else if (stage == preInit) {
      if (!R) {
        DynamicSystem *sys = dynamic_cast<DynamicSystem*>(parent);
        if (sys)
          R = sys->getFrameI();
      }
      for (unsigned int k = 1; k < frame.size(); k++) {
        if(not(static_cast<FixedRelativeFrame*>(frame[k])->getFrameOfReference()))
          static_cast<FixedRelativeFrame*>(frame[k])->setFrameOfReference(I);
      }
      for (unsigned int k = 0; k < contour.size(); k++) {
        if(not(static_cast<RigidContour*>(contour[k]))->getFrameOfReference())
          static_cast<RigidContour*>(contour[k])->setFrameOfReference(I);
      }
      if (R) {
        I->setOrientation(R->evalOrientation() * APF);
        I->setPosition(R->getPosition() + R->getOrientation() * PrPF);
      }
      else {
        DynamicSystem* sys = dynamic_cast<DynamicSystem*>(parent);
        if (sys) {
          I->setOrientation(sys->getFrameI()->evalOrientation() * APF);
          I->setPosition(sys->getFrameI()->getPosition() + sys->getFrameI()->getOrientation() * PrPF);
        }
        else {
          I->setOrientation(getFrameI()->evalOrientation() * APF);
          I->setPosition(getFrameI()->getPosition() + getFrameI()->getOrientation() * PrPF);
        }
      }
    }
    else if (stage == plotting) {
      if (parent)
        updatePlotFeatures();

      if (getPlotFeature(plotRecursive) == enabled) {
        if (getPlotFeature(separateFilePerGroup) == enabled) {
          // We do not use getPath here since separateFilePerGroup is only allowed per Group and all parents of Group's
          // are also Group's (DynamicSystem's) -> Skip the Group[...] for each sub path.
          // We can walk to the top here since stage plotting is done before reorganizeHierarchy.
          string fileName="mbsim.h5";
          const DynamicSystem *ds=this;
          while(ds) {
            fileName=ds->getName()+"."+fileName;
            ds=static_cast<const DynamicSystem*>(ds->getParent());
          }
          // create symbolic link in parent plot file if exist
          if (parent)
            parent->getPlotGroup()->createExternalLink(name, make_pair(boost::filesystem::path(fileName), string("/")));
          // create new plot file (cast needed because of the inadequacy of the HDF5 C++ interface?)
          hdf5File = boost::make_shared<H5::File>(fileName, H5::File::write);
          plotGroup = hdf5File.get();
        }
        else
          plotGroup = parent->getPlotGroup()->createChildObject<H5::Group>(name)();

        plotGroup->createChildAttribute<H5::SimpleAttribute<string> >("Description")()->write("Object of class: " + getType());
        plotVectorSerie = NULL;

#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVGrp = OpenMBV::ObjectFactory::create<OpenMBV::Group>();
        openMBVGrp->setName(name);
        //if(parent) parent->openMBVGrp->addObject(openMBVGrp);
        if (parent)
          parent->getOpenMBVGrp()->addObject(openMBVGrp);
        if (getPlotFeature(separateFilePerGroup) == enabled)
          openMBVGrp->setSeparateFile(true);
#endif

        H5::File *file=dynamic_cast<H5::File*>(plotGroup);
        if(file)
          file->flush();
      }
    }
    else if (stage==unknownStage) {
    }

    for (unsigned i = 0; i < frame.size(); i++)
      frame[i]->init(stage);
    for (unsigned int i = 0; i < contour.size(); i++)
      contour[i]->init(stage);
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->init(stage);
    for (unsigned i = 0; i < object.size(); i++)
      object[i]->init(stage);
    for (unsigned i = 0; i < link.size(); i++)
      link[i]->init(stage);
    for (unsigned i = 0; i < constraint.size(); i++)
      constraint[i]->init(stage);
    for (unsigned i = 0; i < model.size(); i++)
      model[i]->init(stage);
    for (unsigned i = 0; i < inverseKineticsLink.size(); i++)
      inverseKineticsLink[i]->init(stage);
    for (unsigned i = 0; i < observer.size(); i++)
      observer[i]->init(stage);
  }

  int DynamicSystem::solveConstraintsFixpointSingle() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->solveConstraintsFixpointSingle();

    return 0;
  }

  int DynamicSystem::solveImpactsFixpointSingle() {

    for (int i = 0; i < (int) linkSetValuedActive.size(); i++)
      linkSetValuedActive[i]->solveImpactsFixpointSingle();

    return 0;
  }

  int DynamicSystem::solveConstraintsGaussSeidel() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->solveConstraintsGaussSeidel();

    return 0;
  }

  int DynamicSystem::solveImpactsGaussSeidel() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->solveImpactsGaussSeidel();

    return 0;
  }

  int DynamicSystem::solveConstraintsRootFinding() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->solveConstraintsRootFinding();

    return 0;
  }

  int DynamicSystem::solveImpactsRootFinding() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->solveImpactsRootFinding();

    return 0;
  }

  int DynamicSystem::jacobianConstraints() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->jacobianConstraints();

    return 0;
  }

  int DynamicSystem::jacobianImpacts() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->jacobianImpacts();

    return 0;
  }

  void DynamicSystem::checkConstraintsForTermination() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkConstraintsForTermination();
  }

  void DynamicSystem::checkImpactsForTermination() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkImpactsForTermination();
  }

  void DynamicSystem::updaterFactors() {

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterFactors();
  }

  Frame* DynamicSystem::getFrame(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < frame.size(); i++) {
      if (frame[i]->getName() == name)
        return frame[i];
    }
    if (check) {
      if (!(i < frame.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no frame \"" + name + "\"!");
      assert(i < frame.size());
    }
    return NULL;
  }

  Contour* DynamicSystem::getContour(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < contour.size(); i++) {
      if (contour[i]->getName() == name)
        return contour[i];
    }
    if (check) {
      if (!(i < contour.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no contour \"" + name + "\"!");
      assert(i < contour.size());
    }
    return NULL;
  }

  void DynamicSystem::updateqRef(const Vec &qParent) {
    q >> qParent(qInd, qInd + qSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateqRef(qParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateqRef(qParent);
  }

  void DynamicSystem::updateqdRef(const Vec &qdParent) {
    qd >> qdParent(qInd, qInd + qSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateqdRef(qdParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateqdRef(qdParent);
  }

  void DynamicSystem::updatedqRef(const Vec &dqParent) {
    dq >> dqParent(qInd, qInd + qSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedqRef(dqParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatedqRef(dqParent);
  }

  void DynamicSystem::updateuRef(const Vec &uParent) {
    u >> uParent(uInd[0], uInd[0] + uSize[0] - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateuRef(uParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateuRef(uParent);
  }

  void DynamicSystem::updateuallRef(const Vec &uParent) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateuallRef(uParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateuallRef(uParent);
  }

  void DynamicSystem::updateudRef(const Vec &udParent) {
    ud >> udParent(uInd[0], uInd[0] + uSize[0] - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateudRef(udParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateudRef(udParent);
  }

  void DynamicSystem::updateduRef(const Vec &duParent) {
    du >> duParent(uInd[0], uInd[0] + uSize[0] - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateduRef(duParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateduRef(duParent);
  }

  void DynamicSystem::updateudallRef(const Vec &udParent) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updateudallRef(udParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updateudallRef(udParent);
  }

  void DynamicSystem::updatexRef(const Vec &xParent) {
    x >> xParent(xInd, xInd + xSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatexRef(xParent);

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef(xParent);

    for (vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      (**i).updatexRef(xParent);
  }

  void DynamicSystem::updatexdRef(const Vec &xdParent) {
    xd >> xdParent(xInd, xInd + xSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatexdRef(xdParent);

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexdRef(xdParent);

    for (vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      (**i).updatexdRef(xdParent);
  }

  void DynamicSystem::updatedxRef(const Vec &dxParent) {
    dx >> dxParent(xInd, xInd + xSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedxRef(dxParent);

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatedxRef(dxParent);

    for (vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      (**i).updatedxRef(dxParent);
  }

  void DynamicSystem::updatehRef(const Vec &hParent, int j) {
    h[j] >> hParent(hInd[j], hInd[j] + hSize[j] - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatehRef(hParent, j);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatehRef(hParent, j);

    for (vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatehRef(hParent, j);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      if ((**i).hasSmoothPart())
        (**i).updatehRef(hParent, j);
  }

  void DynamicSystem::updaterRef(const Vec &rParent, int j) {
    r[j] >> rParent(hInd[j], hInd[j] + hSize[j] - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updaterRef(rParent, j);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updaterRef(rParent, j);
  }

  void DynamicSystem::updaterdtRef(const Vec &rdtParent) {
    rdt >> rdtParent(hInd[0], hInd[0] + hSize[0] - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updaterdtRef(rdtParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updaterdtRef(rdtParent);
  }

  void DynamicSystem::updateTRef(const Mat& TParent) {
    T >> TParent(Index(qInd, qInd + qSize - 1), Index(uInd[0], uInd[0] + uSize[0] - 1));

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateTRef(TParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->updateTRef(TParent);
  }

  void DynamicSystem::updateMRef(const SymMat& MParent) {
    M >> MParent(Index(hInd[0], hInd[0] + hSize[0] - 1));

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateMRef(MParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->updateMRef(MParent);
  }

  void DynamicSystem::updateLLMRef(const SymMat& LLMParent) {
    LLM >> LLMParent(Index(hInd[0], hInd[0] + hSize[0] - 1));

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateLLMRef(LLMParent);

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->updateLLMRef(LLMParent);
  }

  void DynamicSystem::updategRef(const Vec& gParent) {
    g >> gParent(gInd, gInd + gSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updategRef(gParent);
  }

  void DynamicSystem::updategdRef(const Vec& gdParent) {
    gd >> gdParent(gdInd, gdInd + gdSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updategdRef(gdParent);
  }

  void DynamicSystem::updatelaRef(const Vec &laParent) {
    la >> laParent(laInd, laInd + laSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updatelaRef(laParent);
  }

  void DynamicSystem::updateLaRef(const Vec &LaParent) {
    La >> LaParent(laInd, laInd + laSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateLaRef(LaParent);
  }

  void DynamicSystem::updatelaInverseKineticsRef(const Vec &laParent) {
    laInverseKinetics >> laParent(0, laInverseKineticsSize - 1);

    //for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
    //  (*i)->updatelaRefSpecial(la);

    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updatelaRef(laParent);
  }

  void DynamicSystem::updatewbRef(const Vec &wbParent) {
    wb >> wbParent(laInd, laInd + laSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updatewbRef(wbParent);
  }

  void DynamicSystem::updateWRef(const Mat &WParent, int j) {
    W[j] >> WParent(Index(hInd[j], hInd[j] + hSize[j] - 1), Index(laInd, laInd + laSize - 1));

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateWRef(WParent, j);
  }

  void DynamicSystem::updateWInverseKineticsRef(const Mat &WParent) {
    WInverseKinetics >> WParent(Index(hInd[1], hInd[1] + hSize[1] - 1), Index(0, laInverseKineticsSize - 1));

    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updateWRef(WParent, 1);
  }

  void DynamicSystem::updatebInverseKineticsRef(const Mat &bParent) {
    bInverseKinetics >> bParent(Index(0, bInverseKineticsSize - 1), Index(0, laInverseKineticsSize - 1));

    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updatebRef(bParent);
  }

  void DynamicSystem::updateVRef(const Mat &VParent, int j) {
    V[j] >> VParent(Index(hInd[j], hInd[j] + hSize[j] - 1), Index(laInd, laInd + laSize - 1));

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateVRef(VParent, j);
  }

  void DynamicSystem::updatesvRef(const Vec &svParent) {
    sv >> svParent(svInd, svInd + svSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updatesvRef(svParent);

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatesvRef(svParent);
  }

  void DynamicSystem::updatejsvRef(const VecInt &jsvParent) {
    jsv >> jsvParent(svInd, svInd + svSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updatejsvRef(jsvParent);

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatejsvRef(jsvParent);
  }

  void DynamicSystem::updateresRef(const Vec &resParent) {
    res >> resParent(laInd, laInd + laSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateresRef(resParent);
  }

  void DynamicSystem::updaterFactorRef(const Vec &rFactorParent) {
    rFactor >> rFactorParent(rFactorInd, rFactorInd + rFactorSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterFactorRef(rFactorParent);
  }

  void DynamicSystem::updateLinkStatusRef(const VecInt &LinkStatusParent) {
    LinkStatus >> LinkStatusParent(LinkStatusInd, LinkStatusInd + LinkStatusSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateLinkStatusRef(LinkStatusParent);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateLinkStatusRef(LinkStatusParent);
  }

  void DynamicSystem::updateLinkStatusRegRef(const VecInt &LinkStatusRegParent) {
    LinkStatusReg >> LinkStatusRegParent(LinkStatusRegInd, LinkStatusRegInd + LinkStatusRegSize - 1);

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (*i)->updateLinkStatusRegRef(LinkStatusRegParent);

    for (vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updateLinkStatusRegRef(LinkStatusRegParent);
  }

  void DynamicSystem::initz() {
    for (unsigned i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->initz();
    for (unsigned i = 0; i < object.size(); i++)
      object[i]->initz();
    for (unsigned i = 0; i < link.size(); i++)
      link[i]->initz();
    for (unsigned i = 0; i < constraint.size(); i++)
      constraint[i]->initz();
  }

  void DynamicSystem::writez(H5::GroupBase *parent) {
    for (unsigned i = 0; i < dynamicsystem.size(); i++) {
      H5::Group *group = parent->createChildObject<H5::Group>("System_" + numtostr((int) i))();
      dynamicsystem[i]->writez(group);
    }
    for (unsigned i = 0; i < object.size(); i++) {
      H5::Group *group = parent->createChildObject<H5::Group>("Object_" + numtostr((int) i))();
      object[i]->writez(group);
    }
    for (unsigned i = 0; i < link.size(); i++) {
      H5::Group *group = parent->createChildObject<H5::Group>("Link_" + numtostr((int) i))();
      link[i]->writez(group);
    }
  }

  void DynamicSystem::readz0(H5::GroupBase *parent) {
    for (unsigned i = 0; i < dynamicsystem.size(); i++) {
      H5::Group *group = parent->openChildObject<H5::Group>("System_" + numtostr((int) i));
      dynamicsystem[i]->readz0(group);
    }
    for (unsigned i = 0; i < object.size(); i++) {
      H5::Group *group = parent->openChildObject<H5::Group>("Object_" + numtostr((int) i));
      object[i]->readz0(group);
    }
    for (unsigned i = 0; i < link.size(); i++) {
      H5::Group *group = parent->openChildObject<H5::Group>("Link_" + numtostr((int) i));
      link[i]->readz0(group);
    }
  }

  void DynamicSystem::clearElementLists() {
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->clearElementLists();
    dynamicsystem.clear(); // delete old DynamicSystem list
    object.clear(); // delete old object list
    frame.clear(); // delete old frame list
    contour.clear(); // delete old contour list
    link.clear(); // delete old link list
    constraint.clear(); // delete old constraint list
    inverseKineticsLink.clear(); // delete old link list
    observer.clear(); // delete old link list
  }

  void DynamicSystem::buildListOfDynamicSystems(vector<DynamicSystem*> &sys) {
    for (unsigned int i = 0; i < dynamicsystem.size(); i++) {
      sys.push_back(dynamicsystem[i]);
      dynamicsystem[i]->setPath(dynamicsystem[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfDynamicSystems(sys);
  }

  void DynamicSystem::buildListOfObjects(vector<Object*> &obj) {
    for (unsigned int i = 0; i < object.size(); i++) {
      obj.push_back(object[i]);
      object[i]->setPath(object[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfObjects(obj);
  }

  void DynamicSystem::buildListOfLinks(vector<Link*> &lnk) {
    for (unsigned int i = 0; i < link.size(); i++) {
      lnk.push_back(link[i]);
      link[i]->setPath(link[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfLinks(lnk);
  }

  void DynamicSystem::buildListOfConstraints(vector<Constraint*> &crt) {
    for (unsigned int i = 0; i < constraint.size(); i++) {
      crt.push_back(constraint[i]);
      constraint[i]->setPath(constraint[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfConstraints(crt);
  }

  void DynamicSystem::buildListOfFrames(vector<Frame*> &frm) {
    for (unsigned int i = 0; i < frame.size(); i++) {
      frm.push_back(frame[i]);
      frame[i]->setPath(frame[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfFrames(frm);
  }

  void DynamicSystem::buildListOfContours(vector<Contour*> &cnt) {
    for (unsigned int i = 0; i < contour.size(); i++) {
      cnt.push_back(contour[i]);
      contour[i]->setPath(contour[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfContours(cnt);
  }

  void DynamicSystem::buildListOfModels(std::vector<ModellingInterface*> &modelList) {
    for (unsigned int i = 0; i < model.size(); i++) {
      modelList.push_back(model[i]);
      // Note! setPath is (and must be) done in processModellList
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfModels(modelList);
  }

  void DynamicSystem::buildListOfInverseKineticsLinks(vector<Link*> &iklnk) {
    for (unsigned int i = 0; i < inverseKineticsLink.size(); i++) {
      iklnk.push_back(inverseKineticsLink[i]);
      inverseKineticsLink[i]->setPath(inverseKineticsLink[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfInverseKineticsLinks(iklnk);
  }

  void DynamicSystem::buildListOfObservers(vector<Observer*> &obsrv) {
    for (unsigned int i = 0; i < observer.size(); i++) {
      obsrv.push_back(observer[i]);
      observer[i]->setPath(observer[i]->getPath());
    }
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->buildListOfObservers(obsrv);
  }

  void DynamicSystem::setUpInverseKinetics() {
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->setUpInverseKinetics();

    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (*i)->setUpInverseKinetics();

    for (vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i)
      (*i)->setUpInverseKinetics();
  }

  void DynamicSystem::setUpLinks() {
    for (unsigned int i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->setUpLinks();

    // clear container first, because setUpLinks in called twice from InitStage resize (before and after the reorganization)
    linkSetValued.clear();
    linkSetValuedActive.clear();
    linkSingleValued.clear();
    for (unsigned int i = 0; i < link.size(); i++) {
      bool hasForceLaw = false;
      if (link[i]->isSetValued()) {
        hasForceLaw = true;
        linkSetValued.push_back(link[i]);
        linkSetValuedActive.push_back(link[i]);
      }
      if (link[i]->isSingleValued()) {
        hasForceLaw = true;
        linkSingleValued.push_back(link[i]);
      }
      if (not hasForceLaw) {
        throw new MBSimError("The Link \"" + link[i]->getPath() + "\" comprises now force law!");
      }
    }
  }

  bool DynamicSystem::gActiveChanged() {
    bool changed = false;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      if ((*i)->gActiveChanged())
        changed = true;

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      if ((*i)->gActiveChanged())
        changed = true;

    return changed;
  }

  bool DynamicSystem::gActiveChangedReg() {
    bool changed = false;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      if ((*i)->gActiveChanged())
        changed = true;

    for (vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      if ((*i)->gActiveChanged())
        changed = true;
    
    return changed;
  }

  bool DynamicSystem::detectImpact() {
    bool impact = false;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      if ((*i)->detectImpact())
        impact = true;

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      if ((*i)->detectImpact())
        impact = true;

    return impact;
  }

  void DynamicSystem::calcxSize() {
    xSize = 0;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcxSize();
      xSize += (*i)->getxSize();
    }

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->calcxSize();
      xSize += (*i)->getxSize();
    }

    for (vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i) {
      (*i)->calcxSize();
      xSize += (*i)->getxSize();
    }
  }

  void DynamicSystem::setxInd(int xInd_) {
    xInd = xInd_;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->setxInd(xInd_);
      xInd_ += (*i)->getxSize();
    }

    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->setxInd(xInd_);
      xInd_ += (*i)->getxSize();
    }

    for (vector<Constraint*>::iterator i = constraint.begin(); i != constraint.end(); ++i) {
      (*i)->setxInd(xInd_);
      xInd_ += (*i)->getxSize();
    }
  }

  void DynamicSystem::calcLinkStatusSize() {
    LinkStatusSize = 0;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcLinkStatusSize();
      (*i)->setLinkStatusInd(LinkStatusSize);
      LinkStatusSize += (*i)->getLinkStatusSize();
    }
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcLinkStatusSize();
      (*i)->setLinkStatusInd(LinkStatusSize);
      LinkStatusSize += (*i)->getLinkStatusSize();
    }
  }

  void DynamicSystem::calcLinkStatusRegSize() {
    LinkStatusRegSize = 0;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcLinkStatusRegSize();
      (*i)->setLinkStatusRegInd(LinkStatusRegSize);
      LinkStatusRegSize += (*i)->getLinkStatusRegSize();
    }
    for (vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) {
      (*i)->calcLinkStatusRegSize();
      (*i)->setLinkStatusRegInd(LinkStatusRegSize);
      LinkStatusRegSize += (*i)->getLinkStatusRegSize();
    }
  }

  void DynamicSystem::calcsvSize() {
    svSize = 0;

    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcsvSize();
      svSize += (*i)->getsvSize();
    }
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->calcsvSize();
      svSize += (*i)->getsvSize();
    }
  }

  void DynamicSystem::calclaSize(int j) {
    laSize = 0;

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calclaSize(j);
      (*i)->setlaInd(laSize);
      laSize += (*i)->getlaSize();
    }
  }

  void DynamicSystem::calclaInverseKineticsSize() {
    laInverseKineticsSize = 0;

    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i) {
      (*i)->calclaSize(0);
      (*i)->setlaInd(laInverseKineticsSize);
      laInverseKineticsSize += (*i)->getlaSize();
    }
  }

  void DynamicSystem::calcbInverseKineticsSize() {
    bInverseKineticsSize = 0;

    for (vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i) {
      (*i)->calcbSize();
      (*i)->setbInd(bInverseKineticsSize);
      bInverseKineticsSize += (*i)->getbSize();
    }
  }

  void DynamicSystem::calcgSize(int j) {
    gSize = 0;

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcgSize(j);
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }
  }

  void DynamicSystem::calcgdSize(int j) {
    gdSize = 0;

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcgdSize(j);
      (*i)->setgdInd(gdSize);
      gdSize += (*i)->getgdSize();
    }
  }

  void DynamicSystem::calcrFactorSize(int j) {
    rFactorSize = 0;

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcrFactorSize(j);
      (*i)->setrFactorInd(rFactorSize);
      rFactorSize += (*i)->getrFactorSize();
    }
  }

  void DynamicSystem::setUpActiveLinks() {

    linkSetValuedActive.clear();

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      if ((*i)->isActive())
        linkSetValuedActive.push_back(*i);
    }
  }

  void DynamicSystem::checkActive(int j) {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->checkActive(j);
    setUpActiveLinks();
  }

  void DynamicSystem::checkActiveReg(int j) {
    for (vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (*i)->checkActive(j);
  }

  void DynamicSystem::setgTol(double tol) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).setgTol(tol);
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).setgTol(tol);
  }

  void DynamicSystem::setgdTol(double tol) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).setgdTol(tol);
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).setgdTol(tol);
  }

  void DynamicSystem::setgddTol(double tol) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).setgddTol(tol);
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).setgddTol(tol);
  }

  void DynamicSystem::setlaTol(double tol) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).setlaTol(tol);
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).setlaTol(tol);
  }

  void DynamicSystem::setLaTol(double tol) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).setLaTol(tol);
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).setLaTol(tol);
  }

  void DynamicSystem::setrMax(double rMax) {
    for (vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).setrMax(rMax);
    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).setrMax(rMax);
  }

  void DynamicSystem::addFrame(FixedRelativeFrame *frame) {
    addFrame(static_cast<Frame*>(frame));
  }

  void DynamicSystem::addContour(RigidContour *contour) {
    addContour(static_cast<Contour*>(contour));
  }

  void DynamicSystem::addFrame(Frame *frame_) {
    if (getFrame(frame_->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprises one Frame by the name \"" + name + "\"!");
      assert(getFrame(frame_->getName(),false)==NULL);
    }
    frame.push_back(frame_);
    frame_->setParent(this);
  }

  void DynamicSystem::addContour(Contour* contour_) {
    if (getContour(contour_->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprise one Contour by the name \"" + name + "\"!");
      assert(getContour(contour_->getName(),false)==NULL);
    }
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  int DynamicSystem::frameIndex(const Frame *frame_) const {
    for (unsigned int i = 0; i < frame.size(); i++) {
      if (frame_ == frame[i])
        return i;
    }
    return -1;
  }

  DynamicSystem* DynamicSystem::getGroup(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < dynamicsystem.size(); i++) {
      if (dynamicsystem[i]->getName() == name)
        return dynamicsystem[i];
    }
    if (check) {
      if (!(i < dynamicsystem.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no DynamicSystem \"" + name + "\"!");
      assert(i < dynamicsystem.size());
    }
    return NULL;
  }

  Object* DynamicSystem::getObject(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < object.size(); i++) {
      if (object[i]->getName() == name)
        return object[i];
    }
    if (check) {
      if (!(i < object.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no Object \"" + name + "\"!");
      assert(i < object.size());
    }
    return NULL;
  }

  void DynamicSystem::addLink(Link *lnk) {
    if (getLink(lnk->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprise one Link by the name \"" + lnk->getName() + "\"!");
      assert(getLink(lnk->getName(),false) == NULL);
    }

    link.push_back(lnk);
    lnk->setParent(this);
  }

  void DynamicSystem::addConstraint(Constraint *crt) {
    if (getConstraint(crt->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprise one Constraint by the name \"" + crt->getName() + "\"!");
      assert(getConstraint(crt->getName(),false) == NULL);
    }

    constraint.push_back(crt);
    crt->setParent(this);
  }

  void DynamicSystem::addInverseKineticsLink(Link *lnk) {
    //if(getLink(lnk->getName(),false)) {
    //  cout << "ERROR (DynamicSystem: addLink): The DynamicSystem " << this->name << " can only comprise one Link by the name " <<  lnk->getName() << "!" << endl;
    //  assert(getLink(lnk->getName(),false) == NULL);
    //}
    inverseKineticsLink.push_back(lnk);
    lnk->setParent(this);
  }

  Link* DynamicSystem::getLink(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < link.size(); i++) {
      if (link[i]->getName() == name)
        return link[i];
    }
    if (check) {
      if (!(i < link.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no Link \"" + name + "\"!");
      assert(i < link.size());
    }
    return NULL;
  }

  Constraint* DynamicSystem::getConstraint(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < constraint.size(); i++) {
      if (constraint[i]->getName() == name)
        return constraint[i];
    }
    if (check) {
      if (!(i < constraint.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no Constraint \"" + name + "\"!");
      assert(i < constraint.size());
    }
    return NULL;
  }

  void DynamicSystem::addModel(ModellingInterface *model_) {
    if (getModel(model_->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprise one model by the name \"" + model_->getName() + "\"!");
      assert(getModel(model_->getName(),false) == NULL);
    }
    model.push_back(model_);
    model_->setParent(this);
  }

  ModellingInterface* DynamicSystem::getModel(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < model.size(); i++) {
      if (model[i]->getName() == name)
        return model[i];
    }
    if (check) {
      if (!(i < model.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no model \"" + name + "\"!");
      assert(i < model.size());
    }
    return NULL;
  }

  void DynamicSystem::addGroup(DynamicSystem *sys) {
    if (getGroup(sys->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprise one DynamicSystem by the name \"" + sys->getName() + "\"!");
      assert(getGroup(sys->getName(),false) == NULL);
    }
    dynamicsystem.push_back(sys);
    sys->setParent(this);
  }

  void DynamicSystem::addObject(Object *obj) {
    if (getObject(obj->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprise one Object by the name \"" + obj->getName() + "\"!");
      assert(getObject(obj->getName(),false) == NULL);
    }
    object.push_back(obj);
    obj->setParent(this);
  }

  Element * DynamicSystem::getChildByContainerAndName(const std::string &container, const std::string &name) const {
    if (container == "Object")
      return getObject(name);
    else if (container == "Link")
      return getLink(name);
    else if (container == "Group")
      return getGroup(name);
    else if (container == "Frame")
      return getFrame(name);
    else if (container == "Contour")
      return getContour(name);
    else if (container == "Observer")
      return getObserver(name);
    else
      THROW_MBSIMERROR("Unknown container '"+container+"'.");
  }

  Observer* DynamicSystem::getObserver(const string &name, bool check) const {
    unsigned int i;
    for (i = 0; i < observer.size(); i++) {
      if (observer[i]->getName() == name)
        return observer[i];
    }
    if (check) {
      if (!(i < observer.size()))
        THROW_MBSIMERROR("DynamicSystem comprises no Observer \"" + name + "\"!");
      assert(i < observer.size());
    }
    return NULL;
  }

  void DynamicSystem::addObserver(Observer *ele) {
    if (getObserver(ele->getName(), false)) {
      THROW_MBSIMERROR("DynamicSystem can only comprise one Observer by the name \"" + ele->getName() + "\"!");
      assert(getObserver(ele->getName(),false) == NULL);
    }
    observer.push_back(ele);
    ele->setParent(this);
  }

  void DynamicSystem::updatecorr(int j) {

    for (int i = 0; i < (int) linkSetValued.size(); i++)
      linkSetValued[i]->updatecorr(j);
  }
  
  void DynamicSystem::updatecorrRef(const fmatvec::Vec &ref) {
    corr >> ref(corrInd, corrInd + corrSize - 1);

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updatecorrRef(ref);
  }

  void DynamicSystem::calccorrSize(int j) {
    corrSize = 0;

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calccorrSize(j);
      (*i)->setcorrInd(corrSize);
      corrSize += (*i)->getcorrSize();
    }
  }

  void DynamicSystem::checkRoot() {

    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->checkRoot();
  }

  void DynamicSystem::resetUpToDate() {
    for (unsigned i = 0; i < dynamicsystem.size(); i++)
      dynamicsystem[i]->resetUpToDate();
    for (unsigned i = 0; i < object.size(); i++)
      object[i]->resetUpToDate();
    for (unsigned i = 0; i < link.size(); i++)
      link[i]->resetUpToDate();
    for (unsigned i = 0; i < constraint.size(); i++)
      constraint[i]->resetUpToDate();
    for (unsigned i = 0; i < observer.size(); i++)
      observer[i]->resetUpToDate();
    for (unsigned i = 0; i < inverseKineticsLink.size(); i++)
      inverseKineticsLink[i]->resetUpToDate();
  }

  const fmatvec::Mat& DynamicSystem::getT(bool check) const {
    assert((not check) or (not ds->getUpdateT()));
    return T;
  }

  const fmatvec::Vec& DynamicSystem::geth(int i, bool check) const {
    assert((not check) or (not ds->getUpdateh(i)));
    return h[i];
  }

  const fmatvec::SymMat& DynamicSystem::getM(bool check) const {
    assert((not check) or (not ds->getUpdateM()));
    return M;
  }

  const fmatvec::SymMat& DynamicSystem::getLLM(bool check) const {
    assert((not check) or (not ds->getUpdateLLM()));
    return LLM;
  }

  const fmatvec::Mat& DynamicSystem::getW(int i, bool check) const {
    assert((not check) or (not ds->getUpdateW(i)));
    return W[i];
  }

  const fmatvec::Mat& DynamicSystem::getV(int i, bool check) const {
    assert((not check) or (not ds->getUpdateV(i)));
    return V[i];
  }

  const fmatvec::Vec& DynamicSystem::getg(bool check) const {
    assert((not check) or (not ds->getUpdateg()));
    return g;
  }

  const fmatvec::Vec& DynamicSystem::getgd(bool check) const {
    assert((not check) or (not ds->getUpdategd()));
    return gd;
  }

  const fmatvec::Vec& DynamicSystem::getla(bool check) const {
    assert((not check) or (not ds->getUpdatela()));
    return la;
  }

  const fmatvec::Vec& DynamicSystem::getLa(bool check) const {
    assert((not check) or (not ds->getUpdateLa()));
    return La;
  }

  fmatvec::SymMat& DynamicSystem::getLLM(bool check) {
    assert((not check) or (not ds->getUpdateLLM()));
    return LLM;
  }

  fmatvec::Mat& DynamicSystem::getW(int i, bool check) {
    assert((not check) or (not ds->getUpdateW(i)));
    return W[i];
  }

  fmatvec::Vec& DynamicSystem::getla(bool check) {
    assert((not check) or (not ds->getUpdatela()));
    return la;
  }

  fmatvec::Vec& DynamicSystem::getLa(bool check) {
    assert((not check) or (not ds->getUpdateLa()));
    return La;
  }

//
//  fmatvec::Vec& DynamicSystem::getV(int i, bool check) {
//    assert((not check) or (not ds->updV(i)));
//    return V[i];
//  }

  const Mat& DynamicSystem::evalT() {
    if(ds->getUpdateT()) ds->updateT();
    return T;
  }

  const Vec& DynamicSystem::evalh(int i) {
    if(ds->getUpdateh(i)) ds->updateh(i);
    return h[i];
  }

  const Vec& DynamicSystem::evalr(int i) {
    if(ds->getUpdater(i)) ds->updater(i);
    return r[i];
  }

  const Vec& DynamicSystem::evalrdt() {
    if(ds->getUpdaterdt()) ds->updaterdt();
    return rdt;
  }

  const SymMat& DynamicSystem::evalM() {
    if(ds->getUpdateM()) ds->updateM();
    return M;
  }

  const SymMat& DynamicSystem::evalLLM() {
    if(ds->getUpdateLLM()) ds->updateLLM();
    return LLM;
  }

  const Mat& DynamicSystem::evalW(int i) {
    if(ds->getUpdateW(i)) ds->updateW(i);
    return W[i];
  }

  const Mat& DynamicSystem::evalV(int i) {
    if(ds->getUpdateV(i)) ds->updateV(i);
    return V[i];
  }

  const Vec& DynamicSystem::evalwb() {
    if(ds->getUpdatewb()) ds->updatewb();
    return wb;
  }

  const Vec& DynamicSystem::evalg() {
    if(ds->getUpdateg()) ds->updateg();
    return g;
  }

  const Vec& DynamicSystem::evalgd() {
    if(ds->getUpdategd()) ds->updategd();
    return gd;
  }

  const Mat& DynamicSystem::evalWInverseKinetics() {
    ds->updateWInverseKinetics();
    return WInverseKinetics;
  }

  const Mat& DynamicSystem::evalbInverseKinetics() {
    ds->updatebInverseKinetics();
    return bInverseKinetics;
  }

//  const Mat& DynamicSystem::evalsv() {
//    ds->updateStopVector();
//    return sv;
//  }

}
