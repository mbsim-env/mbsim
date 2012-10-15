/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/modelling_interface.h"
#include "mbsim/extra_dynamic.h"
#include "mbsim/link.h"
#include "mbsim/contour.h"
#include "mbsim/object.h"
#include "mbsim/frame.h"
#include "mbsim/contact.h"
#include "mbsim/joint.h"
#include "mbsim/dynamic_system_solver.h"
#include "hdf5serie/fileserie.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#endif

//#ifdef _OPENMP
//#include <omp.h>
//#endif

#include<algorithm>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  DynamicSystem::DynamicSystem(const string &name) : Element(name), frameParent(0), PrPF(Vec3()), APF(SqrMat3(EYE)), q0(0), u0(0), x0(0), qSize(0), qInd(0), xSize(0), xInd(0), gSize(0), gInd(0), gdSize(0), gdInd(0), laSize(0), laInd(0), rFactorSize(0), rFactorInd(0), svSize(0), svInd(0), LinkStatusSize(0), LinkStatusInd(0), LinkStatusRegSize(0), LinkStatusRegInd(0)
#ifdef HAVE_OPENMBVCPPINTERFACE                      
                                                     , openMBVGrp(0), corrInd(0)
#endif
                                                     {
                                                       uSize[0] = 0;
                                                       uSize[1] = 0;
                                                       uInd[0] = 0;
                                                       uInd[1] = 0;
                                                       hSize[0] = 0;
                                                       hSize[1] = 0;
                                                       hInd[0] = 0;
                                                       hInd[1] = 0;

                                                       I=new Frame("I");
                                                       addFrame(I);

                                                       IrOF.push_back(Vec3());
                                                       AIF.push_back(SqrMat3(EYE));
                                                     }

  DynamicSystem::~DynamicSystem() {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      delete *i;
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      delete *i;
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      delete *i;
    for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i != extraDynamic.end(); ++i)
      delete *i;
    for(vector<Frame*>::iterator i = frame.begin(); i != frame.end(); ++i)
      delete *i;
  }

  void DynamicSystem::updateT(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateT(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateT(t);
  }

  void DynamicSystem::updateh(double t, int j) {
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updateh(t,j); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

    for(int i=0; i<(int)object.size(); i++) {
      try { object[i]->updateh(t,j); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

    for(int i=0; i<(int)linkSingleValued.size(); i++) {
      try { linkSingleValued[i]->updateh(t,j); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

    for(int i=0; i<(int)linkSetValuedNotActiveWithSmoothPart.size(); i++) {
      try { linkSetValuedNotActiveWithSmoothPart[i]->updateh(t,j); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }

  void DynamicSystem::updateh0Fromh1(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateh0Fromh1(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateh0Fromh1(t);
  }

  void DynamicSystem::updateW0FromW1(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateW0FromW1(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateW0FromW1(t);
  }

  void DynamicSystem::updateV0FromV1(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateV0FromV1(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateV0FromV1(t);
  }

  void DynamicSystem::updatehInverseKinetics(double t, int j) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatehInverseKinetics(t,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatehInverseKinetics(t,j);
  }

  void DynamicSystem::updateStateDerivativeDependentVariables(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateStateDerivativeDependentVariables(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateStateDerivativeDependentVariables(t);
  }

  void DynamicSystem::updateM(double t, int j) {
    for(int i=0; i<(int)dynamicsystem.size(); i++) {
      try { dynamicsystem[i]->updateM(t,j); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

    for(int i=0; i<(int)object.size(); i++) {
      try { object[i]->updateM(t,j); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }

  void DynamicSystem::updatedq(double t, double dt) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i)
      (**i).updatedq(t,dt);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
      (**i).updatedq(t,dt);
  }

  void DynamicSystem::sethSize(int hSize_, int j) {
    hSize[j] = hSize_;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j),j);
      //(*i)->sethInd((*i)->getuInd(j),j);
    }

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethSize((*i)->getuSize(j),j);
      //(*i)->sethInd((*i)->getuInd(j),j);
    }
  }

  void DynamicSystem::sethInd(int hInd_, int j) {
    hInd[j] = hInd_;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->sethInd((*i)->getuInd(j),j);
    }

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->sethInd((*i)->getuInd(j),j);
    }
  }

  void DynamicSystem::calcqSize() {
    qSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcqSize();
      //(*i)->setqInd(qSize);
      qSize += (*i)->getqSize();
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcqSize();
      //(*i)->setqInd(qSize);
      qSize += (*i)->getqSize();
    }
  }

  void DynamicSystem::setqInd(int qInd_) {
    qInd = qInd_;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->setqInd(qInd_);
      qInd_ += (*i)->getqSize();
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->setqInd(qInd_);
      qInd_ += (*i)->getqSize();
    }
  }

  void DynamicSystem::calcuSize(int j) {
    uSize[j] = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcuSize(j);
 //     (*i)->setuInd(uSize[j],j);
      uSize[j] += (*i)->getuSize(j);
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->calcuSize(j);
  //    (*i)->setuInd(uSize[j],j);
      uSize[j] += (*i)->getuSize(j);
    }
  }

  void DynamicSystem::setuInd(int uInd_, int j) {
    uInd[j] = uInd_;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->setuInd(uInd_,j);
      uInd_ += (*i)->getuSize(j);
    }
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (*i)->setuInd(uInd_,j);
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
  OpenMBV::Group* DynamicSystem::getOpenMBVGrp() { return openMBVGrp; }
#endif

  void DynamicSystem::updatewb(double t, int j) {

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updatewb(t,j);
  }

  void DynamicSystem::updateW(double t, int j) {

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateW(t,j);
  }

  void DynamicSystem::updateJacobiansInverseKinetics(double t, int j) {

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updateJacobians(t,j);
  }

  void DynamicSystem::updateWInverseKinetics(double t, int j) {
    WInverseKinetics[j].init(0);

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updateW(t,j);
  }

  void DynamicSystem::updatebInverseKinetics(double t) {
    bInverseKinetics.init(0);

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i)
      (**i).updateb(t);
  }

  void DynamicSystem::updateV(double t, int j) {

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updateV(t,j);
  }

  void DynamicSystem::updateg(double t) {

    for(int i=0; i<(int)link.size(); i++) {
      try { link[i]->updateg(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }

  void DynamicSystem::updategInverseKinetics(double t) {

    for(int i=0; i<(int)inverseKineticsLink.size(); i++) {
      try { inverseKineticsLink[i]->updateg(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }

  void DynamicSystem::updategd(double t) {

    for(int i=0; i<(int)linkSingleValued.size(); i++) {
      try { linkSingleValued[i]->updategd(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

    for(int i=0; i<(int)linkSetValuedNotActiveWithSmoothPart.size(); i++) { 
      try { linkSetValuedNotActiveWithSmoothPart[i]->updategd(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

    for(int i=0; i<(int)linkSetValued.size(); i++) { 
      try { linkSetValued[i]->updategd(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }

  void DynamicSystem::updategdInverseKinetics(double t) {

    for(int i=0; i<(int)inverseKineticsLink.size(); i++) { 
      try { inverseKineticsLink[i]->updategd(t); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }

  void DynamicSystem::updatedx(double t, double dt) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatedx(t,dt);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatedx(t,dt);

    for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i!= extraDynamic.end(); ++i) 
      (**i).updatedx(t,dt);

  }

  void DynamicSystem::updateqd(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateqd(t);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateqd(t);
  }

  void DynamicSystem::updatexd(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatexd(t);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexd(t);

    for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i!= extraDynamic.end(); ++i) 
      (**i).updatexd(t);
  }

  void DynamicSystem::updateStopVector(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateStopVector(t);
    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (*i)->updateStopVector(t); 
  }  
 
  void DynamicSystem::updateLinkStatus(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateLinkStatus(t);
    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (*i)->updateLinkStatus(t); 
  } 

  void DynamicSystem::updateLinkStatusReg(double t) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateLinkStatusReg(t);
    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) 
      (*i)->updateLinkStatusReg(t); 
  }

  void DynamicSystem::setDynamicSystemSolver(DynamicSystemSolver* sys) {
    Element::setDynamicSystemSolver(sys);
    for(unsigned i=0; i<dynamicsystem.size(); i++)
      dynamicsystem[i]->setDynamicSystemSolver(sys);

    for(unsigned i=0; i<object.size(); i++)
      object[i]->setDynamicSystemSolver(sys);

    for(unsigned i=0; i<link.size(); i++)
      link[i]->setDynamicSystemSolver(sys);

    for (unsigned i=0; i<extraDynamic.size(); i++)
      extraDynamic[i]->setDynamicSystemSolver(sys);
  }

  void DynamicSystem::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      for(unsigned i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->plot(t,dt);
      for(unsigned i=0; i<object.size(); i++)
        object[i]->plot(t,dt);
      for(unsigned i=0; i<link.size(); i++)
        link[i]->plot(t,dt);
      for(unsigned i=0; i<extraDynamic.size(); i++)
        extraDynamic[i]->plot(t,dt);
      for(unsigned i=0; i<frame.size(); i++)
        frame[i]->plot(t,dt);
      for(unsigned i=0; i<contour.size(); i++)
        contour[i]->plot(t,dt);
      for(unsigned i=0; i<inverseKineticsLink.size(); i++)
        inverseKineticsLink[i]->plot(t,dt);
#ifdef HAVE_OPENMBVCPPINTERFACE
      for(unsigned i=0; i<plotElement.size(); i++)
        plotElement[i]->plot(t,dt);
#endif
    }
  }
  
  void DynamicSystem::plotAtSpecialEvent(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      for(unsigned i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->plotAtSpecialEvent(t,dt);
      for(unsigned i=0; i<object.size(); i++)
        object[i]->plotAtSpecialEvent(t,dt);
      for(unsigned i=0; i<link.size(); i++)
        link[i]->plotAtSpecialEvent(t,dt);
      for(unsigned i=0; i<extraDynamic.size(); i++)
        extraDynamic[i]->plotAtSpecialEvent(t,dt);
      for(unsigned i=0; i<frame.size(); i++)
        frame[i]->plotAtSpecialEvent(t,dt);
      for(unsigned i=0; i<inverseKineticsLink.size(); i++)
        inverseKineticsLink[i]->plotAtSpecialEvent(t,dt);
    }
  }

  void DynamicSystem::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      for(unsigned i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->closePlot();
      for(unsigned i=0; i<object.size(); i++)
        object[i]->closePlot();
      for(unsigned i=0; i<link.size(); i++)
        link[i]->closePlot();
      for(unsigned i=0; i<extraDynamic.size(); i++)
        extraDynamic[i]->closePlot();
      for(unsigned i=0; i<frame.size(); i++)
        frame[i]->closePlot();

      if(getPlotFeature(separateFilePerGroup)==enabled)
        delete (H5::FileSerie*)plotGroup;
      else
        delete (H5::Group*)plotGroup;
    }
  }

  void DynamicSystem::init(InitStage stage) {
    if(stage==relativeFrameContourLocation) {
      // This outer loop is nessesary because the frame hierarchy must not be in the correct order!
      for(size_t k=0; k<saved_refFrameF.size(); k++)
        for(size_t j=0; j<saved_refFrameF.size(); j++) {
          int i = 0;
          if(saved_refFrameF[j]!="") i = frameIndex(getFrame(saved_refFrameF[j]));

          IrOF[j+1]=IrOF[i] + AIF[i]*saved_RrRF[j];
          AIF[j+1]=AIF[i]*saved_ARF[j];
        }
      for(size_t j=0; j<saved_refFrameC.size(); j++) {
        int i = 0;
        if(saved_refFrameC[j]!="") i = frameIndex(getFrame(saved_refFrameC[j]));

        IrOC[j]=IrOF[i] + AIF[i]*saved_RrRC[j];
        AIC[j]=AIF[i]*saved_ARC[j];
      }
    }
    else if(stage==worldFrameContourLocation) {
      if(frameParent) {
        I->setPosition(frameParent->getPosition() + frameParent->getOrientation()*PrPF);
        I->setOrientation(frameParent->getOrientation()*APF);
      }
      else {
        DynamicSystem* sys = dynamic_cast<DynamicSystem*>(parent);
        if(sys) {
          I->setPosition(sys->getFrameI()->getPosition() + sys->getFrameI()->getOrientation()*PrPF);
          I->setOrientation(sys->getFrameI()->getOrientation()*APF);
        }
        else {
          I->setPosition(getFrameI()->getPosition() + getFrameI()->getOrientation()*PrPF);
          I->setOrientation(getFrameI()->getOrientation()*APF);
        }
      }
      for(unsigned int i=1; i<frame.size(); i++) { // kinematics of other frames can be updates from frame I 
        frame[i]->setPosition(I->getPosition() + I->getOrientation()*IrOF[i]);
        frame[i]->setOrientation(I->getOrientation()*AIF[i]);
      }
      for(unsigned int i=0; i<contour.size(); i++) { // kinematics of other contours can be updates from frame I
        contour[i]->setReferencePosition(I->getPosition() + I->getOrientation()*IrOC[i]);
        contour[i]->setReferenceOrientation(I->getOrientation()*AIC[i]);
      }
    }
    else if(stage==MBSim::plot) {
      if(parent)
        updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(separateFilePerGroup)==enabled) {
          // create symbolic link in parent plot file if exist
          if(parent) H5Lcreate_external((getPath()+".mbsim.h5").c_str(), "/",
              parent->getPlotGroup()->getId(), name.c_str(),
              H5P_DEFAULT, H5P_DEFAULT);
          // create new plot file (cast needed because of the inadequacy of the HDF5 C++ interface?)
          plotGroup=(H5::Group*)new H5::FileSerie(getPath()+".mbsim.h5", H5F_ACC_TRUNC);
        }
        else
          plotGroup=new H5::Group(parent->getPlotGroup()->createGroup(name));

        H5::SimpleAttribute<string>::setData(*plotGroup, "Description", "Object of class: "+getType());
        plotVectorSerie=NULL;

#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVGrp=new OpenMBV::Group();
        openMBVGrp->setName(name);
        //if(parent) parent->openMBVGrp->addObject(openMBVGrp);
        if(parent) parent->getOpenMBVGrp()->addObject(openMBVGrp);
        if(getPlotFeature(separateFilePerGroup)==enabled)
          openMBVGrp->setSeparateFile(true);
#endif

        plotGroup->flush(H5F_SCOPE_GLOBAL);
      }
    }

    for(unsigned i=0; i<frame.size(); i++)
      frame[i]->init(stage);
    for(unsigned int i=0; i<contour.size(); i++)
      contour[i]->init(stage);
    for(unsigned int i=0; i<dynamicsystem.size(); i++)
      dynamicsystem[i]->init(stage);
    for(unsigned i=0; i<object.size(); i++)
      object[i]->init(stage);
    for(unsigned i=0; i<link.size(); i++)
      link[i]->init(stage);
    for(unsigned i=0; i<extraDynamic.size(); i++)
      extraDynamic[i]->init(stage);
    for(unsigned i=0; i<model.size(); i++)
      model[i]->init(stage);
    for(unsigned i=0; i<inverseKineticsLink.size(); i++)
      inverseKineticsLink[i]->init(stage);
    for(unsigned i=0; i<plotElement.size(); i++)
      plotElement[i]->init(stage);
  }

  int DynamicSystem::solveConstraintsFixpointSingle() {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsFixpointSingle();

    return 0;
  }

  int DynamicSystem::solveImpactsFixpointSingle(double dt) {

    for(int i=0; i<(int)linkSetValuedActive.size(); i++) {
      try { linkSetValuedActive[i]->solveImpactsFixpointSingle(dt); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }

    return 0;
  }

  int DynamicSystem::solveConstraintsGaussSeidel() {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsGaussSeidel();

    return 0;
  }

  int DynamicSystem::solveImpactsGaussSeidel(double dt) {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->solveImpactsGaussSeidel(dt);

    return 0;
  }

  int DynamicSystem::solveConstraintsRootFinding() {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveConstraintsRootFinding();

    return 0;
  }

  int DynamicSystem::solveImpactsRootFinding(double dt) {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->solveImpactsRootFinding(dt);

    return 0;
  }

  int DynamicSystem::jacobianConstraints() {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->jacobianConstraints();

    return 0;
  }

  int DynamicSystem::jacobianImpacts() {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->jacobianImpacts();

    return 0;
  }

  void DynamicSystem::checkConstraintsForTermination() {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkConstraintsForTermination();
  }

  void DynamicSystem::checkImpactsForTermination(double dt) {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (**i).checkImpactsForTermination(dt);
  }

  void DynamicSystem::updaterFactors() {

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterFactors();
  }

  Frame* DynamicSystem::getFrame(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<frame.size(); i++) {
      if(frame[i]->getName() == name)
        return frame[i];
    }
    if(check) {
      if(!(i<frame.size()))
        throw MBSimError("The object \""+name+"\" comprises no frame \""+name+"\"!");
      assert(i<frame.size());
    }
    return NULL;
  }

  Contour* DynamicSystem::getContour(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name)
        return contour[i];
    }
    if(check) {
      if(!(i<contour.size()))
        throw MBSimError("The object \""+name+"\" comprises no contour \""+name+"\"!");
      assert(i<contour.size());
    }
    return NULL;
  }

  void DynamicSystem::updateqRef(const Vec &qParent) {
    q >> qParent(qInd,qInd+qSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateqRef(qParent);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqRef(qParent);
  }

  void DynamicSystem::updateqdRef(const Vec &qdParent) {
    qd >> qdParent(qInd,qInd+qSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateqdRef(qdParent);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqdRef(qdParent);
  }

  void DynamicSystem::updateuRef(const Vec &uParent) {
    u >> uParent(uInd[0],uInd[0]+uSize[0]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateuRef(uParent);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateuRef(uParent);
  }

  void DynamicSystem::updateuallRef(const Vec &uParent) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateuallRef(uParent);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateuallRef(uParent);
  }

  void DynamicSystem::updateudRef(const Vec &udParent, int j) {
    ud[j] >> udParent(uInd[j],uInd[j]+uSize[j]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateudRef(udParent,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateudRef(udParent,j);
  }

  void DynamicSystem::updateudallRef(const Vec &udParent, int j) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updateudallRef(udParent,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateudallRef(udParent,j);
  }

  void DynamicSystem::updatexRef(const Vec &xParent) {
    x >> xParent(xInd,xInd+xSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatexRef(xParent);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef(xParent);

    for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i!= extraDynamic.end(); ++i) 
      (**i).updatexRef(xParent);
  }

  void DynamicSystem::updatexdRef(const Vec &xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatexdRef(xdParent);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexdRef(xdParent);

    for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i!= extraDynamic.end(); ++i) 
      (**i).updatexdRef(xdParent);
  }

  void DynamicSystem::updatehRef(const Vec &hParent, int j) {
    h[j] >> hParent(hInd[j],hInd[j]+hSize[j]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatehRef(hParent,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatehRef(hParent,j);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatehRef(hParent,j);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      if ((**i).hasSmoothPart())
        (**i).updatehRef(hParent,j);
  }

  void DynamicSystem::updatefRef(const Vec &fParent) {
    f >> fParent(xInd,xInd+xSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updatefRef(fParent);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatefRef(fParent);
  }

  void DynamicSystem::updaterRef(const Vec &rParent, int j) {
    r[j] >> rParent(hInd[j],hInd[j]+hSize[j]-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (**i).updaterRef(rParent,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updaterRef(rParent,j);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterRef(rParent,j);
  }

  void DynamicSystem::updateTRef(const Mat& TParent) {
    T >> TParent(Index(qInd,qInd+qSize-1),Index(uInd[0],uInd[0]+uSize[0]-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateTRef(TParent);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateTRef(TParent);
  }

  void DynamicSystem::updateMRef(const SymMat& MParent, int j) {
    M[j] >> MParent(Index(hInd[j],hInd[j]+hSize[j]-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateMRef(MParent,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateMRef(MParent,j);
  }

  void DynamicSystem::updateLLMRef(const SymMat& LLMParent, int j) {
    LLM[j] >> LLMParent(Index(hInd[j],hInd[j]+hSize[j]-1));

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateLLMRef(LLMParent,j);

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateLLMRef(LLMParent,j);
  }

  void DynamicSystem::updategRef(const Vec& gParent) {
    g >> gParent(gInd,gInd+gSize-1);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updategRef(gParent);
  }

  void DynamicSystem::updategdRef(const Vec& gdParent) {
    gd >> gdParent(gdInd,gdInd+gdSize-1);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updategdRef(gdParent);
  }

  void DynamicSystem::updatelaRef(const Vec &laParent) {
    la >> laParent(laInd,laInd+laSize-1);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updatelaRef(laParent);

    for(vector<Link*>::iterator i = linkSetValuedNotActiveWithSmoothPart.begin(); i != linkSetValuedNotActiveWithSmoothPart.end(); ++i) 
      (**i).deletelaRef();
  }

  void DynamicSystem::updatelaInverseKineticsRef(const Vec &laParent) {
    laInverseKinetics >> laParent(0,laInverseKineticsSize-1);

    //for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
    //  (*i)->updatelaRefSpecial(la);

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i) 
      (**i).updatelaRef(laParent);
  }

  void DynamicSystem::updatewbRef(const Vec &wbParent) {
    wb >> wbParent(laInd,laInd+laSize-1);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updatewbRef(wbParent);
  }

  void DynamicSystem::updateWRef(const Mat &WParent, int j) {
    W[j] >> WParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(laInd,laInd+laSize-1));

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updateWRef(WParent,j);
  }

  void DynamicSystem::updateWnVRefObjects() {

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateWnVRefObjects();

    // TODO: Prüfen ob sauber
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) {
      (**i).updateWRef(ds->getW(0),0);
      (**i).updateVRef(ds->getV(0),0);
      (**i).updateWRef(ds->getW(1),1);
      (**i).updateVRef(ds->getV(1),1);
    }
  }

  void DynamicSystem::updateWInverseKineticsRef(const Mat &WParent, int j) {
    WInverseKinetics[j] >> WParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(0,laInverseKineticsSize-1));

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i) 
      (**i).updateWRef(WParent,j);
  }

  void DynamicSystem::updatebInverseKineticsRef(const Mat &bParent) {
    bInverseKinetics >> bParent(Index(0,bInverseKineticsSize-1),Index(0,laInverseKineticsSize-1));

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i) 
      (**i).updatebRef(bParent);
  }

  void DynamicSystem::updateVRef(const Mat &VParent, int j) {
    V[j] >> VParent(Index(hInd[j],hInd[j]+hSize[j]-1),Index(laInd,laInd+laSize-1));

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updateVRef(VParent,j);
  }

  void DynamicSystem::updatesvRef(const Vec &svParent) {
    sv >> svParent(svInd,svInd+svSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatesvRef(svParent);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatesvRef(svParent);
  }

  void DynamicSystem::updatejsvRef(const VecInt &jsvParent) {
    jsv >> jsvParent(svInd,svInd+svSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updatejsvRef(jsvParent);

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatejsvRef(jsvParent);
  }

  void DynamicSystem::updateresRef(const Vec &resParent) {
    res >> resParent(laInd,laInd+laSize-1);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updateresRef(resParent);
  }

  void DynamicSystem::updaterFactorRef(const Vec &rFactorParent) {
    rFactor >> rFactorParent(rFactorInd,rFactorInd+rFactorSize-1);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updaterFactorRef(rFactorParent);
  }

  void DynamicSystem::updateLinkStatusRef(const VecInt &LinkStatusParent) {
    LinkStatus >> LinkStatusParent(LinkStatusInd,LinkStatusInd+LinkStatusSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateLinkStatusRef(LinkStatusParent);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updateLinkStatusRef(LinkStatusParent);
  }

  void DynamicSystem::updateLinkStatusRegRef(const VecInt &LinkStatusRegParent) {
    LinkStatusReg >> LinkStatusRegParent(LinkStatusRegInd,LinkStatusRegInd+LinkStatusRegSize-1);

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      (*i)->updateLinkStatusRegRef(LinkStatusRegParent);

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) 
      (**i).updateLinkStatusRegRef(LinkStatusRegParent);
  }

  void DynamicSystem::initz() {
    for(unsigned i=0; i<dynamicsystem.size(); i++)
      dynamicsystem[i]->initz();
    for(unsigned i=0; i<object.size(); i++)
      object[i]->initz();
    for(unsigned i=0; i<link.size(); i++)
      link[i]->initz();
    for(unsigned i=0; i<extraDynamic.size(); i++)
      extraDynamic[i]->initz();
  }

  void DynamicSystem::buildListOfObjects(vector<Object*> &obj, bool recursive) {
    for(unsigned int i=0; i<object.size(); i++) 
      obj.push_back(object[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfObjects(obj,recursive);
  }

  void DynamicSystem::buildListOfLinks(vector<Link*> &lnk, bool recursive) {
    for(unsigned int i=0; i<link.size(); i++)
      lnk.push_back(link[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfLinks(lnk,recursive);
  }

  void DynamicSystem::buildListOfSetValuedLinks(vector<Link*> &lnk, bool recursive) {
    for(unsigned int i=0; i<link.size(); i++)
      if(link[i]->isSetValued()) lnk.push_back(link[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfSetValuedLinks(lnk,recursive);
  }

  void DynamicSystem::buildListOfFrames(vector<Frame*> &frm, bool recursive) {
    for(unsigned int i=0; i<frame.size(); i++)
      frm.push_back(frame[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfFrames(frm,recursive);
  }

  void DynamicSystem::buildListOfContours(vector<Contour*> &cnt, bool recursive) {
    for(unsigned int i=0; i<contour.size(); i++)
      cnt.push_back(contour[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfContours(cnt,recursive);
  }

  void DynamicSystem::buildListOfExtraDynamic(vector<ExtraDynamic*> &ed, bool recursive) {
    for(unsigned int i=0; i<extraDynamic.size(); i++)
      ed.push_back(extraDynamic[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfExtraDynamic(ed,recursive);
  }

  void DynamicSystem::buildListOfModels(std::vector<ModellingInterface*> &modelList, bool recursive) {
    for(unsigned int i=0; i<model.size(); i++)
      modelList.push_back(model[i]);
    if(recursive)
      for(unsigned int i=0; i<dynamicsystem.size(); i++)
        dynamicsystem[i]->buildListOfModels(modelList,recursive);
  }

  void DynamicSystem::setUpInverseKinetics() {

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->setUpInverseKinetics();
  }

  void DynamicSystem::setUpLinks() {
    for(unsigned int i=0; i<dynamicsystem.size(); i++) 
      dynamicsystem[i]->setUpLinks();

    // clear container first, because setUpLinks in called twice from InitStage resize (before and after the reorganization)
    linkSetValued.clear();
    linkSetValuedActive.clear();
    linkSetValuedNotActiveWithSmoothPart.clear();
    linkSingleValued.clear();
    for(unsigned int i=0; i<link.size(); i++) {
      bool hasForceLaw = false;
      if(link[i]->isSetValued()) {
        hasForceLaw = true;
        linkSetValued.push_back(link[i]);
        linkSetValuedActive.push_back(link[i]);
      }
      if(link[i]->isSingleValued()) {
        hasForceLaw = true;
        linkSingleValued.push_back(link[i]);
      }
      if(not hasForceLaw) {
        throw new MBSimError("The Link \"" + link[i]->getName() +  "\" comprises now force law!");
      }
    }
  }

  bool DynamicSystem::gActiveChanged() {
    bool changed = false;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      if ((*i)->gActiveChanged())
        changed = true;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      if ((*i)->gActiveChanged())
        changed = true;

    return changed;
  }

 bool DynamicSystem::gActiveChangedReg() {
    bool changed = false;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) 
      if ((*i)->gActiveChanged())
        changed = true;

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) 
      if ((*i)->gActiveChanged())
        changed = true;
    
    return changed;
  }

  void DynamicSystem::calcxSize() {
    xSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcxSize();
      xSize += (*i)->getxSize();
    }

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->calcxSize();
      xSize += (*i)->getxSize();
    }

    for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i != extraDynamic.end(); ++i) {
      (*i)->calcxSize();
      xSize += (*i)->getxSize();
    }
  }

 void DynamicSystem::setxInd(int xInd_) {
    xInd = xInd_;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->setxInd(xInd_);
      xInd_ += (*i)->getxSize();
    }
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->setxInd(xInd_);
      xInd_ += (*i)->getxSize();
    }
    for(vector<ExtraDynamic*>::iterator i = extraDynamic.begin(); i != extraDynamic.end(); ++i) {
      (*i)->setxInd(xInd_);
      xInd_ += (*i)->getxSize();
    }
  }

  void DynamicSystem::calcLinkStatusSize() {
    LinkStatusSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcLinkStatusSize();
      (*i)->setLinkStatusInd(LinkStatusSize);
      LinkStatusSize += (*i)->getLinkStatusSize();
    }
    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcLinkStatusSize();
      (*i)->setLinkStatusInd(LinkStatusSize);
      LinkStatusSize += (*i)->getLinkStatusSize();
    }
  }

  void DynamicSystem::calcLinkStatusRegSize() {
    LinkStatusRegSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcLinkStatusRegSize();
      (*i)->setLinkStatusRegInd(LinkStatusRegSize);
      LinkStatusRegSize += (*i)->getLinkStatusRegSize();
    }
    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i) {
      (*i)->calcLinkStatusRegSize();
      (*i)->setLinkStatusRegInd(LinkStatusRegSize);
      LinkStatusRegSize += (*i)->getLinkStatusRegSize();
    }
  }

  void DynamicSystem::calcsvSize() {
    svSize = 0;

    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->calcsvSize();
      (*i)->setsvInd(svSize);
      svSize += (*i)->getsvSize();
    }
    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
      (*i)->calcsvSize();
      (*i)->setsvInd(svSize);
      svSize += (*i)->getsvSize();
    }
  }

  void DynamicSystem::calclaSize(int j) {
    laSize = 0;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calclaSize(j);
      (*i)->setlaInd(laSize);
      laSize += (*i)->getlaSize();
    }
  }

  void DynamicSystem::calclaInverseKineticsSize() {
    laInverseKineticsSize = 0;

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i) {
      (*i)->calclaSize(0);
      (*i)->setlaInd(laInverseKineticsSize);
      laInverseKineticsSize += (*i)->getlaSize();
    }
  }

  void DynamicSystem::calcbInverseKineticsSize() {
    bInverseKineticsSize = 0;

    for(vector<Link*>::iterator i = inverseKineticsLink.begin(); i != inverseKineticsLink.end(); ++i) {
      (*i)->calcbSize();
      (*i)->setbInd(bInverseKineticsSize);
      bInverseKineticsSize += (*i)->getbSize();
    }
//    cout << name << endl;
//    cout << bInverseKineticsSize << endl;
    //throw;
  }

  void DynamicSystem::calcgSize(int j) {
    gSize = 0;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcgSize(j);
      (*i)->setgInd(gSize);
      gSize += (*i)->getgSize();
    }
  }

  void DynamicSystem::calcgdSize(int j) {
    gdSize = 0;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcgdSize(j);
      (*i)->setgdInd(gdSize);
      gdSize += (*i)->getgdSize();
    }
  }

  void DynamicSystem::calcrFactorSize(int j) {
    rFactorSize = 0;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calcrFactorSize(j);
      (*i)->setrFactorInd(rFactorSize);
      rFactorSize += (*i)->getrFactorSize();
    }
  }

  void DynamicSystem::setUpActiveLinks() {

    linkSetValuedActive.clear();
    linkSetValuedNotActiveWithSmoothPart.clear();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      if((*i)->isActive())
        linkSetValuedActive.push_back(*i);
      else if((*i)->hasSmoothPart())
        linkSetValuedNotActiveWithSmoothPart.push_back(*i);
    }
  }

  void DynamicSystem::checkActive(int j) {
    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->checkActive(j);
    setUpActiveLinks();
  }

  void DynamicSystem::checkActiveReg(int j) {
    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (*i)->checkActive(j);
  }

  void DynamicSystem::setgTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setgTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setgTol(tol);
  }

  void DynamicSystem::setgdTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setgdTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setgdTol(tol);
  }

  void DynamicSystem::setgddTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setgddTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setgddTol(tol);
  }

  void DynamicSystem::setlaTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setlaTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setlaTol(tol);
  }

  void DynamicSystem::setLaTol(double tol) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setLaTol(tol);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setLaTol(tol);
  }

  void DynamicSystem::setrMax(double rMax) {
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i!= dynamicsystem.end(); ++i)
      (**i).setrMax(rMax);
    for(vector<Link*>::iterator i = link.begin(); i!= link.end(); ++i)
      (**i).setrMax(rMax);
  }

  void DynamicSystem::addFrame(Frame* cosy) {
    if(getFrame(cosy->getName(),false)) { 
      throw MBSimError("The DynamicSystem \""+name+"\" can only comprises one Frame by the name \""+name+"\"!");
      assert(getFrame(cosy->getName(),false)==NULL);
    }
    frame.push_back(cosy);
    cosy->setParent(this);
  }

  void DynamicSystem::addFrame(Frame* cosy, const Vec3 &RrRF, const SqrMat3 &ARF, const string& refFrameName) {
    addFrame(cosy);

    saved_refFrameF.push_back(refFrameName);
    saved_RrRF.push_back(RrRF); 
    saved_ARF.push_back(ARF); 
    IrOF.push_back(Vec3());
    AIF.push_back(SqrMat3());
  }

  void DynamicSystem::addFrame(Frame *frame_, const fmatvec::Vec3 &RrRF, const fmatvec::SqrMat3 &ARF, const Frame* refFrame) {
    addFrame(frame_, RrRF, ARF, refFrame?refFrame->getName():"I");
  }

  void DynamicSystem::addFrame(const string &str, const Vec3 &RrRF, const SqrMat3 &ARF, const Frame* refFrame) {
    addFrame(new Frame(str),RrRF,ARF,refFrame);
  }

  void DynamicSystem::addContour(Contour* contour_) {
    if(getContour(contour_->getName(),false)) { 
      throw MBSimError("The DynamicSystem \""+name+"\" can only comprise one Contour by the name \""+name+"\"!");
      assert(getContour(contour_->getName(),false)==NULL);
    }
    contour.push_back(contour_);
    contour_->setParent(this);
  }

  void DynamicSystem::addContour(Contour* contour, const Vec3 &RrRC, const SqrMat3 &ARC, const string& refFrameName) {
    addContour(contour);

    saved_refFrameC.push_back(refFrameName);
    saved_RrRC.push_back(RrRC); 
    saved_ARC.push_back(ARC); 
    IrOC.push_back(Vec3());
    AIC.push_back(SqrMat3());
  }

  void DynamicSystem::addContour(Contour* contour, const fmatvec::Vec3 &RrRC, const fmatvec::SqrMat3 &ARC, const Frame* refFrame) {
    addContour(contour, RrRC, ARC, refFrame?refFrame->getName():"I");
  }

  int DynamicSystem::frameIndex(const Frame *frame_) const {
    for(unsigned int i=0; i<frame.size(); i++) {
      if(frame_==frame[i])
        return i;
    }
    return -1;
  }

  DynamicSystem* DynamicSystem::getGroup(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<dynamicsystem.size(); i++) {
      if(dynamicsystem[i]->getName() == name)
        return dynamicsystem[i];
    }
    if(check){
      if(!(i<dynamicsystem.size()))
        throw MBSimError("The DynamicSystem \""+this->name+"\" comprises no DynamicSystem \""+name+"\"!");
      assert(i<dynamicsystem.size());
    }
    return NULL;
  }

  Object* DynamicSystem::getObject(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<object.size(); i++) {
      if(object[i]->getName() == name)
        return object[i];
    }
    if(check){
      if(!(i<object.size()))
        throw MBSimError("The DynamicSystem \""+this->name+"\" comprises no Object \""+name+"\"!");
      assert(i<object.size());
    }
    return NULL;
  }

  void DynamicSystem::addLink(Link *lnk) {
    if(getLink(lnk->getName(),false)) {
      throw MBSimError("The DynamicSystem \""+name+"\" can only comprise one Link by the name \""+lnk->getName()+"\"!");
      assert(getLink(lnk->getName(),false) == NULL);
    }

    link.push_back(lnk);
    lnk->setParent(this);
  }

  void DynamicSystem::addInverseKineticsLink(Link *lnk) {
    //if(getLink(lnk->getName(),false)) {
    //  cout << "ERROR (DynamicSystem: addLink): The DynamicSystem " << name << " can only comprise one Link by the name " <<  lnk->getName() << "!" << endl;
    //  assert(getLink(lnk->getName(),false) == NULL);
    //}
    inverseKineticsLink.push_back(lnk);
    lnk->setParent(this);
  }

  Link* DynamicSystem::getLink(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<link.size(); i++) {
      if(link[i]->getName() == name)
        return link[i];
    }
    if(check){
      if(!(i<link.size()))
        throw MBSimError("The DynamicSystem \""+this->name+"\" comprises no Link \""+name+"\"!");
      assert(i<link.size());
    }
    return NULL;
  }

  void DynamicSystem::addExtraDynamic(ExtraDynamic *ed_) {
    if(getExtraDynamic(ed_->getName(),false)) {
        throw MBSimError("The DynamicSystem \""+name+"\" can only comprise one ExtraDynamic by the name \""+ed_->getName()+"\"!");
      assert(getExtraDynamic(ed_->getName(),false) == NULL);
    }
    extraDynamic.push_back(ed_);
    ed_->setParent(this);
  }

  ExtraDynamic* DynamicSystem::getExtraDynamic(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<extraDynamic.size(); i++) {
      if(extraDynamic[i]->getName() == name)
        return extraDynamic[i];
    }
    if(check){
      if(!(i<extraDynamic.size()))
        throw MBSimError("The DynamicSystem \""+this->name+"\" comprises no ExtraDynamic \""+name+"\"!");
      assert(i<extraDynamic.size());
    }
    return NULL;
  }

  void DynamicSystem::addModel(ModellingInterface *model_) {
    if(getModel(model_->getName(),false)) {
      throw MBSimError("The DynamicSystem \""+name+"\" can only comprise one model by the name \""+model_->getName()+"\"!");
      assert(getModel(model_->getName(),false) == NULL); 
    }
    model.push_back(model_);
    model_->setParent(this);
  }

  ModellingInterface* DynamicSystem::getModel(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<model.size(); i++) {
      if(model[i]->getName() == name)
        return model[i];
    }
    if(check){
      if(!(i<model.size()))
        throw MBSimError("The DynamicSystem \""+this->name+"\" comprises no model \""+name+"\"!");
      assert(i<model.size());
    }
    return NULL;
  }

  void DynamicSystem::addGroup(DynamicSystem *sys) {
    if(getGroup(sys->getName(),false)) {
      throw MBSimError("The DynamicSystem \""+name+"\" can only comprise one DynamicSystem by the name \""+sys->getName()+"\"!");
      assert(getGroup(sys->getName(),false) == NULL); 
    }
    dynamicsystem.push_back(sys);
    sys->setParent(this);
  }

  void DynamicSystem::addObject(Object *obj) {
    if(getObject(obj->getName(),false)) {
      throw MBSimError("The DynamicSystem \""+name+"\" can only comprise one Object by the name \""+obj->getName()+"\"!");
      assert(getObject(obj->getName(),false) == NULL); 
    }
    object.push_back(obj);
    obj->setParent(this);
  }

  Element * DynamicSystem::getByPathSearch(string path) {
    if (path.substr(0, 1)=="/") { // absolut path
      if(parent)
        return parent->getByPathSearch(path);
      else
        return getByPathSearch(path.substr(1));
    }
    else if (path.substr(0, 3)=="../") // relative path
      return parent->getByPathSearch(path.substr(3));
    else { // local path
      size_t pos0=path.find_first_of("[");
      string container=path.substr(0, pos0);
      size_t pos1=path.find_first_of("]", pos0);
      string searched_name=path.substr(pos0+1, pos1-pos0-1);
      if(path.length()>pos1+1) { // weiter absteigen
        string rest=path.substr(pos1+2);
        if (container=="Object")
          return getObject(searched_name)->getByPathSearch(rest);
        else if (container=="Link")
          return getLink(searched_name)->getByPathSearch(rest);
        else if (container=="ExtraDynamic")
          return getExtraDynamic(searched_name)->getByPathSearch(rest);
        else if (container=="Group")
          return getGroup(searched_name)->getByPathSearch(rest);
        else
          throw MBSimError("Unknown name of container");
      }
      else {
        if (container=="Object")
          return getObject(searched_name);
        else if (container=="Link")
          return getLink(searched_name);
        else if (container=="ExtraDynamic")
          return getExtraDynamic(searched_name);
        else if (container=="Group")
          return getGroup(searched_name);
        else if (container=="Frame")
          return getFrame(searched_name);
        else if (container=="Contour")
          return getContour(searched_name);
        else
          throw MBSimError("Unknown name of container");
      }
    }
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  Element* DynamicSystem::getOpenMBVElement(const string &name, bool check) {
    unsigned int i;
    for(i=0; i<plotElement.size(); i++) {
      if(plotElement[i]->getName() == name)
        return plotElement[i];
    }
    if(check){
      if(!(i<plotElement.size()))
        throw MBSimError("The DynamicSystem \""+this->name+"\" comprises no OpenMBVElement \""+name+"\"!");
      assert(i<plotElement.size());
    }
    return NULL;
  }

  void DynamicSystem::addPlotElement(Element *ele) {
    if(getOpenMBVElement(ele->getName(),false)) {
      throw MBSimError("The DynamicSystem \""+name+"\" can only comprise one OpenMBVElement by the name \""+ele->getName()+"\"!");
      assert(getOpenMBVElement(ele->getName(),false) == NULL); 
    }
    plotElement.push_back(ele);
    ele->setParent(this);
  }
#endif

  void DynamicSystem::updatecorr(int j) {

    for(int i=0; i<(int)linkSetValued.size(); i++) {
      try { linkSetValued[i]->updatecorr(j); }
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
  }
  
  void DynamicSystem::updatecorrRef(const fmatvec::Vec &ref) {
    corr >> ref(corrInd,corrInd+corrSize-1);

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) 
      (**i).updatecorrRef(ref);
  }

  void DynamicSystem::calccorrSize(int j) {
    corrSize = 0;

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
      (*i)->calccorrSize(j);
      (*i)->setcorrInd(corrSize);
      corrSize += (*i)->getcorrSize();
    }
  }

  void DynamicSystem::checkRoot() {

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (*i)->checkRoot();
  }
}

