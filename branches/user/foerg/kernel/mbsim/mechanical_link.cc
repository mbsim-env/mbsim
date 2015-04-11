/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/mechanical_link.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/contour.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>
#endif

using namespace fmatvec;
using namespace std;
using namespace boost;

namespace MBSim {

  MechanicalLink::MechanicalLink(const std::string &name) : Link(name) {
  }

  MechanicalLink::~MechanicalLink() {}

  void MechanicalLink::updatedhdz(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  void MechanicalLink::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      for(unsigned int i=0; i<openMBVArrowF.size(); i++) {
        if(openMBVArrowF[i]) {
          vector<double> data;
          data.push_back(t); 
          Vec3 toPoint=frame[i]->getPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          data.push_back(WF[i](0));
          data.push_back(WF[i](1));
          data.push_back(WF[i](2));
          data.push_back(nrm2(WF[i]));
          openMBVArrowF[i]->append(data);
        }
      }
      for(unsigned int i=0; i<openMBVArrowM.size(); i++) {
        if(openMBVArrowM[i]) {
          vector<double> data;
          data.push_back(t); 
          Vec3 toPoint=frame[i]->getPosition();
          data.push_back(toPoint(0));
          data.push_back(toPoint(1));
          data.push_back(toPoint(2));
          data.push_back(WM[i](0));
          data.push_back(WM[i](1));
          data.push_back(WM[i](2));
          data.push_back(nrm2(WM[i]));
          openMBVArrowM[i]->append(data);
        }
      }
#endif
      Link::plot(t,dt);
    }
  }

  void MechanicalLink::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void MechanicalLink::updateWRef(const Mat& WParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize
      W[j][i]>>WParent(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->gethSize(j)-1);
      W[j][i]>>WParent(I,J);
    }
  } 

  void MechanicalLink::updateVRef(const Mat& VParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1);
      V[j][i]>>VParent(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      V[j][i]>>VParent(I,J);
    }
  } 

  void MechanicalLink::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1);
      h[j][i]>>hParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      h[j][i]>>hParent(I);
    }
  } 

  void MechanicalLink::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void MechanicalLink::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void MechanicalLink::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1);
      dhdt[i]>>dhdtParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void MechanicalLink::updaterRef(const Vec &rParent, int j) {
  for(unsigned i=0; i<frame.size(); i++) {
      int hInd =  frame[i]->gethInd(j);
      Index I = Index(hInd,hInd+frame[i]->getJacobianOfTranslation(j).cols()-1);
      r[j][i]>>rParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      int hInd =  contour[i]->gethInd(j);
      Index I = Index(hInd,hInd+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      r[j][i]>>rParent(I);
    }
  } 

  void MechanicalLink::init(InitStage stage) {
//    if(stage==preInit) {
//      for(unsigned int i=0; i<frame.size(); i++)
//        addDependencies(frame[i]->getDependencies());
//      for(unsigned int i=0; i<contour.size(); i++)
//        addDependencies(contour[i]->getDependencies());
//    }
    if(stage==unknownStage) {
      Link::init(stage);

      for(unsigned int i=0; i<frame.size(); i++) {
        W[0].push_back(Mat(0,0,NONINIT));
        V[0].push_back(Mat(0,0,NONINIT));
        h[0].push_back(Vec(0,NONINIT));
        W[1].push_back(Mat(0,0,NONINIT));
        V[1].push_back(Mat(0,0,NONINIT));
        h[1].push_back(Vec(0,NONINIT));
        r[0].push_back(Vec(0,NONINIT));
        r[1].push_back(Vec(0,NONINIT));
        WF.push_back(Vec3());
        WM.push_back(Vec3());
        fF.push_back(Mat3xV(laSize));
        fM.push_back(Mat3xV(laSize));
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      assert(openMBVArrowF.size()==0 || openMBVArrowF.size()==frame.size());
      assert(openMBVArrowM.size()==0 || openMBVArrowM.size()==frame.size());
#endif

      for(unsigned int i=0; i<contour.size(); i++) {
        W[0].push_back(Mat(0,0,NONINIT));
        V[0].push_back(Mat(0,0,NONINIT));
        h[0].push_back(Vec(0,NONINIT));
        W[1].push_back(Mat(0,0,NONINIT));
        V[1].push_back(Mat(0,0,NONINIT));
        h[1].push_back(Vec(0,NONINIT));
        r[0].push_back(Vec(0,NONINIT));
        r[1].push_back(Vec(0,NONINIT));
        WF.push_back(Vec3());
        WM.push_back(Vec3());
        fF.push_back(Mat3xV(laSize));
        fM.push_back(Mat3xV(laSize));
      }
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVForceGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
        openMBVForceGrp->setExpand(false);
        openMBVForceGrp->setName(name+"_ArrowGroup");
        parent->getOpenMBVGrp()->addObject(openMBVForceGrp);
        for(unsigned int i=0; i<openMBVArrowF.size(); i++) {
          if(openMBVArrowF[i]) {
            openMBVArrowF[i]->setName("Force_"+numtostr((int)i));
            openMBVForceGrp->addObject(openMBVArrowF[i]);
          }
        }
        for(unsigned int i=0; i<openMBVArrowM.size(); i++) {
          if(openMBVArrowM[i]) {
            openMBVArrowM[i]->setName("Moment_"+numtostr((int)i));
            openMBVForceGrp->addObject(openMBVArrowM[i]);
          }
        }
#endif
        Link::init(stage);
      }
    }
    else
      Link::init(stage);
  }

  void MechanicalLink::connect(Frame *frame_) {
    frame.push_back(frame_);
  }

  void MechanicalLink::connect(Contour *contour_) {
    contour.push_back(contour_);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void MechanicalLink::setOpenMBVForceArrow(const shared_ptr<OpenMBV::Arrow> &arrow, const vector<bool>& which) {
    bool flag = true;
    for(unsigned int i=0; i<which.size(); i++) {
      if(which[i]==true) {
        if(flag) {
          openMBVArrowF.push_back(arrow);
          flag = false;
        }
        else 
          openMBVArrowF.push_back(OpenMBV::ObjectFactory::create(arrow));
      }
      else
        openMBVArrowF.push_back(shared_ptr<OpenMBV::Arrow>());
    }
  }

  void MechanicalLink::setOpenMBVMomentArrow(const shared_ptr<OpenMBV::Arrow> &arrow, const vector<bool>& which) {
    bool flag = true;
    for(unsigned int i=0; i<which.size(); i++) {
      if(which[i]==true) {
        if(flag) {
          openMBVArrowM.push_back(arrow);
          flag = false;
        }
        else
          openMBVArrowM.push_back(OpenMBV::ObjectFactory::create(arrow));
      }
      else
        openMBVArrowM.push_back(shared_ptr<OpenMBV::Arrow>());
    }
  }
#endif

}

