/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/link_mechanics.h"
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

namespace MBSim {

  LinkMechanics::LinkMechanics(const std::string &name) : Link(name) {
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVForceGrp=0;
#endif
  }

  LinkMechanics::~LinkMechanics() {}

  void LinkMechanics::updatedhdz(double t) {
    /***************************** FRAMES ***************************************/
    vector<Vec> hLink0, h0;
    for(unsigned int i=0; i<frame.size(); i++) { // save old values
      hLink0.push_back(hLink[i].copy()); 
      h0.push_back(h[i].copy());
    }
    if(frame.size()) updateh(t);
    vector<Vec> hLinkEnd, hEnd;
    for(unsigned int i=0; i<frame.size(); i++) { // save with correct state
      hLinkEnd.push_back(hLink[i].copy());
      hEnd.push_back(h[i].copy());
    }

    /****************** velocity dependent calculations ***********************/
    for(unsigned int i=0; i<frame.size(); i++) 
      for(unsigned int l=0; l<frame.size(); l++) {
        for(int j=0; j<frame[l]->getParent()->getuSize(); j++) {
          hLink[i] = hLink0[i].copy(); // set to old values
          h[i] = h0[i].copy();

          double uParentj = frame[l]->getParent()->getu()(j); // save correct position

          frame[l]->getParent()->getu()(j) += epsroot(); // update with disturbed positions assuming same active links
          frame[l]->getParent()->updateStateDependentVariables(t); 
          updategd(t);
          updateh(t);

          dhdu[i*frame.size()+l].col(j) += (hLink[i]-hLinkEnd[i])/epsroot();
          frame[l]->getParent()->getu()(j) = uParentj;
          frame[l]->getParent()->updateStateDependentVariables(t); 
        }
      }

    /****************** position dependent calculations ***********************/
    for(unsigned int i=0; i<frame.size(); i++) 
      for(unsigned int l=0; l<frame.size(); l++) {
        for(int j=0; j<frame[l]->getParent()->getq().size(); j++) {
          hLink[i] = hLink0[i].copy(); // set to old values
          h[i] = h0[i].copy();

          double qParentj = frame[l]->getParent()->getq()(j); // save correct position

          frame[l]->getParent()->getq()(j) += epsroot(); // update with disturbed positions assuming same active links
          frame[l]->getParent()->updateStateDependentVariables(t); 
          updateg(t);
          updategd(t);
          frame[l]->getParent()->updateT(t); 
          updateJacobians(t);
          updateh(t);

          dhdq[i*frame.size()+l].col(j) += (hLink[i]-hLinkEnd[i])/epsroot();
          frame[l]->getParent()->getq()(j) = qParentj;
          frame[l]->getParent()->updateStateDependentVariables(t); 
          frame[l]->getParent()->updateT(t); 
        }
      }

    /******************** time dependent calculations ***************************/
    // for(unsigned int i=0; i<frame.size(); i++) 
    //   for(unsigned int l=0; l<frame.size(); l++) {
    //     hLink[i] = hLink0[i].copy(); // set to old values
    //     h[i] = h0[i].copy();

    //     double t0 = t; // save correct position

    //     t += epsroot(); // update with disturbed positions assuming same active links
    //     frame[i]->getParent()->updateStateDependentVariables(t); 
    //     updateg(t);
    //     updategd(t);
    //     frame[i]->getParent()->updateT(t); 
    //     updateJacobians(t);
    //     updateh(t);

    //     dhdt[i] += (hLink[i]-hLinkEnd[i])/epsroot();
    //     t = t0;
    //   }

    /************************ back to initial state ******************************/
    for(unsigned int i=0; i<frame.size(); i++) {
      frame[i]->getParent()->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      frame[i]->getParent()->updateT(t); 
      updateJacobians(t);
      hLink[i] = hLinkEnd[i].copy();
      h[i] = hEnd[i].copy();
    }

    /**************************** CONTOURES ****************************************/
    for(unsigned int i=0; i<contour.size(); i++) { // save old values
      hLink0.push_back(hLink[frame.size()+i].copy());
      h0.push_back(h[frame.size()+i].copy());
    }
    if(contour.size()) updateh(t); 
    for(unsigned int i=0; i<contour.size(); i++) { // save with correct state
      hLinkEnd.push_back(hLink[frame.size()+i].copy());
      hEnd.push_back(h[frame.size()+i].copy());
    }

    /****************** velocity dependent calculations *************************/
    for(unsigned int i=0; i<contour.size(); i++) 
      for(unsigned int l=0; l<contour.size(); l++) { 
        for(int j=0; j<contour[l]->getParent()->getuSize(); j++) {
          hLink[frame.size()+i] = hLink0[frame.size()+i].copy(); // set to old values
          h[frame.size()+i] = h0[frame.size()+i].copy();

          double uParentj = contour[l]->getParent()->getu()(j); // save correct position

          contour[l]->getParent()->getu()(j) += epsroot(); // update with disturbed positions assuming same active links
          contour[l]->getParent()->updateStateDependentVariables(t); 
          updategd(t);
          updateh(t);

          dhdu[frame.size()*frame.size()+i*contour.size()+l].col(j) += (hLink[frame.size()+i]-hLinkEnd[frame.size()+i])/epsroot();
          contour[l]->getParent()->getu()(j) = uParentj;
          contour[l]->getParent()->updateStateDependentVariables(t); 
        }
      }

    /****************** position dependent calculations *************************/
    for(unsigned int i=0; i<contour.size(); i++) 
      for(unsigned int l=0; l<contour.size(); l++) { 
        for(int j=0; j<contour[l]->getParent()->getq().size(); j++) {
          hLink[frame.size()+i] = hLink0[frame.size()+i].copy(); // set to old values
          h[frame.size()+i] = h0[frame.size()+i].copy();

          double qParentj = contour[l]->getParent()->getq()(j); // save correct position

          contour[l]->getParent()->getq()(j) += epsroot(); // update with disturbed positions assuming same active links
          contour[l]->getParent()->updateStateDependentVariables(t); 
          updateg(t);
          updategd(t);
          contour[l]->getParent()->updateT(t); 
          updateJacobians(t);
          updateh(t);
          dhdq[frame.size()*frame.size()+i*contour.size()+l].col(j) += (hLink[frame.size()+i]-hLinkEnd[frame.size()+i])/epsroot();
          contour[l]->getParent()->getq()(j) = qParentj;
          contour[l]->getParent()->updateStateDependentVariables(t); 
          contour[l]->getParent()->updateT(t); 
        }
      }

    /******************** time dependent calculations ***************************/
    // for(unsigned int i=0; i<contour.size(); i++) 
    //   for(unsigned int l=0; l<contour.size(); l++) { 
    //     hLink[frame.size()+i] = hLink0[frame.size()+i].copy(); // set to old values
    //     h[frame.size()+i] = h0[frame.size()+i].copy();

    //     double t0 = t; // save correct position

    //     t += epsroot(); // update with disturbed positions assuming same active links
    //     contour[i]->getParent()->updateStateDependentVariables(t); 
    //     updateg(t);
    //     updategd(t);
    //     contour[i]->getParent()->updateT(t); 
    //     updateJacobians(t);
    //     updateh(t);

    //     dhdt[i] += (hLink[frame.size()+i]-hLinkEnd[frame.size()+i])/epsroot();
    //     t = t0;
    //   }

    /************************ back to initial state ******************************/
    for(unsigned int i=0; i<contour.size(); i++) {
      contour[i]->getParent()->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      contour[i]->getParent()->updateT(t); 
      updateJacobians(t);
      hLink[frame.size()+i] = hLinkEnd[frame.size()+i].copy();
      h[frame.size()+i] = hEnd[frame.size()+i].copy();
    }
  }

  void LinkMechanics::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      for(unsigned int i=0; i<openMBVArrowF.size(); i++) {
        if(openMBVArrowF[i]) {
          vector<double> data;
          data.push_back(t); 
          Vec toPoint=frame[i]->getPosition();
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
          Vec toPoint=frame[i]->getPosition();
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

  void LinkMechanics::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void LinkMechanics::updateWRef(const Mat& WParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->getParent()->gethInd(parent,j),frame[i]->getParent()->gethInd(parent,j)+frame[i]->getJacobianOfTranslation().cols()-1);
      W[i].resize()>>WParent(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->getParent()->gethInd(parent,j),contour[i]->getParent()->gethInd(parent,j)+contour[i]->gethSize(j)-1);
      W[i]>>WParent(I,J);
    }
  } 

  void LinkMechanics::updateVRef(const Mat& VParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->getParent()->gethInd(parent,j),frame[i]->getParent()->gethInd(parent,j)+frame[i]->getJacobianOfTranslation().cols()-1);
      V[i].resize()>>VParent(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->getParent()->gethInd(parent,j),contour[i]->getParent()->gethInd(parent,j)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      V[i]>>VParent(I,J);
    }
  } 

  void LinkMechanics::updatehRef(const Vec &hParent, const Vec &hLinkParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->getParent()->gethInd(parent,j),frame[i]->getParent()->gethInd(parent,j)+frame[i]->getJacobianOfTranslation().cols()-1);
      h[i].resize()>>hParent(I);
      hLink[i].resize()>>hLinkParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->getParent()->gethInd(parent,j),contour[i]->getParent()->gethInd(parent,j)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      h[i].resize()>>hParent(I);
      hLink[i].resize()>>hLinkParent(I);
    }
  } 

  void LinkMechanics::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    for(unsigned int i=0; i<frame.size(); i++) 
      for(unsigned int j=0; j<frame.size(); j++) {
        Index I = Index(frame[i]->getParent()->gethInd(parent,k),frame[i]->getParent()->gethInd(parent,k)+frame[i]->getJacobianOfTranslation().cols()-1);
        Index J = Index(frame[j]->getParent()->getqInd(parent),frame[j]->getParent()->getqInd(parent)+frame[j]->getParent()->getqSize()-1);
        dhdq[i*frame.size()+j]>>dhdqParent(I,J);
      }
    for(unsigned int i=0; i<contour.size(); i++) 
      for(unsigned int j=0; j<contour.size(); j++) {
        Index I = Index(contour[i]->getParent()->gethInd(parent,k),contour[i]->getParent()->gethInd(parent,k)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
        Index J = Index(contour[j]->getParent()->getqInd(parent),contour[j]->getParent()->getqInd(parent)+contour[j]->getParent()->getqSize()-1);
        dhdq[frame.size()*frame.size()+i*contour.size()+j]>>dhdqParent(I,J);
      }
  }

  void LinkMechanics::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    for(unsigned int i=0; i<frame.size(); i++)
      for(unsigned int j=0; j<frame.size(); j++) {
        Index I = Index(frame[i]->getParent()->gethInd(parent,k),frame[i]->getParent()->gethInd(parent,k)+frame[i]->getJacobianOfTranslation().cols()-1);
        Index J = Index(frame[j]->getParent()->getuInd(parent,k),frame[j]->getParent()->getuInd(parent,k)+frame[j]->getParent()->getuSize()-1);
        dhdu[i*frame.size()+j]>>dhduParent(I,J);
      }
    for(unsigned int i=0; i<contour.size(); i++)
      for(unsigned int j=0; j<contour.size(); j++) {
        Index I = Index(contour[i]->getParent()->gethInd(parent,k),contour[i]->getParent()->gethInd(parent,k)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
        Index J = Index(contour[j]->getParent()->getuInd(parent,k),contour[j]->getParent()->getuInd(parent,k)+contour[j]->getParent()->getuSize()-1);
        dhdu[frame.size()*frame.size()+i*contour.size()+j]>>dhduParent(I,J);
      }
  }

  void LinkMechanics::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->getParent()->gethInd(parent,j),frame[i]->getParent()->gethInd(parent,j)+frame[i]->getJacobianOfTranslation().cols()-1);
      dhdt[i]>>dhdtParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->getParent()->gethInd(parent,j),contour[i]->getParent()->gethInd(parent,j)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void LinkMechanics::updaterRef(const Vec &rParent, int j) {
  for(unsigned i=0; i<frame.size(); i++) {
      int hInd =  frame[i]->getParent()->gethInd(parent,j);
      Index I = Index(hInd,hInd+frame[i]->getJacobianOfTranslation().cols()-1);
      r[i].resize()>>rParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      int hInd =  contour[i]->getParent()->gethInd(parent,j);
      Index I = Index(hInd,hInd+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      r[i].resize()>>rParent(I);
    }
  } 

  void LinkMechanics::init(InitStage stage) {
    if(stage==unknownStage) {
      Link::init(stage);

      for(unsigned int i=0; i<frame.size(); i++) {
        W.push_back(Mat(frame[i]->getJacobianOfTranslation().cols(),laSize));
        V.push_back(Mat(frame[i]->getJacobianOfTranslation().cols(),laSize));
        h.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
        hLink.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
        for(unsigned int j=0; j<frame.size(); j++) {
          dhdq.push_back(Mat(frame[i]->getJacobianOfTranslation().cols(),frame[j]->getParent()->getqSize()));
          dhdu.push_back(Mat(frame[i]->getJacobianOfTranslation().cols(),frame[j]->getParent()->getuSize()));
        }
        dhdt.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
        r.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
        WF.push_back(Vec(3));
        WM.push_back(Vec(3));
        fF.push_back(Mat(3,laSize));
        fM.push_back(Mat(3,laSize));
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      assert(openMBVArrowF.size()==0 || openMBVArrowF.size()==frame.size());
      assert(openMBVArrowM.size()==0 || openMBVArrowM.size()==frame.size());
#endif

      for(unsigned int i=0; i<contour.size(); i++) {
        W.push_back(Mat(contour[i]->getReferenceJacobianOfTranslation().cols(),laSize));
        V.push_back(Mat(contour[i]->getReferenceJacobianOfTranslation().cols(),laSize));
        h.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
        hLink.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
        for(unsigned int j=0; j<contour.size(); j++) {
          dhdq.push_back(Mat(contour[i]->getReferenceJacobianOfTranslation().cols(),contour[j]->getParent()->getqSize()));
          dhdu.push_back(Mat(contour[i]->getReferenceJacobianOfTranslation().cols(),contour[j]->getParent()->getuSize()));
        }
        dhdt.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
        r.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
        WF.push_back(Vec(3));
        WM.push_back(Vec(3));
        fF.push_back(Mat(3,laSize));
        fM.push_back(Mat(3,laSize));
      }
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVForceGrp=new OpenMBV::Group;
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

  void LinkMechanics::connect(Frame *frame_) {
    frame.push_back(frame_);
  }

  void LinkMechanics::connect(Contour *contour_) {
    contour.push_back(contour_);
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  void LinkMechanics::setOpenMBVForceArrow(OpenMBV::Arrow *arrow, const vector<bool>& which) {
    for(unsigned int i=0; i<which.size(); i++) {
      if(which[i]==true)
        openMBVArrowF.push_back(new OpenMBV::Arrow(*arrow));
      else
        openMBVArrowF.push_back(NULL);
    }
  }

  void LinkMechanics::setOpenMBVMomentArrow(OpenMBV::Arrow *arrow, const vector<bool>& which) {
    for(unsigned int i=0; i<which.size(); i++) {
      if(which[i]==true)
        openMBVArrowM.push_back(new OpenMBV::Arrow(*arrow));
      else
        openMBVArrowM.push_back(NULL);
    }
  }
#endif

}

