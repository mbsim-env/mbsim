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
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/link_mechanics.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/contour.h"
#include "mbsim/utils/eps.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  LinkMechanics::~LinkMechanics() {}

  void LinkMechanics::updatedhdz(double t) {
    /***************************** frames ***************************************/
    for(unsigned int i=0; i<frame.size(); i++) {
      Vec hLink0 = hLink[i].copy(); // save old values
      Vec h0 = h[i].copy();

      updateh(t); // update with correct state
      Vec hLinkEnd = hLink[i].copy();
      Vec hEnd = h[i].copy();

      /****************** velocity dependent calculations ***********************/
      for(int j=0; j<frame[i]->getJacobianOfTranslation().cols(); j++) {
        hLink[i] = hLink0; // set to old values
        h[i] = h0;

        double uParentj = frame[i]->getParent()->getu()(j); // save correct position

        frame[i]->getParent()->getu()(j) += epsroot(); // update with disturbed positions assuming same active links
        frame[i]->getParent()->updateStateDependentVariables(t); 
        updategd(t);
        updateh(t);

        dhdu[i].col(j) += (hLink[i]-hLinkEnd)/epsroot();
        frame[i]->getParent()->getu()(j) = uParentj;
      }

      /****************** position dependent calculations ***********************/
      if(frame[i]->getJacobianOfTranslation().cols() > 0) { // Jacobian using disturbed positions
        for(int j=0; j<frame[i]->getParent()->getq().size(); j++) {
          hLink[i] = hLink0; // set to old values
          h[i] = h0;

          double qParentj = frame[i]->getParent()->getq()(j); // save correct position

          frame[i]->getParent()->getq()(j) += epsroot(); // update with disturbed positions assuming same active links
          frame[i]->getParent()->updateStateDependentVariables(t); 
          updateg(t);
          updategd(t);
          frame[i]->getParent()->updateT(t); 
          updateJacobians(t);
          updateh(t);

          dhdq[i].col(j) += (hLink[i]-hLinkEnd)/epsroot();
          frame[i]->getParent()->getq()(j) = qParentj;
        }
      }

      /******************** time dependent calculations ***************************/
      hLink[i] = hLink0; // set to old values
      h[i] = h0;

      double t0 = t; // save correct position

      t += epsroot(); // update with disturbed positions assuming same active links
      frame[i]->getParent()->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      frame[i]->getParent()->updateT(t); 
      updateJacobians(t);
      updateh(t);

      dhdt[i] += (hLink[i]-hLinkEnd)/epsroot();
      t = t0;

      /************************ back to initial state ******************************/
      frame[i]->getParent()->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      frame[i]->getParent()->updateT(t); 
      updateJacobians(t);
      hLink[i] = hLinkEnd;
      h[i] = hEnd;
    }

    /**************************** contoures ****************************************/
    for(unsigned int i=0; i<contour.size(); i++) {
      Vec hLink0 = hLink[frame.size()+i].copy(); // save old values
      Vec h0 = h[frame.size()+i].copy();

      updateh(t); // update with correct state
      Vec hLinkEnd = hLink[frame.size()+i].copy();
      Vec hEnd = h[frame.size()+i].copy();

      /****************** velocity dependent calculations *************************/
      for(int j=0; j<contour[i]->getReferenceJacobianOfTranslation().cols(); j++) {
        hLink[frame.size()+i] = hLink0; // set to old values
        h[frame.size()+i] = h0;

        double uParentj = contour[i]->getParent()->getu()(j); // save correct position

        contour[i]->getParent()->getu()(j) += epsroot(); // update with disturbed positions assuming same active links
        contour[i]->getParent()->updateStateDependentVariables(t); 
        updategd(t);
        updateh(t);

        dhdu[frame.size()+i].col(j) += (hLink[frame.size()+i]-hLinkEnd)/epsroot();
        contour[i]->getParent()->getu()(j) = uParentj;
      }

      /****************** position dependent calculations *************************/
      if(contour[i]->getReferenceJacobianOfTranslation().cols() > 0) {
        for(int j=0; j<contour[i]->getParent()->getq().size(); j++) {
          hLink[frame.size()+i] = hLink0; // set to old values
          h[frame.size()+i] = h0;

          double qParentj = contour[i]->getParent()->getq()(j); // save correct position

          contour[i]->getParent()->getq()(j) += epsroot(); // update with disturbed positions assuming same active links
          contour[i]->getParent()->updateStateDependentVariables(t); 
          updateg(t);
          updategd(t);
          contour[i]->getParent()->updateT(t); 
          updateJacobians(t);
          updateh(t);
          dhdq[frame.size()+i].col(j) += (hLink[frame.size()+i]-hLinkEnd)/epsroot();
          contour[i]->getParent()->getq()(j) = qParentj;
        }
      }

      /******************** time dependent calculations ***************************/
      hLink[frame.size()+i] = hLink0; // set to old values
      h[frame.size()+i] = h0;

      double t0 = t; // save correct position

      t += epsroot(); // update with disturbed positions assuming same active links
      contour[i]->getParent()->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      contour[i]->getParent()->updateT(t); 
      updateJacobians(t);
      updateh(t);

      dhdt[i] += (hLink[frame.size()+i]-hLinkEnd)/epsroot();
      t = t0;

      /************************ back to initial state ******************************/
      contour[i]->getParent()->updateStateDependentVariables(t); 
      updateg(t);
      updategd(t);
      contour[i]->getParent()->updateT(t); 
      updateJacobians(t);
      hLink[i] = hLinkEnd;
      h[i] = hEnd;
    }
  }

  void LinkMechanics::updater(double t) {

    for(unsigned int i=0; i<frame.size(); i++) 
      r[i] += W[i]*la;

    for(unsigned int i=0; i<contour.size(); i++) 
      r[i] += W[i]*la;
  }

  void LinkMechanics::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      //#ifdef HAVE_AMVIS TODO force visualisation
      //      Vec WrOToPoint;
      //      Vec LoadArrow(6,NONINIT);
      //      for (unsigned int i=0; i<arrowAMVis.size(); i++) {
      //        WrOToPoint = frame[arrowAMVisID[i]]->getPosition();
      //        if(setValued){ 
      //          if (isActive()) {
      //            LoadArrow(0,2) = fF[arrowAMVisID[i]]*la/dt;
      //            LoadArrow(3,5) = fM[arrowAMVisID[i]]*la/dt;
      //          }
      //          else {
      //            LoadArrow = Vec(6,INIT,0.0);
      //            WrOToPoint= Vec(3,INIT,0.0);
      //          }
      //        }
      //        else {
      //          LoadArrow(0,2) = WF[arrowAMVisID[i]];
      //          LoadArrow(3,5) = WM[arrowAMVisID[i]];
      //        }
      //        LoadArrow= LoadArrow/1000*arrowAMVisScale[i]; // scaling: 1KN or 1KNm scaled to arrowlenght one
      //
      //        arrowAMVis[i]->setTime(t);
      //        arrowAMVis[i]->setToPoint(WrOToPoint(0),WrOToPoint(1),WrOToPoint(2));
      //        double color;
      //        if (arrowAMVisMoment[i]) {
      //          arrowAMVis[i]->setDirection(LoadArrow(3),LoadArrow(4),LoadArrow(5));
      //          color=0.5;
      //        }
      //        else {
      //          arrowAMVis[i]->setDirection(LoadArrow(0),LoadArrow(1),LoadArrow(2));
      //          color =1.0;
      //        }
      //        if (arrowAMVisUserFunctionColor[i]) {
      //          color = (*arrowAMVisUserFunctionColor[i])(t)(0);
      //          if (color>1) color=1;
      //          if (color<0) color=0;
      //        }  
      //        arrowAMVis[i]->setColor(color);
      //        arrowAMVis[i]->appendDataset(0);
      //      }
      //
      //      for (unsigned int i=0; i<arrowAMVis.size(); i++) {
      //        if(setValued) { 
      //          if (isActive()) {
      //            LoadArrow(0,2) = fF[arrowAMVisID[i]]*la/dt;
      //            LoadArrow(3,5) = fM[arrowAMVisID[i]]*la/dt;
      //          }
      //          else {
      //            LoadArrow = Vec(6,INIT,0.0);
      //            WrOToPoint= Vec(3,INIT,0.0);
      //          }
      //        }
      //        else {
      //          LoadArrow(0,2) = WF[arrowAMVisID[i]];
      //          LoadArrow(3,5) = WM[arrowAMVisID[i]];
      //        }
      //        LoadArrow(0,2)= LoadArrow(0,2)/1000*arrowAMVisScale[i]; // Scaling: 1KN or 1KNm scaled to arrowlenght one
      //
      //        arrowAMVis[i]->setTime(t);
      //        arrowAMVis[i]->setToPoint(WrOToPoint(0),WrOToPoint(1),WrOToPoint(2));
      //        double color;
      //        if (arrowAMVisMoment[i]) {
      //          arrowAMVis[i]->setDirection(LoadArrow(3),LoadArrow(4),LoadArrow(5));
      //          color=0.5;
      //        }
      //        else {
      //          arrowAMVis[i]->setDirection(LoadArrow(0),LoadArrow(1),LoadArrow(2));
      //          color =1.0;
      //        }
      //        if (arrowAMVisUserFunctionColor[i]) {
      //          color = (*arrowAMVisUserFunctionColor[i])(t)(0);
      //          if (color>1) color=1;
      //          if (color<0) color=0;
      //        }  
      //        arrowAMVis[i]->setColor(color);
      //        arrowAMVis[i]->appendDataset(0);
      //      }
      //#endif
      Link::plot(t,dt);
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
      h[i]>>hParent(I);
      hLink[i]>>hLinkParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->getParent()->gethInd(parent,j),contour[i]->getParent()->gethInd(parent,j)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      h[i]>>hParent(I);
      hLink[i]>>hLinkParent(I);
    }
  } 

  void LinkMechanics::updatedhdqRef(const fmatvec::Mat& dhdqParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->getParent()->gethInd(parent,j),frame[i]->getParent()->gethInd(parent,j)+frame[i]->getJacobianOfTranslation().cols()-1);
      Index J = Index(frame[i]->getParent()->getqInd(),frame[i]->getParent()->getqInd()+(frame[i]->getJacobianOfTranslation().cols() > 0 ? frame[i]->getParent()->getqSize()-1 : (-1)));
      dhdq[i]>>dhdqParent(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->getParent()->gethInd(parent,j),contour[i]->getParent()->gethInd(parent,j)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      Index J = Index(contour[i]->getParent()->getqInd(),contour[i]->getParent()->getqInd()+(contour[i]->getReferenceJacobianOfTranslation().cols() > 0 ? contour[i]->getParent()->getqSize()-1 : (-1)));
      dhdq[i]>>dhdqParent(I,J);
    }
  }

  void LinkMechanics::updatedhduRef(const fmatvec::SqrMat& dhduParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->getParent()->gethInd(parent,j),frame[i]->getParent()->gethInd(parent,j)+frame[i]->getJacobianOfTranslation().cols()-1);
      dhdu[i]>>dhduParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->getParent()->gethInd(parent,j),contour[i]->getParent()->gethInd(parent,j)+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      dhdu[i]>>dhduParent(I);
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

  void LinkMechanics::updaterRef(const Vec &rParent) {
    for(unsigned i=0; i<frame.size(); i++) {
      int hInd =  frame[i]->getParent()->gethInd(parent);
      Index I = Index(hInd,hInd+frame[i]->getJacobianOfTranslation().cols()-1);
      r[i]>>rParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      int hInd =  contour[i]->getParent()->gethInd(parent);
      Index I = Index(hInd,hInd+contour[i]->getReferenceJacobianOfTranslation().cols()-1);
      r[i]>>rParent(I);
    }
  } 

  void LinkMechanics::init() {
    Link::init();

    for(unsigned i=0; i<frame.size(); i++) {
      W.push_back(Mat(frame[i]->getJacobianOfTranslation().cols(),laSize));
      V.push_back(Mat(frame[i]->getJacobianOfTranslation().cols(),laSize));
      h.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
      hLink.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
      dhdq.push_back(Mat(frame[i]->getJacobianOfTranslation().cols(),frame[i]->getJacobianOfTranslation().cols() > 0 ? frame[i]->getParent()->getqSize() : 0));
      dhdu.push_back(SqrMat(frame[i]->getJacobianOfTranslation().cols()));
      dhdt.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
      r.push_back(Vec(frame[i]->getJacobianOfTranslation().cols()));
      WF.push_back(Vec(3));
      WM.push_back(Vec(3));
      fF.push_back(Mat(3,laSize));
      fM.push_back(Mat(3,laSize));
    }

    for(unsigned i=0; i<contour.size(); i++) {
      W.push_back(Mat(contour[i]->getReferenceJacobianOfTranslation().cols(),laSize));
      V.push_back(Mat(contour[i]->getReferenceJacobianOfTranslation().cols(),laSize));
      h.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
      hLink.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
      dhdq.push_back(Mat(contour[i]->getReferenceJacobianOfTranslation().cols(),contour[i]->getReferenceJacobianOfTranslation().cols() > 0 ? contour[i]->getParent()->getqSize() : 0));
      dhdu.push_back(SqrMat(contour[i]->getReferenceJacobianOfTranslation().cols()));
      dhdt.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
      r.push_back(Vec(contour[i]->getReferenceJacobianOfTranslation().cols()));
      WF.push_back(Vec(3));
      WM.push_back(Vec(3));
      fF.push_back(Mat(3,laSize));
      fM.push_back(Mat(3,laSize));
    }
  }

  void LinkMechanics::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
      //#ifdef HAVE_AMVIS
      //      for (unsigned int i=0; i<arrowAMVis.size(); i++)
      //        arrowAMVis[i]->writeBodyFile();
      //#endif
      Link::initPlot();
    }
  }

  void LinkMechanics::connect(Frame *frame_) {
    frame.push_back(frame_);
  }

  void LinkMechanics::connect(Contour *contour_) {
    contour.push_back(contour_);
  }

  //#ifdef HAVE_AMVIS
  //  void LinkMechanics::addAMVisForceArrow(AMVis::Arrow *arrow, double scale, int ID, UserFunction *funcColor) {
  //    assert(ID >= 0);
  //    assert(ID < 2);
  //    arrowAMVis.push_back(arrow);
  //    arrowAMVisScale.push_back(scale);
  //    arrowAMVisID.push_back(ID);
  //    arrowAMVisUserFunctionColor.push_back(funcColor);
  //    arrowAMVisMoment.push_back(false);
  //  }
  //
  //  void LinkMechanics::addAMVisMomentArrow(AMVis::Arrow *arrow,double scale ,int ID, UserFunction *funcColor) {
  //    assert(ID >= 0);
  //    assert(ID < 2);
  //    arrowAMVis.push_back(arrow);
  //    arrowAMVisScale.push_back(scale);
  //    arrowAMVisID.push_back(ID);
  //    arrowAMVisUserFunctionColor.push_back(funcColor);
  //    arrowAMVisMoment.push_back(true);
  //  }
  //#endif

}

