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
#include <mbsim/contact.h>
#include <mbsim/contour.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/utils.h>
#include <mbsim/objectfactory.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/objectfactory.h>
#include <mbsim/utils/eps.h>
#include <mbsim/utils/rotarymatrices.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {
  extern double tP;
  extern bool gflag;

  Contact::Contact(const string &name) :
              LinkMechanics(name), fcl(0), fdf(0), fnil(0), ftil(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
  , openMBVContactGrp(0), openMBVContactFrameSize(0), openMBVContactFrameEnabled(true), contactArrow(NULL), frictionArrow(NULL)
#endif
  , saved_ref1(""), saved_ref2("") {
  }

  Contact::~Contact() {
    for(vector<ContactKinematics*>::iterator iter = contactKinematics.begin(); iter != contactKinematics.end(); ++iter)
      delete *iter;
    /* Delete will fail if the same object is used for more than one Contact.
     * TODO: A delete concept (who deletes what) is still missing in MBSim.
     if(fcl) { delete fcl; fcl=0; }
     if(fdf) { delete fdf; fdf=0; }
     if(fnil) { delete fnil; fnil=0; }
     if(ftil) { delete ftil; ftil=0; }*/


    for (dvec<ContourPointData*>::type::iterator i = cpData.begin(); i != cpData.end(); ++i)
      for(vector<ContourPointData*>::iterator j = i->begin(); j != i->end(); ++j)
        delete[] *j;
    for (dvec<Mat*>::type::iterator i = Wk[0].begin(); i != Wk[0].end(); ++i)
      for(vector<Mat*>::iterator j = i->begin(); j != i->end(); ++j)
        delete[] *j;
    for (dvec<Mat*>::type::iterator i = Vk[0].begin(); i != Vk[0].end(); ++i)
      for(vector<Mat*>::iterator j = i->begin(); j != i->end(); ++j)
        delete[] *j;
    for (dvec<Mat*>::type::iterator i = fF.begin(); i != fF.end(); ++i)
      for(vector<Mat*>::iterator j = i->begin(); j != i->end(); ++j)
        delete[] *j;
    for (dvec<Vec*>::type::iterator i = WF.begin(); i != WF.end(); ++i)
      for(vector<Vec*>::iterator j = i->begin(); j != i->end(); ++j)
        delete[] *j;
    for (dvec<unsigned int*>::type::iterator i = gdActive.begin(); i != gdActive.end(); ++i)
      for(vector<unsigned int*>::iterator j = i->begin(); j != i->end(); ++j)
        delete[] *j;
    for (dvec<unsigned int*>::type::iterator i = gddActive.begin(); i != gddActive.end(); ++i)
      for(vector<unsigned int*>::iterator j = i->begin(); j != i->end(); ++j)
        delete[] *j;
  }

  //  void Contact::updatewb(double t, int j) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gdActive[k]) {
  //        for (unsigned i = 0; i < contour.size(); i++)
  //          wbk[k] += fF[k][i](Index(0, 2), Index(0, laSizek[k] - 1)).T() * cpData[k][i].getFrameOfReference().getGyroscopicAccelerationOfTranslation(j);
  //      }
  //    }
  //    contactKinematics->updatewb(wbk.begin(), gk.begin(), cpData.begin());
  //  }

  void Contact::updatewb(double t, int j) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); ++k) {
        if(gdActive[cK][k]) {
          for (unsigned i = 0; i < 2; ++i) //TODO: only two contours are interacting
            wbk[cK][k] += fF[cK][k][i](Index(0, 2), Index(0, laSizek[cK][k] - 1)).T() * cpData[cK][k][i].getFrameOfReference().getGyroscopicAccelerationOfTranslation(j);
        }
      }
      contactKinematics[cK]->updatewb(wbk[cK].begin(), gk[cK].begin(), cpData[cK].begin());
    }
  }

  //  void Contact::updateW(double t, int j) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //        fF[k][1].col(0) = cpData[k][0].getFrameOfReference().getOrientation().col(0);
  //        if (getFrictionDirections()) {
  //          fF[k][1].col(1) = cpData[k][0].getFrameOfReference().getOrientation().col(1);
  //          if (getFrictionDirections() > 1)
  //            fF[k][1].col(2) = cpData[k][0].getFrameOfReference().getOrientation().col(2);
  //        }
  //
  //        fF[k][0] = -fF[k][1];
  //
  //        for (unsigned int i = 0; i < contour.size(); i++)
  //          Wk[j][k][i] += cpData[k][i].getFrameOfReference().getJacobianOfTranslation(j).T() * fF[k][i](Index(0, 2), Index(0, laSizek[k] - 1));
  //      }
  //    }
  //  }

  void Contact::updateW(double t, int j) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); ++k) {
        if (gActive[cK][k]) {

          int fFstartCol = 0;
          if(fcl->isSetValued())
            fF[cK][k][1].col(0) = cpData[cK][k][0].getFrameOfReference().getOrientation().col(0);
          else
            fFstartCol = 1;

          if (getFrictionDirections()) {
            if(fdf->isSetValued()) {
              fF[cK][k][1].col(1) = cpData[cK][k][0].getFrameOfReference().getOrientation().col(1);
              if (getFrictionDirections() > 1)
                fF[cK][k][1].col(2) = cpData[cK][k][0].getFrameOfReference().getOrientation().col(2);
            }
          }

          fF[cK][k][0] = -fF[cK][k][1];

          for (unsigned int i = 0; i < 2; i++) //TODO: only two contours are interacting at one time?
            Wk[j][cK][k][i] += cpData[cK][k][i].getFrameOfReference().getJacobianOfTranslation(j).T() * fF[cK][k][i](Index(0, 2), Index(fFstartCol, fFstartCol + laSizek[cK][k] - 1));
        }
      }
    }
  }

  //  void Contact::updateV(double t, int j) {
  //    if (getFrictionDirections()) {
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (gdActive[k][0] && !gdActive[k][1]) {
  //          for (unsigned int i = 0; i < contour.size(); i++)
  //            Vk[j][k][i] += cpData[k][i].getFrameOfReference().getJacobianOfTranslation(j).T() * fF[k][i](Index(0, 2), iT) * fdf->dlaTdlaN(gdk[k](1, getFrictionDirections()), lak[k](0));
  //        }
  //      }
  //    }
  //  }

  void Contact::updateV(double t, int j) {
    if (getFrictionDirections()) {
      if(fdf->isSetValued()) {
        for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
          for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
            if (gdActive[cK][k][0] and not gdActive[cK][k][1]) {
              for (unsigned int i = 0; i < 2; i++) //TODO: only two contours are interacting at one time?
                Vk[j][cK][k][i] += cpData[cK][k][i].getFrameOfReference().getJacobianOfTranslation(j).T() * fF[cK][k][i](Index(0, 2), iT) * fdf->dlaTdlaN(gdkT[cK][k], lakN[cK][k](0));
            }
          }
        }
      }
    }
  }

  //  void Contact::updateh(double t, int j) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) { // gActive should not be checked, e.g. because of possible predamping in constitutive laws
  //      lak[k](0) = (*fcl)(gk[k](0), gdk[k](0), this);
  //      if (fdf)
  //        lak[k](1, getFrictionDirections()) = (*fdf)(gdk[k](1, getFrictionDirections()), fabs(lak[k](0)));
  //
  //      WF[k][1] = cpData[k][0].getFrameOfReference().getOrientation().col(0) * lak[k](0);
  //      if (getFrictionDirections()) {
  //        WF[k][1] += cpData[k][0].getFrameOfReference().getOrientation().col(1) * lak[k](1);
  //        if (getFrictionDirections() > 1)
  //          WF[k][1] += cpData[k][0].getFrameOfReference().getOrientation().col(2) * lak[k](2);
  //      }
  //      WF[k][0] = -WF[k][1];
  //      for (unsigned int i = 0; i < contour.size(); i++) {
  //        h[j][i] += cpData[k][i].getFrameOfReference().getJacobianOfTranslation(j).T() * WF[k][i];
  //      }
  //    }
  //  }

  void Contact::updateh(double t, int j) {
    (*fcl).computeSmoothForces(contour, cpData, gk, gdkN, lakN);

    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      int contourIndex = cK * 2;
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) { // gActive should not be checked, e.g. because of possible predamping in constitutive laws

        WF[cK][k][1] = cpData[cK][k][0].getFrameOfReference().getOrientation().col(0) * lakN[cK][k](0);

        if (fdf)
          if(not fdf->isSetValued()) {
            lakT[cK][k] = (*fdf)(gdkT[cK][k], fabs(lakN[cK][k](0)));
            WF[cK][k][1] += cpData[cK][k][0].getFrameOfReference().getOrientation().col(1) * lakT[cK][k](0);
            if (getFrictionDirections() > 1)
              WF[cK][k][1] += cpData[cK][k][0].getFrameOfReference().getOrientation().col(2) * lakT[cK][k](1);
          }

        WF[cK][k][0] = -WF[cK][k][1];
        for (unsigned int i = 0; i < 2; i++) { //TODO only two contours are interacting at one time?
          h[j][contourIndex + i] += cpData[cK][k][i].getFrameOfReference().getJacobianOfTranslation(j).T() * WF[cK][k][i];
        }
      }
    }

  }

  //  void Contact::updateg(double t) {
  //    contactKinematics->updateg(gk.begin(), cpData.begin());
  //  }

  void Contact::updateg(double t) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      contactKinematics[cK]->updateg(gk[cK].begin(), cpData[cK].begin());
    }
  }

  //  void Contact::updategd(double t) {
  //    const bool flag = fcl->isSetValued();
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if ((flag && gdActive[k][0]) || (!flag && fcl->isActive(gk[k](0), 0))) { // TODO: nicer implementation
  //        for (unsigned int i = 0; i < 2; i++)
  //          contour[i]->updateKinematicsForFrame(cpData[k][i], velocities); // angular velocity necessary e.g. see ContactKinematicsSpherePlane::updatewb
  //
  //        Vec Wn = cpData[k][0].getFrameOfReference().getOrientation().col(0);
  //
  //        Vec WvD = cpData[k][1].getFrameOfReference().getVelocity() - cpData[k][0].getFrameOfReference().getVelocity();
  //
  //        gdk[k](0) = Wn.T() * WvD;
  //
  //        if (gdk[k].size() > 1) {
  //          Mat Wt(3, gdk[k].size() - 1);
  //          Wt.col(0) = cpData[k][0].getFrameOfReference().getOrientation().col(1);
  //          if (gdk[k].size() > 2)
  //            Wt.col(1) = cpData[k][0].getFrameOfReference().getOrientation().col(2);
  //
  //          gdk[k](1, gdk[k].size() - 1) = Wt.T() * WvD;
  //        }
  //      }
  //    }
  //  }

  void Contact::updategd(double t) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      int contourIndex = cK * 2;
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if ((fcl->isSetValued() and gdActive[cK][k][0]) or
            (not fcl->isSetValued() and fcl->isActive(gk[cK][k](0), 0))) { // TODO: nicer implementation
          for (unsigned int i = 0; i < 2; i++)
            contour[contourIndex + i]->updateKinematicsForFrame(cpData[cK][k][i], velocities); // angular velocity necessary e.g. see ContactKinematicsSpherePlane::updatewb

          Vec Wn = cpData[cK][k][0].getFrameOfReference().getOrientation().col(0);

          Vec WvD = cpData[cK][k][1].getFrameOfReference().getVelocity() - cpData[cK][k][0].getFrameOfReference().getVelocity();

          gdkN[cK][k](0) = Wn.T() * WvD;

          if (gdkT[cK][k].size()) {
            Mat Wt(3, gdkT[cK][k].size());
            Wt.col(0) = cpData[cK][k][0].getFrameOfReference().getOrientation().col(1);
            if (gdkT[cK][k].size() > 1)
              Wt.col(1) = cpData[cK][k][0].getFrameOfReference().getOrientation().col(2);

            gdkT[cK][k] = Wt.T() * WvD;
          }
        }
      }
    }
  }

  //  void Contact::updateStopVector(double t) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k] != gdActive[k][0])
  //        throw;
  //      if (gActive[k]) {
  //        svk[k](0) = gddk[k](0) > gddTol ? -1 : 1;
  //        if (gdActive[k][1]) {
  //          if (getFrictionDirections()) {
  //            svk[k](1) = nrm2(gddk[k](1, getFrictionDirections())) > gddTol ? -1 : 1;
  //            if ((int) svk[k](1) == -1)
  //              gddkBuf[k] = gddk[k];
  //          }
  //        }
  //        else {
  //          if (getFrictionDirections() == 1)
  //            svk[k](1) = gdk[k](1) > 0 ? 1 : -1;
  //          else if (getFrictionDirections() == 2) {
  //            svk[k](1) = gdk[k](1) + gdk[k](2); // TODO: is there a better concept?
  //          }
  //        }
  //      }
  //      else {
  //        svk[k](0) = gk[k](0) > 0 ? 1 : -1;
  //        if (getFrictionDirections())
  //          svk[k](1) = 1;
  //      }
  //    }
  //  }

  void Contact::updateStopVector(double t) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k] != gdActive[cK][k][0])
          throw;
        if (gActive[cK][k]) {
          svk[cK][k](0) = gddk[cK][k](0) > gddTol ? -1 : 1;
          if (gdActive[cK][k][1]) {
            if (getFrictionDirections()) {
              svk[cK][k](1) = nrm2(gddk[cK][k](1, getFrictionDirections())) > gddTol ? -1 : 1;
              if ((int) svk[cK][k](1) == -1)
                gddkBuf[cK][k] = gddk[cK][k];
            }
          }
          else {
            if (getFrictionDirections() == 1)
              svk[cK][k](1) = gdkT[cK][k](0) > 0 ? 1 : -1;
            else if (getFrictionDirections() == 2) {
              svk[cK][k](1) = gdkT[cK][k](0) + gdkT[cK][k](1); // TODO: is there a better concept?
            }
          }
        }
        else {
          svk[cK][k](0) = gk[cK][k](0) > 0 ? 1 : -1;
          if (getFrictionDirections())
            svk[cK][k](1) = 1;
        }
      }
    }
  }

  //  void Contact::updateJacobians(double t, int j) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      if (gActive[k])
  //        for (unsigned int i = 0; i < 2; i++)
  //          contour[i]->updateJacobiansForFrame(cpData[k][i], j);
  //  }

  void Contact::updateJacobians(double t, int j) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      int contourIndex = cK * 2;
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k])
          for (unsigned int i = 0; i < 2; i++)
            contour[contourIndex + i]->updateJacobiansForFrame(cpData[cK][k][i], j);
      }
    }
  }

  //  void Contact::updateWRef(const Mat& WParent, int j) {
  //    for (unsigned i = 0; i < contour.size(); i++) {
  //      int hInd = contour[i]->gethInd(j);
  //      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
  //      Index J = Index(laInd, laInd + laSize - 1);
  //      W[j][i].resize() >> WParent(I, J);
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        Index Ik = Index(0, W[j][i].rows() - 1);
  //        Index Jk = Index(laIndk[k], laIndk[k] + laSizek[k] - 1);
  //        Wk[j][k][i].resize() >> W[j][i](Ik, Jk);
  //      }
  //    }
  //  }

  void Contact::updateWRef(const Mat& WParent, int j) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
        int contourIndex = 2*cK + i;
        int hInd = contour[contourIndex]->gethInd(j);
        Index I = Index(hInd, hInd + contour[contourIndex]->gethSize(j) - 1);
        Index J = Index(laInd, laInd + laSize - 1);
        W[j][contourIndex].resize() >> WParent(I, J);
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          Index Ik = Index(0, W[j][contourIndex].rows() - 1);
          Index Jk = Index(laIndk[cK][k], laIndk[cK][k] + laSizek[cK][k] - 1);
          Wk[j][cK][k][i].resize() >> W[j][contourIndex](Ik, Jk);
        }
      }
    }
  }

  //  void Contact::updateVRef(const Mat& VParent, int j) {
  //    for (unsigned i = 0; i < contour.size(); i++) {
  //      int hInd = contour[i]->gethInd(j);
  //      Index J = Index(laInd, laInd + laSize - 1);
  //      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
  //      V[j][i].resize() >> VParent(I, J);
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        Index Ik = Index(0, V[j][i].rows() - 1);
  //        Index Jk = Index(laIndk[k], laIndk[k] + laSizek[k] - 1);
  //        Vk[j][k][i].resize() >> V[j][i](Ik, Jk);
  //      }
  //    }
  //  }

  void Contact::updateVRef(const Mat& VParent, int j) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
        int contourIndex = 2*cK + i;
        int hInd = contour[contourIndex]->gethInd(j);
        Index I = Index(hInd, hInd + contour[contourIndex]->gethSize(j) - 1);
        Index J = Index(laInd, laInd + laSize - 1);
        V[j][contourIndex].resize() >> VParent(I, J);
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          Index Ik = Index(0, V[j][contourIndex].rows() - 1);
          Index Jk = Index(laIndk[cK][k], laIndk[cK][k] + laSizek[cK][k] - 1);
          Vk[j][cK][k][i].resize() >> V[j][contourIndex](Ik, Jk);
        }
      }
    }
  }

  //  void Contact::updatehRef(const Vec& hParent, int j) {
  //    for (unsigned i = 0; i < contour.size(); i++) {
  //      int hInd = contour[i]->gethInd(j);
  //      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
  //      h[j][i].resize() >> hParent(I);
  //    }
  //  }

  void Contact::updatehRef(const Vec& hParent, int j) {
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
        int contourIndex = 2*cK + i;
        int hInd = contour[contourIndex]->gethInd(j);
        Index I = Index(hInd, hInd + contour[contourIndex]->gethSize(j) - 1);
        h[j][contourIndex].resize() >> hParent(I);
      }
    }
  }

  //  void Contact::updatewbRef(const Vec& wbParent) {
  //    LinkMechanics::updatewbRef(wbParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      wbk[k].resize() >> wb(laIndk[k], laIndk[k] + laSizek[k] - 1);
  //  }

  void Contact::updatewbRef(const Vec& wbParent) {
    LinkMechanics::updatewbRef(wbParent);
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        wbk[cK][k].resize() >> wb(laIndk[cK][k], laIndk[cK][k] + laSizek[cK][k] - 1);
      }
    }
  }

  //  void Contact::updatelaRef(const Vec& laParent) {
  //    LinkMechanics::updatelaRef(laParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      lak[k].resize() >> la(laIndk[k], laIndk[k] + laSizek[k] - 1);
  //  }

  void Contact::updatelaRef(const Vec& laParent) {
    LinkMechanics::updatelaRef(laParent);
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {

        if(laSizek[cK][k]) {
          int laIndSizeNormal = 0;
          if(fcl->isSetValued()) {
            lakN[cK][k].resize() >> la(laIndk[cK][k], laIndk[cK][k]);
            laIndSizeNormal++;
          }
          if(fdf)
            if(fdf->isSetValued())
              lakT[cK][k].resize() >> la(laIndk[cK][k] + laIndSizeNormal, laIndk[cK][k] + laSizek[cK][k] - 1);
        }
      }
    }
  }

  //  void Contact::updategRef(const Vec& gParent) {
  //    LinkMechanics::updategRef(gParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      gk[k].resize() >> g(gIndk[k], gIndk[k] + gSizek[k] - 1);
  //  }

  void Contact::updategRef(const Vec& gParent) {
    LinkMechanics::updategRef(gParent);
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        gk[cK][k].resize() >> g(gIndk[cK][k], gIndk[cK][k] + gSizek[cK][k] - 1);
      }
    }
  }

  //  void Contact::updategdRef(const Vec& gdParent) {
  //    LinkMechanics::updategdRef(gdParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      gdk[k].resize() >> gd(gdIndk[k], gdIndk[k] + gdSizek[k] - 1);
  //  }

  void Contact::updategdRef(const Vec& gdParent) {
    LinkMechanics::updategdRef(gdParent);
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gdSizek[cK][k]) {
          int gdIndSizeNormal = 0;
          if (fcl->isSetValued()) {
            gdkN[cK][k].resize() >> gd(gdIndk[cK][k], gdIndk[cK][k]);
            gdIndSizeNormal++;
          }
          if (fdf)
            if (fdf->isSetValued())
              gdkT[cK][k].resize() >> gd(gdIndk[cK][k] + gdIndSizeNormal, gdIndk[cK][k] + gdSizek[cK][k] - 1);
        }
      }
    }
  }

  //  void Contact::updaterFactorRef(const Vec& rFactorParent) {
  //    LinkMechanics::updaterFactorRef(rFactorParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      rFactork[k].resize() >> rFactor(rFactorIndk[k], rFactorIndk[k] + rFactorSizek[k] - 1);
  //  }

  void Contact::updaterFactorRef(const Vec& rFactorParent) {
    LinkMechanics::updaterFactorRef(rFactorParent);
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        rFactork[cK][k].resize() >> rFactor(rFactorIndk[cK][k], rFactorIndk[cK][k] + rFactorSizek[cK][k] - 1);
      }
    }
  }

  //  void Contact::updatesvRef(const Vec &svParent) {
  //    LinkMechanics::updatesvRef(svParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      svk[k] >> sv(svIndk[k], svIndk[k] + svSizek[k] - 1);
  //  }

  void Contact::updatesvRef(const Vec &svParent) {
    LinkMechanics::updatesvRef(svParent);
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        svk[cK][k] >> sv(svIndk[cK][k], svIndk[cK][k] + svSizek[cK][k] - 1);
      }
    }
  }

  //  void Contact::updatejsvRef(const Vector<int> &jsvParent) {
  //    LinkMechanics::updatejsvRef(jsvParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      jsvk[k] >> jsv(svIndk[k], svIndk[k] + svSizek[k] - 1);
  //  }

  void Contact::updatejsvRef(const Vector<int> &jsvParent) {
    LinkMechanics::updatejsvRef(jsvParent);
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        jsvk[cK][k] >> jsv(svIndk[cK][k], svIndk[cK][k] + svSizek[cK][k] - 1);
      }
    }
  }

  void Contact::calcxSize() {
    LinkMechanics::calcxSize();
    xSize = 0;
  }

  //  void Contact::calclaSize(int j) {
  //    LinkMechanics::calclaSize(j);
  //    if (j == 0) { // IA
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        laIndk[i] = laSize;
  //        laSizek[i] = 1 + getFrictionDirections();
  //        laSize += laSizek[i];
  //      }
  //    }
  //    else if (j == 1) { // IG
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        laIndk[i] = laSize;
  //        laSizek[i] = gActive[i] * (1 + getFrictionDirections());
  //        laSize += laSizek[i];
  //      }
  //    }
  //    else if (j == 2) { // IB
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        laIndk[i] = laSize;
  //        laSizek[i] = gActive[i] * gdActive[i][0] * (1 + getFrictionDirections());
  //        laSize += laSizek[i];
  //      }
  //    }
  //    else if (j == 3) { // IH
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        laIndk[i] = laSize;
  //        laSizek[i] = gActive[i] * gdActive[i][0] * (1 + gdActive[i][1] * getFrictionDirections());
  //        laSize += laSizek[i];
  //      }
  //    }
  //    else if (j == 4) { // IG
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        laIndk[i] = laSize;
  //        laSizek[i] = gActive[i];
  //        laSize += laSizek[i];
  //      }
  //    }
  //    else if (j == 5) { // IB
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        laIndk[i] = laSize;
  //        laSizek[i] = gActive[i] * gdActive[i][0];
  //        laSize += laSizek[i];
  //      }
  //    }
  //    else
  //      throw;
  //  }

  void Contact::calclaSize(int j) {
    LinkMechanics::calclaSize(j);
    if (j == 0) { // IA
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          laIndk[cK][k] = laSize;

          //Add 1 to lambda size if normal force law is setValued
          if(fcl->isSetValued())
            laSizek[cK][k] = 1;
          else
            laSizek[cK][k] = 0;

          //Add number of friction directions to lambda size if friction force law is setValued
          if(fdf)
            if(fdf->isSetValued())
              laSizek[cK][k] += getFrictionDirections();

          laSize += laSizek[cK][k];
        }
      }
    }
    else if (j == 1) { // IG
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          laIndk[cK][k] = laSize;

          //Add 1 to lambda size if normal force law is setValued
          if(fcl->isSetValued())
            laSizek[cK][k] = 1;
          else
            laSizek[cK][k] = 0;

          //Add number of friction directions to lambda size if friction force law is setValued
          if(fdf)
            if(fdf->isSetValued())
              laSizek[cK][k] += getFrictionDirections();

          //check if contact is active --> else lambda Size will get zero...
          laSizek[cK][k] *= gActive[cK][k];

          laSize += laSizek[cK][k];
        }
      }
    }
    else if (j == 2) { // IB
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          laIndk[cK][k] = laSize;
          //Add 1 to lambda size if normal force law is setValued
          if(fcl->isSetValued())
            laSizek[cK][k] = 1;
          else
            laSizek[cK][k] = 0;

          //Add number of friction directions to lambda size if friction force law is setValued
          if(fdf)
            if(fdf->isSetValued())
              laSizek[cK][k] += getFrictionDirections();

          //check if contact is active --> else lambda Size will get zero...
          laSizek[cK][k] *= gActive[cK][k] * gdActive[cK][k][0];

          laSize += laSizek[cK][k];
        }
      }
    }
    else if (j == 3) { // IH
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          laIndk[cK][k] = laSize;

          //Add 1 to lambda size if normal force law is setValued
          if(fcl->isSetValued())
            laSizek[cK][k] = 1;
          else
            laSizek[cK][k] = 0;

          //Add number of friction directions to lambda size if friction force law is setValued and active
          if(fdf)
            if(fdf->isSetValued())
              laSizek[cK][k] += getFrictionDirections() * gdActive[cK][k][0];

          //check if contact is active --> else lambda Size will get zero...
          laSizek[cK][k] *= gActive[cK][k] * gdActive[cK][k][0];

          laSize += laSizek[cK][k];
        }
      }
    }
    else if (j == 4) { // IG
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          laIndk[cK][k] = laSize;

          //Add 1 to lambda size if normal force law is setValued and active
          if(fcl->isSetValued())
            laSizek[cK][k] = gActive[cK][k];

          laSize += laSizek[cK][k];
        }
      }
    }
    else if (j == 5) { // IB
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          laIndk[cK][k] = laSize;
          //TODO: How to fit to concept of mixed single- and set-valued contacts
          laSizek[cK][k] = gActive[cK][k] * gdActive[cK][k][0];
          laSize += laSizek[cK][k];
        }
      }
    }
    else
      throw;
  }

  //  void Contact::calcgSize(int j) {
  //    LinkMechanics::calcgSize(j);
  //    if (j == 0) { // IA
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gIndk[i] = gSize;
  //        gSizek[i] = 1;
  //        gSize += gSizek[i];
  //      }
  //    }
  //    else if (j == 1) { // IG
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gIndk[i] = gSize;
  //        gSizek[i] = gActive[i];
  //        gSize += gSizek[i];
  //      }
  //    }
  //    else if (j == 2) { // IB
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gIndk[i] = gSize;
  //        gSizek[i] = gActive[i] * gdActive[i][0];
  //        gSize += gSizek[i];
  //      }
  //    }
  //    else
  //      throw;
  //  }


  void Contact::calcgSize(int j) {
    LinkMechanics::calcgSize(j);
    if (j == 0) { // IA
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gIndk[cK][k] = gSize;
          gSizek[cK][k] = 1;
          gSize += gSizek[cK][k];
        }
      }
    }
    else if (j == 1) { // IG
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gIndk[cK][k] = gSize;
          gSizek[cK][k] = gActive[cK][k];
          gSize += gSizek[cK][k];
        }
      }
    }
    else if (j == 2) { // IB
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gIndk[cK][k] = gSize;
          gSizek[cK][k] = gActive[cK][k] * gdActive[cK][k][0];
          gSize += gSizek[cK][k];
        }
      }
    }
    else
      throw;
  }

  //  void Contact::calcgdSize(int j) {
  //    LinkMechanics::calcgdSize(j);
  //    if (j == 0) { // IA
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gdIndk[i] = gdSize;
  //        gdSizek[i] = 1 + getFrictionDirections();
  //        gdSize += gdSizek[i];
  //      }
  //    }
  //    else if (j == 1) { // IG
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gdIndk[i] = gdSize;
  //        gdSizek[i] = gActive[i] * (1 + getFrictionDirections());
  //        gdSize += gdSizek[i];
  //      }
  //    }
  //    else if (j == 2) { // IB
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gdIndk[i] = gdSize;
  //        gdSizek[i] = gActive[i] * gdActive[i][0] * (1 + getFrictionDirections());
  //        gdSize += gdSizek[i];
  //      }
  //    }
  //    else if (j == 3) { // IH
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gdIndk[i] = gdSize;
  //        gdSizek[i] = gActive[i] * gdActive[i][0] * (1 + gdActive[i][1] * getFrictionDirections());
  //        gdSize += gdSizek[i];
  //      }
  //    }
  //    else
  //      throw;
  //  }

  void Contact::calcgdSize(int j) {
    LinkMechanics::calcgdSize(j);
    if (j == 0) { // IA
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gdIndk[cK][k] = gdSize;

          //Add 1 to gd size if normal force law is setValued
          if(fcl->isSetValued())
            gdSizek[cK][k] = 1;
          else
            gdSizek[cK][k] = 0;

          //Add number of friction directions to gd size if friction force law is setValued
          if(fdf)
            if(fdf->isSetValued())
              gdSizek[cK][k] += getFrictionDirections();

          gdSize += gdSizek[cK][k];
        }
      }
    }
    else if (j == 1) { // IG
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gdIndk[cK][k] = gdSize;

          //Add 1 to gd size if normal force law is setValued
          if(fcl->isSetValued())
            gdSizek[cK][k] = 1;
          else
            gdSizek[cK][k] = 0;

          //Add number of friction directions to gd size if friction force law is setValued
          if(fdf)
            if(fdf->isSetValued())
              gdSizek[cK][k] += getFrictionDirections();

          gdSizek[cK][k] *= gActive[cK][k];

          gdSize += gdSizek[cK][k];
        }
      }
    }
    else if (j == 2) { // IB
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gdIndk[cK][k] = gdSize;

          //Add 1 to gd size if normal force law is setValued
          if(fcl->isSetValued())
            gdSizek[cK][k] = 1;
          else
            gdSizek[cK][k] = 0;

          //Add number of friction directions to gd size if friction force law is setValued
          if(fdf)
            if(fdf->isSetValued())
              gdSizek[cK][k] += getFrictionDirections();

          gdSizek[cK][k] *= gActive[cK][k] * gdActive[cK][k][0];

          gdSize += gdSizek[cK][k];
        }
      }
    }
    else if (j == 3) { // IH
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gdIndk[cK][k] = gdSize;

          //Add 1 to gd size if normal force law is setValued
          if(fcl->isSetValued())
            gdSizek[cK][k] = 1;
          else
            gdSizek[cK][k] = 0;

          //Add number of friction directions to gd size if friction force law is setValued
          if(fdf)
            if(fdf->isSetValued())
              gdSizek[cK][k] += gdActive[cK][k][1] * getFrictionDirections();

          gdSizek[cK][k] *= gActive[cK][k] * gdActive[cK][k][0];

          gdSize += gdSizek[cK][k];
        }
      }
    }
    else
      throw;
  }

  //  void Contact::calcrFactorSize(int j) {
  //    LinkMechanics::calcrFactorSize(j);
  //    if (j == 0) { // IA
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        rFactorIndk[i] = rFactorSize;
  //        rFactorSizek[i] = 1 + min(getFrictionDirections(), 1);
  //        rFactorSize += rFactorSizek[i];
  //      }
  //    }
  //    else if (j == 1) { // IG
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        rFactorIndk[i] = rFactorSize;
  //        rFactorSizek[i] = gActive[i] * (1 + min(getFrictionDirections(), 1));
  //        rFactorSize += rFactorSizek[i];
  //      }
  //    }
  //    else if (j == 2) { // IB
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        rFactorIndk[i] = rFactorSize;
  //        rFactorSizek[i] = gActive[i] * gdActive[i][0] * (1 + min(getFrictionDirections(), 1));
  //        rFactorSize += rFactorSizek[i];
  //      }
  //    }
  //    else if (j == 3) { // IB
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        rFactorIndk[i] = rFactorSize;
  //        rFactorSizek[i] = gActive[i] * gdActive[i][0] * (1 + gdActive[i][1] * min(getFrictionDirections(), 1));
  //        rFactorSize += rFactorSizek[i];
  //      }
  //    }
  //  }

  void Contact::calcrFactorSize(int j) {
    LinkMechanics::calcrFactorSize(j);
    if (j == 0) { // IA
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          rFactorIndk[cK][k] = rFactorSize;
          rFactorSizek[cK][k] = 1 + min(getFrictionDirections(), 1);
          rFactorSize += rFactorSizek[cK][k];
        }
      }
    }
    else if (j == 1) { // IG
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          rFactorIndk[cK][k] = rFactorSize;
          rFactorSizek[cK][k] = gActive[cK][k] * (1 + min(getFrictionDirections(), 1));
          rFactorSize += rFactorSizek[cK][k];
        }
      }
    }
    else if (j == 2) { // IB
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          rFactorIndk[cK][k] = rFactorSize;
          rFactorSizek[cK][k] = gActive[cK][k] * gdActive[cK][k][0] * (1 + min(getFrictionDirections(), 1));
          rFactorSize += rFactorSizek[cK][k];
        }
      }
    }
    else if (j == 3) { // IB
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          rFactorIndk[cK][k] = rFactorSize;
          rFactorSizek[cK][k] = gActive[cK][k] * gdActive[cK][k][0] * (1 + gdActive[cK][k][1] * min(getFrictionDirections(), 1));
          rFactorSize += rFactorSizek[cK][k];
        }
      }
    }
  }

  //  void Contact::calcsvSize() {
  //    LinkMechanics::calcsvSize();
  //    svSize = 0;
  //    for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //      svIndk[i] = svSize;
  //      svSizek[i] = isSetValued() ? 1 + min(getFrictionDirections(), 1) : 0;
  //      //svSizek[i] = isSetValued() ? 1 : 0;
  //      svSize += svSizek[i];
  //    }
  //  }

  void Contact::calcsvSize() {
    LinkMechanics::calcsvSize();
    svSize = 0;
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        svIndk[cK][k] = svSize;
        svSizek[cK][k] = isSetValued() ? 1 + min(getFrictionDirections(), 1) : 0;
        //svSizek[cK][k] = isSetValued() ? 1 : 0;
        svSize += svSizek[cK][k];
      }
    }
  }

  //  void Contact::calcLinkStatusSize() {
  //    LinkMechanics::calcLinkStatusSize();
  //    int n = contactKinematics->getNumberOfPotentialContactPoints();
  //    LinkStatusSize = n;
  //    LinkStatus.resize(LinkStatusSize);
  //  }

  void Contact::calcLinkStatusSize() {
    LinkMechanics::calcLinkStatusSize();
    int n = 0;
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      n += contactKinematics[cK]->getNumberOfPotentialContactPoints();
    }
    LinkStatusSize = n;
    LinkStatus.resize(LinkStatusSize);
  }

  //  void Contact::calcLinkStatusRegSize() {
  //    LinkMechanics::calcLinkStatusRegSize();
  //    int n = contactKinematics->getNumberOfPotentialContactPoints();
  //    LinkStatusRegSize = n;
  //    LinkStatusReg.resize(LinkStatusRegSize);
  //  }

  void Contact::calcLinkStatusRegSize() {
    LinkMechanics::calcLinkStatusRegSize();
    int n = 0;
    for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
      n += contactKinematics[cK]->getNumberOfPotentialContactPoints();
    }
    LinkStatusRegSize = n;
    LinkStatusReg.resize(LinkStatusRegSize);
  }

  //  void Contact::init(InitStage stage) {
  //    if (stage == resolveXMLPath) {
  //      if (saved_ref1 != "" && saved_ref2 != "")
  //        connect(getByPath<Contour>(saved_ref1), getByPath<Contour>(saved_ref2));
  //      LinkMechanics::init(stage);
  //    }
  //    else if (stage == resize) {
  //      LinkMechanics::init(stage);
  //      int n = contactKinematics->getNumberOfPotentialContactPoints();
  //
  //      la.resize(n * (1 + getFrictionDirections()));
  //      g.resize(n);
  //      gd.resize(n * (1 + getFrictionDirections()));
  //      gdd.resize(gd.size());
  //      gdn.resize(gd.size());
  //      //LinkStatusSize= n;
  //      //LinkStatus.resize(LinkStatusSize);
  //
  //      for (vector<ContourPointData*>::iterator i = cpData.begin(); i != cpData.end(); ++i)
  //        delete[] *i;
  //      cpData.clear(); // clear container first, because InitStage resize is called twice (before and after the reorganization)
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        if (getFrictionDirections() == 0)
  //          gdActive[i][1] = false;
  //
  //        cpData.push_back(new ContourPointData[2]);
  //
  //        cpData[i][0].getFrameOfReference().setName("0");
  //        cpData[i][1].getFrameOfReference().setName("1");
  //        cpData[i][0].getFrameOfReference().getJacobianOfTranslation(0).resize();
  //        cpData[i][0].getFrameOfReference().getJacobianOfTranslation(1).resize();
  //        cpData[i][0].getFrameOfReference().getJacobianOfRotation(1).resize();
  //        cpData[i][0].getFrameOfReference().getJacobianOfRotation(0).resize();
  //        cpData[i][1].getFrameOfReference().getJacobianOfTranslation(0).resize();
  //        cpData[i][1].getFrameOfReference().getJacobianOfTranslation(1).resize();
  //        cpData[i][1].getFrameOfReference().getJacobianOfRotation(0).resize();
  //        cpData[i][1].getFrameOfReference().getJacobianOfRotation(1).resize();
  //
  //        cpData[i][0].getFrameOfReference().sethSize(contour[0]->gethSize(0), 0);
  //        cpData[i][0].getFrameOfReference().sethSize(contour[0]->gethSize(1), 1);
  //        cpData[i][1].getFrameOfReference().sethSize(contour[1]->gethSize(0), 0);
  //        cpData[i][1].getFrameOfReference().sethSize(contour[1]->gethSize(1), 1);
  //
  //        int laSizek = gdActive[i][0] + gdActive[i][1] * getFrictionDirections();
  //
  //        Wk[0].push_back(new Mat[2]);
  //        Wk[0][i][0].resize(contour[0]->gethSize(), laSizek);
  //        Wk[0][i][1].resize(contour[1]->gethSize(), laSizek);
  //
  //        Vk[0].push_back(new Mat[2]);
  //        Vk[0][i][0].resize(contour[0]->gethSize(), laSizek);
  //        Vk[0][i][1].resize(contour[1]->gethSize(), laSizek);
  //
  //        Wk[1].push_back(new Mat[2]);
  //        Wk[1][i][0].resize(contour[0]->gethSize(1), laSizek);
  //        Wk[1][i][1].resize(contour[1]->gethSize(1), laSizek);
  //
  //        Vk[1].push_back(new Mat[2]);
  //        Vk[1][i][0].resize(contour[0]->gethSize(1), laSizek);
  //        Vk[1][i][1].resize(contour[1]->gethSize(1), laSizek);
  //
  //        fF.push_back(new Mat[2]);
  //        fF[i][0].resize(3, laSizek);
  //        fF[i][1].resize(3, laSizek);
  //
  //        WF.push_back(new Vec[2]);
  //        WF[i][0].resize(3);
  //        WF[i][1].resize(3);
  //      }
  //    }
  //    else if (stage == unknownStage) {
  //      LinkMechanics::init(stage);
  //
  //      iT = Index(1, getFrictionDirections());
  //
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        lak[k].resize() >> la(k * (1 + getFrictionDirections()), (k + 1) * (1 + getFrictionDirections()) - 1);
  //        gdk[k].resize() >> gd(k * (1 + getFrictionDirections()), (k + 1) * (1 + getFrictionDirections()) - 1);
  //        gdnk[k].resize() >> gdn(k * (1 + getFrictionDirections()), (k + 1) * (1 + getFrictionDirections()) - 1);
  //        gddk[k].resize() >> gdd(k * (1 + getFrictionDirections()), (k + 1) * (1 + getFrictionDirections()) - 1);
  //        gk[k].resize() >> g(k, k);
  //        gddkBuf[k].resize(1 + getFrictionDirections());
  //      }
  //    }
  //    else if (stage == preInit) {
  //      LinkMechanics::init(stage);
  //
  //      if (contactKinematics == 0)
  //        contactKinematics = contour[0]->findContactPairingWith(contour[0]->getType(), contour[1]->getType());
  //      if (contactKinematics == 0)
  //        contactKinematics = contour[1]->findContactPairingWith(contour[1]->getType(), contour[0]->getType());
  //      if (contactKinematics == 0)
  //        contactKinematics = contour[0]->findContactPairingWith(contour[1]->getType(), contour[0]->getType());
  //      if (contactKinematics == 0)
  //        contactKinematics = contour[1]->findContactPairingWith(contour[0]->getType(), contour[1]->getType());
  //      if (contactKinematics == 0)
  //        throw MBSimError("ERROR in " + getName() + " (Contact::init): Unknown contact pairing between Contour \"" + contour[0]->getType() + "\" and Contour\"" + contour[1]->getType() + "\"!");
  //
  //      contactKinematics->assignContours(contour[0], contour[1]);
  //
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        gActive.push_back(int(1));
  //        gActive0.push_back(int(1));
  //        gdActive.push_back(new unsigned int[2]);
  //        gddActive.push_back(new unsigned int[2]);
  //        for (int j = 0; j < 1 + min(1, getFrictionDirections()); j++)
  //          gdActive[i][j] = 1;
  //        for (int j = 1 + min(1, getFrictionDirections()); j < 2; j++)
  //          gdActive[i][j] = 0;
  //        for (int j = 0; j < 1 + min(1, getFrictionDirections()); j++)
  //          gddActive[i][j] = 1;
  //        for (int j = 1 + min(1, getFrictionDirections()); j < 2; j++)
  //          gddActive[i][j] = 0;
  //
  //        gk.push_back(Vec(1));
  //        gdk.push_back(Vec(1));
  //        gdnk.push_back(Vec(1));
  //        gddk.push_back(Vec(1));
  //        gddkBuf.push_back(Vec(1));
  //        lak.push_back(Vec());
  //        wbk.push_back(Vec());
  //        svk.push_back(Vec());
  //        jsvk.push_back(Vector<int>());
  //        rFactork.push_back(Vec());
  //        laSizek.push_back(int(0));
  //        laIndk.push_back(int(0));
  //        gSizek.push_back(int(0));
  //        gIndk.push_back(int(0));
  //        gdSizek.push_back(int(0));
  //        gdIndk.push_back(int(0));
  //        svSizek.push_back(int(0));
  //        svIndk.push_back(int(0));
  //        rFactorSizek.push_back(int(0));
  //        rFactorIndk.push_back(int(0));
  //
  //        corrk.push_back(Vec(1));
  //        corrSizek.push_back(int(0));
  //        corrIndk.push_back(int(0));
  //        rootID.push_back(int(0));
  //      }
  //    }
  //    else if (stage == MBSim::plot) {
  //      updatePlotFeatures();
  //      if (getPlotFeature(plotRecursive) == enabled) {
  //#ifdef HAVE_OPENMBVCPPINTERFACE
  //        if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || contactArrow || frictionArrow)) {
  //          openMBVContactGrp = new OpenMBV::Group();
  //          openMBVContactGrp->setName(name + "_ContactGroup");
  //          openMBVContactGrp->setExpand(false);
  //          parent->getOpenMBVGrp()->addObject(openMBVContactGrp);
  //          for (unsigned int i = 0; i < cpData.size(); i++) {
  //            if (openMBVContactFrameSize > epsroot()) {
  //              vector<OpenMBV::Frame*> temp;
  //              temp.push_back(new OpenMBV::Frame);
  //              temp.push_back(new OpenMBV::Frame);
  //              openMBVContactFrame.push_back(temp);
  //              for (unsigned int k = 0; k < 2; k++) { // frames
  //                openMBVContactFrame[i][k]->setOffset(1.);
  //                openMBVContactFrame[i][k]->setSize(openMBVContactFrameSize);
  //                openMBVContactFrame[i][k]->setName("ContactPoint_" + numtostr((int) i) + (k == 0 ? "A" : "B"));
  //                openMBVContactFrame[i][k]->setEnable(openMBVContactFrameEnabled);
  //                openMBVContactGrp->addObject(openMBVContactFrame[i][k]);
  //              }
  //            }
  //            // arrows
  //            OpenMBV::Arrow *arrow;
  //            if (contactArrow) {
  //              arrow = new OpenMBV::Arrow(*contactArrow);
  //              arrow->setName("NormalForce_" + numtostr((int) i) + "B");
  //              openMBVNormalForceArrow.push_back(arrow); // normal force
  //              openMBVContactGrp->addObject(arrow);
  //            }
  //            if (frictionArrow && getFrictionDirections() > 0) { // friction force
  //              arrow = new OpenMBV::Arrow(*frictionArrow);
  //              arrow->setName("FrictionForce_" + numtostr((int) i) + "B");
  //              openMBVFrictionArrow.push_back(arrow);
  //              openMBVContactGrp->addObject(arrow);
  //            }
  //          }
  //        }
  //#endif
  //        if (getPlotFeature(linkKinematics) == enabled) {
  //          for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //            plotColumns.push_back("g[" + numtostr(i) + "](" + numtostr(0) + ")");
  //            for (int j = 0; j < 1 + getFrictionDirections(); ++j)
  //              plotColumns.push_back("gd[" + numtostr(i) + "](" + numtostr(j) + ")");
  //          }
  //        }
  //        if (getPlotFeature(generalizedLinkForce) == enabled) {
  //          for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //            for (int j = 0; j < 1 + getFrictionDirections(); ++j)
  //              plotColumns.push_back("la[" + numtostr(i) + "](" + numtostr(j) + ")");
  //          }
  //        }
  //        PlotFeatureStatus pfKinematics = getPlotFeature(linkKinematics);
  //        PlotFeatureStatus pfKinetics = getPlotFeature(generalizedLinkForce);
  //        setPlotFeature(linkKinematics, disabled);
  //        setPlotFeature(generalizedLinkForce, disabled);
  //        LinkMechanics::init(stage);
  //        setPlotFeature(linkKinematics, pfKinematics);
  //        setPlotFeature(generalizedLinkForce, pfKinetics);
  //      }
  //    }
  //    else
  //      LinkMechanics::init(stage);
  //  }

  void Contact::init(InitStage stage) {
    if (stage == resolveXMLPath) {
      if (saved_ref1 != "" && saved_ref2 != "")
        connect(getByPath<Contour>(saved_ref1), getByPath<Contour>(saved_ref2));
      LinkMechanics::init(stage);
    }
    else if (stage == resize) {
      LinkMechanics::init(stage);

      //At first set the size of the vectors that summarize all contact points of all kinematics
      int n = 0;
      for(size_t cK = 0; cK < contactKinematics.size(); ++cK)
        n += contactKinematics[cK]->getNumberOfPotentialContactPoints();

      //TODO: Change this if la should be the vector of nonsmooth forces
      la.resize(n * (1 + getFrictionDirections()));
      g.resize(n);
      gd.resize(n * (1 + getFrictionDirections()));
      gdd.resize(gd.size());
      gdn.resize(gd.size());
      //LinkStatusSize= n;
      //LinkStatus.resize(LinkStatusSize);


      // clear container first, because InitStage resize is called twice (before and after the reorganization)
      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {
        if (cpData.size()) {

          //clear cpData
          for (vector<ContourPointData*>::iterator i = cpData[cK].begin(); i != cpData[cK].end(); ++i) {
            delete[] *i;
          }
          cpData[cK].clear();

          //clear Wk[0]
          for (vector<Mat*>::iterator i = Wk[0][cK].begin(); i != Wk[0][cK].end(); ++i) {
            delete[] *i;
          }
          Wk[0][cK].clear();

          //clear Wk[1]
          for (vector<Mat*>::iterator i = Wk[1][cK].begin(); i != Wk[1][cK].end(); ++i) {
            delete[] *i;
          }
          Wk[1][cK].clear();

          //clear Vk[0]
          for (vector<Mat*>::iterator i = Vk[0][cK].begin(); i != Vk[0][cK].end(); ++i) {
            delete[] *i;
          }
          Vk[0][cK].clear();

          //clear Vk[1]
          for (vector<Mat*>::iterator i = Vk[1][cK].begin(); i != Vk[1][cK].end(); ++i) {
            delete[] *i;
          }
          Vk[1][cK].clear();

          //clear fF
          for (vector<Mat*>::iterator i = fF[cK].begin(); i != fF[cK].end(); ++i) {
            delete[] *i;
          }
          fF[cK].clear();

          //clear WF
          for (vector<Vec*>::iterator i = WF[cK].begin(); i != WF[cK].end(); ++i) {
            delete[] *i;
          }
          WF[cK].clear();
        }
      }

      cpData.clear();
      Wk[0].clear();
      Wk[1].clear();
      Vk[0].clear();
      Vk[1].clear();
      fF.clear();
      WF.clear();


      for (size_t cK = 0; cK != contactKinematics.size(); ++cK) {

        cpData.push_back(vector<ContourPointData*>());
        Wk[0].push_back(vector<Mat*>());
        Wk[1].push_back(vector<Mat*>());
        Vk[0].push_back(vector<Mat*>());
        Vk[1].push_back(vector<Mat*>());
        fF.push_back(vector<Mat*>());
        WF.push_back(vector<Vec*>());

        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (getFrictionDirections() == 0)
            gdActive[cK][k][1] = false;

          int contourIndex = 2 * cK;

          cpData[cK].push_back(new ContourPointData[2]);

          cpData[cK][k][0].getFrameOfReference().setName("0");
          cpData[cK][k][1].getFrameOfReference().setName("1");
          cpData[cK][k][0].getFrameOfReference().getJacobianOfTranslation(0).resize();
          cpData[cK][k][0].getFrameOfReference().getJacobianOfTranslation(1).resize();
          cpData[cK][k][0].getFrameOfReference().getJacobianOfRotation(1).resize();
          cpData[cK][k][0].getFrameOfReference().getJacobianOfRotation(0).resize();
          cpData[cK][k][1].getFrameOfReference().getJacobianOfTranslation(0).resize();
          cpData[cK][k][1].getFrameOfReference().getJacobianOfTranslation(1).resize();
          cpData[cK][k][1].getFrameOfReference().getJacobianOfRotation(0).resize();
          cpData[cK][k][1].getFrameOfReference().getJacobianOfRotation(1).resize();

          cpData[cK][k][0].getFrameOfReference().sethSize(contour[contourIndex]->gethSize(0), 0);
          cpData[cK][k][0].getFrameOfReference().sethSize(contour[contourIndex]->gethSize(1), 1);
          cpData[cK][k][1].getFrameOfReference().sethSize(contour[contourIndex + 1]->gethSize(0), 0);
          cpData[cK][k][1].getFrameOfReference().sethSize(contour[contourIndex + 1]->gethSize(1), 1);

          int laSizek = gdActive[cK][k][0] + gdActive[cK][k][1] * getFrictionDirections();

          Wk[0][cK].push_back(new Mat[2]);
          Wk[0][cK][k][0].resize(contour[contourIndex]->gethSize(), laSizek);
          Wk[0][cK][k][1].resize(contour[contourIndex + 1]->gethSize(), laSizek);

          Vk[0][cK].push_back(new Mat[2]);
          Vk[0][cK][k][0].resize(contour[contourIndex]->gethSize(), laSizek);
          Vk[0][cK][k][1].resize(contour[contourIndex + 1]->gethSize(), laSizek);

          Wk[1][cK].push_back(new Mat[2]);
          Wk[1][cK][k][0].resize(contour[contourIndex]->gethSize(1), laSizek);
          Wk[1][cK][k][1].resize(contour[contourIndex + 1]->gethSize(1), laSizek);

          Vk[1][cK].push_back(new Mat[2]);
          Vk[1][cK][k][0].resize(contour[contourIndex]->gethSize(1), laSizek);
          Vk[1][cK][k][1].resize(contour[contourIndex + 1]->gethSize(1), laSizek);

          fF[cK].push_back(new Mat[2]);
          fF[cK][k][0].resize(3, laSizek);
          fF[cK][k][1].resize(3, laSizek);

          WF[cK].push_back(new Vec[2]);
          WF[cK][k][0].resize(3);
          WF[cK][k][1].resize(3);
        }
      }
    }
    else if (stage == unknownStage) {
      LinkMechanics::init(stage);

      iT = Index(1, getFrictionDirections());

      //TODO: check if indices are set correctly?
      int startIndexFriction = 0;
      int endIndexFriction = 0;
      int indexNormal = 0;
      for (size_t cK = 0; cK < contactKinematics.size(); ++cK) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          endIndexFriction = startIndexFriction + getFrictionDirections();
          lakN[cK][k].resize() >> la(startIndexFriction, startIndexFriction);
          lakT[cK][k].resize() >> la(startIndexFriction+1, endIndexFriction);

          gdkN[cK][k].resize() >> gd(startIndexFriction, startIndexFriction);
          gdkT[cK][k].resize() >> gd(startIndexFriction+1, endIndexFriction);

          gdnkN[cK][k].resize() >> gd(startIndexFriction, startIndexFriction);
          gdnkT[cK][k].resize() >> gd(startIndexFriction+1, endIndexFriction);

          gddk[cK][k].resize() >> gdd(startIndexFriction, endIndexFriction);
          gk[cK][k].resize() >> g(indexNormal, indexNormal);
          gddkBuf[cK][k].resize(1 + getFrictionDirections());
          startIndexFriction = endIndexFriction + 1;
          indexNormal++;
        }
      }
    }
    else if (stage == preInit) {
      LinkMechanics::init(stage);


      /*Set the sizes of the different vectors*/
      for(size_t cK = 0; cK < contactKinematics.size(); ++cK) {

        int contourIndex = 2 * cK;

        //assign Contours in every contact kinematic
        contactKinematics[cK]->assignContours(contour[contourIndex], contour[contourIndex + 1]);

        gActive.push_back(vector<unsigned int>());
        gActive0.push_back(vector<unsigned int>());
        gdActive.push_back(vector<unsigned int*>());
        gddActive.push_back(vector<unsigned int*>());

        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gActive[cK].push_back(int(1));
          gActive0[cK].push_back(int(1));
          gdActive[cK].push_back(new unsigned int[2]);
          gddActive[cK].push_back(new unsigned int[2]);
          for (int j = 0; j < 1 + min(1, getFrictionDirections()); j++)
            gdActive[cK][k][j] = 1;
          for (int j = 1 + min(1, getFrictionDirections()); j < 2; j++)
            gdActive[cK][j] = 0;
          for (int j = 0; j < 1 + min(1, getFrictionDirections()); j++)
            gddActive[cK][k][j] = 1;
          for (int j = 1 + min(1, getFrictionDirections()); j < 2; j++)
            gddActive[cK][j] = 0;

          gk.push_back(vector<Vec>());
          gk[cK].push_back(Vec(1));


          gdkN.push_back(vector<Vec>());
          gdkN[cK].push_back(Vec(1));

          gdkT.push_back(vector<Vec>());
          gdkT[cK].push_back(Vec(getFrictionDirections()));

          gdnkN.push_back(vector<Vec>());
          gdnkN[cK].push_back(Vec(1));

          gdnkT.push_back(vector<Vec>());
          gdnkT[cK].push_back(Vec(getFrictionDirections()));

          gddk.push_back(vector<Vec>());
          gddk[cK].push_back(Vec(1));

          gddkBuf.push_back(vector<Vec>());
          gddkBuf[cK].push_back(Vec(1));

          lakN.push_back(vector<Vec>());
          lakN[cK].push_back(Vec(1));

          lakT.push_back(vector<Vec>());
          lakT[cK].push_back(Vec(getFrictionDirections()));

          wbk.push_back(vector<Vec>());
          wbk[cK].push_back(Vec());

          svk.push_back(vector<Vec>());
          svk[cK].push_back(Vec());

          jsvk.push_back(vector<Vector<int> >());
          jsvk[cK].push_back(Vector<int>());

          rFactork.push_back(vector<Vec>());
          rFactork[cK].push_back(Vec());

          laSizek.push_back(vector<int>());
          laSizek[cK].push_back(int(0));

          laIndk.push_back(vector<int>());
          laIndk[cK].push_back(int(0));

          gSizek.push_back(vector<int>());
          gSizek[cK].push_back(int(0));

          gIndk.push_back(vector<int>());
          gIndk[cK].push_back(int(0));

          gdSizek.push_back(vector<int>());
          gdSizek[cK].push_back(int(0));

          gdIndk.push_back(vector<int>());
          gdIndk[cK].push_back(int(0));

          svSizek.push_back(vector<int>());
          svSizek[cK].push_back(int(0));

          svIndk.push_back(vector<int>());
          svIndk[cK].push_back(int(0));

          rFactorSizek.push_back(vector<int>());
          rFactorSizek[cK].push_back(int(0));

          rFactorIndk.push_back(vector<int>());
          rFactorIndk[cK].push_back(int(0));

          corrk.push_back(vector<Vec>());
          corrk[cK].push_back(Vec(1));

          corrSizek.push_back(vector<int>());
          corrSizek[cK].push_back(int(0));

          corrIndk.push_back(vector<int>());
          corrIndk[cK].push_back(int(0));

          rootID.push_back(vector<int>());
          rootID[cK].push_back(int(0));
        }
      }
    }
    else if (stage == MBSim::plot) {
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || contactArrow || frictionArrow)) {
          for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
            //One contact group for (at least) every contact kinematics:
            // 1st reason: not to much data for one H5-Group (it could be too much for many contacts)
            // 2nd reason: names for every contactKinematics (pair of two contours or contourPairing) is possible
            openMBVContactGrp.push_back(new OpenMBV::Group());
            openMBVContactGrp[cK]->setName(name + numtostr((int) cK) + "_ContactGroup"); //TODO: enable names for at least every contactKinematic (use a concept similar to the contourPairing, maybe just for plotting, or maybe for every vector thing we have now in this class)
            openMBVContactGrp[cK]->setExpand(false);
            parent->getOpenMBVGrp()->addObject(openMBVContactGrp[cK]);

            for (unsigned int k = 0; k < cpData[cK].size(); k++) {
              if (openMBVContactFrameSize > epsroot()) {
                openMBVContactFrame.push_back(vector<vector<OpenMBV::Frame*> >());

                vector<OpenMBV::Frame*> temp;
                temp.push_back(new OpenMBV::Frame);
                temp.push_back(new OpenMBV::Frame);

                openMBVContactFrame[cK].push_back(temp);
                for (unsigned int i = 0; i < 2; i++) { // frames
                  openMBVContactFrame[cK][k][i]->setOffset(1.);
                  openMBVContactFrame[cK][k][i]->setSize(openMBVContactFrameSize);
                  openMBVContactFrame[cK][k][i]->setName("ContactPoint_" + numtostr((int) k) + (i == 0 ? "A" : "B"));
                  openMBVContactFrame[cK][k][i]->setEnable(openMBVContactFrameEnabled);
                  openMBVContactGrp[cK]->addObject(openMBVContactFrame[cK][k][i]);
                }
              }
              // arrows
              OpenMBV::Arrow *arrow;
              if (contactArrow) {
                openMBVNormalForceArrow.push_back(vector<OpenMBV::Arrow*>());
                arrow = new OpenMBV::Arrow(*contactArrow);
                arrow->setName("NormalForce_" + numtostr((int) k) + "B");
                openMBVNormalForceArrow[cK].push_back(arrow); // normal force
                openMBVContactGrp[cK]->addObject(arrow);
              }
              if (frictionArrow && getFrictionDirections() > 0) { // friction force
                openMBVFrictionArrow.push_back(vector<OpenMBV::Arrow*>());
                arrow = new OpenMBV::Arrow(*frictionArrow);
                arrow->setName("FrictionForce_" + numtostr((int) k) + "B");
                openMBVFrictionArrow[cK].push_back(arrow);
                openMBVContactGrp[cK]->addObject(arrow);
              }
            }
          }
        }
#endif
        for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
          if (getPlotFeature(linkKinematics) == enabled) {
            for (int i = 0; i < contactKinematics[cK]->getNumberOfPotentialContactPoints(); i++) {
              plotColumns.push_back("g[" + numtostr(i) + "](" + numtostr(0) + ")");
              for (int j = 0; j < 1 + getFrictionDirections(); ++j)
                plotColumns.push_back("gd[" + numtostr(i) + "](" + numtostr(j) + ")");
            }
          }
          if (getPlotFeature(generalizedLinkForce) == enabled) {
            for (int i = 0; i < contactKinematics[cK]->getNumberOfPotentialContactPoints(); i++) {
              for (int j = 0; j < 1 + getFrictionDirections(); ++j)
                plotColumns.push_back("la[" + numtostr(i) + "](" + numtostr(j) + ")");
            }
          }
        }
        PlotFeatureStatus pfKinematics = getPlotFeature(linkKinematics);
        PlotFeatureStatus pfKinetics = getPlotFeature(generalizedLinkForce);
        setPlotFeature(linkKinematics, disabled);
        setPlotFeature(generalizedLinkForce, disabled);
        LinkMechanics::init(stage);
        setPlotFeature(linkKinematics, pfKinematics);
        setPlotFeature(generalizedLinkForce, pfKinetics);
      }

    }
    else
      LinkMechanics::init(stage);
  }

  bool Contact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if (fdf)
      flag |= fdf->isSetValued();
    return flag;
  }

  bool Contact::isSingleValued() const {
    if(fcl->isSetValued()) {
      if(fdf) {
        return not fdf->isSetValued();
      }
      return false;
    }
    return true;
  }

  //  void Contact::updateLinkStatus(double t) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //        LinkStatus(k) = 2;
  //        if (ftil) {
  //          if (ftil->isSticking(lak[k](1, getFrictionDirections()), gdnk[k](1, getFrictionDirections()), gdk[k](1, getFrictionDirections()), lak[k](0), LaTol, gdTol))
  //            LinkStatus(k) = 3;
  //          else
  //            LinkStatus(k) = 4;
  //        }
  //      }
  //      else
  //        LinkStatus(k) = 1;
  //    }
  //  }

  void Contact::updateLinkStatus(double t) {
    int linkStatusIndex = 0;
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {
          LinkStatus(linkStatusIndex) = 2;
          if (ftil) {
            if (ftil->isSticking(lakT[cK][k], gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0), LaTol, gdTol))
              LinkStatus(linkStatusIndex) = 3;
            else
              LinkStatus(linkStatusIndex) = 4;
          }
        }
        else
          LinkStatus(linkStatusIndex) = 1;
        linkStatusIndex++;
      }
    }
  }

  //  void Contact::updateLinkStatusReg(double t) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //        LinkStatusReg(k) = 2;
  //      }
  //      else {
  //        LinkStatusReg(k) = 1;
  //      }
  //    }
  //  }

  void Contact::updateLinkStatusReg(double t) {
    int linkStatusIndex = 0;
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {
          LinkStatusReg(linkStatusIndex) = 2;
        }
        else {
          LinkStatusReg(linkStatusIndex) = 1;
        }
        linkStatusIndex++;
      }
    }
  }

  //  bool Contact::isActive() const {
  //    for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //      if (gActive[i])
  //        return true;
  //    }
  //    return false;
  //  }

  bool Contact::isActive() const {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k])
          return true;
      }
    }
    return false;
  }

  //  bool Contact::gActiveChanged() {
  //    bool changed = false;
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive0[k] != gActive[k])
  //        changed = true;
  //      gActive0[k] = gActive[k];
  //    }
  //    return changed;
  //  }

  bool Contact::gActiveChanged() {
    bool changed = false;
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive0[cK][k] != gActive[cK][k])
          changed = true;
        gActive0[cK][k] = gActive[cK][k];
      }
    }
    return changed;
  }

  //  void Contact::plot(double t, double dt) {
  //    if (getPlotFeature(plotRecursive) == enabled) {
  //#ifdef HAVE_OPENMBVCPPINTERFACE
  //      if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || contactArrow || frictionArrow)) {
  //        for (unsigned int i = 0; i < cpData.size(); i++) {
  //          // frames
  //          if (openMBVContactFrameSize > epsroot()) {
  //            for (unsigned int k = 0; k < 2; k++) {
  //              vector<double> data;
  //              data.push_back(t);
  //              data.push_back(cpData[i][k].getFrameOfReference().getPosition()(0));
  //              data.push_back(cpData[i][k].getFrameOfReference().getPosition()(1));
  //              data.push_back(cpData[i][k].getFrameOfReference().getPosition()(2));
  //              Vec cardan = AIK2Cardan(cpData[i][k].getFrameOfReference().getOrientation());
  //              data.push_back(cardan(0));
  //              data.push_back(cardan(1));
  //              data.push_back(cardan(2));
  //              data.push_back(0);
  //              openMBVContactFrame[i][k]->append(data);
  //            }
  //          }
  //          // arrows
  //          // normal force
  //          vector<double> data;
  //          if (contactArrow) {
  //            data.push_back(t);
  //            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(0));
  //            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(1));
  //            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(2));
  //            Vec F(3, INIT, 0);
  //            if (isSetValued()) {
  //              if (gActive[i])
  //                F = fF[i][1].col(0) * lak[i](0) / dt;
  //            }
  //            else
  //              F = cpData[i][0].getFrameOfReference().getOrientation().col(0) * lak[i](0);
  //            data.push_back(F(0));
  //            data.push_back(F(1));
  //            data.push_back(F(2));
  //            data.push_back(nrm2(F));
  //            openMBVNormalForceArrow[i]->append(data);
  //          }
  //          if (frictionArrow && getFrictionDirections() > 0) { // friction force
  //            data.clear();
  //            data.push_back(t);
  //            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(0));
  //            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(1));
  //            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(2));
  //            Vec F(3, INIT, 0);
  //            if (isSetValued()) {                    // TODO switch between stick and slip not possible with TimeStepper
  //              if (gActive[i] && lak[i].size() > 1) { // stick friction
  //                F = fF[i][1].col(1) * lak[i](1) / dt;
  //                if (getFrictionDirections() > 1)
  //                  F += fF[i][1].col(2) * lak[i](2) / dt;
  //              }
  //              if (gActive[i] && lak[i].size() == 1) // slip friction
  //                F = fF[i][1](Index(0, 2), iT) * fdf->dlaTdlaN(gdk[i](1, getFrictionDirections()), lak[i](0)) * lak[i](0) / dt;
  //            }
  //            else {
  //              F = cpData[i][0].getFrameOfReference().getOrientation().col(1) * lak[i](1);
  //              if (getFrictionDirections() > 1)
  //                F += cpData[i][0].getFrameOfReference().getOrientation().col(2) * lak[i](2);
  //            }
  //            data.push_back(F(0));
  //            data.push_back(F(1));
  //            data.push_back(F(2));
  //            data.push_back((isSetValued() && lak[i].size() > 1) ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
  //            openMBVFrictionArrow[i]->append(data);
  //          }
  //        }
  //      }
  //#endif
  //      if (getPlotFeature(linkKinematics) == enabled) {
  //        bool flag = fcl->isSetValued();
  //        for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //          plotVector.push_back(gk[i](0)); //gN
  //          if ((flag && gActive[i]) || (!flag && fcl->isActive(gk[i](0), 0))) {
  //            for (int j = 0; j < 1 + getFrictionDirections(); j++)
  //              plotVector.push_back(gdk[i](j)); //gd
  //          }
  //          else {
  //            for (int j = 0; j < 1 + getFrictionDirections(); j++)
  //              plotVector.push_back(NAN); //gd
  //          }
  //        }
  //      }
  //      if (getPlotFeature(generalizedLinkForce) == enabled) {
  //        for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //          if (gActive[i] && gdActive[i][0]) {
  //            plotVector.push_back(lak[i](0) / (isSetValued() ? dt : 1.));
  //            if (gdActive[i][1]) {
  //              for (int j = 0; j < getFrictionDirections(); j++)
  //                plotVector.push_back(lak[i](1 + j) / (isSetValued() ? dt : 1.));
  //            }
  //            else {
  //              if (fdf) {
  //                Vec buf = fdf->dlaTdlaN(gdk[i](1, getFrictionDirections()), lak[i](0)) * lak[i](0);
  //                for (int j = 0; j < getFrictionDirections(); j++)
  //                  plotVector.push_back(buf(j) / (isSetValued() ? dt : 1.));
  //              }
  //            }
  //          }
  //          else {
  //            for (int j = 0; j < 1 + getFrictionDirections(); j++)
  //              plotVector.push_back(0);
  //          }
  //        }
  //      }
  //      PlotFeatureStatus pfKinematics = getPlotFeature(linkKinematics);
  //      PlotFeatureStatus pfKinetics = getPlotFeature(generalizedLinkForce);
  //      setPlotFeature(linkKinematics, disabled);
  //      setPlotFeature(generalizedLinkForce, disabled);
  //      LinkMechanics::plot(t, dt);
  //      setPlotFeature(linkKinematics, pfKinematics);
  //      setPlotFeature(generalizedLinkForce, pfKinetics);
  //    }
  //  }

  void Contact::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || contactArrow || frictionArrow)) {
        for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
          for (unsigned int k = 0; k < cpData[cK].size(); k++) {
            // frames
            if (openMBVContactFrameSize > epsroot()) {
              for (unsigned int i = 0; i < 2; i++) {
                vector<double> data;
                data.push_back(i);
                data.push_back(cpData[cK][k][i].getFrameOfReference().getPosition()(0));
                data.push_back(cpData[cK][k][i].getFrameOfReference().getPosition()(1));
                data.push_back(cpData[cK][k][i].getFrameOfReference().getPosition()(2));
                Vec cardan = AIK2Cardan(cpData[cK][k][i].getFrameOfReference().getOrientation());
                data.push_back(cardan(0));
                data.push_back(cardan(1));
                data.push_back(cardan(2));
                data.push_back(0);
                openMBVContactFrame[cK][k][i]->append(data);
              }
            }
            // arrows
            // normal force
            vector<double> data;
            if (contactArrow) {
              data.push_back(t);
              data.push_back(cpData[cK][k][1].getFrameOfReference().getPosition()(0));
              data.push_back(cpData[cK][k][1].getFrameOfReference().getPosition()(1));
              data.push_back(cpData[cK][k][1].getFrameOfReference().getPosition()(2));
              Vec F(3, INIT, 0);
              if (fcl->isSetValued()) {
                if (gActive[cK][k])
                  F = fF[cK][k][1].col(0) * lakN[cK][k] / dt;
              }
              else
                F = cpData[cK][k][0].getFrameOfReference().getOrientation().col(0) * lakN[cK][k];
              data.push_back(F(0));
              data.push_back(F(1));
              data.push_back(F(2));
              data.push_back(nrm2(F));
              openMBVNormalForceArrow[cK][k]->append(data);
            }
            if (frictionArrow && getFrictionDirections() > 0) { // friction force
              data.clear();
              data.push_back(t);
              data.push_back(cpData[cK][k][1].getFrameOfReference().getPosition()(0));
              data.push_back(cpData[cK][k][1].getFrameOfReference().getPosition()(1));
              data.push_back(cpData[cK][k][1].getFrameOfReference().getPosition()(2));
              Vec F(3, INIT, 0);
              if (fdf->isSetValued()) {                    // TODO switch between stick and slip not possible with TimeStepper
                if (gActive[cK][k] && lakT[cK][k].size()) { // stick friction
                  F = fF[cK][k][1].col(1) * lakT[cK][k](0) / dt;
                  if (getFrictionDirections() > 1)
                    F += fF[cK][k][1].col(2) * lakT[cK][k](1) / dt;
                }
                if (gActive[cK][k] && lakT[cK][k].size() == 0) // slip friction
                  F = fF[cK][k][1](Index(0, 2), iT) * fdf->dlaTdlaN(gdkT[cK][k], lakN[cK][k](0)) * lakN[cK][k](0) / dt;
              }
              else {
                F = cpData[cK][k][0].getFrameOfReference().getOrientation().col(1) * lakT[cK][k](0);
                if (getFrictionDirections() > 1)
                  F += cpData[cK][k][0].getFrameOfReference().getOrientation().col(2) * lakT[cK][k](1);
              }
              data.push_back(F(0));
              data.push_back(F(1));
              data.push_back(F(2));
              data.push_back((fdf->isSetValued() && lakT[cK][k].size()) ? 1 : 0.5); // draw in green if slipping and draw in red if sticking
              openMBVFrictionArrow[cK][k]->append(data);
            }
          }
        }
      }
#endif
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        if (getPlotFeature(linkKinematics) == enabled) {
          bool flag = fcl->isSetValued();
          for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
            plotVector.push_back(gk[cK][k](0)); //gN
            if ((flag && gActive[cK][k]) || (!flag && fcl->isActive(gk[cK][k](0), 0))) {
              plotVector.push_back(gdkN[cK][k](0)); //gd-Normal
              for (int j = 0; j < getFrictionDirections(); j++)
                plotVector.push_back(gdkT[cK][k](j)); //gd-Tangential
            }
            else {
              for (int j = 0; j < 1 + getFrictionDirections(); j++)
                plotVector.push_back(NAN); //gd
            }
          }
        }
        if (getPlotFeature(generalizedLinkForce) == enabled) {
          for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
            if (gActive[cK][k] && gdActive[cK][k][0]) {
              plotVector.push_back(lakN[cK][k](0) / (fcl->isSetValued() ? dt : 1.));
              if (gdActive[cK][k][1]) {
                for (int j = 0; j < getFrictionDirections(); j++)
                  plotVector.push_back(lakT[cK][k](j) / (fdf->isSetValued() ? dt : 1.));
              }
              else {
                if (fdf) {
                  Vec buf = fdf->dlaTdlaN(gdkT[cK][k], lakN[cK][k](0)) * lakN[cK][k](0);
                  for (int j = 0; j < getFrictionDirections(); j++)
                    plotVector.push_back(buf(j) / (fdf->isSetValued() ? dt : 1.));
                }
              }
            }
            else {
              for (int j = 0; j < 1 + getFrictionDirections(); j++)
                plotVector.push_back(0);
            }
          }
        }
      }
      PlotFeatureStatus pfKinematics = getPlotFeature(linkKinematics);
      PlotFeatureStatus pfKinetics = getPlotFeature(generalizedLinkForce);
      setPlotFeature(linkKinematics, disabled);
      setPlotFeature(generalizedLinkForce, disabled);
      LinkMechanics::plot(t, dt);
      setPlotFeature(linkKinematics, pfKinematics);
      setPlotFeature(generalizedLinkForce, pfKinetics);
    }
  }

  void Contact::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      LinkMechanics::closePlot();
    }
  }

  //  void Contact::solveImpactsFixpointSingle(double dt) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        gdnk[k](0) = b(laInd + laIndk[k]);
  //        for (int j = ia[laInd + laIndk[k]]; j < ia[laInd + laIndk[k] + 1]; j++)
  //          gdnk[k](0) += a[j] * laMBS(ja[j]);
  //
  //        lak[k](0) = fnil->project(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));
  //
  //        for (int i = 1; i <= getFrictionDirections(); i++) {
  //          gdnk[k](i) = b(laInd + laIndk[k] + i);
  //          for (int j = ia[laInd + laIndk[k] + i]; j < ia[laInd + laIndk[k] + 1 + i]; j++)
  //            gdnk[k](i) += a[j] * laMBS(ja[j]);
  //        }
  //
  //        if (ftil)
  //          lak[k](1, getFrictionDirections()) = ftil->project(lak[k](1, getFrictionDirections()), gdnk[k](1, getFrictionDirections()), gdk[k](1, getFrictionDirections()), lak[k](0), rFactork[k](1));
  //      }
  //    }
  //  }

  void Contact::solveImpactsFixpointSingle(double dt) {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          int addIndexNormal = 0;
          double scaleFactorN = dt;
          if(fcl->isSetValued()) {
            scaleFactorN = 1;
            addIndexNormal++;
            gdnkN[cK][k](0) = b(laInd + laIndk[cK][k]);
            for (int j = ia[laInd + laIndk[cK][k]]; j < ia[laInd + laIndk[cK][k] + 1]; j++)
              gdnkN[cK][k](0) += a[j] * laMBS(ja[j]);

            lakN[cK][k](0) = fnil->project(lakN[cK][k](0), gdnkN[cK][k](0), gdkN[cK][k](0), rFactork[cK][k](0));
          }

          if(fdf->isSetValued()) {
            for (int i = 0; i < getFrictionDirections(); i++) {
              gdnkT[cK][k](i) = b(laInd + laIndk[cK][k] + addIndexNormal + i);
              for (int j = ia[laInd + laIndk[cK][k] + i + addIndexNormal]; j < ia[laInd + laIndk[cK][k] + 1 + i + addIndexNormal]; j++)
                gdnkT[cK][k](i) += a[j] * laMBS(ja[j]);
            }

//            if (ftil) //There must be a ftil coming with a setValued fdf
            lakT[cK][k] = ftil->project(lakT[cK][k], gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0) * scaleFactorN, rFactork[cK][k](1));
          }
        }
      }
    }
  }

  //  void Contact::solveConstraintsFixpointSingle() {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gdActive[k][0]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        gddk[k](0) = b(laInd + laIndk[k]);
  //        for (int j = ia[laInd + laIndk[k]]; j < ia[laInd + laIndk[k] + 1]; j++)
  //          gddk[k](0) += a[j] * laMBS(ja[j]);
  //
  //        lak[k](0) = fcl->project(lak[k](0), gddk[k](0), rFactork[k](0));
  //
  //        if (gdActive[k][1]) {
  //          for (int i = 1; i <= getFrictionDirections(); i++) {
  //            gddk[k](i) = b(laInd + laIndk[k] + i);
  //            for (int j = ia[laInd + laIndk[k] + i]; j < ia[laInd + laIndk[k] + 1 + i]; j++)
  //              gddk[k](i) += a[j] * laMBS(ja[j]);
  //          }
  //
  //          if (fdf)
  //            lak[k](1, getFrictionDirections()) = fdf->project(lak[k](1, getFrictionDirections()), gddk[k](1, getFrictionDirections()), lak[k](0), rFactork[k](1));
  //        }
  //      }
  //    }
  //  }

  void Contact::solveConstraintsFixpointSingle() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gdActive[cK][k][0]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          int addIndexnormal = 0;
          if(fcl->isSetValued()) {
            addIndexnormal++;
            gddk[cK][k](0) = b(laInd + laIndk[cK][k]);
            for (int j = ia[laInd + laIndk[cK][k]]; j < ia[laInd + laIndk[cK][k] + 1]; j++)
              gddk[cK][k](0) += a[j] * laMBS(ja[j]);

            lakN[cK][k](0) = fcl->project(lakN[cK][k](0), gddk[cK][k](0), rFactork[cK][k](0));
          }

          if (fdf) {
            if (fdf->isSetValued() and gdActive[cK][k][1]) {
              for (int i = 1; i < getFrictionDirections() + 1; i++) {
                gddk[cK][k](i) = b(laInd + laIndk[cK][k] + i + addIndexnormal);
                for (int j = ia[laInd + laIndk[cK][k] + i + addIndexnormal]; j < ia[laInd + laIndk[cK][k] + 1 + i + addIndexnormal]; j++)
                  gddk[cK][k](i) += a[j] * laMBS(ja[j]);
              }

              lakT[cK][k] = fdf->project(lakT[cK][k], gddk[cK][k](1, getFrictionDirections()), lakN[cK][k](0), rFactork[cK][k](1));
            }
          }
        }
      }
    }
  }

  //  void Contact::solveImpactsGaussSeidel(double dt) {
  //    assert(getFrictionDirections() <= 1);
  //
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        gdnk[k](0) = b(laInd + laIndk[k]);
  //        for (int j = ia[laInd + laIndk[k]] + 1; j < ia[laInd + laIndk[k] + 1]; j++)
  //          gdnk[k](0) += a[j] * laMBS(ja[j]);
  //
  //        const double om = 1.0;
  //        const double buf = fnil->solve(a[ia[laInd + laIndk[k]]], gdnk[k](0), gdk[k](0));
  //        lak[k](0) += om * (buf - lak[k](0));
  //
  //        if (getFrictionDirections()) {
  //          gdnk[k](1) = b(laInd + laIndk[k] + 1);
  //          for (int j = ia[laInd + laIndk[k] + 1] + 1; j < ia[laInd + laIndk[k] + 2]; j++)
  //            gdnk[k](1) += a[j] * laMBS(ja[j]);
  //
  //          if (ftil) {
  //            Vec buf = ftil->solve(ds->getG()(Index(laInd + laIndk[k] + 1, laInd + laIndk[k] + getFrictionDirections())), gdnk[k](1, getFrictionDirections()), gdk[k](1, getFrictionDirections()), lak[k](0));
  //            lak[k](1, getFrictionDirections()) += om * (buf - lak[k](1, getFrictionDirections()));
  //          }
  //        }
  //      }
  //    }
  //  }

  void Contact::solveImpactsGaussSeidel(double dt) {
    assert(getFrictionDirections() <= 1);
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          //TODO: check indices (in other solution algorithms too!)
          int addIndexnormal = 0;
          if(fcl->isSetValued()) {
            addIndexnormal++;
            gdnkN[cK][k](0) = b(laInd + laIndk[cK][k]);
            for (int j = ia[laInd + laIndk[cK][k]] + 1; j < ia[laInd + laIndk[cK][k] + 1]; j++)
              gdnkN[cK][k](0) += a[j] * laMBS(ja[j]);

            const double om = 1.0;
            const double buf = fnil->solve(a[ia[laInd + laIndk[cK][k]]], gdnkN[cK][k](0), gdkN[cK][k](0));
            lakN[cK][k](0) += om * (buf - lakN[cK][k](0));
          }

          if (getFrictionDirections()) {
            if(fdf->isSetValued()) {
              gdnkT[cK][k](0) = b(laInd + laIndk[cK][k] + addIndexnormal);
              for (int j = ia[laInd + laIndk[cK][k] + addIndexnormal] + 1; j < ia[laInd + laIndk[cK][k] + addIndexnormal + 1]; j++)
                gdnkT[cK][k](0) += a[j] * laMBS(ja[j]);

              if (ftil) {
                Vec buf = ftil->solve(ds->getG()(Index(laInd + laIndk[cK][k] + addIndexnormal, laInd + laIndk[cK][k] + getFrictionDirections())), gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0));
                lakT[cK][k] += 1.0 * (buf - lakT[cK][k]); //TODO: why om?
              }
            }
          }
        }
      }
    }
  }

  //  void Contact::solveConstraintsGaussSeidel() {
  //    assert(getFrictionDirections() <= 1);
  //
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gdActive[k][0]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        gddk[k](0) = b(laInd + laIndk[k]);
  //        for (int j = ia[laInd + laIndk[k]] + 1; j < ia[laInd + laIndk[k] + 1]; j++)
  //          gddk[k](0) += a[j] * laMBS(ja[j]);
  //
  //        const double om = 1.0; // relaxation parameter omega (cf. Foerg, dissertation, p. 102)
  //        const double buf = fcl->solve(a[ia[laInd + laIndk[k]]], gddk[k](0));
  //        lak[k](0) += om * (buf - lak[k](0));
  //
  //        if (getFrictionDirections() && gdActive[k][1]) {
  //          gddk[k](1) = b(laInd + laIndk[k] + 1);
  //          for (int j = ia[laInd + laIndk[k] + 1] + 1; j < ia[laInd + laIndk[k] + 2]; j++)
  //            gddk[k](1) += a[j] * laMBS(ja[j]);
  //
  //          if (fdf) {
  //            Vec buf = fdf->solve(ds->getG()(Index(laInd + laIndk[k] + 1, laInd + laIndk[k] + getFrictionDirections())), gddk[k](1, getFrictionDirections()), lak[k](0));
  //            lak[k](1, getFrictionDirections()) += om * (buf - lak[k](1, getFrictionDirections()));
  //          }
  //        }
  //      }
  //    }
  //  }

  void Contact::solveConstraintsGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gdActive[cK][k][0]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          gddk[cK][k](0) = b(laInd + laIndk[cK][k]);
          for (int j = ia[laInd + laIndk[cK][k]] + 1; j < ia[laInd + laIndk[cK][k] + 1]; j++)
            gddk[cK][k](0) += a[j] * laMBS(ja[j]);

          const double om = 1.0; // relaxation parameter omega (cf. Foerg, dissertation, p. 102)
          const double buf = fcl->solve(a[ia[laInd + laIndk[cK][k]]], gddk[cK][k](0));
          lakN[cK][k](0) += om * (buf - lakN[cK][k](0));

          if (getFrictionDirections() && gdActive[cK][k][1]) {
            gddk[cK][k](1) = b(laInd + laIndk[cK][k] + 1);
            for (int j = ia[laInd + laIndk[cK][k] + 1] + 1; j < ia[laInd + laIndk[cK][k] + 2]; j++)
              gddk[cK][k](1) += a[j] * laMBS(ja[j]);

            if (fdf) {
              Vec buf = fdf->solve(ds->getG()(Index(laInd + laIndk[cK][k] + 1, laInd + laIndk[cK][k] + getFrictionDirections())), gddk[cK][k](1, getFrictionDirections()), lakN[cK][k](0));
              lakT[cK][k] += om * (buf - lakT[cK][k]);
            }
          }
        }
      }
    }
  }

  //  void Contact::solveImpactsRootFinding(double dt) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        for (int i = 0; i < 1 + getFrictionDirections(); i++) {
  //          gdnk[k](i) = b(laInd + laIndk[k] + i);
  //          for (int j = ia[laInd + laIndk[k] + i]; j < ia[laInd + laIndk[k] + 1 + i]; j++)
  //            gdnk[k](i) += a[j] * laMBS(ja[j]);
  //        }
  //
  //        res(0) = lak[k](0) - fnil->project(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));
  //        if (ftil)
  //          res(1, getFrictionDirections()) = lak[k](1, getFrictionDirections()) - ftil->project(lak[k](1, getFrictionDirections()), gdnk[k](1, getFrictionDirections()), gdk[k](1, getFrictionDirections()), lak[k](0), rFactork[k](1));
  //      }
  //    }
  //  }

  void Contact::solveImpactsRootFinding(double dt) {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          //compute residuum for normal direction
          int addIndexnormal = 0;
          if(fcl->isSetValued()) {
            addIndexnormal++;
            gdnkN[cK][k](0) = b(laInd + laIndk[cK][k]);
            for (int j = ia[laInd + laIndk[cK][k]]; j < ia[laInd + laIndk[cK][k] + 1]; j++)
              gdnkN[cK][k](0) += a[j] * laMBS(ja[j]);

            res(0) = lakN[cK][k](0) - fnil->project(lakN[cK][k](0), gdnkN[cK][k](0), gdkN[cK][k](0), rFactork[cK][k](0));
          }

          //compute residuum for tangential directions
          if(fdf->isSetValued()) {
            for (int i = 0; i < getFrictionDirections(); i++) {
              gdnkT[cK][k](i) = b(laInd + laIndk[cK][k] + i + addIndexnormal);
              for (int j = ia[laInd + laIndk[cK][k] + i + addIndexnormal]; j < ia[laInd + laIndk[cK][k] + 1 + i + addIndexnormal]; j++)
                gdnkT[cK][k](i) += a[j] * laMBS(ja[j]);
            }
//            if (ftil) There must be a frictional impact law if fdf is set valued!
            res(addIndexnormal, addIndexnormal + getFrictionDirections() - 1) = lakT[cK][k] - ftil->project(lakT[cK][k], gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0), rFactork[cK][k](1));
          }
        }
      }
    }
  }

  //  void Contact::solveConstraintsRootFinding() {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gdActive[k][0]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        for (int i = 0; i < 1 + getFrictionDirections(); i++) {
  //          gddk[k](i) = b(laInd + laIndk[k] + i);
  //          for (int j = ia[laInd + laIndk[k] + i]; j < ia[laInd + laIndk[k] + 1 + i]; j++)
  //            gddk[k](i) += a[j] * laMBS(ja[j]);
  //        }
  //
  //        res(0) = lak[k](0) - fcl->project(lak[k](0), gddk[k](0), rFactork[k](0));
  //        if (fdf)
  //          res(1, getFrictionDirections()) = lak[k](1, getFrictionDirections()) - fdf->project(lak[k](1, getFrictionDirections()), gddk[k](1, getFrictionDirections()), lak[k](0), rFactork[k](1));
  //      }
  //    }
  //  }

  void Contact::solveConstraintsRootFinding() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gdActive[cK][k][0]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          for (int i = 0; i < 1 + getFrictionDirections(); i++) {
            gddk[cK][k](i) = b(laInd + laIndk[cK][k] + i);
            for (int j = ia[laInd + laIndk[cK][k] + i]; j < ia[laInd + laIndk[cK][k] + 1 + i]; j++)
              gddk[cK][k](i) += a[j] * laMBS(ja[j]);
          }

          res(0) = lakN[cK][k](0) - fcl->project(lakN[cK][k](0), gddk[cK][k](0), rFactork[cK][k](0));
          if (fdf)
            res(1, getFrictionDirections()) = lakT[cK][k] - fdf->project(lakT[cK][k], gddk[cK][k](1, getFrictionDirections()), lakN[cK][k](0), rFactork[cK][k](1));
        }
      }
    }
  }

  //  void Contact::jacobianConstraints() {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gdActive[k][0]) {
  //
  //        const SqrMat Jprox = ds->getJprox();
  //        const SqrMat G = ds->getG();
  //
  //        RowVec jp1 = Jprox.row(laInd + laIndk[k]);
  //        RowVec e1(jp1.size());
  //        e1(laInd + laIndk[k]) = 1;
  //        Vec diff = fcl->diff(lak[k](0), gddk[k](0), rFactork[k](0));
  //
  //        jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk[k])
  //        for (int i = 0; i < G.size(); i++)
  //          jp1(i) -= diff(1) * G(laInd + laIndk[k], i);
  //
  //        if (getFrictionDirections() == 1) {
  //          Mat diff = fdf->diff(lak[k](1, 1), gddk[k](1, 1), lak[k](0), rFactork[k](1));
  //          RowVec jp2 = Jprox.row(laInd + laIndk[k] + 1);
  //          RowVec e2(jp2.size());
  //          e2(laInd + laIndk[k] + 1) = 1;
  //          Mat e(2, jp2.size());
  //          e(0, laInd + laIndk[k]) = 1;
  //          e(1, laInd + laIndk[k] + 1) = 1;
  //          jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk[k])
  //          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk[k])
  //          for (int i = 0; i < G.size(); i++)
  //            jp2(i) -= diff(0, 1) * G(laInd + laIndk[k] + 1, i);
  //
  //        }
  //        else if (getFrictionDirections() == 2) {
  //          Mat diff = ftil->diff(lak[k](1, 2), gddk[k](1, 2), gdk[k](1, 2), lak[k](0), rFactork[k](1));
  //          Mat jp2 = Jprox(Index(laInd + laIndk[k] + 1, laInd + laIndk[k] + 2), Index(0, Jprox.cols() - 1));
  //          Mat e2(2, jp2.cols());
  //          e2(0, laInd + laIndk[k] + 1) = 1;
  //          e2(1, laInd + laIndk[k] + 2) = 1;
  //          jp2 = e2 - diff(Index(0, 1), Index(4, 4)) * e1 - diff(Index(0, 1), Index(0, 1)) * e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+2),Index(0,G.size()-1))
  //          for (int i = 0; i < G.size(); i++) {
  //            jp2(0, i) = diff(0, 2) * G(laInd + laIndk[k] + 1, i) + diff(0, 3) * G(laInd + laIndk[k] + 2, i);
  //            jp2(1, i) = diff(1, 2) * G(laInd + laIndk[k] + 1, i) + diff(1, 3) * G(laInd + laIndk[k] + 2, i);
  //          }
  //        }
  //      }
  //    }
  //  }

  void Contact::jacobianConstraints() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gdActive[cK][k][0]) {

          const SqrMat Jprox = ds->getJprox();
          const SqrMat G = ds->getG();

          RowVec jp1 = Jprox.row(laInd + laIndk[cK][k]);
          RowVec e1(jp1.size());
          e1(laInd + laIndk[cK][k]) = 1;
          Vec diff = fcl->diff(lakN[cK][k](0), gddk[cK][k](0), rFactork[cK][k](0));

          jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk[cK][k])
          for (int i = 0; i < G.size(); i++)
            jp1(i) -= diff(1) * G(laInd + laIndk[cK][k], i);

          if (getFrictionDirections() == 1) {
            Mat diff = fdf->diff(lakT[cK][k], gddk[cK][k](1, 1), lakN[cK][k](0), rFactork[cK][k](1));
            RowVec jp2 = Jprox.row(laInd + laIndk[cK][k] + 1);
            RowVec e2(jp2.size());
            e2(laInd + laIndk[cK][k] + 1) = 1;
            Mat e(2, jp2.size());
            e(0, laInd + laIndk[cK][k]) = 1;
            e(1, laInd + laIndk[cK][k] + 1) = 1;
            jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk[cK][k])
            //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk[cK][k])
            for (int i = 0; i < G.size(); i++)
              jp2(i) -= diff(0, 1) * G(laInd + laIndk[cK][k] + 1, i);

          }
          else if (getFrictionDirections() == 2) {
            Mat diff = ftil->diff(lakT[cK][k], gddk[cK][k](1, 2), gdkT[cK][k], lakN[cK][k](0), rFactork[cK][k](1));
            Mat jp2 = Jprox(Index(laInd + laIndk[cK][k] + 1, laInd + laIndk[cK][k] + 2), Index(0, Jprox.cols() - 1));
            Mat e2(2, jp2.cols());
            e2(0, laInd + laIndk[cK][k] + 1) = 1;
            e2(1, laInd + laIndk[cK][k] + 2) = 1;
            jp2 = e2 - diff(Index(0, 1), Index(4, 4)) * e1 - diff(Index(0, 1), Index(0, 1)) * e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk[cK][k]+1,laInd+laIndk[cK][k]+2),Index(0,G.size()-1))
            for (int i = 0; i < G.size(); i++) {
              jp2(0, i) = diff(0, 2) * G(laInd + laIndk[cK][k] + 1, i) + diff(0, 3) * G(laInd + laIndk[cK][k] + 2, i);
              jp2(1, i) = diff(1, 2) * G(laInd + laIndk[cK][k] + 1, i) + diff(1, 3) * G(laInd + laIndk[cK][k] + 2, i);
            }
          }
        }
      }
    }
  }

  //  void Contact::jacobianImpacts() {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //
  //        const SqrMat Jprox = ds->getJprox();
  //        const SqrMat G = ds->getG();
  //
  //        RowVec jp1 = Jprox.row(laInd + laIndk[k]);
  //        RowVec e1(jp1.size());
  //        e1(laInd + laIndk[k]) = 1;
  //        Vec diff = fnil->diff(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));
  //
  //        jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk[k])
  //        for (int i = 0; i < G.size(); i++)
  //          jp1(i) -= diff(1) * G(laInd + laIndk[k], i);
  //
  //        if (getFrictionDirections() == 1) {
  //          Mat diff = ftil->diff(lak[k](1, 1), gdnk[k](1, 1), gdk[k](1, 1), lak[k](0), rFactork[k](1));
  //          RowVec jp2 = Jprox.row(laInd + laIndk[k] + 1);
  //          RowVec e2(jp2.size());
  //          e2(laInd + laIndk[k] + 1) = 1;
  //          Mat e(2, jp2.size());
  //          e(0, laInd + laIndk[k]) = 1;
  //          e(1, laInd + laIndk[k] + 1) = 1;
  //          jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk[k])
  //          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk[k])
  //          for (int i = 0; i < G.size(); i++)
  //            jp2(i) -= diff(0, 1) * G(laInd + laIndk[k] + 1, i);
  //
  //        }
  //        else if (getFrictionDirections() == 2) {
  //          Mat diff = ftil->diff(lak[k](1, 2), gdnk[k](1, 2), gdk[k](1, 2), lak[k](0), rFactork[k](1));
  //          Mat jp2 = Jprox(Index(laInd + laIndk[k] + 1, laInd + laIndk[k] + 2), Index(0, Jprox.cols() - 1));
  //          Mat e2(2, jp2.cols());
  //          e2(0, laInd + laIndk[k] + 1) = 1;
  //          e2(1, laInd + laIndk[k] + 2) = 1;
  //          jp2 = e2 - diff(Index(0, 1), Index(4, 4)) * e1 - diff(Index(0, 1), Index(0, 1)) * e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+2),Index(0,G.size()-1))
  //          for (int i = 0; i < G.size(); i++) {
  //            jp2(0, i) = diff(0, 2) * G(laInd + laIndk[k] + 1, i) + diff(0, 3) * G(laInd + laIndk[k] + 2, i);
  //            jp2(1, i) = diff(1, 2) * G(laInd + laIndk[k] + 1, i) + diff(1, 3) * G(laInd + laIndk[k] + 2, i);
  //          }
  //        }
  //      }
  //    }
  //  }

  void Contact::jacobianImpacts() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {

          const SqrMat Jprox = ds->getJprox();
          const SqrMat G = ds->getG();

          RowVec jp1 = Jprox.row(laInd + laIndk[cK][k]);
          RowVec e1(jp1.size());
          e1(laInd + laIndk[cK][k]) = 1;
          Vec diff = fnil->diff(lakN[cK][k](0), gdnkN[cK][k](0), gdkN[cK][k](0), rFactork[cK][k](0));

          jp1 = e1 - diff(0) * e1; // -diff(1)*G.row(laInd+laIndk[cK][k])
          for (int i = 0; i < G.size(); i++)
            jp1(i) -= diff(1) * G(laInd + laIndk[cK][k], i);

          if (getFrictionDirections() == 1) {
            Mat diff = ftil->diff(lakT[cK][k], gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0), rFactork[cK][k](1));
            RowVec jp2 = Jprox.row(laInd + laIndk[cK][k] + 1);
            RowVec e2(jp2.size());
            e2(laInd + laIndk[cK][k] + 1) = 1;
            Mat e(2, jp2.size());
            e(0, laInd + laIndk[cK][k]) = 1;
            e(1, laInd + laIndk[cK][k] + 1) = 1;
            jp2 = e2 - diff(0, 2) * e1 - diff(0, 0) * e2; // -diff(1)*G.row(laInd+laIndk[cK][k])
            //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk[cK][k])
            for (int i = 0; i < G.size(); i++)
              jp2(i) -= diff(0, 1) * G(laInd + laIndk[cK][k] + 1, i);

          }
          else if (getFrictionDirections() == 2) {
            Mat diff = ftil->diff(lakT[cK][k], gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0), rFactork[cK][k](1));
            Mat jp2 = Jprox(Index(laInd + laIndk[cK][k] + 1, laInd + laIndk[cK][k] + 2), Index(0, Jprox.cols() - 1));
            Mat e2(2, jp2.cols());
            e2(0, laInd + laIndk[cK][k] + 1) = 1;
            e2(1, laInd + laIndk[cK][k] + 2) = 1;
            jp2 = e2 - diff(Index(0, 1), Index(4, 4)) * e1 - diff(Index(0, 1), Index(0, 1)) * e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk[cK][k]+1,laInd+laIndk[cK][k]+2),Index(0,G.size()-1))
            for (int i = 0; i < G.size(); i++) {
              jp2(0, i) = diff(0, 2) * G(laInd + laIndk[cK][k] + 1, i) + diff(0, 3) * G(laInd + laIndk[cK][k] + 2, i);
              jp2(1, i) = diff(1, 2) * G(laInd + laIndk[cK][k] + 1, i) + diff(1, 3) * G(laInd + laIndk[cK][k] + 2, i);
            }
          }
        }
      }
    }
  }

  //  void Contact::updaterFactors() {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gdActive[k][0]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //
  //        double sumN = 0;
  //        for (int j = ia[laInd + laIndk[k]] + 1; j < ia[laInd + laIndk[k] + 1]; j++)
  //          sumN += fabs(a[j]);
  //        const double aN = a[ia[laInd + laIndk[k]]];
  //        if (aN > sumN) {
  //          rFactorUnsure(0) = 0;
  //          rFactork[k](0) = 1.0 / aN;
  //        }
  //        else {
  //          rFactorUnsure(0) = 1;
  //          rFactork[k](0) = rMax / aN;
  //        }
  //        double sumT1 = 0;
  //        double sumT2 = 0;
  //        double aT1, aT2;
  //        if (fdf && gdActive[k][1]) {
  //          if (getFrictionDirections() == 1) {
  //            for (int j = ia[laInd + laIndk[k] + 1] + 1; j < ia[laInd + laIndk[k] + 2]; j++)
  //              sumT1 += fabs(a[j]);
  //            aT1 = a[ia[laInd + laIndk[k] + 1]];
  //            if (aT1 > sumT1) {
  //              rFactorUnsure(1) = 0;
  //              rFactork[k](1) = 1.0 / aT1;
  //            }
  //            else {
  //              rFactorUnsure(1) = 1;
  //              rFactork[k](1) = rMax / aT1;
  //            }
  //          }
  //          else if (getFrictionDirections() == 2) {
  //            for (int j = ia[laInd + laIndk[k] + 1] + 1; j < ia[laInd + laIndk[k] + 2]; j++)
  //              sumT1 += fabs(a[j]);
  //            for (int j = ia[laInd + laIndk[k] + 2] + 1; j < ia[laInd + laIndk[k] + 3]; j++)
  //              sumT2 += fabs(a[j]);
  //            aT1 = a[ia[laInd + laIndk[k] + 1]];
  //            aT2 = a[ia[laInd + laIndk[k] + 2]];
  //
  //            // TODO rFactorUnsure
  //            if (aT1 - sumT1 >= aT2 - sumT2)
  //              if (aT1 + sumT1 >= aT2 + sumT2)
  //                rFactork[k](1) = 2.0 / (aT1 + aT2 + sumT1 - sumT2);
  //              else
  //                rFactork[k](1) = 1.0 / aT2;
  //            else if (aT1 + sumT1 < aT2 + sumT2)
  //              rFactork[k](1) = 2.0 / (aT1 + aT2 - sumT1 + sumT2);
  //            else
  //              rFactork[k](1) = 1.0 / aT1;
  //          }
  //        }
  //      }
  //    }
  //  }

  void Contact::updaterFactors() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gdActive[cK][k][0]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();

          int addIndexnormal = 0;
          if(fcl->isSetValued()) {
            addIndexnormal++;
            double sumN = 0;
            for (int j = ia[laInd + laIndk[cK][k]] + 1; j < ia[laInd + laIndk[cK][k] + 1]; j++)
              sumN += fabs(a[j]);
            const double aN = a[ia[laInd + laIndk[cK][k]]];
            if (aN > sumN) {
              rFactorUnsure(0) = 0;
              rFactork[cK][k](0) = 1.0 / aN;
            }
            else {
              rFactorUnsure(0) = 1;
              rFactork[cK][k](0) = rMax / aN;
            }
          }

          if (fdf && gdActive[cK][k][1]) {
            if(fdf->isSetValued()) {
              double sumT1 = 0;
              double sumT2 = 0;
              double aT1, aT2;
              if (getFrictionDirections() == 1) {
                for (int j = ia[laInd + laIndk[cK][k] + addIndexnormal] + 1; j < ia[laInd + laIndk[cK][k] + addIndexnormal +1]; j++)
                  sumT1 += fabs(a[j]);
                aT1 = a[ia[laInd + laIndk[cK][k] + addIndexnormal]];
                if (aT1 > sumT1) {
                  rFactorUnsure(1) = 0;
                  rFactork[cK][k](1) = 1.0 / aT1;
                }
                else {
                  rFactorUnsure(1) = 1;
                  rFactork[cK][k](1) = rMax / aT1;
                }
              }
              else if (getFrictionDirections() == 2) {
                for (int j = ia[laInd + laIndk[cK][k] + addIndexnormal] + 1; j < ia[laInd + laIndk[cK][k] + addIndexnormal + 1]; j++)
                  sumT1 += fabs(a[j]);
                for (int j = ia[laInd + laIndk[cK][k] + addIndexnormal + 1] + 1; j < ia[laInd + laIndk[cK][k] + addIndexnormal + 2]; j++)
                  sumT2 += fabs(a[j]);
                aT1 = a[ia[laInd + laIndk[cK][k] + addIndexnormal]];
                aT2 = a[ia[laInd + laIndk[cK][k] + addIndexnormal + 1]];

                // TODO rFactorUnsure
                if (aT1 - sumT1 >= aT2 - sumT2)
                  if (aT1 + sumT1 >= aT2 + sumT2)
                    rFactork[cK][k](1) = 2.0 / (aT1 + aT2 + sumT1 - sumT2);
                  else
                    rFactork[cK][k](1) = 1.0 / aT2;
                else if (aT1 + sumT1 < aT2 + sumT2)
                  rFactork[cK][k](1) = 2.0 / (aT1 + aT2 - sumT1 + sumT2);
                else
                  rFactork[cK][k](1) = 1.0 / aT1;
              }
            }
          }
        }
      }
    }
  }

  //  void Contact::checkConstraintsForTermination() {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gdActive[k][0]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        for (unsigned int i = 0; i < 1 + gdActive[k][1] * getFrictionDirections(); i++) {
  //          gddk[k](i) = b(laInd + laIndk[k] + i);
  //          for (int j = ia[laInd + laIndk[k] + i]; j < ia[laInd + laIndk[k] + 1 + i]; j++)
  //            gddk[k](i) += a[j] * laMBS(ja[j]);
  //        }
  //
  //        if (!fcl->isFulfilled(lak[k](0), gddk[k](0), laTol, gddTol)) {
  //          ds->setTermination(false);
  //          return;
  //        }
  //        if (fdf && gdActive[k][1]) {
  //          if (!fdf->isFulfilled(lak[k](1, getFrictionDirections()), gddk[k](1, getFrictionDirections()), lak[k](0), laTol, gddTol)) {
  //            ds->setTermination(false);
  //            return;
  //          }
  //        }
  //      }
  //    }
  //  }

  void Contact::checkConstraintsForTermination() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gdActive[cK][k][0]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          int addIndexnormal = 0;
          if(fcl->isSetValued()) {
            addIndexnormal++;

            gddk[cK][k](0) = b(laInd + laIndk[cK][k]);
            for (int j = ia[laInd + laIndk[cK][k]]; j < ia[laInd + laIndk[cK][k] + 1]; j++)
              gddk[cK][k](0) += a[j] * laMBS(ja[j]);

            if (!fcl->isFulfilled(lakN[cK][k](0), gddk[cK][k](0), laTol, gddTol)) {
              ds->setTermination(false);
              return;
            }
          }

          if (fdf && gdActive[cK][k][1]) {

            for (unsigned int i = 1; i < gdActive[cK][k][1] * getFrictionDirections() + 1; i++) { //TODO: Is there any other number than 0 or one for gdActive? otherwithe the multiplication could be deleted again...
              gddk[cK][k](i) = b(laInd + laIndk[cK][k] + i);
              for (int j = ia[laInd + laIndk[cK][k] + i]; j < ia[laInd + laIndk[cK][k] + 1 + i]; j++)
                gddk[cK][k](i) += a[j] * laMBS(ja[j]);
            }

            if (!fdf->isFulfilled(lakT[cK][k], gddk[cK][k](1, getFrictionDirections()), lakN[cK][k](0), laTol, gddTol)) {
              ds->setTermination(false);
              return;
            }
          }
        }
      }
    }
  }

  //  void Contact::checkImpactsForTermination(double dt) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //
  //        const double *a = ds->getGs()();
  //        const int *ia = ds->getGs().Ip();
  //        const int *ja = ds->getGs().Jp();
  //        const Vec &laMBS = ds->getla();
  //        const Vec &b = ds->getb();
  //
  //        for (int i = 0; i < 1 + getFrictionDirections(); i++) {
  //          gdnk[k](i) = b(laInd + laIndk[k] + i);
  //          for (int j = ia[laInd + laIndk[k] + i]; j < ia[laInd + laIndk[k] + 1 + i]; j++)
  //            gdnk[k](i) += a[j] * laMBS(ja[j]);
  //        }
  //
  //        if (!fnil->isFulfilled(lak[k](0), gdnk[k](0), gdk[k](0), LaTol, gdTol)) {
  //          ds->setTermination(false);
  //          return;
  //        }
  //        if (ftil) {
  //          if (!ftil->isFulfilled(lak[k](1, getFrictionDirections()), gdnk[k](1, getFrictionDirections()), gdk[k](1, getFrictionDirections()), lak[k](0), LaTol, gdTol)) {
  //            ds->setTermination(false);
  //            return;
  //          }
  //        }
  //      }
  //    }
  //  }

  void Contact::checkImpactsForTermination(double dt) {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {

          const double *a = ds->getGs()();
          const int *ia = ds->getGs().Ip();
          const int *ja = ds->getGs().Jp();
          const Vec &laMBS = ds->getla();
          const Vec &b = ds->getb();

          int addIndexnormal = 0;
          double scaleFactorN = dt;
          if(fcl->isSetValued()) {
            scaleFactorN = 1.;
            addIndexnormal++;
            gdnkN[cK][k](0) = b(laInd + laIndk[cK][k]);
            for (int j = ia[laInd + laIndk[cK][k]]; j < ia[laInd + laIndk[cK][k] + 1]; j++)
              gdnkN[cK][k](0) += a[j] * laMBS(ja[j]);
            if (!fnil->isFulfilled(lakN[cK][k](0), gdnkN[cK][k](0), gdkN[cK][k](0), LaTol, gdTol)) {
              ds->setTermination(false);
              return;
            }
          }

          if (ftil) {
            for (int i = 0; i < getFrictionDirections(); i++) {
              gdnkT[cK][k](i) = b(laInd + laIndk[cK][k] + i + addIndexnormal);
              for (int j = ia[laInd + laIndk[cK][k] + i + addIndexnormal]; j < ia[laInd + laIndk[cK][k] + 1 + i + addIndexnormal]; j++)
                gdnkT[cK][k](i) += a[j] * laMBS(ja[j]);
            }
            if (!ftil->isFulfilled(lakT[cK][k], gdnkT[cK][k], gdkT[cK][k], lakN[cK][k](0) * scaleFactorN, LaTol, gdTol)) {
              ds->setTermination(false);
              return;
            }
          }
        }
      }
    }
  }

  //  void Contact::checkActive(int j) {
  //    if (j == 1) { // formerly checkActiveg()
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        gActive[k] = fcl->isActive(gk[k](0), gTol) ? 1 : 0;
  //        gdActive[k][0] = gActive[k];
  //        gdActive[k][1] = gdActive[k][0];
  //      }
  //    }
  //    else if (j == 2) { // formerly checkActivegd()
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        gdActive[k][0] = gActive[k] ? (fcl->remainsActive(gdk[k](0), gdTol) ? 1 : 0) : 0;
  //        gdActive[k][1] = getFrictionDirections() && gdActive[k][0] ? (fdf->isSticking(gdk[k](1, getFrictionDirections()), gdTol) ? 1 : 0) : 0;
  //        gddActive[k][0] = gdActive[k][0];
  //        gddActive[k][1] = gdActive[k][1];
  //      }
  //    }
  //    else if (j == 3) { // formerly checkActivegdn()
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (gActive[k]) { // Contact was closed
  //          if (gdnk[k](0) <= gdTol) { // Contact stays closed // TODO bilateral contact
  //            gdActive[k][0] = true;
  //            gddActive[k][0] = true;
  //            if (getFrictionDirections()) {
  //              if (nrm2(gdnk[k](1, getFrictionDirections())) <= gdTol) {
  //                gdActive[k][1] = true;
  //                gddActive[k][1] = true;
  //              }
  //              else {
  //                gdActive[k][1] = false;
  //                gddActive[k][1] = false;
  //              }
  //            }
  //          }
  //          else { // Contact will open
  //            gdActive[k][0] = false;
  //            gdActive[k][1] = false;
  //            gddActive[k][0] = false;
  //            gddActive[k][1] = false;
  //          }
  //        }
  //      }
  //    }
  //    else if (j == 4) { // formerly checkActivegdd()
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (gActive[k]) {
  //          if (gdActive[k][0]) {
  //            if (gddk[k](0) <= gddTol) { // Contact stays closed
  //              gddActive[k][0] = true;
  //              if (getFrictionDirections()) {
  //                if (gdActive[k][1]) {
  //                  if (nrm2(gddk[k](1, getFrictionDirections())) <= gddTol)
  //                    gddActive[k][1] = true;
  //                  else {
  //                    gddActive[k][1] = false;
  //                  }
  //                }
  //              }
  //            }
  //            else { // Contact will open
  //              gddActive[k][0] = false;
  //              gddActive[k][1] = false;
  //            }
  //          }
  //        }
  //      }
  //    }
  //    else if (j == 5) {
  //      for (int i = 0; i < contactKinematics->getNumberOfPotentialContactPoints(); i++) {
  //        if (gActive[i]) {
  //          if (gdActive[i][0]) {
  //            if (gdActive[i][1]) {
  //              if (!gddActive[i][1]) {
  //                gdActive[i][1] = false;
  //              }
  //            }
  //            if (!gddActive[i][0]) {
  //              gActive[i] = false;
  //              gdActive[i][0] = false;
  //              gdActive[i][1] = false;
  //            }
  //          }
  //          else
  //            gActive[i] = false;
  //        }
  //      }
  //    }
  //    else if (j == 6) { // nur nach schließenden Kontakten schauen
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (rootID[k] == 3) {
  //          gActive[k] = true;
  //          gdActive[k][0] = true;
  //          gdActive[k][1] = true;
  //          gddActive[k][0] = true;
  //          gddActive[k][1] = true;
  //        }
  //      }
  //    }
  //    else if (j == 7) { // nur nach Gleit-Haft-Übergängen schauen
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (getFrictionDirections()) {
  //          if (rootID[k] == 2) {
  //            gdActive[k][1] = true;
  //            gddActive[k][1] = true;
  //          }
  //        }
  //      }
  //    }
  //    else if (j == 8) { // nur nach öffnenden Kontakten und Haft-Gleit-Übergängen schauen
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (jsvk[k](0) && rootID[k] == 1) { // Kontakt öffnet
  //          gddActive[k][0] = false;
  //          gddActive[k][1] = false;
  //        }
  //        if (getFrictionDirections()) {
  //          if (jsvk[k](1) && rootID[k] == 1) { // Haft-Gleitübergang
  //            gddActive[k][1] = false;
  //          }
  //        }
  //      }
  //    }
  //    else
  //      throw;
  //  }

  void Contact::checkActive(int j) {
    if (j == 1) { // formerly checkActiveg()
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gActive[cK][k] = fcl->isActive(gk[cK][k](0), gTol) ? 1 : 0;
          gdActive[cK][k][0] = gActive[cK][k];
          gdActive[cK][k][1] = gdActive[cK][k][0];
        }
      }
    }
    else if (j == 2) { // formerly checkActivegd()
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          gdActive[cK][k][0] = gActive[cK][k] ? (fcl->remainsActive(gdkN[cK][k](0), gdTol) ? 1 : 0) : 0;
          gdActive[cK][k][1] = getFrictionDirections() && gdActive[cK][k][0] ? (fdf->isSticking(gdkT[cK][k], gdTol) ? 1 : 0) : 0;
          gddActive[cK][k][0] = gdActive[cK][k][0];
          gddActive[cK][k][1] = gdActive[cK][k][1];
        }
      }
    }
    else if (j == 3) { // formerly checkActivegdn()
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (gActive[cK][k]) { // Contact was closed
            if (gdnkN[cK][k](0) <= gdTol) { // Contact stays closed // TODO bilateral contact
              gdActive[cK][k][0] = true;
              gddActive[cK][k][0] = true;
              if (getFrictionDirections()) {
                if (nrm2(gdnkT[cK][k]) <= gdTol) {
                  gdActive[cK][k][1] = true;
                  gddActive[cK][k][1] = true;
                }
                else {
                  gdActive[cK][k][1] = false;
                  gddActive[cK][k][1] = false;
                }
              }
            }
            else { // Contact will open
              gdActive[cK][k][0] = false;
              gdActive[cK][k][1] = false;
              gddActive[cK][k][0] = false;
              gddActive[cK][k][1] = false;
            }
          }
        }
      }
    }
    else if (j == 4) { // formerly checkActivegdd()
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (gActive[cK][k]) {
            if (gdActive[cK][k][0]) {
              if (gddk[cK][k](0) <= gddTol) { // Contact stays closed
                gddActive[cK][k][0] = true;
                if (getFrictionDirections()) {
                  if (gdActive[cK][k][1]) {
                    if (nrm2(gddk[cK][k](1, getFrictionDirections())) <= gddTol)
                      gddActive[cK][k][1] = true;
                    else {
                      gddActive[cK][k][1] = false;
                    }
                  }
                }
              }
              else { // Contact will open
                gddActive[cK][k][0] = false;
                gddActive[cK][k][1] = false;
              }
            }
          }
        }
      }
    }
    else if (j == 5) {
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (gActive[cK][k]) {
            if (gdActive[cK][k][0]) {
              if (gdActive[cK][k][1]) {
                if (!gddActive[cK][k][1]) {
                  gdActive[cK][k][1] = false;
                }
              }
              if (!gddActive[cK][k][0]) {
                gActive[cK][k] = false;
                gdActive[cK][k][0] = false;
                gdActive[cK][k][1] = false;
              }
            }
            else
              gActive[cK][k] = false;
          }
        }
      }
    }
    else if (j == 6) { // nur nach schließenden Kontakten schauen
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (rootID[cK][k] == 3) {
            gActive[cK][k] = true;
            gdActive[cK][k][0] = true;
            gdActive[cK][k][1] = true;
            gddActive[cK][k][0] = true;
            gddActive[cK][k][1] = true;
          }
        }
      }
    }
    else if (j == 7) { // nur nach Gleit-Haft-Übergängen schauen
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (getFrictionDirections()) {
            if (rootID[cK][k] == 2) {
              gdActive[cK][k][1] = true;
              gddActive[cK][k][1] = true;
            }
          }
        }
      }
    }
    else if (j == 8) { // nur nach öffnenden Kontakten und Haft-Gleit-Übergängen schauen
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (jsvk[cK][k](0) && rootID[cK][k] == 1) { // Kontakt öffnet
            gddActive[cK][k][0] = false;
            gddActive[cK][k][1] = false;
          }
          if (getFrictionDirections()) {
            if (jsvk[cK][k](1) && rootID[cK][k] == 1) { // Haft-Gleitübergang
              gddActive[cK][k][1] = false;
            }
          }
        }
      }
    }
    else
      throw;
  }

  int Contact::getFrictionDirections() {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  int Contact::connect(Contour *contour0, Contour* contour1, ContactKinematics* contactKinematics_ /*=0*/) {
    LinkMechanics::connect(contour0);
    LinkMechanics::connect(contour1);
    contactKinematics.push_back(contactKinematics_);

    int cK = contactKinematics.size() - 1;

    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour0->findContactPairingWith(contour0->getType(), contour1->getType());
    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour1->findContactPairingWith(contour1->getType(), contour0->getType());
    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour0->findContactPairingWith(contour1->getType(), contour0->getType());
    if (contactKinematics[cK] == 0)
      contactKinematics[cK] = contour1->findContactPairingWith(contour0->getType(), contour1->getType());
    if (contactKinematics[cK] == 0)
      throw MBSimError("ERROR in " + getName() + " (Contact::init): Unknown contact pairing between Contour \"" + contour0->getType() + "\" and Contour\"" + contour1->getType() + "\"!");

    return cK;
  }

  void Contact::computeCurvatures(Vec & r, int contactKinematicsIndex) const {
    contactKinematics[contactKinematicsIndex]->computeCurvatures(r, cpData[contactKinematicsIndex][0]);
  }

  //  void Contact::LinearImpactEstimation(Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k]) {
  //        gAct_(*IndActive_) = gk[k](0);
  //        (*IndActive_)++;
  //      }
  //      else {
  //        for (unsigned int i = 0; i < 2; i++)
  //          contour[i]->updateKinematicsForFrame(cpData[k][i], velocities);
  //        Vec Wn = cpData[k][0].getFrameOfReference().getOrientation().col(0);
  //        Vec WvD = cpData[k][1].getFrameOfReference().getVelocity() - cpData[k][0].getFrameOfReference().getVelocity();
  //        gdInActive_(*IndInActive_) = Wn.T() * WvD;
  //        gInActive_(*IndInActive_) = gk[k](0);
  //        (*IndInActive_)++;
  //      }
  //    }
  //
  //  }

  void Contact::LinearImpactEstimation(Vec &gInActive_, Vec &gdInActive_, int *IndInActive_, Vec &gAct_, int *IndActive_) {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      int contourIndex = cK * 2;

      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k]) {
          gAct_(*IndActive_) = gk[cK][k](0);
          (*IndActive_)++;
        }
        else {
          for (unsigned int i = 0; i < 2; i++)
            contour[contourIndex + i]->updateKinematicsForFrame(cpData[cK][k][i], velocities);
          Vec Wn = cpData[cK][k][0].getFrameOfReference().getOrientation().col(0);
          Vec WvD = cpData[cK][k][1].getFrameOfReference().getVelocity() - cpData[cK][k][0].getFrameOfReference().getVelocity();
          gdInActive_(*IndInActive_) = Wn.T() * WvD;
          gInActive_(*IndInActive_) = gk[cK][k](0);
          (*IndInActive_)++;
        }
      }
    }

  }

  //  void Contact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      if (gActive[k])
  //        (*sizeActive_)++;
  //      else
  //        (*sizeInActive_)++;
  //    }
  //  }

  void Contact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        if (gActive[cK][k])
          (*sizeActive_)++;
        else
          (*sizeInActive_)++;
      }
    }
  }

  void Contact::initializeUsingXML(TiXmlElement *element) {
    //TODO?!
    LinkMechanics::initializeUsingXML(element);
    TiXmlElement *e;
    e = element->FirstChildElement(MBSIMNS"contactForceLaw");
    GeneralizedForceLaw *gfl = ObjectFactory::getInstance()->createGeneralizedForceLaw(e->FirstChildElement());
    setContactForceLaw(gfl);
    gfl->initializeUsingXML(e->FirstChildElement());
    e = e->NextSiblingElement();
    GeneralizedImpactLaw *gifl = ObjectFactory::getInstance()->createGeneralizedImpactLaw(e->FirstChildElement());
    if (gifl) {
      setContactImpactLaw(gifl);
      gifl->initializeUsingXML(e->FirstChildElement());
      e = e->NextSiblingElement();
    }
    FrictionForceLaw *ffl = ObjectFactory::getInstance()->createFrictionForceLaw(e->FirstChildElement());
    if (ffl) {
      setFrictionForceLaw(ffl);
      ffl->initializeUsingXML(e->FirstChildElement());
      e = e->NextSiblingElement();
    }
    FrictionImpactLaw *fil = ObjectFactory::getInstance()->createFrictionImpactLaw(e->FirstChildElement());
    if (fil) {
      setFrictionImpactLaw(fil);
      fil->initializeUsingXML(e->FirstChildElement());
    }
    e = element->FirstChildElement(MBSIMNS"connect");
    saved_ref1 = e->Attribute("ref1");
    saved_ref2 = e->Attribute("ref2");
#ifdef HAVE_OPENMBVCPPINTERFACE
    if (element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints"))
      enableOpenMBVContactPoints(getDouble(element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints")));
    e = element->FirstChildElement(MBSIMNS"openMBVNormalForceArrow");
    if (e) {
      OpenMBV::Arrow *arrow = dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVNormalForceArrow(arrow);
      e = e->NextSiblingElement();
    }
    e = element->FirstChildElement(MBSIMNS"openMBVFrictionArrow");
    if (e) {
      OpenMBV::Arrow *arrow = dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
      setOpenMBVFrictionArrow(arrow);
      e = e->NextSiblingElement();
    }
#endif
  }

  //  void Contact::updatecorrRef(const Vec& corrParent) {
  //    LinkMechanics::updatecorrRef(corrParent);
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++)
  //      corrk[k].resize() >> corr(corrIndk[k], corrIndk[k] + corrSizek[k] - 1);
  //  }

  void Contact::updatecorrRef(const Vec& corrParent) {
    LinkMechanics::updatecorrRef(corrParent);
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        corrk[cK][k].resize() >> corr(corrIndk[cK][k], corrIndk[cK][k] + corrSizek[cK][k] - 1);
      }
    }
  }

  //  void Contact::updatecorr(int j) {
  //    if (j == 1) { // IG position
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (gActive[k]) { // Contact was closed
  //          if (gdActive[k][0])
  //            corrk[k](0) = 0; // Contact stays closed, regular projection
  //          else
  //            corrk[k](0) = 1e-14; // Contact opens, projection to positive normal distance
  //        }
  //      }
  //    }
  //    else if (j == 2) {
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (gActive[k] && gdActive[k][0]) { // Contact was closed
  //          if (gddActive[k][0])
  //            corrk[k](0) = 0; // Contact stays closed, regular projection
  //          else
  //            corrk[k](0) = 1e-14; // Contact opens, projection to positive normal distance
  //        }
  //      }
  //    }
  //    else if (j == 4) {
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        if (rootID[k] == 1)
  //          gddk[k] = gddkBuf[k];
  //        if (gActive[k] && gdActive[k][0]) { // Contact was closed
  //          if (gddActive[k][0])
  //            corrk[k](0) = 0; // Contact stays closed, regular projection
  //          else
  //            corrk[k](0) = 1e-16; // Contact opens, projection to positive normal distance
  //          if (getFrictionDirections()) {
  //            if (gdActive[k][1]) { // Contact was sticking
  //              if (gddActive[k][1]) {
  //                corrk[k](1) = 0; // Contact stays sticking, regular projection
  //                if (getFrictionDirections() > 1)
  //                  corrk[k](2) = 0; // Contact stays sticking, regular projection
  //              }
  //              else {
  //                corrk[k](1) = gddk[k](1) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
  //                if (getFrictionDirections() > 1)
  //                  corrk[k](2) = gddk[k](2) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
  //              }
  //            }
  //          }
  //        }
  //      }
  //    }
  //    else
  //      throw;
  //  }

  void Contact::updatecorr(int j) {
    if (j == 1) { // IG position
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (gActive[cK][k]) { // Contact was closed
            if (gdActive[cK][k][0])
              corrk[cK][k](0) = 0; // Contact stays closed, regular projection
            else
              corrk[cK][k](0) = 1e-14; // Contact opens, projection to positive normal distance
          }
        }
      }
    }
    else if (j == 2) {
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (gActive[cK][k] && gdActive[cK][k][0]) { // Contact was closed
            if (gddActive[cK][k][0])
              corrk[cK][k](0) = 0; // Contact stays closed, regular projection
            else
              corrk[cK][k](0) = 1e-14; // Contact opens, projection to positive normal distance
          }
        }
      }
    }
    else if (j == 4) {
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          if (rootID[cK][k] == 1)
            gddk[cK][k] = gddkBuf[cK][k];
          if (gActive[cK][k] && gdActive[cK][k][0]) { // Contact was closed
            if (gddActive[cK][k][0])
              corrk[cK][k](0) = 0; // Contact stays closed, regular projection
            else
              corrk[cK][k](0) = 1e-16; // Contact opens, projection to positive normal distance
            if (getFrictionDirections()) {
              if (gdActive[cK][k][1]) { // Contact was sticking
                if (gddActive[cK][k][1]) {
                  corrk[cK][k](1) = 0; // Contact stays sticking, regular projection
                  if (getFrictionDirections() > 1)
                    corrk[cK][k](2) = 0; // Contact stays sticking, regular projection
                }
                else {
                  corrk[cK][k](1) = gddk[cK][k](1) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
                  if (getFrictionDirections() > 1)
                    corrk[cK][k](2) = gddk[cK][k](2) > 0 ? 1e-16 : -1e-16; // Contact slides, projection to positive normal velocity
                }
              }
            }
          }
        }
      }
    }
    else
      throw;
  }

  //  void Contact::calccorrSize(int j) {
  //    LinkMechanics::calccorrSize(j);
  //    if (j == 1) { // IG
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        corrIndk[k] = corrSize;
  //        corrSizek[k] = gActive[k];
  //        corrSize += corrSizek[k];
  //      }
  //    }
  //    else if (j == 2) { // IB
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        corrIndk[k] = corrSize;
  //        corrSizek[k] = gActive[k] * gdActive[k][0];
  //        corrSize += corrSizek[k];
  //      }
  //    }
  //    //    else if(j==3) { // IG
  //    //      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //    //        corrIndk[k] = corrSize;
  //    //        corrSizek[k] = gActive[i]*(1+getFrictionDirections());
  //    //        corrSize += corrSizek[k];
  //    //      }
  //    //    }
  //    else if (j == 4) { // IH
  //      for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //        corrIndk[k] = corrSize;
  //        corrSizek[k] = gActive[k] * gdActive[k][0] * (1 + gdActive[k][1] * getFrictionDirections());
  //        corrSize += corrSizek[k];
  //      }
  //    }
  //    else
  //      throw;
  //  }

  void Contact::calccorrSize(int j) {
    LinkMechanics::calccorrSize(j);
    if (j == 1) { // IG
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          corrIndk[cK][k] = corrSize;
          corrSizek[cK][k] = gActive[cK][k];
          corrSize += corrSizek[cK][k];
        }
      }
    }
    else if (j == 2) { // IB
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          corrIndk[cK][k] = corrSize;
          corrSizek[cK][k] = gActive[cK][k] * gdActive[cK][k][0];
          corrSize += corrSizek[cK][k];
        }
      }
    }
    //    else if(j==3) { // IG
    //      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
    //        corrIndk[cK][k] = corrSize;
    //        corrSizek[cK][k] = gActive[i]*(1+getFrictionDirections());
    //        corrSize += corrSizek[cK][k];
    //      }
    //    }
    else if (j == 4) { // IH
      for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
        for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
          corrIndk[cK][k] = corrSize;
          corrSizek[cK][k] = gActive[cK][k] * gdActive[cK][k][0] * (1 + gdActive[cK][k][1] * getFrictionDirections());
          corrSize += corrSizek[cK][k];
        }
      }
    }
    else
      throw;
  }

  //  void Contact::checkRoot() {
  //    for (int k = 0; k < contactKinematics->getNumberOfPotentialContactPoints(); k++) {
  //      rootID[k] = 0;
  //      if (jsvk[k](0)) {
  //        if (gActive[k])
  //          rootID[k] = 1; // Contact was closed -> opening
  //        else
  //          rootID[k] = 3; // Contact was open -> impact
  //      }
  //      if (getFrictionDirections()) {
  //        if (jsvk[k](1)) {
  //          if (gdActive[k][1])
  //            rootID[k] = 1; // Contact was sticking -> sliding
  //          else {
  //            if (getFrictionDirections() == 1)
  //              rootID[k] = 2; // Contact was sliding -> sticking
  //            else if (nrm2(gdk[k](1, getFrictionDirections())) <= gdTol)
  //              rootID[k] = 2; // Contact was sliding -> sticking
  //          }
  //        }
  //      }
  //      ds->setRootID(max(ds->getRootID(), rootID[k]));
  //    }
  //  }

  void Contact::checkRoot() {
    for (size_t cK = 0; cK < contactKinematics.size(); cK++) {
      for (int k = 0; k < contactKinematics[cK]->getNumberOfPotentialContactPoints(); k++) {
        rootID[cK][k] = 0;
        if (jsvk[cK][k](0)) {
          if (gActive[cK][k])
            rootID[cK][k] = 1; // Contact was closed -> opening
          else
            rootID[cK][k] = 3; // Contact was open -> impact
        }
        if (getFrictionDirections()) {
          if (jsvk[cK][k](1)) {
            if (gdActive[cK][k][1])
              rootID[cK][k] = 1; // Contact was sticking -> sliding
            else {
              if (getFrictionDirections() == 1)
                rootID[cK][k] = 2; // Contact was sliding -> sticking
              else if (nrm2(gdkT[cK][k]) <= gdTol)
                rootID[cK][k] = 2; // Contact was sliding -> sticking
            }
          }
        }
        ds->setRootID(max(ds->getRootID(), rootID[cK][k]));
      }
    }
  }

}

