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
#include <mbsim/contact.h>
#include <mbsim/contour.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/utils/contact_utils.h>
//#include <mbsim/class_factory.h> TODO

using namespace std;
using namespace fmatvec;

namespace MBSim {

  double sign(double x) {
    if(x>0)
      return 1.0;
    else if(x<0)
      return -1.0;
    else 
      return 0;
  }

  int min(int i, int j) {
    return i<j?i:j;
  }

  Contact::Contact(const string &name) : LinkMechanics(name), contactKinematics(0), fcl(0), fdf(0), fnil(0), ftil(0) {}

  Contact::~Contact() {
    if (contactKinematics) delete contactKinematics;
  }

  void Contact::updater(double t) {

    for(unsigned i=0; i<contour.size(); i++) 
      r[i] += V[i]*la;
  }

  void Contact::updatewb(double t) {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {
        for(unsigned i=0; i<contour.size(); i++) 
          wbk[k] += trans(fF[k][i](Index(0,2),Index(0,laSizek[k]-1)))*cpData[k][i].getFrameOfReference().getGyroscopicAccelerationOfTranslation();

        //	contactKinematics->updatewb(wb,g,cpData[k]);
      }
    }
    contactKinematics->updatewb(wbk,gk,cpData);
  }

  void Contact::updateW(double t) {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {
        fF[k][1].col(0) = cpData[k][0].getFrameOfReference().getOrientation().col(0);
        if(getFrictionDirections()) {
          fF[k][1].col(1) = cpData[k][0].getFrameOfReference().getOrientation().col(1);
          if(getFrictionDirections() > 1)
            fF[k][1].col(2) = cpData[k][0].getFrameOfReference().getOrientation().col(2);
        }

        fF[k][0] = -fF[k][1];

        for(unsigned int i=0; i<contour.size(); i++) 
          Wk[k][i] += trans(cpData[k][i].getFrameOfReference().getJacobianOfTranslation())*fF[k][i](Index(0,2),Index(0,laSizek[k]-1));
      }
    }
  }

  void Contact::updateV(double t) {

    if(getFrictionDirections()) {
      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
        if(gActive[k] && !gdActive[k][1]) {
          for(unsigned int i=0; i<contour.size(); i++) 
            Vk[k][i] += trans(cpData[k][i].getFrameOfReference().getJacobianOfTranslation())*fF[k][i](Index(0,2),iT)*fdf->dlaTdlaN(gdk[k](1,getFrictionDirections()), lak[k](0));
        }
      }
    }
  }
  
  void Contact::updateh(double t) {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      // TODO: check if(gActive[k]) {
      lak[k](0) = (*fcl)(gk[k](0),gdk[k](0));
      if(fdf)
        lak[k](1,getFrictionDirections()) = (*fdf)(gdk[k](1,getFrictionDirections()),fabs(lak[k](0)));

      WF[k][1] =  cpData[k][0].getFrameOfReference().getOrientation().col(0)*lak[k](0);
      if(getFrictionDirections()) {
        WF[k][1] += cpData[k][0].getFrameOfReference().getOrientation().col(1)*lak[k](1);
        if(getFrictionDirections() > 1)
          WF[k][1] += cpData[k][0].getFrameOfReference().getOrientation().col(2)*lak[k](2);
      }
      WF[k][0] = -WF[k][1];
      for(unsigned int i=0; i<contour.size(); i++)
        h[i] += trans(cpData[k][i].getFrameOfReference().getJacobianOfTranslation())*WF[k][i];
    }
  }

  void Contact::updateg(double t) {
    //for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
    contactKinematics->updateg(gk,cpData);
  }

  void Contact::updategd(double t) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {
        for(unsigned int i=0; i<2; i++) {
          Vec WrPC = cpData[k][i].getFrameOfReference().getPosition() - contour[i]->getFrame()->getPosition();
          cpData[k][i].getFrameOfReference().setAngularVelocity(contour[i]->getFrame()->getAngularVelocity());
          cpData[k][i].getFrameOfReference().setVelocity(contour[i]->getFrame()->getVelocity() + crossProduct(contour[i]->getFrame()->getAngularVelocity(),WrPC));
        }

        Vec Wn = cpData[k][0].getFrameOfReference().getOrientation().col(0);

        Vec WvD = cpData[k][1].getFrameOfReference().getVelocity() - cpData[k][0].getFrameOfReference().getVelocity();

        gdk[k](0) = trans(Wn)*WvD;

        if(gdk[k].size()>1) {
          Mat Wt(3,gdk[k].size()-1);
          Wt.col(0) = cpData[k][0].getFrameOfReference().getOrientation().col(1);
          if(gdk[k].size() > 2)
            Wt.col(1) = cpData[k][0].getFrameOfReference().getOrientation().col(2);

          gdk[k](1,gdk[k].size()-1) = trans(Wt)*WvD;
        }
      }
    }
  }

    void Contact::updateStopVector(double t) {

      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
        if(gActive[k]) {
          svk[k](0) = lak[k](0);
          if(gdActive[k][1]) {
            svk[k](1) = nrm2(lak[k](1,getFrictionDirections())) - fdf->getFrictionCoefficient(nrm2(gdk[k](1,getFrictionDirections())))*lak[k](0);
          } 
          else {
            if(getFrictionDirections() == 1)
              svk[k](1) = gdk[k](1);
            else if(getFrictionDirections() == 2) 
              svk[k](1) = gdk[k](1)+gdk[k](2);
          }
        }
        else {
          svk[k](0) = gk[k](0);
          //sv(1,getFrictionDirections()).init(1);
          if(getFrictionDirections())
            svk[k](1) = 1;
        }
      }
    }
  
    void Contact::updateJacobians(double t) {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {
        for(unsigned int i=0; i<2; i++) {
          Vec WrPC = cpData[k][i].getFrameOfReference().getPosition() - contour[i]->getFrame()->getPosition();
          Mat tWrPC = tilde(WrPC);
          cpData[k][i].getFrameOfReference().setJacobianOfTranslation(contour[i]->getFrame()->getJacobianOfTranslation() - tWrPC*contour[i]->getFrame()->getJacobianOfRotation());
          cpData[k][i].getFrameOfReference().setJacobianOfRotation(contour[i]->getFrame()->getJacobianOfRotation());
          cpData[k][i].getFrameOfReference().setGyroscopicAccelerationOfTranslation(contour[i]->getFrame()->getGyroscopicAccelerationOfTranslation() - tWrPC*contour[i]->getFrame()->getGyroscopicAccelerationOfRotation() + crossProduct(contour[i]->getFrame()->getAngularVelocity(),crossProduct(contour[i]->getFrame()->getAngularVelocity(),WrPC)));
          cpData[k][i].getFrameOfReference().setGyroscopicAccelerationOfRotation(contour[i]->getFrame()->getGyroscopicAccelerationOfRotation());
        }
      }
    }
  }

  void Contact::updateWRef(const Mat& WParent, int j) {
    for(unsigned i=0; i<contour.size(); i++) {
      int hInd =  contour[i]->getParent()->gethInd(parent,j);
      Index I = Index(hInd,hInd+contour[i]->gethSize(j)-1);
      Index J = Index(laInd,laInd+laSize-1);
      W[i].resize()>>WParent(I,J);
      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
        Index Ik = Index(0,W[i].rows()-1);
        Index Jk = Index(laIndk[k],laIndk[k]+laSizek[k]-1);
        Wk[k][i].resize()>>W[i](Ik,Jk);
      }
    }
  } 

  void Contact::updateVRef(const Mat& VParent, int j) {
    for(unsigned i=0; i<contour.size(); i++) {
      int hInd =  contour[i]->getParent()->gethInd(parent,j);
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(hInd,hInd+contour[i]->gethSize(j)-1);
      V[i].resize()>>VParent(I,J);
      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
        Index Ik = Index(0,V[i].rows()-1);
        Index Jk = Index(laIndk[k],laIndk[k]+laSizek[k]-1);
        Vk[k][i].resize()>>V[i](Ik,Jk);
      }
    } 
  }

  void Contact::updatehRef(const Vec& hParent, int j) {
    for(unsigned i=0; i<contour.size(); i++) {
      int hInd =  contour[i]->getParent()->gethInd(parent,j);
      Index I = Index(hInd,hInd+contour[i]->gethSize(j)-1);
      h[i].resize()>>hParent(I);
    }
  } 

  void Contact::updatewbRef(const Vec& wbParent) {
    LinkMechanics::updatewbRef(wbParent);
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      wbk[k].resize() >> wb(laIndk[k],laIndk[k]+laSizek[k]-1);
  }

  void Contact::updatelaRef(const Vec& laParent) {
    LinkMechanics::updatelaRef(laParent);
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      lak[k].resize() >> la(laIndk[k],laIndk[k]+laSizek[k]-1);
  }

  void Contact::updategRef(const Vec& gParent) {
    LinkMechanics::updategRef(gParent);
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      gk[k].resize() >> g(gIndk[k],gIndk[k]+gSizek[k]-1);
  }

  void Contact::updategdRef(const Vec& gdParent) {
    LinkMechanics::updategdRef(gdParent);
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      gdk[k].resize() >> gd(gdIndk[k],gdIndk[k]+gdSizek[k]-1);
  }

  void Contact::updaterFactorRef(const Vec& rFactorParent) {
    LinkMechanics::updaterFactorRef(rFactorParent);
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      rFactork[k].resize() >> rFactor(rFactorIndk[k],rFactorIndk[k]+rFactorSizek[k]-1);
  }

  void Contact::updatesvRef(const Vec &svParent) {
    LinkMechanics::updatesvRef(svParent);
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      svk[k] >> sv(svIndk[k],svIndk[k]+svSizek[k]-1);
  }

  void Contact::updatejsvRef(const Vector<int> &jsvParent) {
    LinkMechanics::updatejsvRef(jsvParent);
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      jsvk[k] >> jsv(svIndk[k],svIndk[k]+svSizek[k]-1);
  }


  void Contact::calcxSize() {
    LinkMechanics::calcxSize();
    xSize = 0;
  }

  void Contact::calclaSize() {
    LinkMechanics::calclaSize();
    laSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      laIndk[i] = laSize;
      laSizek[i] = gdActive[i][0]+gdActive[i][1]*getFrictionDirections();
      laSize += laSizek[i];
    }
  }

  void Contact::calclaSizeForActiveg() {
    LinkMechanics::calclaSizeForActiveg();
    laSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      laIndk[i] = laSize;
      laSizek[i] = gActive[i];
      laSize += laSizek[i];
    }
  }

  void Contact::calcgSize() {
    LinkMechanics::calcgSize();
    gSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gIndk[i] = gSize;
      gSizek[i] = 1;
      gSize += gSizek[i];
    }
  }

  void Contact::calcgSizeActive() {
    LinkMechanics::calcgSizeActive();
    gSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gIndk[i] = gSize;
      gSizek[i] = gActive[i];
      gSize += gSizek[i];
    }
  }

  void Contact::calcgdSize() {
    LinkMechanics::calcgdSize();
    gdSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gdIndk[i] = gdSize;
      gdSizek[i] = gdActive[i][0]*(1+getFrictionDirections());
      gdSize += gdSizek[i];
    }
  }

  void Contact::calcgdSizeActive() {
    LinkMechanics::calcgdSizeActive();
    gdSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gdIndk[i] = gdSize;
      gdSizek[i] = gdActive[i][0]+gdActive[i][1]*getFrictionDirections();
      gdSize += gdSizek[i];
    }
  }

  void Contact::calcrFactorSize() {
    LinkMechanics::calcrFactorSize();
    rFactorSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      rFactorIndk[i] = rFactorSize;
      rFactorSizek[i] = gdActive[i][0]+min(getFrictionDirections(),1)*gdActive[i][1];
      rFactorSize += rFactorSizek[i];
    }
  }

  void Contact::calcsvSize() {
    LinkMechanics::calcsvSize();
    svSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      svIndk[i] = svSize;
      svSizek[i] = isSetValued() ? 1+min(getFrictionDirections(),1) : 0;
      svSize += svSizek[i];
    }
  }

  void Contact::init() {
    LinkMechanics::init();

    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {

      if(getFrictionDirections() == 0)
        gdActive[i][1] = false;

      cpData.push_back(new ContourPointData[2]);

      cpData[i][0].getFrameOfReference().setName("0");
      cpData[i][1].getFrameOfReference().setName("1");
      cpData[i][0].getFrameOfReference().getJacobianOfTranslation().resize(3,contour[0]->getWJP().cols());
      cpData[i][0].getFrameOfReference().getJacobianOfRotation().resize(3,contour[0]->getWJR().cols());
      cpData[i][1].getFrameOfReference().getJacobianOfTranslation().resize(3,contour[1]->getWJP().cols());
      cpData[i][1].getFrameOfReference().getJacobianOfRotation().resize(3,contour[1]->getWJR().cols());

      int laSizek = gdActive[i][0]+gdActive[i][1]*getFrictionDirections();

      Wk.push_back(new Mat[2]);
      Wk[i][0].resize(contour[0]->gethSize(),laSizek);
      Wk[i][1].resize(contour[1]->gethSize(),laSizek);

      Vk.push_back(new Mat[2]);
      Vk[i][0].resize(contour[0]->gethSize(),laSizek);
      Vk[i][1].resize(contour[1]->gethSize(),laSizek);

      fF.push_back(new Mat[2]);
      fF[i][0].resize(3,laSizek);
      fF[i][1].resize(3,laSizek);

      WF.push_back(new Vec[2]);
      WF[i][0].resize(3);
      WF[i][1].resize(3);
    }

    iT = Index(1,getFrictionDirections());

    int n = contactKinematics->getNumberOfPotentialContactPoints();

    la.resize(n*(1+getFrictionDirections()));
    g.resize(n);
    gd.resize(n*(1+getFrictionDirections()));
    gdd.resize(gd.size());
    gdn.resize(gd.size());

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      lak[k].resize() >> la(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
      gdk[k].resize() >> gd(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
      gdnk[k].resize() >> gdn(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
      gddk[k].resize() >> gdd(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
      gk[k].resize() >> g(k,k);
    }

  }

  void Contact::preinit() {
    LinkMechanics::preinit();

    if(contactKinematics);
    else contactKinematics = findContactPairing(contour[0],contour[1]);
    if(contactKinematics==0) {
      cout << "unknown contact pairing" << endl;
      throw 5;
    }

    contactKinematics->assignContours(contour[0],contour[1]);

    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gActive.push_back(int(1));
      gActive0.push_back(int(1));
      gdActive.push_back(new unsigned int[1+min(1,getFrictionDirections())]);
      for (int j=0; j<1+min(1,getFrictionDirections()); j++)
        gdActive[i][j] = 1;

      gk.push_back(Vec(1));
      gdk.push_back(Vec(1));
      gdnk.push_back(Vec(1));
      gddk.push_back(Vec(1));
      lak.push_back(Vec());
      wbk.push_back(Vec());
      svk.push_back(Vec());
      jsvk.push_back(Vector<int>());
      rFactork.push_back(Vec());
      laSizek.push_back(int(0));
      laIndk.push_back(int(0));
      gSizek.push_back(int(0));
      gIndk.push_back(int(0));
      gdSizek.push_back(int(0));
      gdIndk.push_back(int(0));
      svSizek.push_back(int(0));
      svIndk.push_back(int(0));
      rFactorSizek.push_back(int(0));
      rFactorIndk.push_back(int(0));
    }
  }

  bool Contact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if(fdf) 
      flag |= fdf->isSetValued();
    return flag;
  }

  bool Contact::isActive() const {
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      if(gActive[i])
        return true;
    }
    return false;
  }

  bool Contact::gActiveChanged() {
    bool changed = false;
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive0[k] != gActive[k])
        changed = true;
      gActive0[k] = gActive[k];
    }
    return changed;
  }

  void Contact::solveImpactsFixpointSingle() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        gdnk[k](0) = b(laIndDS+laIndk[k]);
        for(int j=ia[laIndDS+laIndk[k]]; j<ia[laIndDS+laIndk[k]+1]; j++)
          gdnk[k](0) += a[j]*laMBS(ja[j]);

        lak[k](0) = fnil->project(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));

        for(int i=1; i<=getFrictionDirections(); i++) {
          gdnk[k](i) = b(laIndDS+laIndk[k]+i);
          for(int j=ia[laIndDS+laIndk[k]+i]; j<ia[laIndDS+laIndk[k]+1+i]; j++)
            gdnk[k](i) += a[j]*laMBS(ja[j]);
        }

        if(ftil)
          lak[k](1,getFrictionDirections()) = ftil->project(lak[k](1,getFrictionDirections()), gdnk[k](1,getFrictionDirections()), gdk[k](1,getFrictionDirections()), lak[k](0), rFactork[k](1));
      }
    }
  }

  void Contact::solveConstraintsFixpointSingle() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        gddk[k](0) = b(laIndDS+laIndk[k]);
        for(int j=ia[laIndDS+laIndk[k]]; j<ia[laIndDS+laIndk[k]+1]; j++)
          gddk[k](0) += a[j]*laMBS(ja[j]);

        lak[k](0) = fcl->project(lak[k](0), gddk[k](0), rFactork[k](0));

        if(gdActive[k][1]) {
          for(int i=1; i<=getFrictionDirections(); i++) {
            gddk[k](i) = b(laIndDS+laIndk[k]+i);
            for(int j=ia[laIndDS+laIndk[k]+i]; j<ia[laIndDS+laIndk[k]+1+i]; j++)
              gddk[k](i) += a[j]*laMBS(ja[j]);
          }

          if(fdf)
            lak[k](1,getFrictionDirections()) = fdf->project(lak[k](1,getFrictionDirections()), gddk[k](1,getFrictionDirections()), lak[k](0), rFactork[k](1));
        }
      }
    }
  } 

  void Contact::solveImpactsGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        gdnk[k](0) = b(laIndDS+laIndk[k]);
        for(int j=ia[laIndDS+laIndk[k]]+1; j<ia[laIndDS+laIndk[k]+1]; j++)
          gdnk[k](0) += a[j]*laMBS(ja[j]);

        double om = 1.0;
        double buf = fnil->solve(a[ia[laIndDS+laIndk[k]]], gdnk[k](0), gdk[k](0));
        lak[k](0) += om*(buf - lak[k](0));

        if(getFrictionDirections()) {
          gdnk[k](1) = b(laIndDS+laIndk[k]+1);
          for(int j=ia[laIndDS+laIndk[k]+1]+1; j<ia[laIndDS+laIndk[k]+2]; j++)
            gdnk[k](1) += a[j]*laMBS(ja[j]);

          if(ftil) {
            Vec buf = ftil->solve(ds->getG()(Index(laIndDS+laIndk[k]+1,laIndDS+laIndk[k]+getFrictionDirections())), gdnk[k](1,getFrictionDirections()), gdk[k](1,getFrictionDirections()), lak[k](0));
            lak[k](1,getFrictionDirections()) += om*(buf - lak[k](1,getFrictionDirections()));
          }
        }
      }
    }
  }

  void Contact::solveConstraintsGaussSeidel() {

    assert(getFrictionDirections() <= 1);

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        gddk[k](0) = b(laIndDS+laIndk[k]);
        for(int j=ia[laIndDS+laIndk[k]]+1; j<ia[laIndDS+laIndk[k]+1]; j++)
          gddk[k](0) += a[j]*laMBS(ja[j]);

        double om = 1.0;
        double buf = fcl->solve(a[ia[laIndDS+laIndk[k]]], gddk[k](0));
        lak[k](0) += om*(buf - lak[k](0));

        if(getFrictionDirections() && gdActive[k][1]) {
          gddk[k](1) = b(laIndDS+laIndk[k]+1);
          for(int j=ia[laIndDS+laIndk[k]+1]+1; j<ia[laIndDS+laIndk[k]+2]; j++)
            gddk[k](1) += a[j]*laMBS(ja[j]);

          if(fdf) {
            Vec buf = fdf->solve(ds->getG()(Index(laIndDS+laIndk[k]+1,laIndDS+laIndk[k]+getFrictionDirections())), gddk[k](1,getFrictionDirections()), lak[k](0));
            lak[k](1,getFrictionDirections()) += om*(buf - lak[k](1,getFrictionDirections()));
          }
        }
      }
    }
  }

  void Contact::solveImpactsRootFinding() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        for(int i=0; i < 1+getFrictionDirections(); i++) {
          gdnk[k](i) = b(laIndDS+laIndk[k]+i);
          for(int j=ia[laIndDS+laIndk[k]+i]; j<ia[laIndDS+laIndk[k]+1+i]; j++)
            gdnk[k](i) += a[j]*laMBS(ja[j]);
        }

        res(0) = lak[k](0) - fnil->project(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));
        if(ftil) 
          res(1,getFrictionDirections()) = lak[k](1,getFrictionDirections()) - ftil->project(lak[k](1,getFrictionDirections()), gdnk[k](1,getFrictionDirections()), gdk[k](1,getFrictionDirections()), lak[k](0), rFactork[k](1));
      }
    }
  }

  void Contact::solveConstraintsRootFinding() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        for(int i=0; i < 1+getFrictionDirections(); i++) {
          gddk[k](i) = b(laIndDS+laIndk[k]+i);
          for(int j=ia[laIndDS+laIndk[k]+i]; j<ia[laIndDS+laIndk[k]+1+i]; j++)
            gddk[k](i) += a[j]*laMBS(ja[j]);
        }

        res(0) = lak[k](0) - fcl->project(lak[k](0), gddk[k](0), rFactork[k](0));
        if(fdf) 
          res(1,getFrictionDirections()) = lak[k](1,getFrictionDirections()) - fdf->project(lak[k](1,getFrictionDirections()), gddk[k](1,getFrictionDirections()), lak[k](0), rFactork[k](1));
      }
    }
  }

  void Contact::jacobianConstraints() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        SqrMat Jprox = ds->getJprox();
        SqrMat G = ds->getG();

        RowVec jp1=Jprox.row(laIndDS+laIndk[k]);
        RowVec e1(jp1.size());
        e1(laIndDS+laIndk[k]) = 1;
        Vec diff = fcl->diff(lak[k](0), gddk[k](0), rFactork[k](0));

        jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndDS+laIndk[k])
        for(int i=0; i<G.size(); i++) 
          jp1(i) -= diff(1)*G(laIndDS+laIndk[k],i);

        if(getFrictionDirections() == 1) {
          Mat diff = fdf->diff(lak[k](1,1), gddk[k](1,1), lak[k](0), rFactork[k](1));
          RowVec jp2=Jprox.row(laIndDS+laIndk[k]+1);
          RowVec e2(jp2.size());
          e2(laIndDS+laIndk[k]+1) = 1;
          Mat e(2,jp2.size());
          e(0,laIndDS+laIndk[k]) = 1;
          e(1,laIndDS+laIndk[k]+1) = 1;
          jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laIndDS+laIndk[k])
          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laIndDS+laIndk[k])
          for(int i=0; i<G.size(); i++) 
            jp2(i) -= diff(0,1)*G(laIndDS+laIndk[k]+1,i);

        }
        else if(getFrictionDirections() == 2) {
          Mat diff = ftil->diff(lak[k](1,2), gddk[k](1,2), gdk[k](1,2), lak[k](0), rFactork[k](1));
          Mat jp2=Jprox(Index(laIndDS+laIndk[k]+1,laIndDS+laIndk[k]+2),Index(0,Jprox.cols()));
          Mat e2(2,jp2.cols());
          e2(0,laIndDS+laIndk[k]+1) = 1;
          e2(1,laIndDS+laIndk[k]+2) = 1;
          jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laIndDS+laIndk[k]+1,laIndDS+laIndk[k]+2),Index(0,G.size()-1))
          for(int i=0; i<G.size(); i++) {
            jp2(0,i) = diff(0,2)*G(laIndDS+laIndk[k]+1,i)+diff(0,3)*G(laIndDS+laIndk[k]+2,i);
            jp2(1,i) = diff(1,2)*G(laIndDS+laIndk[k]+1,i)+diff(1,3)*G(laIndDS+laIndk[k]+2,i);
          }
        }
      }
    }
  }

  void Contact::jacobianImpacts() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        SqrMat Jprox = ds->getJprox();
        SqrMat G = ds->getG();

        RowVec jp1=Jprox.row(laIndDS+laIndk[k]);
        RowVec e1(jp1.size());
        e1(laIndDS+laIndk[k]) = 1;
        Vec diff = fnil->diff(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));

        jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndDS+laIndk[k])
        for(int i=0; i<G.size(); i++) 
          jp1(i) -= diff(1)*G(laIndDS+laIndk[k],i);

        if(getFrictionDirections() == 1) {
          Mat diff = ftil->diff(lak[k](1,1), gdnk[k](1,1), gdk[k](1,1), lak[k](0), rFactork[k](1));
          RowVec jp2=Jprox.row(laIndDS+laIndk[k]+1);
          RowVec e2(jp2.size());
          e2(laIndDS+laIndk[k]+1) = 1;
          Mat e(2,jp2.size());
          e(0,laIndDS+laIndk[k]) = 1;
          e(1,laIndDS+laIndk[k]+1) = 1;
          jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laIndDS+laIndk[k])
          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laIndDS+laIndk[k])
          for(int i=0; i<G.size(); i++) 
            jp2(i) -= diff(0,1)*G(laIndDS+laIndk[k]+1,i);

        } else if(getFrictionDirections() == 2) {
          Mat diff = ftil->diff(lak[k](1,2), gdnk[k](1,2), gdk[k](1,2), lak[k](0), rFactork[k](1));
          Mat jp2=Jprox(Index(laIndDS+laIndk[k]+1,laIndDS+laIndk[k]+2),Index(0,Jprox.cols()));
          Mat e2(2,jp2.cols());
          e2(0,laIndDS+laIndk[k]+1) = 1;
          e2(1,laIndDS+laIndk[k]+2) = 1;
          jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laIndDS+laIndk[k]+1,laIndDS+laIndk[k]+2),Index(0,G.size()-1))
          for(int i=0; i<G.size(); i++) {
            jp2(0,i) = diff(0,2)*G(laIndDS+laIndk[k]+1,i)+diff(0,3)*G(laIndDS+laIndk[k]+2,i);
            jp2(1,i) = diff(1,2)*G(laIndDS+laIndk[k]+1,i)+diff(1,3)*G(laIndDS+laIndk[k]+2,i);
          }
        }
      }
    }
  }

    void Contact::updaterFactors() {

      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
        if(gActive[k]) {

          double *a = ds->getGs()();
          int *ia = ds->getGs().Ip();
          double sumN = 0;

          for(int j=ia[laIndDS+laIndk[k]]+1; j<ia[laIndDS+laIndk[k]+1]; j++)
            sumN += fabs(a[j]);
          double aN = a[ia[laIndDS+laIndk[k]]];
          if(aN > sumN) {
            rFactorUnsure(0) = 0;
            rFactork[k](0) = 1.0/aN;
          } else {
            rFactorUnsure(0) = 1;
            rFactork[k](0) = rMax/aN;
          }
          double sumT1 = 0;
          double sumT2 = 0;
          double aT1, aT2;
          if(fdf && gdActive[k][1]) {
            if(getFrictionDirections() == 1) {
              for(int j=ia[laIndDS+laIndk[k]+1]+1; j<ia[laIndDS+laIndk[k]+2]; j++)
                sumT1 += fabs(a[j]);
              aT1 = a[ia[laIndDS+laIndk[k]+1]];
              if(aT1 > sumT1) {
                rFactorUnsure(1)=0;
                rFactork[k](1) = 1.0/aT1;
              } else {
                rFactorUnsure(1)=1;
                rFactork[k](1) = rMax/aT1;
              }
            } else if(getFrictionDirections() == 2) {
              for(int j=ia[laIndDS+laIndk[k]+1]+1; j<ia[laIndDS+laIndk[k]+2]; j++)
                sumT1 += fabs(a[j]);
              for(int j=ia[laIndDS+laIndk[k]+2]+1; j<ia[laIndDS+laIndk[k]+3]; j++)
                sumT2 += fabs(a[j]);
              aT1 = a[ia[laIndDS+laIndk[k]+1]];
              aT2 = a[ia[laIndDS+laIndk[k]+2]];

              // TODO rFactorUnsure
              if(aT1 - sumT1 >= aT2 - sumT2) 
                if(aT1 + sumT1 >= aT2 + sumT2) 
                  rFactork[k](1) = 2.0/(aT1+aT2+sumT1-sumT2);
                else 
                  rFactork[k](1) = 1.0/aT2;
              else 
                if(aT1 + sumT1 < aT2 + sumT2) 
                  rFactork[k](1) = 2.0/(aT1+aT2-sumT1+sumT2);
                else 
                  rFactork[k](1) = 1.0/aT1;
            }
          }
        }
      }
    }

  void Contact::checkConstraintsForTermination() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) { // TODO check if gdActive[k][0]

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        for(unsigned int i=0; i < 1+ gdActive[k][1]*getFrictionDirections(); i++) {
          gddk[k](i) = b(laIndDS+laIndk[k]+i);
          for(int j=ia[laIndDS+laIndk[k]+i]; j<ia[laIndDS+laIndk[k]+1+i]; j++)
            gddk[k](i) += a[j]*laMBS(ja[j]);
        }

        if(!fcl->isFullfield(lak[k](0),gddk[k](0),laTol,gddTol)) {
          ds->setTermination(false);
          return;
        }
        if(fdf && gdActive[k][1]) 
          if(!fdf->isFullfield(lak[k](1,getFrictionDirections()),gddk[k](1,getFrictionDirections()),lak[k](0),laTol,gddTol)) {
            ds->setTermination(false);
            return;
          }
      }
    }
  }

  void Contact::checkImpactsForTermination() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        double *a = ds->getGs()();
        int *ia = ds->getGs().Ip();
        int *ja = ds->getGs().Jp();
        Vec &laMBS = ds->getla();
        Vec &b = ds->getb();

        for(int i=0; i < 1+getFrictionDirections(); i++) {
          gdnk[k](i) = b(laIndDS+laIndk[k]+i);
          for(int j=ia[laIndDS+laIndk[k]+i]; j<ia[laIndDS+laIndk[k]+1+i]; j++)
            gdnk[k](i) += a[j]*laMBS(ja[j]);
        }

        if(!fnil->isFullfield(lak[k](0),gdnk[k](0),gdk[k](0),LaTol,gdTol)) {
          ds->setTermination(false);
          return;
        }
        if(ftil) 
          if(!ftil->isFullfield(lak[k](1,getFrictionDirections()),gdnk[k](1,getFrictionDirections()),gdk[k](1,getFrictionDirections()),lak[k](0),LaTol,gdTol)) {
            ds->setTermination(false);
            return;
          }
      }
    }
  }
  
  void Contact::checkActiveg() { 
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      gActive[k] = fcl->isActive(gk[k](0),0) ? 1 : 0; 
  }

  void Contact::checkActivegd() { 
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gdActive[i][0] = gActive[i] ? (fcl->remainsActive(gdk[i](0),gdTol) ? 1 : 0) : 0; 
      gdActive[i][1] = getFrictionDirections() && gdActive[i][0] ? (fdf->isSticking(gdk[i](1,getFrictionDirections()),gdTol) ? 1 : 0) : 0; 
    }
  }

  void Contact::checkActivegdn() { 
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {
        if(gdnk[k](0) <= gdTol) {
          gActive[k] = true;
          gdActive[k][0] = true;
        } 
        else {
          gActive[k] = false;
          gdActive[k][0] = false;
        }
      }
      if(getFrictionDirections()) {
        if(gdActive[k][0])
          if(nrm2(gdnk[k](1,getFrictionDirections())) <= gdTol)
            gdActive[k][1] = true;
          else
            gdActive[k][1] = false;
        else
          gdActive[k][1] = false;
      }
    }
  }

  void Contact::checkActivegdd() { 
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gdActive[k][0]) {
        if(gddk[k](0) <= gddTol) {
          gActive[k] = true;
          gdActive[k][0] = true;
        }
        else {
          gActive[k] = false;
          gdActive[k][0] = false;
        }
      }
      if(getFrictionDirections())
        if(gdActive[k][0]) {
          if(gdActive[k][1]) 
            if(nrm2(gddk[k](1,getFrictionDirections())) <= gddTol)
              gdActive[k][1] = true;
            else
              gdActive[k][1] = false;
          else 
            gdActive[k][1] = false;
        }
    }
  }

  void Contact::checkAllgd() { 
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gdActive[i][0] = gActive[i] ? 1 : 0; 
      gdActive[i][1] = getFrictionDirections() && gActive[i] ? 1 : 0; 
    }
  }

  void Contact::updateCondition() {

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(jsvk[k](0)) {
        if(gActive[k]) {
          gActive[k] = false;
          gdActive[k][0] = false;
          if(getFrictionDirections())
            gdActive[k][1] = false;
          return;
        }
        else {// TODO if(gdk[k](0)<=0) { // pseudo collision because of penetration
          gActive[k] = true;
          gdActive[k][0] = true;
          if(getFrictionDirections())
            gdActive[k][1] = true;
          ds->setImpact(true);
          return;
        }
        }
        if(getFrictionDirections())
          if(jsvk[k](1)) {
            if(gdActive[k][1]) {
              gdActive[k][1] = false;
            } 
            else {
              gdActive[k][1] = true;
              ds->setSticking(true);
            }
          }
      }
    }

  void Contact::resizeJacobians(int j) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {
        for(unsigned int i=0; i<2; i++) 
          cpData[k][i].getFrameOfReference().resizeJacobians(j);
      }
    }
  }

  //void Contact::load(const string& path, ifstream &inputfile) {
  //  LinkMechanics::load(path,inputfile);
  //  string dummy;
  //  int s = inputfile.tellg();
  //  getline(inputfile,dummy); // # Type of contact law:
  //  getline(inputfile,dummy); // Type of contact law 
  //  inputfile.seekg(s,ios::beg);
  //  ClassFactory cf;
  //  setContactForceLaw(cf.getGeneralizedForceLaw(dummy));
  //  fcl->load(path, inputfile);

  //  s = inputfile.tellg();
  //  getline(inputfile,dummy); // # Type of friction law:
  //  getline(inputfile,dummy); // Type of friction law 
  //  inputfile.seekg(s,ios::beg);
  //  if(dummy.empty()) {
  //    getline(inputfile,dummy); // # Type of friction law
  //    getline(inputfile,dummy); // End of line
  //  } else {
  //    setFrictionForceLaw(cf.getFrictionForceLaw(dummy));
  //    fdf->load(path, inputfile);
  //  }

  //  s = inputfile.tellg();
  //  getline(inputfile,dummy); // # Type of normal impact law:
  //  cout << dummy << endl;
  //  getline(inputfile,dummy); // Type of normal impact law 
  //  cout << dummy << endl;
  //  inputfile.seekg(s,ios::beg);
  //  setContactImpactLaw(cf.getGeneralizedImpactLaw(dummy));
  //  fnil->load(path, inputfile);

  //  s = inputfile.tellg();
  //  getline(inputfile,dummy); // # Type of tangential impact law:
  //  getline(inputfile,dummy); // Type of tangential impact law 
  //  inputfile.seekg(s,ios::beg);
  //  if(dummy.empty()) {
  //    getline(inputfile,dummy); // # Type of friction law
  //    getline(inputfile,dummy); // End of line
  //  } else {
  //    setFrictionImpactLaw(cf.getFrictionImpactLaw(dummy));
  //    ftil->load(path, inputfile);
  //  }
  //}

  //void Contact::save(const string& path, ofstream &outputfile) {
  //  LinkMechanics::save(path, outputfile);

  //  fcl->save(path,outputfile);
  //  if(fdf)
  //    fdf->save(path,outputfile);
  //  else {
  //    outputfile << "# Type of friction force law:" << endl << endl;
  //  }

  //  if(fnil)
  //    fnil->save(path,outputfile);
  //  else
  //    outputfile << "# Type of contact impact law:" << endl << endl;

  //  if(ftil)
  //    ftil->save(path,outputfile);
  //  else {
  //    outputfile << "# Type of friction impact law:" << endl << endl;
  //  }
  //}

    int Contact::getFrictionDirections() {
      if(fdf) 
        return fdf->getFrictionDirections();
      else
        return 0;
    }

    void Contact::connect(Contour *contour0, Contour* contour1) {
      LinkMechanics::connect(contour0);
      LinkMechanics::connect(contour1);
    }

  }

