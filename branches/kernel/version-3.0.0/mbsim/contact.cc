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

  Contact::Contact(const string &name) : LinkMechanics(name), contactKinematics(0), fcl(0), fdf(0), fnil(0), ftil(0), cpData(0), gActive(0), gActive0(0), gdActive(0), gk(0), gdk(0), gdnk(0), gddk(0), lak(0), wbk(0), svk(0), rFactork(0), jsvk(0), fF(0), WF(0), Vk(0), Wk(0), laSizek(0), laIndk(0), gSizek(0), gIndk(0), gdSizek(0), gdIndk(0), svSizek(0), svIndk(0), rFactorSizek(0), rFactorIndk(0)
#ifdef HAVE_OPENMBVCPPINTERFACE
                                         , openMBVContactGrp(0), openMBVContactFrame(0), openMBVNormalForceArrow(0), openMBVFrictionArrow(0), openMBVContactFrameSize(0), openMBVContactFrameEnabled(true), contactArrow(NULL), frictionArrow(NULL)
#endif
                                           , saved_ref1(""), saved_ref2("")
                                           {
 watchg = true; 
 watchgd[0] = true; 
 watchgd[1] = true; 
 slide_right = true;
					   }

  Contact::~Contact() {
    if(contactKinematics) { delete contactKinematics; contactKinematics=0; }
    /* Delete will fail if the same object is used for more than one Contact.
     * TODO: A delete concept (who deletes what) is still missing in MBSim.
    if(fcl) { delete fcl; fcl=0; }
    if(fdf) { delete fdf; fdf=0; }
    if(fnil) { delete fnil; fnil=0; }
    if(ftil) { delete ftil; ftil=0; }*/

    for(vector<ContourPointData*>::iterator i = cpData.begin(); i != cpData.end(); ++i)
      delete[] *i;
    for(vector<Mat*>::iterator i = Wk.begin(); i != Wk.end(); ++i)
      delete[] *i;
    for(vector<Mat*>::iterator i = Vk.begin(); i != Vk.end(); ++i)
      delete[] *i;
    for(vector<Mat*>::iterator i = fF.begin(); i != fF.end(); ++i)
      delete[] *i;
    for(vector<Vec*>::iterator i = WF.begin(); i != WF.end(); ++i)
      delete[] *i;
    for(vector<unsigned int*>::iterator i = gdActive.begin(); i != gdActive.end(); ++i)
      delete[] *i;
#ifdef HAVE_OPENMBVCPPINTERFACE
    for(vector<OpenMBV::Arrow *>::iterator i = openMBVNormalForceArrow.begin(); i != openMBVNormalForceArrow.end(); ++i)
      delete *i;
    for(vector<OpenMBV::Arrow *>::iterator i = openMBVFrictionArrow.begin(); i != openMBVFrictionArrow.end(); ++i)
      delete *i;
#endif
  }

  void Contact::updatewb(double t) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {
        for(unsigned i=0; i<contour.size(); i++) 
          wbk[k] += fF[k][i](Index(0,2),Index(0,laSizek[k]-1)).T()*cpData[k][i].getFrameOfReference().getGyroscopicAccelerationOfTranslation();
      }
    }
    contactKinematics->updatewb(wbk.begin(),gk.begin(),cpData.begin());
  }

  void Contact::updateW(double t) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) { // TODO
        fF[k][1].col(0) = cpData[k][0].getFrameOfReference().getOrientation().col(0);
        if(getFrictionDirections()) {
          fF[k][1].col(1) = cpData[k][0].getFrameOfReference().getOrientation().col(1);
          if(getFrictionDirections() > 1)
            fF[k][1].col(2) = cpData[k][0].getFrameOfReference().getOrientation().col(2);
        }

        fF[k][0] = -fF[k][1];

        for(unsigned int i=0; i<contour.size(); i++) 
          Wk[k][i] += cpData[k][i].getFrameOfReference().getJacobianOfTranslation().T()*fF[k][i](Index(0,2),Index(0,laSizek[k]-1));
      }
    }
  }

  void Contact::updateV(double t) {
    if(getFrictionDirections()) {
      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
        if(gdActive[k][0] && !gdActive[k][1]) {
          for(unsigned int i=0; i<contour.size(); i++) 
            Vk[k][i] += cpData[k][i].getFrameOfReference().getJacobianOfTranslation().T()*fF[k][i](Index(0,2),iT)*fdf->dlaTdlaN(gdk[k](1,getFrictionDirections()), lak[k](0));
        }
      }
    }
  }

  void Contact::updateh(double t) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) { // gActive should not be checked, e.g. because of possible predamping in constitutive laws
      lak[k](0) = (*fcl)(gk[k](0),gdk[k](0), this);
      if(fdf) lak[k](1,getFrictionDirections()) = (*fdf)(gdk[k](1,getFrictionDirections()),fabs(lak[k](0)));

      WF[k][1] =  cpData[k][0].getFrameOfReference().getOrientation().col(0)*lak[k](0);
      if(getFrictionDirections()) {
        WF[k][1] += cpData[k][0].getFrameOfReference().getOrientation().col(1)*lak[k](1);
        if(getFrictionDirections() > 1)
          WF[k][1] += cpData[k][0].getFrameOfReference().getOrientation().col(2)*lak[k](2);
      }
      WF[k][0] = -WF[k][1];
      for(unsigned int i=0; i<contour.size(); i++) {
        h[i] += cpData[k][i].getFrameOfReference().getJacobianOfTranslation().T()*WF[k][i];
      }
    }
  }

  void Contact::updateg(double t) {
    contactKinematics->updateg(gk.begin(),cpData.begin());
  }

  void Contact::updategd(double t) {
    const bool flag = fcl->isSetValued();
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if((flag && gActive[k]) || (!flag && fcl->isActive(gk[k](0),0))) { // TODO
        for(unsigned int i=0; i<2; i++) contour[i]->updateKinematicsForFrame(cpData[k][i],velocities); // angular velocity necessary e.g. see ContactKinematicsSpherePlane::updatewb

        Vec Wn = cpData[k][0].getFrameOfReference().getOrientation().col(0);

        Vec WvD = cpData[k][1].getFrameOfReference().getVelocity() - cpData[k][0].getFrameOfReference().getVelocity();

        gdk[k](0) = Wn.T()*WvD;

        if(gdk[k].size()>1) {
          Mat Wt(3,gdk[k].size()-1);
          Wt.col(0) = cpData[k][0].getFrameOfReference().getOrientation().col(1);
          if(gdk[k].size() > 2)
            Wt.col(1) = cpData[k][0].getFrameOfReference().getOrientation().col(2);

          gdk[k](1,gdk[k].size()-1) = Wt.T()*WvD;
        }
	//cout << name << " updategd at t = " << t << endl;
	//cout << gk[0] << endl;
	//cout << gdk[0] << endl;
      }
    }
  }

  void Contact::updateStopVector(double t) {
    //cout << name << " updateStopVector at t= " << t << endl;
    //cout << gk[0] << endl;
    //cout << gdk[0] << endl;
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
       //svk[k](0) = (fcl->remainsActive(gdk[k](0),gdTol)&& fcl->isActive(gk[k](0),1e-10)) ? 1 : -1; // TODO
       //svk[k](0) = gk[k](0); // TODO
       //svk[k](0) =  fcl->isActive(gk[k](0),0) && gdk[k](0)<-gdTol ? 1 : -1; // TODO
       //svk[k](0) =  fcl->isActive(gk[k](0),gTol) ? 1 : -1; // TODO
       if(watchg) {
	 svk[k](0) =  gk[k](0)>0 ? 1 : -1; // TODO
       } else {
	 if(watchgd[0]) {
	   svk[k](0) =  gdk[k](0)>0 ? 1 : -1; // TODO
	 } else {
	   if(gdk[k](0)>gdTol)
	     watchgd[0]= true;
	   svk[k](0) = 1;
	 }
	 if(gk[k](0)>gTol)
	   watchg = true;
       }
       if(getFrictionDirections()){
	 //svk[k](1) = (fcl->isActive(gk[k](0),gTol) && abs(gdk[k](0))<gdTol && fdf->isSticking(gdk[k](1,getFrictionDirections()),gdTol)) ? 1 : -1;
	 //svk[k](1) = (fcl->isActive(gk[k](0),gTol) && fdf->isSticking(gdk[k](1,getFrictionDirections()),gdTol)) ? 1 : -1;
	 if(!watchg && !watchgd[0]) {
	   if(watchgd[1]) {
	     if(slide_right)
	       svk[k](1) =  gdk[k](1)>0 ? 1 : -1; // TODO
	     else
	       svk[k](1) =  gdk[k](1)>0 ? -1 : 1; // TODO
	   } else {
	       svk[k](1) = 1;
	       if(abs(gdk[k](1))>gdTol) {
		 watchgd[1]= true;
		 slide_right = gdk[k](1) > 0;
	       }
	   }
	 } else
	   svk[k](1) = 1;
       //svk[k](1) = 1;
    }
    }
    //cout << sv << endl;
  }

  void Contact::updateJacobians(double t) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      if(gActive[k]) 
        for(unsigned int i=0; i<2; i++) contour[i]->updateJacobiansForFrame(cpData[k][i]);
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
    //cout << "calclaSize" << endl;
    //cout << name << endl;
    //cout<< gdActive[0][0] << endl;
    LinkMechanics::calclaSize();
    laSize = 0;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      laIndk[i] = laSize;
      laSizek[i] = gdActive[i][0]+gdActive[i][1]*getFrictionDirections();
      laSize += laSizek[i];
    }
    //cout<< laSize << endl;
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
      //svSizek[i] = isSetValued() ? 1 : 0;
      svSize += svSizek[i];
    }
  }

  void Contact::calcLinkStatusSize() {
    LinkMechanics::calcLinkStatusSize();
    int n = contactKinematics->getNumberOfPotentialContactPoints();
    LinkStatusSize= n;
    LinkStatus.resize(LinkStatusSize);
  }

  void Contact::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Contour>(saved_ref1), getByPath<Contour>(saved_ref2));
      LinkMechanics::init(stage);
    }
    else if(stage==resize) {
      LinkMechanics::init(stage);
      int n = contactKinematics->getNumberOfPotentialContactPoints();

      la.resize(n*(1+getFrictionDirections()));
      g.resize(n);
      gd.resize(n*(1+getFrictionDirections()));
      gdd.resize(gd.size());
      gdn.resize(gd.size());
      //LinkStatusSize= n;
      //LinkStatus.resize(LinkStatusSize);

      for(vector<ContourPointData*>::iterator i = cpData.begin(); i != cpData.end(); ++i) delete[] *i;
      cpData.clear(); // clear container first, because InitStage resize is called twice (before and after the reorganization)
      for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
        if(getFrictionDirections() == 0)
          gdActive[i][1] = false;

        cpData.push_back(new ContourPointData[2]);

        cpData[i][0].getFrameOfReference().setName("0");
        cpData[i][1].getFrameOfReference().setName("1");
        cpData[i][0].getFrameOfReference().getJacobianOfTranslation().resize();
        cpData[i][0].getFrameOfReference().getJacobianOfRotation().resize();
        cpData[i][1].getFrameOfReference().getJacobianOfTranslation().resize();
        cpData[i][1].getFrameOfReference().getJacobianOfRotation().resize();

	cpData[i][0].getFrameOfReference().sethSize(contour[0]->gethSize(0),0);
        cpData[i][0].getFrameOfReference().sethSize(contour[0]->gethSize(1),1);
        cpData[i][1].getFrameOfReference().sethSize(contour[1]->gethSize(0),0);
        cpData[i][1].getFrameOfReference().sethSize(contour[1]->gethSize(1),1);

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
    }
    else if(stage==unknownStage) {
      LinkMechanics::init(stage);

      iT = Index(1,getFrictionDirections());

      for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
        lak[k].resize() >> la(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
        gdk[k].resize() >> gd(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
        gdnk[k].resize() >> gdn(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
        gddk[k].resize() >> gdd(k*(1+getFrictionDirections()),(k+1)*(1+getFrictionDirections())-1);
        gk[k].resize() >> g(k,k);
      }
    }
    else if(stage==preInit) {
      LinkMechanics::init(stage);

      if(contactKinematics==0)
        contactKinematics = contour[0]->findContactPairingWith(contour[0]->getType(), contour[1]->getType());
      if(contactKinematics==0)
        contactKinematics = contour[1]->findContactPairingWith(contour[1]->getType(), contour[0]->getType());
      if(contactKinematics==0)
        contactKinematics = contour[0]->findContactPairingWith(contour[1]->getType(), contour[0]->getType());
      if(contactKinematics==0)
        contactKinematics = contour[1]->findContactPairingWith(contour[0]->getType(), contour[1]->getType());
      if(contactKinematics==0)
        throw MBSimError("Unknown contact pairing between Contour \""+contour[0]->getType()+"\" and Contour\""+contour[1]->getType()+"\"!");

      contactKinematics->assignContours(contour[0],contour[1]);

      for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
        gActive.push_back(int(1));
        gActive0.push_back(int(1));
        gdActive.push_back(new unsigned int[2]);
        for(int j=0; j<1+min(1,getFrictionDirections()); j++) gdActive[i][j] = 1;
        for(int j=1+min(1,getFrictionDirections()); j<2; j++) gdActive[i][j] = 0;

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
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);
      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && (openMBVContactFrameSize>epsroot() || contactArrow || frictionArrow)) {
          openMBVContactGrp = new OpenMBV::Group();
          openMBVContactGrp->setName(name+"_ContactGroup");
          openMBVContactGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVContactGrp);
          for(unsigned int i=0; i<cpData.size(); i++) {
            if(openMBVContactFrameSize>epsroot()) {
              openMBVContactFrame.push_back(new OpenMBV::Frame[2]);
              for(unsigned int k=0; k<2; k++) { // frames
                openMBVContactFrame[i][k].setOffset(1.);
                openMBVContactFrame[i][k].setSize(openMBVContactFrameSize);
                openMBVContactFrame[i][k].setName("ContactPoint_"+numtostr((int)i)+(k==0?"A":"B"));
                openMBVContactFrame[i][k].setEnable(openMBVContactFrameEnabled);
                openMBVContactGrp->addObject(&openMBVContactFrame[i][k]);
              }
            }
            // arrows
            OpenMBV::Arrow *arrow;
            if(contactArrow) {
              arrow=new OpenMBV::Arrow(*contactArrow);
              arrow->setName("NormalForce_"+numtostr((int)i)+"B");
              openMBVNormalForceArrow.push_back(arrow); // normal force
              openMBVContactGrp->addObject(arrow);
            }
            if(frictionArrow && getFrictionDirections()>0) { // friction force
              arrow=new OpenMBV::Arrow(*frictionArrow);
              arrow->setName("FrictionForce_"+numtostr((int)i)+"B");
              openMBVFrictionArrow.push_back(arrow);
              openMBVContactGrp->addObject(arrow);
            }
          }
        }
#endif
        if(getPlotFeature(generalizedLinkForce)==enabled) {
          for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
            for(int j=0; j<1+getFrictionDirections(); ++j)
              plotColumns.push_back("la["+numtostr(i)+"]("+numtostr(j)+")");
          }
        }
        if(getPlotFeature(linkKinematics)==enabled) {
          for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
            plotColumns.push_back("g["+numtostr(i)+"]("+numtostr(0)+")");
            for(int j=0; j<1+getFrictionDirections(); ++j) 
              plotColumns.push_back("gd["+numtostr(i)+"]("+numtostr(j)+")");
          }
        }
        LinkMechanics::init(stage);
      }
    }
    else
      LinkMechanics::init(stage);
  }

  bool Contact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if(fdf) 
      flag |= fdf->isSetValued();
    return flag;
  }

 
  void Contact::updateLinkStatus(double t) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if (gActive[k])  {
        LinkStatus(k) = 2;
        if (ftil) {
          if (ftil->isSticking(lak[k](1,getFrictionDirections()),gdnk[k](1,getFrictionDirections()),gdk[k](1,getFrictionDirections()),lak[k](0),LaTol,gdTol)) LinkStatus(k) = 3;
          else LinkStatus(k) = 4;
        }
      }
      else LinkStatus(k) = 1;
    }
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

  void Contact::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && (openMBVContactFrameSize>epsroot() || contactArrow || frictionArrow)) {
        for(unsigned int i=0; i<cpData.size(); i++) {
          // frames
          if(openMBVContactFrameSize>epsroot()) {
            for (unsigned int k=0; k<2; k++) {
              vector<double> data;
              data.push_back(t);
              data.push_back(cpData[i][k].getFrameOfReference().getPosition()(0));
              data.push_back(cpData[i][k].getFrameOfReference().getPosition()(1));
              data.push_back(cpData[i][k].getFrameOfReference().getPosition()(2));
              Vec cardan=AIK2Cardan(cpData[i][k].getFrameOfReference().getOrientation());
              data.push_back(cardan(0));
              data.push_back(cardan(1));
              data.push_back(cardan(2));
              data.push_back(0);
              openMBVContactFrame[i][k].append(data);
            }
          }
          // arrows
          // normal force
          vector<double> data;
          if(contactArrow) {
            data.push_back(t);
            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(0));
            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(1));
            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(2));
            Vec F(3,INIT,0);
            if(isSetValued()) {
              if(gActive[i]) F=fF[i][1].col(0)*lak[i](0)/dt;
            }
            else
              F=cpData[i][0].getFrameOfReference().getOrientation().col(0)*lak[i](0);
            data.push_back(F(0));
            data.push_back(F(1));
            data.push_back(F(2));
            data.push_back(nrm2(F));
            openMBVNormalForceArrow[i]->append(data);
          }
          if(frictionArrow && getFrictionDirections()>0) { // friction force
            data.clear();
            data.push_back(t);
            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(0));
            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(1));
            data.push_back(cpData[i][1].getFrameOfReference().getPosition()(2));
            Vec F(3,INIT,0);
            if(isSetValued()) {                    // TODO switch between stick and slip not possible with TimeStepper
              if(gActive[i] && lak[i].size()>1) { // stick friction
                F=fF[i][1].col(1)*lak[i](1)/dt;
                if(getFrictionDirections()>1)
                  F+=fF[i][1].col(2)*lak[i](2)/dt;
              }
              if(gActive[i] && lak[i].size()==1) // slip friction
                F=fF[i][1](Index(0,2),iT)*fdf->dlaTdlaN(gdk[i](1,getFrictionDirections()), lak[i](0))*lak[i](0)/dt;
            }
            else {
              F=cpData[i][0].getFrameOfReference().getOrientation().col(1)*lak[i](1);
              if(getFrictionDirections()>1)
                F+=cpData[i][0].getFrameOfReference().getOrientation().col(2)*lak[i](2);
            }
            data.push_back(F(0));
            data.push_back(F(1));
            data.push_back(F(2));
            data.push_back((isSetValued() && lak[i].size()>1)?1:0.5); // draw in green if slipping and draw in red if sticking
            openMBVFrictionArrow[i]->append(data);
          }
        }
      }
#endif
    if(getPlotFeature(generalizedLinkForce)==enabled) {
        for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
          if(gActive[i] && gdActive[i][0]) {
            plotVector.push_back(lak[i](0)/(isSetValued()?dt:1.));
            if(gdActive[i][1]) {
              for(int j=0; j<getFrictionDirections(); j++)
                plotVector.push_back(lak[i](1+j)/(isSetValued()?dt:1.));
            }
            else {
              if(fdf) {
                Vec buf = fdf->dlaTdlaN(gdk[i](1,getFrictionDirections()), lak[i](0))*lak[i](0);
                for(int j=0; j<getFrictionDirections(); j++)
                  plotVector.push_back(buf(j)/(isSetValued()?dt:1.));
              }
            }
          } 
          else {
            for(int j=0; j<1+getFrictionDirections() ; j++)
              plotVector.push_back(0);
          }
        }
      }
      if(getPlotFeature(linkKinematics)==enabled) {
        bool flag = fcl->isSetValued();
        for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
          plotVector.push_back(gk[i](0)); //gN
          if((flag && gActive[i]) || (!flag && fcl->isActive(gk[i](0),0))) {
            for(int j=0; j<1+getFrictionDirections(); j++)
              plotVector.push_back(gdk[i](j)); //gd
          } 
          else {
            for(int j=0; j<1+getFrictionDirections(); j++)
              plotVector.push_back(NAN); //gd
          }
        }
      }
      LinkMechanics::plot(t, dt);
    }
  }

  void Contact::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      LinkMechanics::closePlot();
    }
#ifdef HAVE_OPENMBVCPPINTERFACE
    for(unsigned int i=0; i<openMBVContactFrame.size(); i++) {
      delete[] openMBVContactFrame[i];
      openMBVContactFrame[i]=0;
    }
    if(contactArrow) { delete contactArrow; contactArrow=0; }
    if(frictionArrow) { delete frictionArrow; frictionArrow=0; }
    if(openMBVContactGrp) { delete openMBVContactGrp; openMBVContactGrp=0; }
#endif
  }

  void Contact::solveImpactsFixpointSingle(double dt) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        gdnk[k](0) = b(laInd+laIndk[k]);
        for(int j=ia[laInd+laIndk[k]]; j<ia[laInd+laIndk[k]+1]; j++)
          gdnk[k](0) += a[j]*laMBS(ja[j]);

        lak[k](0) = fnil->project(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));

        for(int i=1; i<=getFrictionDirections(); i++) {
          gdnk[k](i) = b(laInd+laIndk[k]+i);
          for(int j=ia[laInd+laIndk[k]+i]; j<ia[laInd+laIndk[k]+1+i]; j++)
            gdnk[k](i) += a[j]*laMBS(ja[j]);
        }

        if(ftil)
          lak[k](1,getFrictionDirections()) = ftil->project(lak[k](1,getFrictionDirections()), gdnk[k](1,getFrictionDirections()), gdk[k](1,getFrictionDirections()), lak[k](0), rFactork[k](1));
      }
    }
  }

  void Contact::solveConstraintsFixpointSingle() {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gdActive[k][0]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        gddk[k](0) = b(laInd+laIndk[k]);
        for(int j=ia[laInd+laIndk[k]]; j<ia[laInd+laIndk[k]+1]; j++)
          gddk[k](0) += a[j]*laMBS(ja[j]);

        lak[k](0) = fcl->project(lak[k](0), gddk[k](0), rFactork[k](0));

        if(gdActive[k][1]) {
          for(int i=1; i<=getFrictionDirections(); i++) {
            gddk[k](i) = b(laInd+laIndk[k]+i);
            for(int j=ia[laInd+laIndk[k]+i]; j<ia[laInd+laIndk[k]+1+i]; j++)
              gddk[k](i) += a[j]*laMBS(ja[j]);
          }

          if(fdf)
            lak[k](1,getFrictionDirections()) = fdf->project(lak[k](1,getFrictionDirections()), gddk[k](1,getFrictionDirections()), lak[k](0), rFactork[k](1));
        }
      }
    }
  } 

  void Contact::solveImpactsGaussSeidel(double dt) {
    assert(getFrictionDirections() <= 1);

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        gdnk[k](0) = b(laInd+laIndk[k]);
        for(int j=ia[laInd+laIndk[k]]+1; j<ia[laInd+laIndk[k]+1]; j++)
          gdnk[k](0) += a[j]*laMBS(ja[j]);

        const double om = 1.0;
        const double buf = fnil->solve(a[ia[laInd+laIndk[k]]], gdnk[k](0), gdk[k](0));
        lak[k](0) += om*(buf - lak[k](0));

        if(getFrictionDirections()) {
          gdnk[k](1) = b(laInd+laIndk[k]+1);
          for(int j=ia[laInd+laIndk[k]+1]+1; j<ia[laInd+laIndk[k]+2]; j++)
            gdnk[k](1) += a[j]*laMBS(ja[j]);

          if(ftil) {
            Vec buf = ftil->solve(ds->getG()(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+getFrictionDirections())), gdnk[k](1,getFrictionDirections()), gdk[k](1,getFrictionDirections()), lak[k](0));
            lak[k](1,getFrictionDirections()) += om*(buf - lak[k](1,getFrictionDirections()));
          }
        }
      }
    }
  }

  void Contact::solveConstraintsGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gdActive[k][0]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        gddk[k](0) = b(laInd+laIndk[k]);
        for(int j=ia[laInd+laIndk[k]]+1; j<ia[laInd+laIndk[k]+1]; j++)
          gddk[k](0) += a[j]*laMBS(ja[j]);

        const double om = 1.0; // relaxation parameter omega (cf. Foerg, dissertation, p. 102)
        const double buf = fcl->solve(a[ia[laInd+laIndk[k]]], gddk[k](0));
        lak[k](0) += om*(buf - lak[k](0));

        if(getFrictionDirections() && gdActive[k][1]) {
          gddk[k](1) = b(laInd+laIndk[k]+1);
          for(int j=ia[laInd+laIndk[k]+1]+1; j<ia[laInd+laIndk[k]+2]; j++)
            gddk[k](1) += a[j]*laMBS(ja[j]);

          if(fdf) {
            Vec buf = fdf->solve(ds->getG()(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+getFrictionDirections())), gddk[k](1,getFrictionDirections()), lak[k](0));
            lak[k](1,getFrictionDirections()) += om*(buf - lak[k](1,getFrictionDirections()));
          }
        }
      }
    }
  }

  void Contact::solveImpactsRootFinding(double dt) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        for(int i=0; i < 1+getFrictionDirections(); i++) {
          gdnk[k](i) = b(laInd+laIndk[k]+i);
          for(int j=ia[laInd+laIndk[k]+i]; j<ia[laInd+laIndk[k]+1+i]; j++)
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
      if(gdActive[k][0]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        for(int i=0; i < 1+getFrictionDirections(); i++) {
          gddk[k](i) = b(laInd+laIndk[k]+i);
          for(int j=ia[laInd+laIndk[k]+i]; j<ia[laInd+laIndk[k]+1+i]; j++)
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
      if(gdActive[k][0]) {

        const SqrMat Jprox = ds->getJprox();
        const SqrMat G = ds->getG();

        RowVec jp1=Jprox.row(laInd+laIndk[k]);
        RowVec e1(jp1.size());
        e1(laInd+laIndk[k]) = 1;
        Vec diff = fcl->diff(lak[k](0), gddk[k](0), rFactork[k](0));

        jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laInd+laIndk[k])
        for(int i=0; i<G.size(); i++) 
          jp1(i) -= diff(1)*G(laInd+laIndk[k],i);

        if(getFrictionDirections() == 1) {
          Mat diff = fdf->diff(lak[k](1,1), gddk[k](1,1), lak[k](0), rFactork[k](1));
          RowVec jp2=Jprox.row(laInd+laIndk[k]+1);
          RowVec e2(jp2.size());
          e2(laInd+laIndk[k]+1) = 1;
          Mat e(2,jp2.size());
          e(0,laInd+laIndk[k]) = 1;
          e(1,laInd+laIndk[k]+1) = 1;
          jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laInd+laIndk[k])
          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk[k])
          for(int i=0; i<G.size(); i++) 
            jp2(i) -= diff(0,1)*G(laInd+laIndk[k]+1,i);

        }
        else if(getFrictionDirections() == 2) {
          Mat diff = ftil->diff(lak[k](1,2), gddk[k](1,2), gdk[k](1,2), lak[k](0), rFactork[k](1));
          Mat jp2=Jprox(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+2),Index(0,Jprox.cols()-1));
          Mat e2(2,jp2.cols());
          e2(0,laInd+laIndk[k]+1) = 1;
          e2(1,laInd+laIndk[k]+2) = 1;
          jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+2),Index(0,G.size()-1))
          for(int i=0; i<G.size(); i++) {
            jp2(0,i) = diff(0,2)*G(laInd+laIndk[k]+1,i)+diff(0,3)*G(laInd+laIndk[k]+2,i);
            jp2(1,i) = diff(1,2)*G(laInd+laIndk[k]+1,i)+diff(1,3)*G(laInd+laIndk[k]+2,i);
          }
        }
      }
    }
  }

  void Contact::jacobianImpacts() {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        const SqrMat Jprox = ds->getJprox();
        const SqrMat G = ds->getG();

        RowVec jp1=Jprox.row(laInd+laIndk[k]);
        RowVec e1(jp1.size());
        e1(laInd+laIndk[k]) = 1;
        Vec diff = fnil->diff(lak[k](0), gdnk[k](0), gdk[k](0), rFactork[k](0));

        jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laInd+laIndk[k])
        for(int i=0; i<G.size(); i++) 
          jp1(i) -= diff(1)*G(laInd+laIndk[k],i);

        if(getFrictionDirections() == 1) {
          Mat diff = ftil->diff(lak[k](1,1), gdnk[k](1,1), gdk[k](1,1), lak[k](0), rFactork[k](1));
          RowVec jp2=Jprox.row(laInd+laIndk[k]+1);
          RowVec e2(jp2.size());
          e2(laInd+laIndk[k]+1) = 1;
          Mat e(2,jp2.size());
          e(0,laInd+laIndk[k]) = 1;
          e(1,laInd+laIndk[k]+1) = 1;
          jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laInd+laIndk[k])
          //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laInd+laIndk[k])
          for(int i=0; i<G.size(); i++) 
            jp2(i) -= diff(0,1)*G(laInd+laIndk[k]+1,i);

        } 
        else if(getFrictionDirections() == 2) {
          Mat diff = ftil->diff(lak[k](1,2), gdnk[k](1,2), gdk[k](1,2), lak[k](0), rFactork[k](1));
          Mat jp2=Jprox(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+2),Index(0,Jprox.cols()-1));
          Mat e2(2,jp2.cols());
          e2(0,laInd+laIndk[k]+1) = 1;
          e2(1,laInd+laIndk[k]+2) = 1;
          jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laInd+laIndk[k]+1,laInd+laIndk[k]+2),Index(0,G.size()-1))
          for(int i=0; i<G.size(); i++) {
            jp2(0,i) = diff(0,2)*G(laInd+laIndk[k]+1,i)+diff(0,3)*G(laInd+laIndk[k]+2,i);
            jp2(1,i) = diff(1,2)*G(laInd+laIndk[k]+1,i)+diff(1,3)*G(laInd+laIndk[k]+2,i);
          }
        }
      }
    }
  }

  void Contact::updaterFactors() {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k] && gdActive[k][0]) { // TODO

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();

        double sumN = 0;
        for(int j=ia[laInd+laIndk[k]]+1; j<ia[laInd+laIndk[k]+1]; j++)
          sumN += fabs(a[j]);
        const double aN = a[ia[laInd+laIndk[k]]];
        if(aN > sumN) {
          rFactorUnsure(0) = 0;
          rFactork[k](0) = 1.0/aN;
        } 
        else {
          rFactorUnsure(0) = 1;
          rFactork[k](0) = rMax/aN;
        }
        double sumT1 = 0;
        double sumT2 = 0;
        double aT1, aT2;
        if(fdf && gdActive[k][1]) {
          if(getFrictionDirections() == 1) {
            for(int j=ia[laInd+laIndk[k]+1]+1; j<ia[laInd+laIndk[k]+2]; j++)
              sumT1 += fabs(a[j]);
            aT1 = a[ia[laInd+laIndk[k]+1]];
            if(aT1 > sumT1) {
              rFactorUnsure(1)=0;
              rFactork[k](1) = 1.0/aT1;
            }
            else {
              rFactorUnsure(1)=1;
              rFactork[k](1) = rMax/aT1;
            }
          } 
          else if(getFrictionDirections() == 2) {
            for(int j=ia[laInd+laIndk[k]+1]+1; j<ia[laInd+laIndk[k]+2]; j++)
              sumT1 += fabs(a[j]);
            for(int j=ia[laInd+laIndk[k]+2]+1; j<ia[laInd+laIndk[k]+3]; j++)
              sumT2 += fabs(a[j]);
            aT1 = a[ia[laInd+laIndk[k]+1]];
            aT2 = a[ia[laInd+laIndk[k]+2]];

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
      if(gdActive[k][0]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        for(unsigned int i=0; i < 1+ gdActive[k][1]*getFrictionDirections(); i++) {
          gddk[k](i) = b(laInd+laIndk[k]+i);
          for(int j=ia[laInd+laIndk[k]+i]; j<ia[laInd+laIndk[k]+1+i]; j++)
            gddk[k](i) += a[j]*laMBS(ja[j]);
        }

        if(!fcl->isFulfilled(lak[k](0),gddk[k](0),laTol,gddTol)) {
          ds->setTermination(false);
          return;
        }
        if(fdf && gdActive[k][1]) { 
          if(!fdf->isFulfilled(lak[k](1,getFrictionDirections()),gddk[k](1,getFrictionDirections()),lak[k](0),laTol,gddTol)) {
            ds->setTermination(false);
            return;
          }
        }
      }
    }
  }

  void Contact::checkImpactsForTermination(double dt) {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gActive[k]) {

        const double *a = ds->getGs()();
        const int *ia = ds->getGs().Ip();
        const int *ja = ds->getGs().Jp();
        const Vec &laMBS = ds->getla();
        const Vec &b = ds->getb();

        for(int i=0; i < 1+getFrictionDirections(); i++) {
          gdnk[k](i) = b(laInd+laIndk[k]+i);
          for(int j=ia[laInd+laIndk[k]+i]; j<ia[laInd+laIndk[k]+1+i]; j++)
            gdnk[k](i) += a[j]*laMBS(ja[j]);
        }

        if(!fnil->isFulfilled(lak[k](0),gdnk[k](0),gdk[k](0),LaTol,gdTol)) {
          ds->setTermination(false);
          return;
        }
        if(ftil) { 
          if(!ftil->isFulfilled(lak[k](1,getFrictionDirections()),gdnk[k](1,getFrictionDirections()),gdk[k](1,getFrictionDirections()),lak[k](0),LaTol,gdTol)) {
            ds->setTermination(false);
            return;
          }
        }
      }
    }
  }

  void Contact::checkActiveg() { 
    //cout << "checkActiveg" << endl;
    //cout << name << endl;
    //cout << gk[0] << endl;
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) 
      gActive[k] = fcl->isActive(gk[k](0),gTol) ? 1 : 0; 
    //cout << gActive[0] << endl;
  }

  void Contact::checkActivegd() { 
  //  cout << "checkActivegd" << endl;
  //  cout << name << endl;
  //  cout << gdk[0] << endl;
    for(int i=0; i<contactKinematics->getNumberOfPotentialContactPoints(); i++) {
      gdActive[i][0] = (gActive[i] && gdk[i](0) < gdTol) ? 1 : 0;  // TODO
      gdActive[i][1] = getFrictionDirections() && gdActive[i][0] ? (fdf->isSticking(gdk[i](1,getFrictionDirections()),gdTol) ? 1 : 0) : 0; 
    }
  //  cout << gdActive[0][0] << " ";
  //  cout << gdActive[0][1] << endl;
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
      if(getFrictionDirections()) {
	if(gdActive[k][0]) {
	  if(gdActive[k][1]) 
	    if(nrm2(gddk[k](1,getFrictionDirections())) <= gddTol)
	      gdActive[k][1] = true;
	    else
	      gdActive[k][1] = false;
	  else 
	    gdActive[k][1] = false;
	}
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

  // überprüfen, was der Root Finder beobachten soll
  void Contact::checkState() {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(gk[k](0) > gTol) {
	watchg = true;
	watchgd[0] = true;
	watchgd[1] = true;
      } else {
	watchg = false;
	if(gdk[k](0) > gdTol) {
	  watchgd[0] = true;
	  watchgd[1] = true;
	} else {
	  watchgd[0] = false;
	  if(abs(gdk[k](1)) > gdTol)
	    watchgd[1] = true;
	  else
	    watchgd[1] = false;
	}
      }
      //cout << name << " checkState" << endl; 
      //cout << watchg << " " << watchgd[1] << endl; 
    }
  }

  void Contact::updateCondition() {
    for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
      if(jsvk[k](0)) {
	ds->setImpact(true);
      }
      if(jsvk[k](1)) {
      }
      //        if(gActive[k]) {
      //          gActive[k] = false;
      //          gdActive[k][0] = false;
      //          if(getFrictionDirections())
      //            gdActive[k][1] = false;
      //          return;
      //        }
      //        else {// TODO if(gdk[k](0)<=0)  // pseudo collision because of penetration
      //          gActive[k] = true;
      //          gdActive[k][0] = false;
      //          if(getFrictionDirections())
      //            gdActive[k][1] = false;
      //          ds->setImpact(true);
      //          return;
      //        }
      //      }
      //      if(getFrictionDirections()) {
      //        if(jsvk[k](1)) {
      //          if(gdActive[k][1]) {
      //            gdActive[k][1] = false;
      //          } 
      //          else if(nrm2(gdk[k](1,getFrictionDirections()))<gdTol) {
      //            gdActive[k][1] = true;
      //            ds->setSticking(true);
      //          }
      //        }
      //      }
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

void Contact::computeCurvatures(Vec & r) const {
  contactKinematics->computeCurvatures(r, cpData[0]);
}

void Contact::LinearImpactEstimation(Vec &gInActive_,Vec &gdInActive_,int *IndInActive_,Vec &gAct_,int *IndActive_) {
  for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
    if(gActive[k]) {
      gAct_(*IndActive_) = gk[k](0);
      (*IndActive_)++;
    }
    else {
      for(unsigned int i=0; i<2; i++) contour[i]->updateKinematicsForFrame(cpData[k][i],velocities); 
      Vec Wn = cpData[k][0].getFrameOfReference().getOrientation().col(0);
      Vec WvD = cpData[k][1].getFrameOfReference().getVelocity() - cpData[k][0].getFrameOfReference().getVelocity();
      gdInActive_(*IndInActive_) = Wn.T()*WvD;
      gInActive_(*IndInActive_) = gk[k](0);
      (*IndInActive_)++;
    }
  } 

}

void Contact::SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) { 
  for(int k=0; k<contactKinematics->getNumberOfPotentialContactPoints(); k++) {
    if(gActive[k]) (*sizeActive_)++;
    else (*sizeInActive_)++;
  } 
}

void Contact::initializeUsingXML(TiXmlElement *element) {
  LinkMechanics::initializeUsingXML(element);
  TiXmlElement *e;
  e=element->FirstChildElement(MBSIMNS"contactForceLaw");
  GeneralizedForceLaw *gfl=ObjectFactory::getInstance()->createGeneralizedForceLaw(e->FirstChildElement());
  setContactForceLaw(gfl);
  gfl->initializeUsingXML(e->FirstChildElement());
  e=e->NextSiblingElement();
  GeneralizedImpactLaw *gifl=ObjectFactory::getInstance()->createGeneralizedImpactLaw(e->FirstChildElement());
  if(gifl) {
    setContactImpactLaw(gifl);
    gifl->initializeUsingXML(e->FirstChildElement());
    e=e->NextSiblingElement();
  }
  FrictionForceLaw *ffl=ObjectFactory::getInstance()->createFrictionForceLaw(e->FirstChildElement());
  if(ffl) {
    setFrictionForceLaw(ffl);
    ffl->initializeUsingXML(e->FirstChildElement());
    e=e->NextSiblingElement();
  }
  FrictionImpactLaw *fil=ObjectFactory::getInstance()->createFrictionImpactLaw(e->FirstChildElement());
  if(fil) {
    setFrictionImpactLaw(fil);
    fil->initializeUsingXML(e->FirstChildElement());
  }
  e=element->FirstChildElement(MBSIMNS"connect");
  saved_ref1=e->Attribute("ref1");
  saved_ref2=e->Attribute("ref2");
#ifdef HAVE_OPENMBVCPPINTERFACE
  if(element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints"))
    enableOpenMBVContactPoints(getDouble(element->FirstChildElement(MBSIMNS"enableOpenMBVContactPoints")));
  e=element->FirstChildElement(MBSIMNS"openMBVNormalForceArrow");
  if(e) {
    OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
    arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
    setOpenMBVNormalForceArrow(arrow);
    e=e->NextSiblingElement();
  }
  e=element->FirstChildElement(MBSIMNS"openMBVFrictionArrow");
  if(e) {
    OpenMBV::Arrow *arrow=dynamic_cast<OpenMBV::Arrow*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
    arrow->initializeUsingXML(e->FirstChildElement()); // first initialize, because setOpenMBVForceArrow calls the copy constructor on arrow
    setOpenMBVFrictionArrow(arrow);
    e=e->NextSiblingElement();
  }
#endif
}
}

