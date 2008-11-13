/* Copyright (C) 2004-2006  Martin Förg

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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include "joint.h"
#include "coordinate_system.h"
#include "data_interface_base.h"
#include "constitutive_laws.h"
#include "multi_body_system.h"

#ifdef HAVE_AMVIS
#include "coilspring.h"
using namespace AMVis;
#endif

namespace MBSim {

  Joint::Joint(const string &name) : Link(name), ffl(0), fml(0), fifl(0), fiml(0), coilspringAMVis(0), coilspringAMVisUserFunctionColor(0) {
  }

  Joint::~Joint() { 
#ifdef HAVE_AMVIS   
    delete coilspringAMVis;
    delete coilspringAMVisUserFunctionColor;
#endif
  }
  void Joint::calcxSize() {
    Link::calcxSize();
    xSize = momentDir.cols();
  }

  void Joint::calcgSize() {
    Link::calcgSize();
    gSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calcgdSize() {
    Link::calcgdSize();
    gdSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calclaSize() {
    Link::calclaSize();
    laSize = forceDir.cols()+momentDir.cols();
  }

  void Joint::calcrFactorSize() {
    Link::calcrFactorSize();
    rFactorSize = setValued ? forceDir.cols()+momentDir.cols() : 0;
  }

  void Joint::init() {
    Link::init();

    g.resize(forceDir.cols()+momentDir.cols());
    gd.resize(forceDir.cols()+momentDir.cols());
    la.resize(forceDir.cols()+momentDir.cols());
    gdd.resize(gdSize);
    gdn.resize(gdSize);

    IT = Index(0,forceDir.cols()-1);
    IR = Index(forceDir.cols(),forceDir.cols()+momentDir.cols()-1);
    if(forceDir.cols()) 
      Wf = forceDir;
    else {
      forceDir.resize(3,0);
      Wf.resize(3,0);
    }
    if(momentDir.cols())
      Wm = momentDir;
    else {
      momentDir.resize(3,0);
      Wm.resize(3,0);
    }
    //fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
    //fF[1](Index(0,2),Index(0,Wf.cols()-1)) = Wf;
    //fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
    //fM[1](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = Wm;
  }

  bool Joint::isSetValued() const {
    bool flag = false;
    if(ffl) 
      flag |= ffl->isSetValued();
    if(fml) 
      flag |= fml->isSetValued();
    cout << name << endl;
    cout << ffl->isSetValued() << endl;
    return flag;
  }

  void Joint::connect(CoordinateSystem *port0, CoordinateSystem* port1) {
    Link::connect(port0,0);
    Link::connect(port1,1);
  }

  void Joint::updateg(double t) {
    Wf = port[0]->getOrientation()*forceDir;
    Wm = port[0]->getOrientation()*momentDir;
    WrP0P1 = port[1]->getPosition()-port[0]->getPosition();
    g(IT) = trans(Wf)*WrP0P1;
    g(IR) = x;
  }

  void Joint::updategd(double t) {
    WvP0P1 = port[1]->getVelocity()-port[0]->getVelocity();
    WomP0P1 = port[1]->getAngularVelocity()-port[0]->getAngularVelocity();
    gd(IT) = trans(Wf)*(WvP0P1 - crossProduct(port[0]->getAngularVelocity(), WrP0P1));
    gd(IR) = trans(Wm)*WomP0P1;
  }

  void Joint::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.col(i) = forceDir.col(i)/nrm2(fd.col(i));
  }

  void Joint::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.col(i) = momentDir.col(i)/nrm2(md.col(i));
  }

  void Joint::updatexd(double t) {
    xd = gd(IR);
  }

  void Joint::updatedx(double t, double dt) {
    xd = gd(IR)*dt;
  }

  void Joint::initPlotFiles() {

    Link::initPlotFiles();

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      coilspringAMVis->writeBodyFile();
    }
#endif
  }

  void Joint::plot(double t,double dt) {
    Link::plot(t,dt);

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      Vec WrOToPoint;
      Vec WrOFromPoint;

      WrOFromPoint = port[0]->getPosition();
      WrOToPoint   = port[1]->getPosition();
      if (coilspringAMVisUserFunctionColor) {
	double color;
	color = ((*coilspringAMVisUserFunctionColor)(t))(0);
	if (color>1) color=1;
	if (color<0) color=0;
	coilspringAMVis->setColor(color);
      } 
      coilspringAMVis->setTime(t); 
      coilspringAMVis->setFromPoint(WrOFromPoint(0), WrOFromPoint(1), WrOFromPoint(2));
      coilspringAMVis->setToPoint(WrOToPoint(0), WrOToPoint(1), WrOToPoint(2));
      coilspringAMVis->appendDataset(0);
    }
  }
#endif

   void Joint::updateW(double t) {
    fF[0](Index(0,2),Index(0,Wf.cols()-1)) = -Wf;
    fM[0](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = -Wm;
    fM[0](Index(0,2),Index(0,Wf.cols()-1)) = -tilde(WrP0P1)*Wf;
    fF[1](Index(0,2),Index(0,Wf.cols()-1)) = Wf;
    fM[1](Index(0,2),Index(Wf.cols(),Wf.cols()+Wm.cols()-1)) = Wm;

    for(unsigned int i=0; i<port.size(); i++) 
      W[i] += trans(port[i]->getJacobianOfTranslation())*fF[i] + trans(port[i]->getJacobianOfRotation())*fM[i];
  }

  void Joint::updatewb(double t) {

    for(unsigned i=0; i<port.size(); i++) 
      wb += trans(fF[i])*port[i]->getGyroscopicAccelerationOfTranslation() + trans(fM[i])*port[i]->getGyroscopicAccelerationOfRotation();

    wb(0,Wf.cols()-1) += trans(Wf)*(crossProduct(port[0]->getAngularVelocity(),crossProduct(port[0]->getAngularVelocity(),WrP0P1)) - 2*crossProduct(port[0]->getAngularVelocity(),WvP0P1));
    wb(Wf.cols(),Wm.cols()+Wf.cols()-1) -=  trans(Wm)*crossProduct(port[0]->getAngularVelocity(),WomP0P1);
  }

  void Joint::solveConstraintsFixpointSingle() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = ffl->project(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = fml->project(la(i), gdd(i), rFactor(i));
    }
  }

  void Joint::solveImpactsFixpointSingle() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::solveConstraintsGaussSeidel() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = ffl->solve(a[ia[laIndMBS+i]], gdd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      la(i) = fml->solve(a[ia[laIndMBS+i]], gdd(i));
    }
  }

  void Joint::solveImpactsGaussSeidel() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fifl->solve(a[ia[laIndMBS+i]], gdn(i), gd(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      la(i) = fiml->solve(a[ia[laIndMBS+i]], gdn(i), gd(i));
    }
  }

  void Joint::solveConstraintsRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - ffl->project(la(i), gdd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fml->project(la(i), gdd(i), rFactor(i));
    }
  }
  void Joint::solveImpactsRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();
  
    for(int i=0; i<forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fifl->project(la(i), gdn(i), gd(i), rFactor(i));
    }
    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      res(i) = la(i) - fiml->project(la(i), gdn(i), gd(i), rFactor(i));
    }
  }

  void Joint::jacobianConstraints() {
    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = ffl->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = fml->diff(la(i), gdd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }
  }

  void Joint::jacobianImpacts() {
    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    for(int i=0; i<forceDir.cols(); i++) {
      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = fifl->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++) {

      RowVec jp1=Jprox.row(laIndMBS+i);
      RowVec e1(jp1.size());
      e1(laIndMBS+i) = 1;
      Vec diff = fiml->diff(la(i), gdn(i), gd(i), rFactor(i));

      jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS+i)
      for(int j=0; j<G.size(); j++) 
	jp1(j) -= diff(1)*G(laIndMBS+i,j);
    }
  }

  void Joint::updaterFactors() {
    if(isActive()) {
      double *a = mbs->getGs()();
      int *ia = mbs->getGs().Ip();

      for(int i=0; i<rFactorSize; i++) {
	double sum = 0;
	for(int j=ia[laIndMBS+i]+1; j<ia[laIndMBS+i+1]; j++)
	  sum += fabs(a[j]);
	double ai = a[ia[laIndMBS+i]];
	if(ai > sum) {
	  rFactorUnsure(i) = 0;
	  rFactor(i) = 1.0/ai;
	} else {
	  rFactorUnsure(i) = 1;
	  rFactor(i) = 1.0/ai;
	}
      }
    }
  }

  void Joint::checkConstraintsForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < forceDir.cols(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      if(!ffl->isFullfield(la(i),gdd(i),laTol,gddTol)) {
	mbs->setTermination(false);
	return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdd(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);

      if(!fml->isFullfield(la(i),gdd(i),laTol,gddTol)) {
	mbs->setTermination(false);
	return;
      }
    }
  }

  void Joint::checkImpactsForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < forceDir.cols(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      if(!fifl->isFullfield(la(i),gdn(i),gd(i),LaTol,gdTol)) {
	mbs->setTermination(false);
	return;
      }
    }
    for(int i=forceDir.cols(); i < forceDir.cols() + momentDir.cols(); i++) {
      gdn(i) = b(i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);

      if(!fiml->isFullfield(la(i),gdn(i),gd(i),LaTol,gdTol)) {
	mbs->setTermination(false);
	return;
      }
    }
  }

  void Joint::updateh(double t) {
    for(int i=0; i<forceDir.cols(); i++) 
      la(i) = (*ffl)(g(i), gd(i));

    for(int i=forceDir.cols(); i<forceDir.cols() + momentDir.cols(); i++)
      la(i) = (*fml)(g(i), gd(i));

    WF[1] = Wf*la(IT);
    WM[1] = Wm*la(IR);
    WF[0] = -WF[1];
    WM[0] = -WM[1]+crossProduct(WrP0P1,WF[0]);
    for(unsigned int i=0; i<port.size(); i++)
      h[i] += trans(port[i]->getJacobianOfTranslation())*WF[i] + trans(port[i]->getJacobianOfRotation())*WM[i];
  }

  //double Joint::computePotentialEnergy() {
  //  return 0.5*trans(la)*g;
  //}

}
