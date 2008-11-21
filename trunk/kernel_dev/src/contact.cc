/* Copyright (C) 2004-2006  Martin Förg, Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */
#include <config.h> 
#include "contact.h"
#include "object.h"
#include "contour.h"
#include "functions_contact.h"
#include "multi_body_system.h"
#include "nonlinear_algebra.h"
#include "constitutive_laws.h"
#include "class_factory.h"

// --- List of contact kinematic implementations - BEGIN ---
#include "point_line.h"
#include "circlesolid_contour1s.h"
#include "circlesolid_line.h"
#include "circlesolid_plane.h"
#include "point_plane.h"
#include "point_area.h"
#include "edge_edge.h"
#include "circlehollow_cylinderflexible.h"
#include "point_cylinderflexible.h"
#include "sphere_plane.h"
#include "point_contourinterpolation.h"
#include "point_contour1s.h"
#include "line_contour1s.h"
#include "circlesolid_circlehollow.h"
#include "circlesolid_circlesolid.h"
#include "sphere_sphere.h"
#include "sphere_frustum.h"
#include "point_frustum.h"
#include "circlesolid_frustum2d.h"
// --- List of contact kinematic implementations -  END  ---

namespace MBSim {

  double sign(double x) {
    if(x>0)
      return 1.0;
    else if(x<0)
      return -1.0;
    else 
      return 0;
    // return x>=0?1:-1;
  }

  int min(int i, int j) {
    return i<j?i:j;
  }


  Contact::Contact(const string &name) : Link(name), contactKinematics(0), argT(2), fcl(0), fdf(0), fnil(0), ftil(0) {

    gActive = 1;
    gdActive[0] = 1;
    gdActive[1] = 1;

    gActive0 = 1;
    gdActive0[0] = 1;
    gdActive0[1] = 1;
  }

  Contact::~Contact() {
    if (contactKinematics) delete contactKinematics;
  }
  
  void Contact::calcxSize() {
    Link::calcxSize();
    xSize = 0;
  }

  void Contact::calclaSize() {
    Link::calclaSize();
    laSize = gdActive[0]+gdActive[1]*getFrictionDirections();
  }

  void Contact::calcgSize() {
    Link::calcgSize();
    gSize = gActive;
  }

  void Contact::calcgdSize() {
    Link::calcgdSize();
    gdSize = gdActive[0]+gdActive[1]*getFrictionDirections();
  }

  void Contact::calcrFactorSize() {
    Link::calcrFactorSize();
    rFactorSize = gdActive[0]+min(getFrictionDirections(),1)*gdActive[1];
  }

  void Contact::calcsvSize() {
    Link::calcsvSize();
    svSize = 1+getFrictionDirections();
  }

  void Contact::init() {
    Link::init();

    gdd.resize(gdSize);
    gdn.resize(gdSize);

    g.resize(1);
    gd.resize(1+getFrictionDirections());
    la.resize(1+getFrictionDirections());

    if(getFrictionDirections() == 0)
      gdActive[1] = false;

    if(contactKinematics);

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Line*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Line*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointLine; 

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Line*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Line*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleSolidLine;

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Plane*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Plane*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleSolidPlane;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Plane*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Plane*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointPlane;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Area*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Area*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointArea;

    else if(dynamic_cast<Edge*>(contour[0]) && dynamic_cast<Edge*>(contour[1])) 
      contactKinematics = new ContactKinematicsEdgeEdge;

    else if((dynamic_cast<Sphere*>(contour[0]) && dynamic_cast<Plane*>(contour[1])) || (dynamic_cast<Sphere*>(contour[1]) && dynamic_cast<Plane*>(contour[0]))) 
      contactKinematics = new ContactKinematicsSpherePlane;

    // INTERPOLATIONSGESCHICHTEN - Interpol-Point
    else if((dynamic_cast<Point*>(contour[0]) &&  dynamic_cast<ContourInterpolation*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) &&  dynamic_cast<ContourInterpolation*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointContourInterpolation;
    // INTERPOLATIONSGESCHICHTEN

    else if((dynamic_cast<CircleHollow*>(contour[0]) && dynamic_cast<CylinderFlexible*>(contour[1])) || (dynamic_cast<CircleHollow*>(contour[1]) && dynamic_cast<CylinderFlexible*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleHollowCylinderFlexible;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<CylinderFlexible*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<CylinderFlexible*>(contour[0])))
      contactKinematics = new ContactKinematicsPointCylinderFlexible;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Contour1s*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Contour1s*>(contour[0]))) 
      contactKinematics = new ContactKinematicsPointContour1s;

    else if((dynamic_cast<Line*>(contour[0]) && dynamic_cast<Contour1s*>(contour[1])) || (dynamic_cast<Line*>(contour[1]) && dynamic_cast<Contour1s*>(contour[0]))) 
      contactKinematics = new ContactKinematicsLineContour1s;

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Contour1s*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Contour1s*>(contour[0])))
      contactKinematics = new ContactKinematicsCircleSolidContour1s;


    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<CircleHollow*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<CircleHollow*>(contour[0])))
      contactKinematics = new ContactKinematicsCircleSolidCircleHollow;

    else if(dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<CircleSolid*>(contour[1]))
      contactKinematics = new ContactKinematicsCircleSolidCircleSolid;

    else if(dynamic_cast<Sphere*>(contour[0]) && dynamic_cast<Sphere*>(contour[1]))
      contactKinematics = new ContactKinematicsSphereSphere;

    else if((dynamic_cast<Sphere*>(contour[0]) && dynamic_cast<Frustum*>(contour[1])) || (dynamic_cast<Sphere*>(contour[1]) && dynamic_cast<Frustum*>(contour[0])))
      contactKinematics = new ContactKinematicsSphereFrustum;

    else if((dynamic_cast<CircleSolid*>(contour[0]) && dynamic_cast<Frustum2D*>(contour[1])) || (dynamic_cast<CircleSolid*>(contour[1]) && dynamic_cast<Frustum2D*>(contour[0]))) 
      contactKinematics = new ContactKinematicsCircleSolidFrustum2D;

    else if((dynamic_cast<Point*>(contour[0]) && dynamic_cast<Frustum*>(contour[1])) || (dynamic_cast<Point*>(contour[1]) && dynamic_cast<Frustum*>(contour[0])))
      contactKinematics = new ContactKinematicsPointFrustum;  

    else {
      cout << "Unkown contact pairing" <<endl;
      throw 5;
    }

    //cpData.push_back(new ContourPointData);
    //cpData.push_back(new ContourPointData);
    cpData[0].type = CONTINUUM; // default-Wert
    cpData[0].Wn.resize(3,1);
    cpData[0].Wt.resize(3,getFrictionDirections());
    cpData[1].type = CONTINUUM; // default-Wert
    cpData[1].Wn.resize(3,1);
    cpData[1].Wt.resize(3,getFrictionDirections());
    
    cpData[0].cosy.setName("0");
    cpData[1].cosy.setName("1");
    cpData[0].cosy.getJacobianOfTranslation().resize(3,contour[0]->getWJP().cols());
    cpData[0].cosy.getJacobianOfRotation().resize(3,contour[0]->getWJR().cols());
    cpData[1].cosy.getJacobianOfTranslation().resize(3,contour[1]->getWJP().cols());
    cpData[1].cosy.getJacobianOfRotation().resize(3,contour[1]->getWJR().cols());

    //connectHitSpheres(contour[0],contour[1]);

    iT = Index(1,getFrictionDirections());
    contactKinematics->assignContours(contour);
  }

  int Contact::getFrictionDirections() {
    if(fdf) 
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void Contact::connect(Contour *contour0, Contour* contour1) {
    Link::connect(contour0,0);
    Link::connect(contour1,1);
  }

  void Contact::updateWRef(const Mat& WParent) {
   for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(),contour[i]->gethInd()+contour[i]->gethSize()-1);
      W[i].resize()>>WParent(I,J);
    }
  } 

  void Contact::updateVRef(const Mat& VParent) {
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(),contour[i]->gethInd()+contour[i]->getWJP().cols()-1);
      V[i].resize()>>VParent(I,J);
    }
  } 

  void Contact::updateh(double t) {
 //   if(isActive()) {
      la(0) = (*fcl)(g(0),gd(0));
      if(fdf)
	la(1,getFrictionDirections()) = (*fdf)(gd(1,getFrictionDirections()),fabs(la(0)));

      WF[1] =  cpData[1].cosy.getOrientation().col(1)*la(0);
      if(getFrictionDirections()) {
	WF[1] += cpData[1].cosy.getOrientation().col(0)*la(1);
	if(getFrictionDirections() > 1)
	  WF[1] += cpData[1].cosy.getOrientation().col(2)*la(2);
      }
      WF[0] = -WF[1];
      for(unsigned int i=0; i<contour.size(); i++)
	h[i] += trans(cpData[i].cosy.getJacobianOfTranslation())*WF[i];
  //    cout << t << endl;
  //    cout << g << endl;
  //    cout << name << endl;
  //    cout << la << endl;
  //  }
  }

  void Contact::updater(double t) {

    for(unsigned i=0; i<contour.size(); i++) 
      r[i] += V[i]*la;
  }


  void Contact::updateg(double t) {
    contactKinematics->updateg(g,cpData);
    for(unsigned int i=0; i<2; i++) {
      Vec WrPC = cpData[i].cosy.getPosition() - contour[i]->getCoordinateSystem()->getPosition();

      cpData[i].cosy.setAngularVelocity(contour[i]->getCoordinateSystem()->getAngularVelocity());
      cpData[i].cosy.setVelocity(contour[i]->getCoordinateSystem()->getVelocity() + crossProduct(contour[i]->getCoordinateSystem()->getAngularVelocity(),WrPC));

      Mat tWrPC = tilde(WrPC);
      cpData[i].cosy.setJacobianOfTranslation(contour[i]->getCoordinateSystem()->getJacobianOfTranslation() - tWrPC*contour[i]->getCoordinateSystem()->getJacobianOfRotation());
      cpData[i].cosy.setJacobianOfRotation(contour[i]->getCoordinateSystem()->getJacobianOfRotation());
      cpData[i].cosy.setGyroscopicAccelerationOfTranslation(contour[i]->getCoordinateSystem()->getGyroscopicAccelerationOfTranslation() - tWrPC*contour[i]->getCoordinateSystem()->getGyroscopicAccelerationOfRotation() + crossProduct(contour[i]->getCoordinateSystem()->getAngularVelocity(),crossProduct(contour[i]->getCoordinateSystem()->getAngularVelocity(),WrPC)));
      cpData[i].cosy.setGyroscopicAccelerationOfRotation(contour[i]->getCoordinateSystem()->getGyroscopicAccelerationOfRotation());
    }

  }

  void Contact::updategd(double t) {
  
    Vec Wn = cpData[1].cosy.getOrientation().col(1);

    Vec WvD = cpData[1].cosy.getVelocity() - cpData[0].cosy.getVelocity();

    gd(0) = trans(Wn)*WvD;

    if(gd.size()>1) {
      Mat Wt(3,gd.size()-1);
      Wt.col(0) = cpData[1].cosy.getOrientation().col(0);
      if(gd.size() > 2)
	Wt.col(1) = cpData[1].cosy.getOrientation().col(2);

      gd(1,gd.size()-1) = trans(Wt)*WvD;
    }

   }

  bool Contact::gActiveChanged() {
    bool changed = gActive0 != gActive;
    gActive0 = gActive;
    return changed;
  }


  void Contact::checkActiveg() { 
    gActive = fcl->isActive(g(0),0) ? 1 : 0; 
  }

  void Contact::checkActivegd() { 
    gdActive[0] = gActive ? (fcl->remainsActive(gd(0),gdTol) ? 1 : 0) : 0; 
    gdActive[1] = getFrictionDirections() && gdActive[0] ? (fdf->isSticking(gd(1,getFrictionDirections()),gdTol) ? 1 : 0) : 0; 
  }

  void Contact::checkAllgd() { 
    gdActive[0] = gActive ? 1 : 0; 
    gdActive[1] = getFrictionDirections() && gActive ? 1 : 0; 
  }

  bool Contact::isSetValued() const {
    bool flag = fcl->isSetValued();
    if(fdf) 
      flag |= fdf->isSetValued();
    return flag;
  }

  void Contact::save(const string& path, ofstream &outputfile) {
    Link::save(path, outputfile);

    fcl->save(path,outputfile);
    if(fdf)
      fdf->save(path,outputfile);
    else {
      outputfile << "# Type of friction force law:" << endl << endl;
    }

    if(fnil)
      fnil->save(path,outputfile);
    else
      outputfile << "# Type of contact impact law:" << endl << endl;

    if(ftil)
      ftil->save(path,outputfile);
    else {
      outputfile << "# Type of friction impact law:" << endl << endl;
    }
  }

  void Contact::load(const string& path, ifstream &inputfile) {
    Link::load(path,inputfile);
    string dummy;
    int s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of contact law:
    getline(inputfile,dummy); // Type of contact law 
    inputfile.seekg(s,ios::beg);
    ClassFactory cf;
    setContactForceLaw(cf.getGeneralizedForceLaw(dummy));
    fcl->load(path, inputfile);

    s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of friction law:
    getline(inputfile,dummy); // Type of friction law 
    inputfile.seekg(s,ios::beg);
    if(dummy.empty()) {
      getline(inputfile,dummy); // # Type of friction law
      getline(inputfile,dummy); // End of line
    } else {
      setFrictionForceLaw(cf.getFrictionForceLaw(dummy));
      fdf->load(path, inputfile);
    }

    s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of normal impact law:
    cout << dummy << endl;
    getline(inputfile,dummy); // Type of normal impact law 
    cout << dummy << endl;
    inputfile.seekg(s,ios::beg);
    setContactImpactLaw(cf.getGeneralizedImpactLaw(dummy));
    fnil->load(path, inputfile);

    s = inputfile.tellg();
    getline(inputfile,dummy); // # Type of tangential impact law:
    getline(inputfile,dummy); // Type of tangential impact law 
    inputfile.seekg(s,ios::beg);
    if(dummy.empty()) {
      getline(inputfile,dummy); // # Type of friction law
      getline(inputfile,dummy); // End of line
    } else {
      setFrictionImpactLaw(cf.getFrictionImpactLaw(dummy));
      ftil->load(path, inputfile);
    }
  }


  void Contact::updatewb(double t) {

    for(unsigned i=0; i<contour.size(); i++) 
      wb += trans(fF[i](Index(0,2),Index(0,laSize-1)))*cpData[i].cosy.getGyroscopicAccelerationOfTranslation();

    contactKinematics->updatewb(wb,g,cpData);
  }

  void Contact::updateW(double t) {
    
    fF[1].col(0) = cpData[1].cosy.getOrientation().col(1);
    if(getFrictionDirections()) {
      fF[1].col(1) = cpData[1].cosy.getOrientation().col(0);
      if(getFrictionDirections() > 1)
	fF[1].col(2) = cpData[1].cosy.getOrientation().col(2);
    }

    fF[0] = -fF[1];

    for(unsigned int i=0; i<contour.size(); i++) 
      W[i] += trans(cpData[i].cosy.getJacobianOfTranslation())*fF[i](Index(0,2),Index(0,laSize-1));
  }

  void Contact::updateV(double t) {

    if(getFrictionDirections() && !gdActive[1]) 
      for(unsigned int i=0; i<contour.size(); i++) 
	V[i] += trans(cpData[i].cosy.getJacobianOfTranslation())*fF[i](Index(0,2),iT)*fdf->dlaTdlaN(gd(1,getFrictionDirections()), la(0));
  }

  void Contact::checkActivegdn() { 
   // cout << name << endl;
   // cout <<"gdn = "<< gdn << endl;
   // cout <<"la = "<< la << endl;
    if(gActive) {
      if(gdn(0) <= gdTol) {
	gActive = true;
	gdActive[0] = true;
      } 
      else {
	gActive = false;
	gdActive[0] = false;
      }
    }
  //  else { // Ist wahrscheinlich unnötig
  //    gActive = false;
  //    gdActive[0] = false;
  //  }
    if(getFrictionDirections())
      if(gdActive[0])
	if(nrm2(gdn(1,getFrictionDirections())) <= gdTol)
	  gdActive[1] = true;
	else
	  gdActive[1] = false;
      else
	  gdActive[1] = false;
  }

  void Contact::checkActivegdd() { 
    //cout << name << endl;
    //cout <<"gdd = "<< gdd << endl;
    //cout <<"la = "<< la << endl;
    if(gdActive[0]) {
      if(gdd(0) <= gddTol) {
	gActive = true;
	gdActive[0] = true;
      }
      else {
	gActive = false;
	gdActive[0] = false;
      }
    }
    if(getFrictionDirections())
      if(gdActive[0]) 
	if(gdActive[1]) 
	  if(nrm2(gdd(1,getFrictionDirections())) <= gddTol)
	    gdActive[1] = true;
	  else
	    gdActive[1] = false;
	else 
	  gdActive[1] = false;
  }

  void Contact::updateCondition() {

    //cout<<"before " << "gA = " << gActive << " gd0 = " << gdActive[0] << " gd1 = " << gdActive[1] << endl;
    if(jsv(0)) {
      if(gActive) {
	gActive = false;
	gdActive[0] = false;
	if(getFrictionDirections())
	  gdActive[1] = false;
	return;
      }
      else {// if(gd(0)<=0) { // evtl. zur Abfrage zur Vermeidung von Schein-Kollisionen wegen Eindringen
	gActive = true;
	gdActive[0] = true;
	if(getFrictionDirections())
	  gdActive[1] = true;
	mbs->setImpact(true);
	return;
      }
    }
    if(getFrictionDirections())
      if(jsv(1)) {
	if(gdActive[1]) {
	  gdActive[1] = false;
	} 
	else {
	  gdActive[1] = true;
	  mbs->setSticking(true);
	}
      }
  //  cout<<"after " << "gA = " << gActive << " gd0 = " << gdActive[0] << " gd1 = " << gdActive[1] << endl;
  }

  void Contact::updateStopVector(double t) {
    if(gActive) {
      sv(0) = la(0);
      if(gdActive[1]) {
	sv(1) = nrm2(la(1,getFrictionDirections())) - fdf->getFrictionCoefficient(nrm2(gd(1,getFrictionDirections())))*la(0);
      } 
      else
	sv(1,getFrictionDirections()) = gd(1,getFrictionDirections());
    }
    else {
      sv(0) = g(0);
      sv(1,getFrictionDirections()).init(1);
    }
  }

  void Contact::solveConstraintsFixpointSingle() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdd(0) = b(laIndMBS);
    for(int j=ia[laIndMBS]; j<ia[laIndMBS+1]; j++)
      gdd(0) += a[j]*laMBS(ja[j]);

    la(0) = fcl->project(la(0), gdd(0), rFactor(0));

    for(int i=1; i<=getFrictionDirections(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);
    }

    if(fdf)
      la(1,getFrictionDirections()) = fdf->project(la(1,getFrictionDirections()), gdd(1,getFrictionDirections()), la(0), rFactor(1));
  } 
  
  void Contact::solveImpactsFixpointSingle() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdn(0) = b(laIndMBS);
    for(int j=ia[laIndMBS]; j<ia[laIndMBS+1]; j++)
      gdn(0) += a[j]*laMBS(ja[j]);

    la(0) = fnil->project(la(0), gdn(0), gd(0), rFactor(0));

    for(int i=1; i<=getFrictionDirections(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    if(ftil)
      la(1,getFrictionDirections()) = ftil->project(la(1,getFrictionDirections()), gdn(1,getFrictionDirections()), gd(1,getFrictionDirections()), la(0), rFactor(1));
  }


  void Contact::solveConstraintsGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    int laInd = laIndMBS;
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdd(0) = b(laIndMBS);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdd(0) += a[j]*laMBS(ja[j]);

    double om = 1.0;
    double buf = fcl->solve(a[ia[laInd]], gdd(0));
    la(0) += om*(buf - la(0));

    if(getFrictionDirections() && gdActive[1]) {
      gdd(1) = b(laIndMBS+1);
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	gdd(1) += a[j]*laMBS(ja[j]);

      if(fdf) {
	Vec buf = fdf->solve(mbs->getG()(Index(laInd+1,laInd+getFrictionDirections())), gdd(1,getFrictionDirections()), la(0));
	la(1,getFrictionDirections()) += om*(buf - la(1,getFrictionDirections()));
      }
    }
  }

  void Contact::solveImpactsGaussSeidel() {
    assert(getFrictionDirections() <= 1);

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    int laInd = laIndMBS;
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    gdn(0) = b(laIndMBS);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*laMBS(ja[j]);

    double om = 1.0;
    double buf = fnil->solve(a[ia[laInd]], gdn(0), gd(0));
    la(0) += om*(buf - la(0));

    if(getFrictionDirections()) {
      gdn(1) = b(laIndMBS+1);
      for(int j=ia[laInd+1]+1; j<ia[laInd+2]; j++)
	gdn(1) += a[j]*laMBS(ja[j]);

      if(ftil) {
	Vec buf = ftil->solve(mbs->getG()(Index(laInd+1,laInd+getFrictionDirections())), gdn(1,getFrictionDirections()), gd(1,getFrictionDirections()), la(0));
	la(1,getFrictionDirections()) += om*(buf - la(1,getFrictionDirections()));
      }
    }
  }

  void Contact::solveConstraintsRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < 1+getFrictionDirections(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);
    }

    res(0) = la(0) - fcl->project(la(0), gdd(0), rFactor(0));
    if(fdf) 
      res(1,getFrictionDirections()) = la(1,getFrictionDirections()) - fdf->project(la(1,getFrictionDirections()), gdd(1,getFrictionDirections()), la(0), rFactor(1));
  }

  void Contact::solveImpactsRootFinding() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < 1+getFrictionDirections(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    res(0) = la(0) - fnil->project(la(0), gdn(0), gd(0), rFactor(0));
    if(ftil) 
      res(1,getFrictionDirections()) = la(1,getFrictionDirections()) - ftil->project(la(1,getFrictionDirections()), gdn(1,getFrictionDirections()), gd(1,getFrictionDirections()), la(0), rFactor(1));
  }

  void Contact::jacobianConstraints() {

    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    RowVec jp1=Jprox.row(laIndMBS);
    RowVec e1(jp1.size());
    e1(laIndMBS) = 1;
    Vec diff = fcl->diff(la(0), gdd(0), rFactor(0));

    jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS)
    for(int i=0; i<G.size(); i++) 
      jp1(i) -= diff(1)*G(laIndMBS,i);

    if(getFrictionDirections() == 1) {
      Mat diff = fdf->diff(la(1,1), gdd(1,1), la(0), rFactor(1));
      RowVec jp2=Jprox.row(laIndMBS+1);
      RowVec e2(jp2.size());
      e2(laIndMBS+1) = 1;
      Mat e(2,jp2.size());
      e(0,laIndMBS) = 1;
      e(1,laIndMBS+1) = 1;
      jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laIndMBS)
      //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laIndMBS)
      for(int i=0; i<G.size(); i++) 
	jp2(i) -= diff(0,1)*G(laIndMBS+1,i);

    } else if(getFrictionDirections() == 2) {
      Mat diff = ftil->diff(la(1,2), gdd(1,2), gd(1,2), la(0), rFactor(1));
      Mat jp2=Jprox(Index(laIndMBS+1,laIndMBS+2),Index(0,Jprox.cols()));
      Mat e2(2,jp2.cols());
      e2(0,laIndMBS+1) = 1;
      e2(1,laIndMBS+2) = 1;
      jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laIndMBS+1,laIndMBS+2),Index(0,G.size()-1))
      for(int i=0; i<G.size(); i++) {
	jp2(0,i) = diff(0,2)*G(laIndMBS+1,i)+diff(0,3)*G(laIndMBS+2,i);
	jp2(1,i) = diff(1,2)*G(laIndMBS+1,i)+diff(1,3)*G(laIndMBS+2,i);
      }
    }
  }

  void Contact::jacobianImpacts() {

    SqrMat Jprox = mbs->getJprox();
    SqrMat G = mbs->getG();

    RowVec jp1=Jprox.row(laIndMBS);
    RowVec e1(jp1.size());
    e1(laIndMBS) = 1;
    Vec diff = fnil->diff(la(0), gdn(0), gd(0), rFactor(0));

    jp1 = e1-diff(0)*e1; // -diff(1)*G.row(laIndMBS)
    for(int i=0; i<G.size(); i++) 
      jp1(i) -= diff(1)*G(laIndMBS,i);

    if(getFrictionDirections() == 1) {
      Mat diff = ftil->diff(la(1,1), gdn(1,1), gd(1,1), la(0), rFactor(1));
      RowVec jp2=Jprox.row(laIndMBS+1);
      RowVec e2(jp2.size());
      e2(laIndMBS+1) = 1;
      Mat e(2,jp2.size());
      e(0,laIndMBS) = 1;
      e(1,laIndMBS+1) = 1;
      jp2 = e2-diff(0,2)*e1-diff(0,0)*e2; // -diff(1)*G.row(laIndMBS)
      //jp2 = e2-diff.row(0)(0,1)*e; // -diff(1)*G.row(laIndMBS)
      for(int i=0; i<G.size(); i++) 
	jp2(i) -= diff(0,1)*G(laIndMBS+1,i);

    } else if(getFrictionDirections() == 2) {
      Mat diff = ftil->diff(la(1,2), gdn(1,2), gd(1,2), la(0), rFactor(1));
      Mat jp2=Jprox(Index(laIndMBS+1,laIndMBS+2),Index(0,Jprox.cols()));
      Mat e2(2,jp2.cols());
      e2(0,laIndMBS+1) = 1;
      e2(1,laIndMBS+2) = 1;
      jp2 = e2-diff(Index(0,1),Index(4,4))*e1-diff(Index(0,1),Index(0,1))*e2; // -diff(Index(0,1),Index(4,5))*G(Index(laIndMBS+1,laIndMBS+2),Index(0,G.size()-1))
      for(int i=0; i<G.size(); i++) {
	jp2(0,i) = diff(0,2)*G(laIndMBS+1,i)+diff(0,3)*G(laIndMBS+2,i);
	jp2(1,i) = diff(1,2)*G(laIndMBS+1,i)+diff(1,3)*G(laIndMBS+2,i);
      }
    }
  }

  void Contact::checkConstraintsForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(unsigned int i=0; i < 1+ gdActive[1]*getFrictionDirections(); i++) {
      gdd(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdd(i) += a[j]*laMBS(ja[j]);
    }

    if(!fcl->isFullfield(la(0),gdd(0),laTol,gddTol)) {
      mbs->setTermination(false);
      return;
    }
    if(fdf && gdActive[1]) 
      if(!fdf->isFullfield(la(1,getFrictionDirections()),gdd(1,getFrictionDirections()),la(0),laTol,gddTol)) {
	mbs->setTermination(false);
	return;
      }
  }

  void Contact::checkImpactsForTermination() {

    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    int *ja = mbs->getGs().Jp();
    Vec &laMBS = mbs->getla();
    Vec &b = mbs->getb();

    for(int i=0; i < 1+getFrictionDirections(); i++) {
      gdn(i) = b(laIndMBS+i);
      for(int j=ia[laIndMBS+i]; j<ia[laIndMBS+1+i]; j++)
	gdn(i) += a[j]*laMBS(ja[j]);
    }

    if(!fnil->isFullfield(la(0),gdn(0),gd(0),LaTol,gdTol)) {
      mbs->setTermination(false);
      return;
    }
    if(ftil) 
      if(!ftil->isFullfield(la(1,getFrictionDirections()),gdn(1,getFrictionDirections()),gd(1,getFrictionDirections()),la(0),LaTol,gdTol)) {
	mbs->setTermination(false);
	return;
      }
  }

  void Contact::updaterFactors() {
    double *a = mbs->getGs()();
    int *ia = mbs->getGs().Ip();
    double sumN = 0;

    for(int j=ia[laIndMBS]+1; j<ia[laIndMBS+1]; j++)
      sumN += fabs(a[j]);
    double aN = a[ia[laIndMBS]];
    if(aN > sumN) {
      rFactorUnsure(0) = 0;
      rFactor(0) = 1.0/aN;
    } else {
      rFactorUnsure(0) = 1;
      rFactor(0) = rMax/aN;
    }
    double sumT1 = 0;
    double sumT2 = 0;
    double aT1, aT2;
    if(getFrictionDirections() == 1) {
      for(int j=ia[laIndMBS+1]+1; j<ia[laIndMBS+2]; j++)
	sumT1 += fabs(a[j]);
      aT1 = a[ia[laIndMBS+1]];
      if(aT1 > sumT1) {
	rFactorUnsure(1)=0;
	rFactor(1) = 1.0/aT1;
      } else {
	rFactorUnsure(1)=1;
	rFactor(1) = rMax/aT1;
      }
    } else if(getFrictionDirections() == 2) {
      for(int j=ia[laIndMBS+1]+1; j<ia[laIndMBS+2]; j++)
	sumT1 += fabs(a[j]);
      for(int j=ia[laIndMBS+2]+1; j<ia[laIndMBS+3]; j++)
	sumT2 += fabs(a[j]);
      aT1 = a[ia[laIndMBS+1]];
      aT2 = a[ia[laIndMBS+2]];

      // TODO rFactorUnsure
      if(aT1 - sumT1 >= aT2 - sumT2) 
	if(aT1 + sumT1 >= aT2 + sumT2) 
	  rFactor(1) = 2.0/(aT1+aT2+sumT1-sumT2);
	else 
	  rFactor(1) = 1.0/aT2;
      else 
	if(aT1 + sumT1 < aT2 + sumT2) 
	  rFactor(1) = 2.0/(aT1+aT2-sumT1+sumT2);
	else 
	  rFactor(1) = 1.0/aT1;
    }
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// ZUR NUMERISCHEN BESTIMMUNG DER GENERALISIERTEN KRAFTRICHTUNGEN WN
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//       if (false) {
// // Numerische Ableitung gp(0) -> WN    
//     static bool rootCall = true;
//     static double eps = 5.e-9;

//     if(rootCall) {
// 	rootCall = false;

// 	int WNSize = cylinder->getObject()->getqSize();
// 	Mat WN(WNSize,1);
// 	const double gAlt = g(0);

// 	for(int i=0;i<WNSize;i++) {
// 	    Vec qTemp = cylinder->getObject()->getq();
// 	    qTemp(i) += eps;
// 	    cylinder->getObject()->setq(qTemp);
// 	    pairingCircleHollowCylinderFlexibleStage1(icircle,icylinder);
// 	    WN(i,0) = (g(0) - gAlt) / eps ;
// 	    qTemp(i) -= eps;
// 	    cylinder->getObject()->setq(qTemp);
// 	}

// 	// reset
// 	pairingCircleHollowCylinderFlexibleStage2(icircle,icylinder);

// // 	cout << "numerisches WN = " << trans(WN) << endl;

// 	rootCall = true;
//     }

//       }
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

