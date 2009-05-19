/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#include <config.h>
#include "mbsim/contour.h"
#include "mbsim/object.h"
#include "mbsim/utils/rotarymatrices.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <rigid_body.h>
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/sphere.h>
#endif

using namespace fmatvec;
using namespace std;

namespace MBSim {

  /* Contour */
  Contour::Contour(const string &name) : Element(name), parent(0), R("R")
# ifdef HAVE_OPENMBVCPPINTERFACE
   , openMBVRigidBody(0)
# endif
                                           {
                                             // no canonic output...
                                             hSize[0] = 0;
                                             hSize[1] = 0;
                                             hInd[0] = 0;
                                             hInd[1] = 0;
                                           }

  Contour::~Contour() {}

  void Contour::init() {
    getFrame()->getJacobianOfTranslation().resize(3,hSize[0]);
    getFrame()->getJacobianOfRotation().resize(3,hSize[0]);
  }

  void Contour::resizeJacobians(int j) {
    getFrame()->getJacobianOfTranslation().resize(3,hSize[j]);
    getFrame()->getJacobianOfRotation().resize(3,hSize[j]);
  }

  void Contour::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVRigidBody && !openMBVRigidBody->isHDF5Link()) {
        vector<double> data;
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        data.push_back(0);
        openMBVRigidBody->append(data);
      }
#endif
      Element::plot(t,dt);
    }
  }

  void Contour::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
        openMBVRigidBody->setName(name);
        RigidBody *rigidBody;
        parent->getOpenMBVGrp()->addObject(openMBVRigidBody);
        if((rigidBody=dynamic_cast<RigidBody*>(parent))!=0) {
          if(rigidBody->getOpenMBVBody()==0) {
            cout<<"To visualize a contour on a rigid body, the body must at least have a OpenMBV::InvisibleBody!"<<endl;
            _exit(1);
          }
          openMBVRigidBody->setHDF5LinkTarget(rigidBody->getOpenMBVBody());
          openMBVRigidBody->setInitialTranslation((rigidBody->getContainerForContourPositions())[rigidBody->contourIndex(this)]);
          openMBVRigidBody->setInitialRotation(AIK2Cardan((rigidBody->getContainerForContourOrientations())[rigidBody->contourIndex(this)]));
        }
      }
#endif
      Element::initPlot(parent);
    }
  }

  /* Rigid Contour */
  void RigidContour::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    if(ff == velocities) {
      Vec WrPC = cp.getFrameOfReference().getPosition() - R.getPosition();
      cp.getFrameOfReference().setVelocity(R.getVelocity() + crossProduct(R.getAngularVelocity(),WrPC));
      cp.getFrameOfReference().setAngularVelocity(R.getAngularVelocity());
    }
    else throw new MBSimError("ERROR (RigidContour::updateKinematicsForFrame): FrameFeature not implemented!");
  }

  void RigidContour::updateJacobiansForFrame(ContourPointData &cp) {
    Vec WrPC = cp.getFrameOfReference().getPosition() - R.getPosition();
    Mat tWrPC = tilde(WrPC);

    cp.getFrameOfReference().setJacobianOfTranslation(R.getJacobianOfTranslation() - tWrPC*R.getJacobianOfRotation());
    cp.getFrameOfReference().setJacobianOfRotation(R.getJacobianOfRotation());
    cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(R.getGyroscopicAccelerationOfTranslation() - tWrPC*R.getGyroscopicAccelerationOfRotation() + crossProduct(R.getAngularVelocity(),crossProduct(R.getAngularVelocity(),WrPC)));
    cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(R.getGyroscopicAccelerationOfRotation());

    // adapt dimensions if necessary
    if(cp.getFrameOfReference().getJacobianOfTranslation().rows() == 0) cp.getFrameOfReference().getJacobianOfTranslation().resize(3,R.getJacobianOfTranslation().cols());
    if(cp.getFrameOfReference().getJacobianOfRotation().rows() == 0) cp.getFrameOfReference().getJacobianOfRotation().resize(3,R.getJacobianOfRotation().cols());
  }

  /* Circle Solid */
#ifdef HAVE_OPENMBVCPPINTERFACE
  void CircleSolid::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Frustum;
      ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(r);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(r);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setHeight(0);
    }
    else openMBVRigidBody=0;
  }
#endif

    /* Circle Hollow */
#ifdef HAVE_OPENMBVCPPINTERFACE
  void CircleHollow::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Frustum;
      ((OpenMBV::Frustum*)openMBVRigidBody)->setInnerBaseRadius(r);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setInnerTopRadius(r);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setBaseRadius(1.1*r);
      ((OpenMBV::Frustum*)openMBVRigidBody)->setTopRadius(1.1*r);

      ((OpenMBV::Frustum*)openMBVRigidBody)->setHeight(0);
    }
    else openMBVRigidBody=0;
  }
#endif

  /* Area */
  Area::Area(const string &name) : RigidContour(name), lim1(1), lim2(1), Cn(3), Cd1(3), Cd2(3) {}
  void Area::setCd1(const Vec &d) {Cd1 = d/nrm2(d);}
  void Area::setCd2(const Vec &d) {Cd2 = d/nrm2(d);}
  void Area::init() {
    RigidContour::init();
    Cn = crossProduct(Cd1,Cd2);
    Cn = Cn/nrm2(Cn);
  }

  /* Edge */
  Edge::Edge(const string &name) : RigidContour(name), lim(1), Cn(3), Cd(3), Ce(3) {}
  void Edge::setCd(const Vec &d) {Cd = d/nrm2(d);}
  void Edge::setCe(const Vec &e) {Ce = e/nrm2(e);}

  /* Sphere */
#ifdef HAVE_OPENMBVCPPINTERFACE
  void Sphere::enableOpenMBV(bool enable) {
    if(enable) {
      openMBVRigidBody=new OpenMBV::Sphere;
      ((OpenMBV::Sphere*)openMBVRigidBody)->setRadius(r);
    }
    else openMBVRigidBody=0;
  }
#endif

  void Sphere::initializeUsingXML(TiXmlElement *element) {
    RigidContour::initializeUsingXML(element);
    TiXmlElement* e;
    e=element->FirstChildElement(MBSIMNS"radius");
    setRadius(atof(e->GetText()));
    e=e->NextSiblingElement();
    if(e && e->ValueStr()==MBSIMNS"enableOpenMBV")
      enableOpenMBV();
  }

  /* Contour1s Analytical */
  void Contour1sAnalytical::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
    if(ff==cosy || ff==position_cosy || velocity_cosy || velocities_cosy || ff==all) {
      cp.getFrameOfReference().getOrientation().col(0)= funcCrPC->computeN(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation().col(1)= funcCrPC->computeT(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation().col(2)= -funcCrPC->computeB(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getOrientation() = cp.getFrameOfReference().getOrientation() * R.getOrientation();
    }
    if(ff==position || ff==position_cosy || ff==all) cp.getFrameOfReference().getPosition() = R.getPosition() + R.getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0));
    if(ff==velocity || velocity_cosy || ff==velocities || velocities_cosy || ff==all) cp.getFrameOfReference().getVelocity() = R.getVelocity() + crossProduct(R.getAngularVelocity(),R.getOrientation()*(*funcCrPC)(cp.getLagrangeParameterPosition()(0)));
    if(ff==angularVelocity || ff==velocities || velocities_cosy || ff==all) throw new MBSimError("ERROR (Contour1sAnalytical::updateKinematicsForFrame): Angular velocity not implemented!");
  }

  /* Contour Interpolation */
  ContourInterpolation::ContourInterpolation(const string &name,int parameters_,int nPoints_) : Contour(name),contourParameters(parameters_),numberOfPoints(nPoints_) {}
  void ContourInterpolation::setPoint(Point *point_, int n) {
    assert(0 <= n);
    assert(n <  numberOfPoints);
    iPoints[n] = point_;
  }

  Vec ContourInterpolation::computePointWeights(const Vec &s) {
    Vec weights(numberOfPoints);
    for(int i=0;i<numberOfPoints; i++) weights(i) = computePointWeight(s,i);
    return weights;
  }

  Vec ContourInterpolation::computeWrOC(const ContourPointData &cp) {
    const Vec &s = cp.getLagrangeParameterPosition();
    Vec r(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) r += computePointWeight(s,i) * iPoints[i]->getReferencePosition();
    return r;
  }

  Vec ContourInterpolation::computeWvC(const ContourPointData &cp) {
    const Vec &s = cp.getLagrangeParameterPosition();
    Vec v(3,INIT,0.0);
    for(int i=0; i<numberOfPoints;i++) v += computePointWeight(s,i) * iPoints[i]->getReferenceVelocity();
    return v;
  }

  Mat ContourInterpolation::computeWt(const ContourPointData &cp) {
    const Vec &s = cp.getLagrangeParameterPosition();
    Mat t(3,contourParameters,INIT,0.0);

    for(int i=0; i<contourParameters; i++) {
      Vec tTemp = t.col(i);
      for(int j=0; j<numberOfPoints;j++) tTemp += computePointWeight(s,j,i) * iPoints[j]->getReferencePosition();
      tTemp /= nrm2(tTemp);
    }   
    return t;
  }

Vec ContourInterpolation::computeWrOC(const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWrOC(cp);}
Vec ContourInterpolation::computeWvC (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWvC (cp);}
Mat ContourInterpolation::computeWt  (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWt  (cp);}
Vec ContourInterpolation::computeWn  (const Vec& s) {ContourPointData cp; cp.getContourParameterType()=EXTINTERPOL;cp.getLagrangeParameterPosition()=s; return computeWn  (cp);}


/* ContourQuad */
ContourQuad::ContourQuad(const string &name) : ContourInterpolation(name,2,4) {
  iPoints.resize(numberOfPoints);
}

void ContourQuad::init() {
  Contour::init();
}

bool ContourQuad::testInsideBounds(const ContourPointData &cp) {
  if( cp.getLagrangeParameterPosition().size()!=2 ) {
    return false;
  }
  if( 0 <= cp.getLagrangeParameterPosition()(0) && cp.getLagrangeParameterPosition()(0) <= 1 && 0 <= cp.getLagrangeParameterPosition()(1) && cp.getLagrangeParameterPosition()(1) <= 1) return true;
  else return false;
}

double ContourQuad::computePointWeight(const Vec &s, int i) {
  double xi  = s(0);
  double eta = s(1);

  switch(i) {
    case 0: return (1-xi)*(1-eta);
    case 1: return (  xi)*(1-eta);
    case 2: return (  xi)*(  eta);
    case 3: return (1-xi)*(  eta);
    default: return 0.0;  // new, probably not OK
  }
}

double ContourQuad::computePointWeight(const Vec &s, int i, int diff) {
  double xi  = s(0);
  double eta = s(1);

  if (diff == 0) // Ableitungen nach xi
    switch(i) {
      case 0: return -(1-eta);
      case 1: return  (1-eta);
      case 2: return  (  eta);
      case 3: return -(  eta);
      default: return 0.0;  // new, probably not OK
    }
  else // Ableitungen nach eta
    switch(i) {
      case 0: return - (1-xi);
      case 1: return - (  xi);
      case 2: return   (  xi);
      case 3: return   (1-xi);
      default: return 0.0;  // new, probably not OK
    }
}

Vec ContourQuad::computeWn(const ContourPointData &cp) {
  const Vec &s = cp.getLagrangeParameterPosition();
  Mat tTemp = computeWt(s);
  return crossProduct(tTemp.col(1),tTemp.col(0)); // Achtung: Interpoation mit einem Konturparameter-> t.col(1) = Cb;
}

CompoundContour::CompoundContour(const string &name) : Contour(name) {
}

void CompoundContour::addContourElement(Contour* c, const Vec& Kr_) {
  element.push_back(c);
  Kr.push_back(Kr_);
  Wr.push_back(Vec(3));
}

void CompoundContour::setReferencePosition(const Vec &WrOP) {
  Contour::setReferencePosition(WrOP);
  for(unsigned int i=0; i<element.size(); i++) 
    element[i]->setReferencePosition(R.getPosition() + Wr[i]);
}

void CompoundContour::setReferenceVelocity(const Vec &WvP) {
  Contour::setReferenceVelocity(WvP);
  for(unsigned int i=0; i<element.size(); i++) 
    element[i]->setReferenceVelocity(R.getVelocity() + crossProduct(R.getAngularVelocity(), Wr[i]));
}

void CompoundContour::setReferenceAngularVelocity(const Vec &WomegaC) {
  Contour::setReferenceAngularVelocity(WomegaC);
  for(unsigned int i=0; i<element.size(); i++) 
    element[i]->setReferenceAngularVelocity(R.getAngularVelocity());
}

void CompoundContour::setReferenceOrientation(const SqrMat &AWC) {
  Contour::setReferenceOrientation(AWC);
  for(unsigned int i=0; i<element.size(); i++) {
    element[i]->setReferenceOrientation(R.getOrientation());
    Wr[i] = R.getOrientation()*Kr[i];
  }
}

void CompoundContour::setReferenceJacobianOfTranslation(const Mat &WJP) {
  Contour::setReferenceJacobianOfTranslation(WJP);
  for(unsigned int i=0; i<element.size(); i++) 
    element[i]->setReferenceJacobianOfTranslation(R.getJacobianOfTranslation() - tilde(Wr[i])*R.getJacobianOfRotation());
}

void CompoundContour::setReferenceGyroscopicAccelerationOfTranslation(const Vec &WjP) {
  Contour::setReferenceGyroscopicAccelerationOfTranslation(WjP);
  for(unsigned int i=0; i<element.size(); i++) 
    element[i]->setReferenceGyroscopicAccelerationOfTranslation(R.getGyroscopicAccelerationOfTranslation() - tilde(Wr[i])*R.getGyroscopicAccelerationOfRotation() + crossProduct(R.getAngularVelocity(),crossProduct(R.getAngularVelocity(),Wr[i])));
}

void CompoundContour::setReferenceJacobianOfRotation(const Mat &WJR) {
  Contour::setReferenceJacobianOfRotation(WJR);
  for(unsigned int i=0; i<element.size(); i++) 
    element[i]->setReferenceJacobianOfRotation(R.getJacobianOfRotation());
}

void CompoundContour::setReferenceGyroscopicAccelerationOfRotation(const Vec &WjR) {
  Contour::setReferenceGyroscopicAccelerationOfRotation(WjR);
  for(unsigned int i=0; i<element.size(); i++) 
    element[i]->setReferenceGyroscopicAccelerationOfRotation(R.getGyroscopicAccelerationOfRotation());
}

void CompoundContour::init() {
  Contour::init();
  for(unsigned int i=0; i<element.size(); i++) {
    element[i]->sethSize(hSize[0]);
    element[i]->init();
  }
}

Cuboid::Cuboid(const string &name) : CompoundContour(name) {}

void Cuboid::preinit() {
  Vec Kr[8];
  for(int i=0; i<8; i++) {
    Kr[i] = Vec(3);
  }
  Kr[0](0) = l/2;
  Kr[0](1) = d/2;
  Kr[0](2) = h/2;

  Kr[1](0) = -l/2.0;
  Kr[1](1) = d/2.0;
  Kr[1](2) = h/2.0;

  Kr[2](0) = -l/2.0;
  Kr[2](1) = -d/2.0;
  Kr[2](2) = h/2.0;

  Kr[3](0) = l/2.0;
  Kr[3](1) = -d/2.0;
  Kr[3](2) = h/2.0;

  Kr[4](0) = l/2.0;
  Kr[4](1) = d/2.0;
  Kr[4](2) = -h/2.0;

  Kr[5](0) = -l/2.0;
  Kr[5](1) = d/2.0;
  Kr[5](2) = -h/2.0;

  Kr[6](0) = -l/2.0;
  Kr[6](1) = -d/2.0;
  Kr[6](2) = -h/2.0;

  Kr[7](0) = l/2.0;
  Kr[7](1) = -d/2.0;
  Kr[7](2) = -h/2.0;

  for(int i=0; i<8; i++) {
    stringstream s;
    s << i+1;
    addContourElement(new Point(s.str()),Kr[i]);
  }
}

}

