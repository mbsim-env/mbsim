#include <config.h>
#include "tyre_contact.h"
#include "mbsim/constitutive_laws/tyre_model.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/tyre.h"
#include "mbsim/contours/plane.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, TyreContact)

  TyreContact::TyreContact(const std::string &name) : ContourLink(name) {
    M.resize(2);
  }

  TyreContact::~TyreContact() {
    delete model;
  }

  void TyreContact::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if(stage==preInit) {
      if(not model)
	throwError("(TyreContact::init): tyre model must be defined");
      if(not dynamic_cast<Plane*>(contour[0]))
	throwError("(TyreContact::init): first contour must be of type Plane");
      if(not dynamic_cast<Tyre*>(contour[1]))
	throwError("(TyreContact::init): second contour must be of type Tyre");
      DF.resize(3,NONINIT);
      DM.resize(1,NONINIT);
      iF = RangeV(0,2);
      iM = RangeV(3,3);
    }
    ContourLink::init(stage,config);
    model->init(stage, config);
  }

  void TyreContact::initializeUsingXML(xercesc::DOMElement *element) {
    ContourLink::initializeUsingXML(element);
    xercesc::DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"tyreModel");
    setTyreModel(ObjectFactory::createAndInit<TyreModel>(e->getFirstElementChild()));

  }

  void TyreContact::setTyreModel(TyreModel *model_) {
    model = model_;
    model->setParent(this);
  } 

  void TyreContact::calcSize() {
    ng = 1;
    ngd = 4;
    nla = ngd;
    updSize = false;
  }

 void TyreContact::updatePositions(Frame *frame) {
    if(updrrel)
      updateGeneralizedPositions();
  }

  void TyreContact::updateGeneralizedPositions() {
    Plane *plane = static_cast<Plane*>(contour[0]);
    Tyre *tyre = static_cast<Tyre*>(contour[1]);

    Vec3 Wn = plane->getFrame()->evalOrientation().col(0);
    Vec3 Wb = tyre->getFrame()->evalOrientation().col(1);
    double t_EC = Wn.T()*Wb;
    Vec3 z_EC = Wn - t_EC*Wb;
    double z_EC_nrm2 = nrm2(z_EC);
    Vec3 Wd = tyre->getFrame()->getPosition() - (tyre->getRimRadius()/z_EC_nrm2)*z_EC - plane->getFrame()->getPosition();
    double g = Wn.T()*Wd - tyre->getUnloadedRadius() + tyre->getRimRadius();
    cFrame[1]->setPosition(tyre->getFrame()->getPosition() - (tyre->getRimRadius()/z_EC_nrm2)*z_EC - (tyre->getUnloadedRadius() - tyre->getRimRadius() + min(g,0.))*Wn);
    cFrame[0]->setPosition(cFrame[1]->getPosition(false) - Wn*max(g,0.));
    rrel(0) = g;

    Vec3 nx = crossProduct(Wb,Wn);
    nx /= nrm2(nx);

    cFrame[0]->setOrientation(plane->getFrame()->getOrientation());
    cFrame[1]->getOrientation(false).set(0, -plane->getFrame()->getOrientation().col(0));
    cFrame[1]->getOrientation(false).set(2, nx);
    cFrame[1]->getOrientation(false).set(1, crossProduct(cFrame[1]->getOrientation(false).col(2),cFrame[1]->getOrientation(false).col(0)));

    cFrame[0]->getOrientation(false).set(0, cFrame[1]->getOrientation(false).col(2));
    cFrame[0]->getOrientation(false).set(1, cFrame[1]->getOrientation(false).col(1));
    cFrame[0]->getOrientation(false).set(2, -cFrame[1]->getOrientation(false).col(0));
    cFrame[1]->setOrientation(cFrame[0]->getOrientation(false));

    updrrel = false;
  }

  void TyreContact::updateGeneralizedVelocities() {
    Vec3 Wn = cFrame[0]->evalOrientation().col(2);
    Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();
    vrel(2) = Wn.T() * WvD;
    Mat3xV Wt(2,NONINIT);
    Wt.set(0, cFrame[0]->getOrientation().col(0));
    Wt.set(1, cFrame[0]->getOrientation().col(1));
    vrel.set(RangeV(0,1), Wt.T() * WvD);
    updvrel = false;
  }

  void TyreContact::updateForwardVelocity() {
    Vec3 Wv = static_cast<Tyre*>(contour[1])->getFrame()->evalVelocity();
    Vec3 rSC = cFrame[1]->evalPosition() - static_cast<Tyre*>(contour[1])->getFrame()->evalPosition();
    Vec3 Wom = static_cast<Tyre*>(contour[1])->getFrame()->evalAngularVelocity();
    Vec3 Kom = static_cast<Tyre*>(contour[1])->getFrame()->evalOrientation().T()*Wom;
    Kom(1) = 0;
    Wom = static_cast<Tyre*>(contour[1])->getFrame()->evalOrientation()*Kom;
    RvC = cFrame[0]->evalOrientation().T()*(Wv + crossProduct(Wom,rSC));
    updfvel = false;
  }

  void TyreContact::updateGeneralizedForces() {
    model->updateGeneralizedForces();
    updla = false;
  }

  void TyreContact::updateForceDirections() {
    DF.set(0,cFrame[0]->evalOrientation().col(0));
    DF.set(1,cFrame[0]->getOrientation().col(1));
    DF.set(2,cFrame[0]->getOrientation().col(2));
    DM.set(0,cFrame[0]->getOrientation().col(2));
    updDF = false;
  }

  void TyreContact::updateh(int j) {
    Vec3 F = evalGlobalForceDirection()*evalGeneralizedForce()(iF);
    Vec3 M = evalGlobalMomentDirection()*evalGeneralizedForce()(iM);

    h[j][0] -= cFrame[0]->evalJacobianOfTranslation(j).T() * F + cFrame[0]->evalJacobianOfRotation(j).T() * M;
    h[j][1] += cFrame[1]->evalJacobianOfTranslation(j).T() * F + cFrame[1]->evalJacobianOfRotation(j).T() * M;
  }

  void TyreContact::updatexd() {
    const Vec3& v = evalForwardVelocity();
    xd(0) = (atan(v(1)/v(0)) - x(0))*v(0)/evalsRelax();
  }

  void TyreContact::resetUpToDate() {
    ContourLink::resetUpToDate();
    updfvel = true;
  }
}
