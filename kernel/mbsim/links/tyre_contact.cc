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
    if(t_EC>0) {
      Wb *= -1.;
      t_EC *= -1;	
    }
    Vec3 z_EC = Wn - t_EC*Wb;
    double z_EC_nrm2 = nrm2(z_EC);
    Vec3 Wd = tyre->getFrame()->getPosition() - (tyre->getRimRadius()/z_EC_nrm2)*z_EC - plane->getFrame()->getPosition();
    double g = Wn.T()*Wd - tyre->getCrownRadius();
    cFrame[1]->setPosition(tyre->getFrame()->getPosition() - (tyre->getRimRadius()/z_EC_nrm2)*z_EC - (tyre->getCrownRadius() + min(g,0.))*Wn);
    cFrame[0]->setPosition(cFrame[1]->getPosition(false) - Wn*max(g,0.));
    rrel(0) = g;

    Vec3 nx = crossProduct(Wb,Wn); 
    nx /= nrm2(nx);

    cFrame[0]->getOrientation(false).set(0, plane->getFrame()->getOrientation().col(0));
    cFrame[0]->getOrientation(false).set(1, -nx);
    cFrame[0]->getOrientation(false).set(2, crossProduct(cFrame[0]->getOrientation(false).col(0),cFrame[0]->getOrientation(false).col(1)));
    cFrame[1]->getOrientation(false).set(0, nx);
    cFrame[1]->getOrientation(false).set(2, -plane->getFrame()->getOrientation().col(0));
    cFrame[1]->getOrientation(false).set(1, crossProduct(cFrame[1]->getOrientation(false).col(2),cFrame[1]->getOrientation(false).col(0)));

    updrrel = false;
  }

  void TyreContact::updateGeneralizedVelocities() {
    Vec3 Wn = cFrame[0]->evalOrientation().col(0);
    Vec3 WvD = cFrame[1]->evalVelocity() - cFrame[0]->evalVelocity();
    vrel(0) = Wn.T() * WvD;
    Mat3xV Wt(2,NONINIT);
    Wt.set(0, cFrame[0]->getOrientation().col(1));
    Wt.set(1, cFrame[0]->getOrientation().col(2));
    vrel.set(RangeV(1,2), Wt.T() * WvD);
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
    Tyre *tyre = static_cast<Tyre*>(contour[1]);

    lambda(0) = max(0.1,-c_z_RW*evalGeneralizedRelativePosition()(0)-d_z_RW*evalGeneralizedRelativeVelocity()(0));

    double RvSx = cFrame[0]->evalOrientation().col(1).T()*cFrame[1]->evalVelocity();
    double slip = -RvSx/evalForwardVelocity()(1);

    double dfz = (lambda(0) - Fz0_RW)/Fz0_RW;
    double Dx = (pDx1 + pDx2*dfz)*lambda(0);
    double Ex = (pEx1 + pEx2*dfz+pEx3*pow(dfz,2))*(1 - pEx4*sgn(slip));
    double Kxka = lambda(0)*(pKx1 + pKx2*dfz)*exp(pKx3*dfz);
    double Bx = Kxka/(Cx*Dx);
    double Fx0 = Dx*sin(Cx*atan(Bx*slip - Ex*(Bx*slip - atan(Bx*slip))));
    double Bxal = rBx1*cos(atan(rBx2*slip));
    lambda(1) = cos(Cxal*atan(Bxal*x(0)))*Fx0;

    double phi = asin(static_cast<Tyre*>(contour[1])->getFrame()->getOrientation().col(1).T()*cFrame[0]->getOrientation().col(0));
    Kyalr = pKy1*Fz0_RW*sin(pKy2*atan(lambda(0)/((pKy3+pKy4*pow(phi,2))*Fz0_RW)))/(1+pKy5*pow(phi,2));
    double Dy = lambda(0)*pDy1*exp(pDy2*dfz)/(1 + pDy3*pow(phi,2));
    double Ey = pEy1 + pEy2*pow(phi,2) + pEy4*phi*sgn(x(0));
    double By = Kyalr/(Cy*Dy);
    double Kyga = (pKy6 + pKy7*dfz)*lambda(0);
    double Bga = Kyga/(Cga*Dy);
    double Fy0 = Dy*sin(Cy*atan(By*x(0) - Ey*(By*x(0) - atan(By*x(0)))) + Cga*atan(Bga*phi - Ega*(Bga*phi - atan(Bga*phi))));

    double Byka = rBy1*cos(atan(rBy2*(x(0) - rBy3)));
    lambda(2) = -cos(Cyka*atan(Byka*slip))*Fy0;

    double ga = 0;
    Dy = lambda(0)*pDy1*exp(pDy2*dfz)/(1 + pDy3*pow(ga,2));
    Ey = pEy1+pEy2*pow(ga,2) + pEy4*ga*sgn(x(0));
    double Kyal = pKy1*Fz0_RW*sin(pKy2*atan(lambda(0)/((pKy3 + pKy4*pow(ga,2))*Fz0_RW)))/(1 + pKy5*pow(ga,2));
    By = Kyal/(Cy*Dy);
    Kyga = (pKy6 + pKy7*dfz)*lambda(0);
    Bga = Kyga/(Cga*Dy);
    Fy0 = Dy*sin(Cy*atan(By*x(0) - Ey*(By*x(0) - atan(By*x(0)))) + Cga*atan(Bga*ga - Ega*(Bga*ga - atan(Bga*ga))));

    ga = phi;

    double SHr = (qHz3+qHz4*dfz)*ga;
    double Bt = (qBz1 + qBz2*dfz)*(1 + qBz5*abs(ga) + qBz6*pow(ga,2));
    double Dt = lambda(0)*(tyre->getCrownRadius()/Fz0_RW)*(qDz1 + qDz2*dfz)*(1 + qDz3*abs(ga)+qDz4*pow(ga,2));
    double Et = (qEz1+qEz2*dfz)*(1 + qEz5*ga*(2./M_PI)*atan(Bt*Ct*x(0)));
    double Br = qBz9 + qBz10*By*Cy;
    double Dr = lambda(0)*tyre->getCrownRadius()*((qDz8 + qDz9*dfz)*ga + (qDz10+qDz11*dfz)*ga*abs(ga))/sqrt(1 + pow(x(0),2));

    double Fy = cos(Cyka*atan(Byka*slip))*Fy0;
    double lat = sqrt(pow(x(0),2) + pow(Kxka*slip/Kyal,2))*sgn(x(0));
    double lar = sqrt(pow(x(0) + SHr,2) + pow(Kxka*slip/Kyal,2))*sgn(x(0) + SHr);
    double Mzr = Dr*cos(atan(Br*lar));
    lambda(3) = Dt*cos(Ct*atan(Bt*lat - Et*(Bt*lat - atan(Bt*lat))))/sqrt(1 + pow(x(0),2))*Fy - Mzr;

    updla = false;
  }

  void TyreContact::updateForceDirections() {
    DF.set(0,cFrame[0]->evalOrientation().col(0));
    DF.set(1,cFrame[0]->getOrientation().col(1));
    DF.set(2,cFrame[0]->getOrientation().col(2));
    DM.set(0,cFrame[0]->getOrientation().col(0));
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
    double sRelax = evalKyalr()*(9.694e-6 - 1.333*1e-8*v(1) + 1.898e-9*pow(v(1),2));
    xd(0) = (atan(v(2)/v(1)) - x(0))*v(1)/sRelax;
  }

  void TyreContact::resetUpToDate() {
    ContourLink::resetUpToDate();
    updfvel = true;
  }
}
