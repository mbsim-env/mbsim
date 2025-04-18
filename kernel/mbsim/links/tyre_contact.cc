#include <config.h>
#include "tyre_contact.h"
#include "mbsim/constitutive_laws/tyre_model.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/tyre.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/spatial_contour.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "mbsim/functions/contact/funcpair_spatialcontour_spatialcontour.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, TyreContact)

  class FuncPairSpatialContourCircularShape : public Function<Vec(Vec)> {
    public:
      FuncPairSpatialContourCircularShape(double r_, double rc_, Tyre *tyre_, Contour *contour_) : r(r_), rc(rc_), contour(contour_), tyre(tyre_) { }
      Vec operator()(const Vec &alpha) override;
    private:
      double r, rc;
      Contour *contour;
      Tyre *tyre;
  };

  Vec FuncPairSpatialContourCircularShape::operator()(const Vec &alpha) {
    Vec3 Wn = contour->evalWn(alpha);
    Vec3 Wb = tyre->getFrame()->evalOrientation().col(1);
    Vec3 Wc = Wn - (Wn.T()*Wb)*Wb;
    Vec3 WrD = (tyre->getFrame()->getPosition() - ((r-rc)/nrm2(Wc))*Wc) - contour->evalPosition(alpha);
    Vec3 Wu = contour->evalWu(alpha);
    Vec3 Wv = contour->evalWv(alpha);
    Vec2 Wt(NONINIT);
    Wt(0) = Wu.T() * WrD;
    Wt(1) = Wv.T() * WrD;
    return Wt;
  }

  class FuncPairSpatialContourEllipticalShape : public Function<Vec(Vec)> {
    public:
      FuncPairSpatialContourEllipticalShape(double r_, double a_, double b_, Tyre *tyre_, Contour *contour_) : r(r_), a(a_), b(b_), contour(contour_), tyre(tyre_) { }
      Vec operator()(const Vec &alpha) override;
    private:
      double r, a, b;
      Contour *contour;
      Tyre *tyre;
  };

  Vec FuncPairSpatialContourEllipticalShape::operator()(const Vec &alpha) {
    Vec3 Wn = contour->evalWn(alpha);
    Vec3 Wb = tyre->getFrame()->evalOrientation().col(1);
    Vec3 nx = crossProduct(Wb,Wn);
    nx /= nrm2(nx);
    SqrMat3 AIB(NONINIT);
    AIB.set(0, nx);
    AIB.set(1, crossProduct(Wn,nx));
    AIB.set(2, Wn);
    double ga = asin(Wb.T()*Wn);
    double xi = -atan(a/b*tan(ga));
    double y = a*sin(xi);
    double z = b - b*cos(xi) - r;
    Vec3 BrSC(NONINIT);
    BrSC(0) = 0;
    BrSC(1) = y*cos(ga) - z*sin(ga);
    BrSC(2) = y*sin(ga) + z*cos(ga);
    Vec WrQ = tyre->getFrame()->getPosition() + AIB*BrSC;
    Vec3 WrD = WrQ - contour->evalPosition(alpha);
    Vec3 Wu = contour->evalWu(alpha);
    Vec3 Wv = contour->evalWv(alpha);
    Vec2 Wt(NONINIT);
    Wt(0) = Wu.T() * WrD;
    Wt(1) = Wv.T() * WrD;
    return Wt;
  }

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
      if(model->motorcycleKinematics() and dynamic_cast<Tyre*>(contour[1])) {
        if(static_cast<Tyre*>(contour[1])->getShapeOfCrossSectionContour()==Tyre::flat)
          throwError("(TyreContact::init): shape of tyre contour must be circular or elliptical");
      }
      DF.resize(3,NONINIT);
      DM.resize(model->getDMSize(),NONINIT);
      iF = RangeV(0,2);
      iM = RangeV(3,3+model->getDMSize()-1);
    }
    else if(stage==plotting) {
      if(plotFeature[plotRecursive] and plotFeature[MBSim::generalizedForce]) {
	model->initPlot(plotColumns);
      }
    }
    else if(stage==unknownStage) {
      if(isSize)
        curis = zeta0.row(0).T();
    }
    ContourLink::init(stage,config);
    model->init(stage, config);
  }

  void TyreContact::initializeUsingXML(xercesc::DOMElement *element) {
    ContourLink::initializeUsingXML(element);
    xercesc::DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"tyreModel");
    setTyreModel(ObjectFactory::createAndInit<TyreModel>(e->getFirstElementChild()));
    e = E(element)->getFirstElementChildNamed(MBSIM%"initialGuess");
    if(e) setInitialGuess(E(e)->getText<MatV>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"tolerance");
    if(e) setTolerance(E(e)->getText<double>());
  }

  void TyreContact::plot() {
    if(plotFeature[plotRecursive] and plotFeature[MBSim::generalizedForce]) {
      model->plot(plotVector);
    }
    ContourLink::plot();
  }

  void TyreContact::setTyreModel(TyreModel *model_) {
    model = model_;
    model->setParent(this);
  } 

  void TyreContact::calcSize() {
    ng = 1;
    ngd = 3;
    nla = 3+model->getDMSize();
    updSize = false;
  }

  void TyreContact::calcxSize() {
    xSize = model->getxSize();
  }

  void TyreContact::calcisSize() {
    ContourLink::calcisSize();
    if(dynamic_cast<Tyre*>(contour[1])) {
      if(not dynamic_cast<Plane*>(contour[0]))
        isSize += 2;
    }
    else
      isSize += 4;
  }

  void TyreContact::updatePositions(Frame *frame) {
    if(updrrel)
      updateGeneralizedPositions();
  }

  void TyreContact::updateGeneralizedPositions() {
    double g = 1;
    Vec3 Wb = getRigidContour(1)->getFrame()->evalOrientation().col(1);
    if(dynamic_cast<Tyre*>(contour[1])) {
      Tyre *tyre = static_cast<Tyre*>(contour[1]);
      double r = model->evalFreeRadius();
      if(dynamic_cast<Plane*>(contour[0])) {
        Plane *plane = static_cast<Plane*>(contour[0]);
        Vec3 Wn = plane->getFrame()->evalOrientation().col(0);
        Vec3 Wc = Wn - (Wn.T()*Wb)*Wb;
        Vec3 nx = crossProduct(Wb,Wn);
        nx /= nrm2(nx);
        cFrame[0]->getOrientation(false).set(0, nx);
        cFrame[0]->getOrientation(false).set(1, crossProduct(Wn,nx));
        cFrame[0]->getOrientation(false).set(2, Wn);
        cFrame[1]->setOrientation(cFrame[0]->getOrientation(false));
        if(model->motorcycleKinematics()) {
          if(tyre->getShapeOfCrossSectionContour()==Tyre::circular) {
            double rc = model->getContourParameters()(0);
            Vec WrCW = tyre->getFrame()->getPosition() - ((r-rc)/nrm2(Wc))*Wc;
            Vec3 Wd = WrCW - plane->getFrame()->getPosition();
            g = Wn.T()*Wd - rc;
            cFrame[1]->setPosition(WrCW - (rc+min(g,0.))*Wn);
          }
          else if(tyre->getShapeOfCrossSectionContour()==Tyre::elliptical) {
            double ga = asin(Wb.T()*Wn);
            double a = model->getContourParameters()(0);
            double b = model->getContourParameters()(1);
            //	  double y = a*tan(ga)/sqrt(pow(tan(ga),2) + pow(b/a,2));
            //	  double z = r - b + sqrt(b*b - pow(y*b/a,2));
            double xi = -atan(a/b*tan(ga));
            double y = a*sin(xi);
            double z = b - b*cos(xi) - r;
            Vec3 BrSC(NONINIT);
            BrSC(0) = 0;
            BrSC(1) = y*cos(ga) - z*sin(ga);
            BrSC(2) = y*sin(ga) + z*cos(ga);
            Vec WrQ = tyre->getFrame()->getPosition() + cFrame[1]->getOrientation(false)*BrSC;
            Vec3 Wd = WrQ - plane->getFrame()->getPosition();
            g = Wn.T()*Wd;
            cFrame[1]->setPosition(WrQ - min(g,0.)*Wn);
          }
        }
        else {
          Vec WrCW = tyre->getFrame()->getPosition() - (r/nrm2(Wc))*Wc;
          Vec3 Wd = WrCW - plane->getFrame()->getPosition();
          g = Wn.T()*Wd;
          cFrame[1]->setPosition(WrCW - (min(g,0.)/cos(asin(Wb.T()*Wn))/nrm2(Wc))*Wc);
        }
        cFrame[0]->setPosition(cFrame[1]->getPosition(false) - Wn*max(g,0.));
      }
      else {
        if(model->motorcycleKinematics()) {
          if(tyre->getShapeOfCrossSectionContour()==Tyre::circular) {
            double rc = model->getContourParameters()(0);
            auto func = new FuncPairSpatialContourCircularShape(r,rc,tyre,contour[0]);
            MultiDimNewtonMethod search(func, nullptr);
            search.setTolerance(tol);
            nextis = search.solve(curis);
            if(search.getInfo()!=0)
              throw std::runtime_error("(TyreContact::updateGeneralizedPositions): contact search failed!");
            cFrame[0]->setZeta(nextis);
            Vec3 Wn = contour[0]->evalWn(cFrame[0]->getZeta(false));
            Vec3 Wc = Wn - (Wn.T()*Wb)*Wb;
            Vec3 nx = crossProduct(Wb,Wn);
            nx /= nrm2(nx);
            cFrame[0]->getOrientation(false).set(0, nx);
            cFrame[0]->getOrientation(false).set(1, crossProduct(Wn,nx));
            cFrame[0]->getOrientation(false).set(2, Wn);
            cFrame[1]->setOrientation(cFrame[0]->getOrientation(false));
            cFrame[0]->setPosition(contour[0]->evalPosition(cFrame[0]->getZeta(false)));
            Vec WrCW = tyre->getFrame()->getPosition() - ((r-rc)/nrm2(Wc))*Wc;
            Vec3 Wd = WrCW - cFrame[0]->getPosition(false);
            g = contour[0]->isZetaOutside(cFrame[0]->getZeta(false))?1:Wn.T()*Wd-rc;
            if(g < -contour[0]->getThickness()) g = 1;
            cFrame[1]->setPosition(WrCW - (rc+min(g,0.))*Wn);
          }
          if(tyre->getShapeOfCrossSectionContour()==Tyre::elliptical) {
            double a = model->getContourParameters()(0);
            double b = model->getContourParameters()(1);
            auto func = new FuncPairSpatialContourEllipticalShape(r,a,b,tyre,contour[0]);
            MultiDimNewtonMethod search(func, nullptr);
            search.setTolerance(tol);
            nextis = search.solve(curis);
            if(search.getInfo()!=0)
              throw std::runtime_error("(TyreContact::updateGeneralizedPositions): contact search failed!");
            cFrame[0]->setZeta(nextis);
            Vec3 Wn = contour[0]->evalWn(cFrame[0]->getZeta(false));
            Vec3 nx = crossProduct(Wb,Wn);
            nx /= nrm2(nx);
            cFrame[0]->getOrientation(false).set(0, nx);
            cFrame[0]->getOrientation(false).set(1, crossProduct(Wn,nx));
            cFrame[0]->getOrientation(false).set(2, Wn);
            cFrame[1]->setOrientation(cFrame[0]->getOrientation(false));
            cFrame[0]->setPosition(contour[0]->evalPosition(cFrame[0]->getZeta(false)));
            double ga = asin(Wb.T()*Wn);
            double xi = -atan(a/b*tan(ga));
            double y = a*sin(xi);
            double z = b - b*cos(xi) - r;
            Vec3 BrSC(NONINIT);
            BrSC(0) = 0;
            BrSC(1) = y*cos(ga) - z*sin(ga);
            BrSC(2) = y*sin(ga) + z*cos(ga);
            Vec WrQ = tyre->getFrame()->getPosition() + cFrame[1]->getOrientation(false)*BrSC;
            Vec3 Wd = WrQ - cFrame[0]->getPosition(false);
            g = contour[0]->isZetaOutside(cFrame[0]->getZeta(false))?1:Wn.T()*Wd;
            if(g < -contour[0]->getThickness()) g = 1;
            cFrame[1]->setPosition(WrQ - min(g,0.)*Wn);
          }
        }
        else {
          auto func = new FuncPairSpatialContourCircularShape(r,0,tyre,contour[0]);
          MultiDimNewtonMethod search(func, nullptr);
          search.setTolerance(tol);
          nextis = search.solve(curis);
          if(search.getInfo()!=0)
            throw std::runtime_error("(TyreContact::updateGeneralizedPositions): contact search failed!");
          cFrame[0]->setZeta(nextis);
          Vec3 Wn = contour[0]->evalWn(cFrame[0]->getZeta(false));
          Vec3 Wc = Wn - (Wn.T()*Wb)*Wb;
          Vec3 nx = crossProduct(Wb,Wn);
          nx /= nrm2(nx);
          cFrame[0]->getOrientation(false).set(0, nx);
          cFrame[0]->getOrientation(false).set(1, crossProduct(Wn,nx));
          cFrame[0]->getOrientation(false).set(2, Wn);
          cFrame[1]->setOrientation(cFrame[0]->getOrientation(false));
          cFrame[0]->setPosition(contour[0]->evalPosition(cFrame[0]->getZeta(false)));
          Vec WrCW = tyre->getFrame()->getPosition() - (r/nrm2(Wc))*Wc;
          Vec3 Wd = WrCW - cFrame[0]->getPosition(false);
          g = contour[0]->isZetaOutside(cFrame[0]->getZeta(false))?1:Wn.T()*Wd;
          if(g < -contour[0]->getThickness()) g = 1;
          cFrame[1]->setPosition(WrCW - (min(g,0.)/cos(asin(Wb.T()*Wn))/nrm2(Wc))*Wc);
        }
      }
    }
    else {
      auto func = new FuncPairSpatialContourSpatialContour(contour[0],contour[1]);
      MultiDimNewtonMethod search(func, nullptr);
      search.setTolerance(tol);
      nextis = search.solve(curis);
      if(search.getInfo()!=0)
        throw std::runtime_error("(RailwayWheelContact::updateGeneralizedPositions): contact search failed!");
      cFrame[0]->setZeta(nextis(RangeV(0,1)));
      cFrame[1]->setZeta(nextis(RangeV(2,3)));
      auto Wn = contour[0]->evalWn(cFrame[0]->getZeta(false));
      auto nx = crossProduct(Wb,Wn);
      cFrame[0]->getOrientation(false).set(0, nx);
      cFrame[0]->getOrientation(false).set(1, crossProduct(Wn,nx));
      cFrame[0]->getOrientation(false).set(2, Wn);
      cFrame[1]->setOrientation(cFrame[0]->getOrientation(false));
      cFrame[0]->setPosition(contour[0]->evalPosition(cFrame[0]->getZeta(false)));
      cFrame[1]->setPosition(contour[1]->evalPosition(cFrame[1]->getZeta(false)));

      if(contour[0]->isZetaOutside(cFrame[0]->getZeta(false)) or contour[1]->isZetaOutside(cFrame[1]->getZeta(false)))
        g = 1;
      else
        g = Wn.T()*(cFrame[1]->getPosition(false) - cFrame[0]->getPosition(false));
      if(g < -contour[0]->getThickness() or g < -contour[1]->getThickness()) g = 1;
    }
    rrel(0) = g;

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
    Vec3 Wv = getRigidContour(1)->getFrame()->evalVelocity();
    Vec3 rSC = cFrame[1]->evalPosition() - getRigidContour(1)->getFrame()->evalPosition();
    Vec3 Wom = getRigidContour(1)->getFrame()->evalAngularVelocity();
    Vec3 Kom = getRigidContour(1)->getFrame()->evalOrientation().T()*Wom;
    Kom(1) = 0;
    Wom = getRigidContour(1)->getFrame()->evalOrientation()*Kom;
    RvC = cFrame[0]->evalOrientation().T()*(Wv + crossProduct(Wom,rSC));
    updfvel = false;
  }

  void TyreContact::updateGeneralizedForces() {
    model->updateGeneralizedForces();
    updla = false;
  }

  void TyreContact::updateForceDirections() {
    DF = cFrame[0]->evalOrientation();
    if(model->getDMSize()==3)
      DM = cFrame[0]->getOrientation();
    else
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
    model->updatexd();
  }

  void TyreContact::resetUpToDate() {
    ContourLink::resetUpToDate();
    model->resetUpToDate();
    updfvel = true;
  }

  RigidContour* TyreContact::getRigidContour(int i) {
    return static_cast<RigidContour*>(getContour(i));
  }

}
