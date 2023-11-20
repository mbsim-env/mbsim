/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "magic_formula_sharp.h"
#include "mbsim/links/tyre_contact.h"
#include "mbsim/contours/tyre.h"
#include "mbsim/frames/contour_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MagicFormulaSharp)

  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  void MagicFormulaSharp::init(InitStage stage, const InitConfigSet &config) {
    if(stage==unknownStage) {
      TyreContact *contact = static_cast<TyreContact*>(parent);
      Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
      double R0_ = tyre->getEllipseParameters()(1)-Fz0/cz;
      if(abs(R0_-R0)>1e-6)
	msg(Warn) << "(MagicFormulaSharp::init): crown radius R0 (" << R0 << ") is different to estimated crown radius of " << tyre->getPath() << " (" << R0_ << ")." << endl;
    }
    TyreModel::init(stage, config);
  }

  void MagicFormulaSharp::initPlot(vector<string> &plotColumns) {
    plotColumns.emplace_back("camber angle");
    plotColumns.emplace_back("rolling velocity");
    plotColumns.emplace_back("spin component of longitudinal velocity");
    plotColumns.emplace_back("longitudinal slip");
    plotColumns.emplace_back("cornering stiffness");
    plotColumns.emplace_back("relaxation length");
    plotColumns.emplace_back("slip angle");
    plotColumns.emplace_back("scrub radius");
  }

  void MagicFormulaSharp::plot(vector<double> &plotVector) {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    plotVector.push_back(ga);
    plotVector.push_back(vx);
    plotVector.push_back(vsx-vx);
    plotVector.push_back(ka);
    plotVector.push_back(Kyal);
    plotVector.push_back(si);
    plotVector.push_back(be);
    plotVector.push_back(Rs);
  }

  void MagicFormulaSharp::initializeUsingXML(DOMElement *element) {
    TyreModel::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"cz");
    setcz(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"dz");
    setdz(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Fz0");
    setFz0(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"R0");
    setR0(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKy1");
    setpKy1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKy2");
    setpKy2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKy3");
    setpKy3(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKy4");
    setpKy4(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKy5");
    setpKy5(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKy6");
    setpKy6(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKy7");
    setpKy7(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pDx1");
    setpDx1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pDx2");
    setpDx2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pEx1");
    setpEx1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pEx2");
    setpEx2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pEx3");
    setpEx3(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pEx4");
    setpEx4(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKx1");
    setpKx1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKx2");
    setpKx2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pKx3");
    setpKx3(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Cx");
    setCx(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Cy");
    setCy(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"rBx1");
    setrBx1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"rBx2");
    setrBx2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Cxal");
    setCxal(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pDy1");
    setpDy1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pDy2");
    setpDy2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pDy3");
    setpDy3(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pEy1");
    setpEy1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pEy2");
    setpEy2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"pEy4");
    setpEy4(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Cga");
    setCga(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Ega");
    setEga(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"rBy1");
    setrBy1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"rBy2");
    setrBy2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"rBy3");
    setrBy3(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Cyka");
    setCyka(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qHz3");
    setqHz3(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qHz4");
    setqHz4(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qBz1");
    setqBz1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qBz2");
    setqBz2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qBz5");
    setqBz5(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qBz6");
    setqBz6(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qBz9");
    setqBz9(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qBz10");
    setqBz10(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz1");
    setqDz1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz2");
    setqDz2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz3");
    setqDz3(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz4");
    setqDz4(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz8");
    setqDz8(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz9");
    setqDz9(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz10");
    setqDz10(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qDz11");
    setqDz11(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qEz1");
    setqEz1(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qEz2");
    setqEz2(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"qEz5");
    setqEz5(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Ct");
    setCt(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"c1Rel");
    setc1Rel(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"c2Rel");
    setc2Rel(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"c3Rel");
    setc3Rel(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForCamberStiffness");
    if(e) setScaleFactorForCamberStiffness(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLongitudinalForce");
    if(e) setScaleFactorForLongitudinalForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLateralForce");
    if(e) setScaleFactorForLateralForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForAligningMoment");
    if(e) setScaleFactorForAligningMoment(E(e)->getText<double>());
  }

  void MagicFormulaSharp::updatexd() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    contact->getxd(false)(0) = (atan(vsy/vx) - contact->getx()(0))*vx/si;
  }

  void MagicFormulaSharp::updateGeneralizedForces() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double Fx, Fy, Mz;
    double Fz = -cz*contact->evalGeneralizedRelativePosition()(0)-dz*contact->evalGeneralizedRelativeVelocity()(2);
    vsx = contact->getGeneralizedRelativeVelocity()(0);
    vsy = contact->getGeneralizedRelativeVelocity()(1);
    vx = abs(contact->evalForwardVelocity()(0));
    if(Fz>0) {
      if(Fz<1) Fz = 1;
      double dfz = (Fz-Fz0)/Fz0;
      ka = -vsx/vx;
      be = contact->getx()(0);
      ga = asin(tyre->getFrame()->getOrientation().col(1).T()*contact->getContourFrame(0)->getOrientation().col(2));

      double Dx = (pDx1+pDx2*dfz)*Fz;
      double Ex = (pEx1+pEx2*dfz+pEx3*pow(dfz,2))*(1-pEx4*sgn(ka));
      double Kxka = Fz*(pKx1+pKx2*dfz)*exp(pKx3*dfz);
      double Bx = Kxka/(Cx*Dx);
      double Fx0 = Dx*sin(Cx*atan(Bx*ka-Ex*(Bx*ka-atan(Bx*ka))));
      double Bxal = rBx1*cos(atan(rBx2*ka));
      Fx = cos(Cxal*atan(Bxal*be))*Fx0*sfFx;

      Kyal = pKy1*Fz0*sin(pKy2*atan(Fz/((pKy3+pKy4*pow(ga,2))*Fz0)))/(1+pKy5*pow(ga,2));
      double Dy = Fz*pDy1*exp(pDy2*dfz)/(1+pDy3*pow(ga,2));
      double Ey = pEy1+pEy2*pow(ga,2)+pEy4*ga*sgn(be);
      double By = Kyal/(Cy*Dy);
      double Kyga = (pKy6+pKy7*dfz)*Fz*sfKyga;
      double Bga = Kyga/(Cga*Dy);
      double Fy0 = Dy*sin(Cy*atan(By*be-Ey*(By*be-atan(By*be)))+Cga*atan(Bga*ga-Ega*(Bga*ga-atan(Bga*ga))));
      double Byka = rBy1*cos(atan(rBy2*(be-rBy3)));
      Fy = -cos(Cyka*atan(Byka*ka))*Fy0*sfFy;

      double Dy0 = Fz*pDy1*exp(pDy2*dfz);
      double Ey0 = pEy1;
      double Kyal0 = pKy1*Fz0*sin(pKy2*atan(Fz/(pKy3*Fz0)));
      double By0 = Kyal0/(Cy*Dy0);
      double Fy00 = Dy0*sin(Cy*atan(By0*be-Ey0*(By0*be-atan(By0*be))));
      double SHr = (qHz3+qHz4*dfz)*ga;
      double Bt = (qBz1+qBz2*dfz)*(1+qBz5*abs(ga)+qBz6*pow(ga,2));
      double Dt = Fz*(R0/Fz0)*(qDz1+qDz2*dfz)*(1+qDz3*abs(ga)+qDz4*pow(ga,2));
      double Et = (qEz1+qEz2*dfz)*(1+qEz5*ga*(2./M_PI)*atan(Bt*Ct*be));
      double Br = qBz9+qBz10*By0*Cy;
      double Dr = Fz*R0*((qDz8+qDz9*dfz)*ga+(qDz10+qDz11*dfz)*ga*abs(ga))/sqrt(1+pow(be,2));
      Fy0 = cos(Cyka*atan(Byka*ka))*Fy00;
      double lat = sqrt(pow(be,2)+pow(Kxka*ka/Kyal0,2))*sgn(be);
      double lar = sqrt(pow(be+SHr,2)+pow(Kxka*ka/Kyal0,2))*sgn(be+SHr);
      double Mzr = Dr*cos(atan(Br*lar));
      Mz = (Dt*cos(Ct*atan(Bt*lat-Et*(Bt*lat-atan(Bt*lat))))/sqrt(1+pow(be,2))*Fy0-Mzr)*sfMz;

      si = Kyal*(c1Rel+c2Rel*vx+c3Rel*pow(vx,2));

      Rs = R0*sin(ga);
    }
    else {
      Fz = 0;
      Fx = 0;
      Fy = 0;
      Mz = 0;
      ga = 0;
      ka = 0;
      Kyal = 0;
      si = 1;
      be = 0;
      Rs = 0;
    }

    contact->getGeneralizedForce(false)(0) = Fx;
    contact->getGeneralizedForce(false)(1) = Fy;
    contact->getGeneralizedForce(false)(2) = Fz;
    contact->getGeneralizedForce(false)(3) = Mz;
  }

  VecV MagicFormulaSharp::getData() const {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    VecV data(8,NONINIT);
    data(0) = ga;
    data(1) = vx;
    data(2) = vsx-vx;
    data(3) = ka;
    data(4) = Kyal;
    data(5) = si;
    data(6) = be;
    data(7) = Rs;
    return data;
  }

}
