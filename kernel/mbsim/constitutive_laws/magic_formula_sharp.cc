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

  void MagicFormulaSharp::initPlot(vector<string> &plotColumns) {
    plotColumns.emplace_back("camber angle");
    plotColumns.emplace_back("rolling velocity");
    plotColumns.emplace_back("spin component of longitudinal velocity");
    plotColumns.emplace_back("longitudinal slip");
    plotColumns.emplace_back("Kyal");
    plotColumns.emplace_back("sSelax");
    plotColumns.emplace_back("slip angle");
    plotColumns.emplace_back("scrub radius");
  }

  void MagicFormulaSharp::plot(vector<double> &plotVector) {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    plotVector.push_back(phi);
    plotVector.push_back(vRoll);
    plotVector.push_back(RvSx-vRoll);
    plotVector.push_back(slip);
    plotVector.push_back(Kyalr);
    plotVector.push_back(sRelax);
    plotVector.push_back(slipAnglePT1);
    plotVector.push_back(rScrub);
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLongitudinalForce");
    if(e) setScaleFactorForLongitudinalForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLateralForce");
    if(e) setScaleFactorForLateralForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForAligningMoment");
    if(e) setScaleFactorForAligningMoment(E(e)->getText<double>());
  }

  void MagicFormulaSharp::updateGeneralizedForces() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double FN = max(0.1,-cz*contact->evalGeneralizedRelativePosition()(0)-dz*contact->evalGeneralizedRelativeVelocity()(2));

    RvSx = contact->getContourFrame(1)->evalOrientation().col(0).T()*contact->getContourFrame(1)->evalVelocity();
    vRoll = contact->evalForwardVelocity()(0);
    slip = -RvSx/vRoll;

    double dfz = (FN - Fz0)/Fz0;
    double Dx = (pDx1 + pDx2*dfz)*FN;
    double Ex = (pEx1 + pEx2*dfz+pEx3*pow(dfz,2))*(1 - pEx4*sgn(slip));
    double Kxka = FN*(pKx1 + pKx2*dfz)*exp(pKx3*dfz);
    double Bx = Kxka/(Cx*Dx);
    double Fx0 = Dx*sin(Cx*atan(Bx*slip - Ex*(Bx*slip - atan(Bx*slip))));
    double Bxal = rBx1*cos(atan(rBx2*slip));
    slipAnglePT1 = contact->getx()(0);
    double FLo = cos(Cxal*atan(Bxal*slipAnglePT1))*Fx0;
    phi = asin(static_cast<Tyre*>(contact->getContour(1))->getFrame()->getOrientation().col(1).T()*contact->getContourFrame(0)->getOrientation().col(2));
    Kyalr = pKy1*Fz0*sin(pKy2*atan(FN/((pKy3+pKy4*pow(phi,2))*Fz0)))/(1+pKy5*pow(phi,2));
    double Dy = FN*pDy1*exp(pDy2*dfz)/(1 + pDy3*pow(phi,2));
    double Ey = pEy1 + pEy2*pow(phi,2) + pEy4*phi*sgn(slipAnglePT1);
    double By = Kyalr/(Cy*Dy);
    double Kyga = (pKy6 + pKy7*dfz)*FN;
    double Bga = Kyga/(Cga*Dy);
    double Fy0 = Dy*sin(Cy*atan(By*slipAnglePT1 - Ey*(By*slipAnglePT1 - atan(By*slipAnglePT1))) + Cga*atan(Bga*phi - Ega*(Bga*phi - atan(Bga*phi))));

    double Byka = rBy1*cos(atan(rBy2*(slipAnglePT1 - rBy3)));
    double FLa = -cos(Cyka*atan(Byka*slip))*Fy0;

    double ga = 0;
    Dy = FN*pDy1*exp(pDy2*dfz)/(1 + pDy3*pow(ga,2));
    Ey = pEy1+pEy2*pow(ga,2) + pEy4*ga*sgn(slipAnglePT1);
    double Kyal = pKy1*Fz0*sin(pKy2*atan(FN/((pKy3 + pKy4*pow(ga,2))*Fz0)))/(1 + pKy5*pow(ga,2));
    By = Kyal/(Cy*Dy);
    Kyga = (pKy6 + pKy7*dfz)*FN;
    Bga = Kyga/(Cga*Dy);
    Fy0 = Dy*sin(Cy*atan(By*slipAnglePT1 - Ey*(By*slipAnglePT1 - atan(By*slipAnglePT1))) + Cga*atan(Bga*ga - Ega*(Bga*ga - atan(Bga*ga))));

    ga = phi;

    double rCrown = tyre->getUnloadedRadius() - tyre->getRimRadius();
    double SHr = (qHz3+qHz4*dfz)*ga;
    double Bt = (qBz1 + qBz2*dfz)*(1 + qBz5*abs(ga) + qBz6*pow(ga,2));
    double Dt = FN*(rCrown/Fz0)*(qDz1 + qDz2*dfz)*(1 + qDz3*abs(ga)+qDz4*pow(ga,2));
    double Et = (qEz1+qEz2*dfz)*(1 + qEz5*ga*(2./M_PI)*atan(Bt*Ct*slipAnglePT1));
    double Br = qBz9 + qBz10*By*Cy;
    double Dr = FN*rCrown*((qDz8 + qDz9*dfz)*ga + (qDz10+qDz11*dfz)*ga*abs(ga))/sqrt(1 + pow(slipAnglePT1,2));

    double Fy = cos(Cyka*atan(Byka*slip))*Fy0;
    double lat = sqrt(pow(slipAnglePT1,2) + pow(Kxka*slip/Kyal,2))*sgn(slipAnglePT1);
    double lar = sqrt(pow(slipAnglePT1 + SHr,2) + pow(Kxka*slip/Kyal,2))*sgn(slipAnglePT1 + SHr);
    double Mzr = Dr*cos(atan(Br*lar));
    double M = Dt*cos(Ct*atan(Bt*lat - Et*(Bt*lat - atan(Bt*lat))))/sqrt(1 + pow(slipAnglePT1,2))*Fy - Mzr;

    sRelax = Kyalr*(9.694e-6 - 1.333*1e-8*contact->getForwardVelocity()(0) + 1.898e-9*pow(contact->getForwardVelocity()(0),2));

    rScrub = (rCrown+contact->getGeneralizedRelativePosition()(0))*sin(phi);

    contact->getGeneralizedForce(false)(0) = sfFLo*FLo;
    contact->getGeneralizedForce(false)(1) = sfFLa*FLa;
    contact->getGeneralizedForce(false)(2) = FN;
    contact->getGeneralizedForce(false)(3) = sfM*M;

    contact->getsRelax(false) = sRelax;
  }
}
