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

  void MagicFormulaSharp::updateGeneralizedForces() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double FN = max(0.1,-c_z_RW*contact->evalGeneralizedRelativePosition()(0)-d_z_RW*contact->evalGeneralizedRelativeVelocity()(2));

    double RvSx = contact->getContourFrame(1)->evalOrientation().col(0).T()*contact->getContourFrame(1)->evalVelocity();
    double slip = -RvSx/contact->evalForwardVelocity()(0);

    double dfz = (FN - Fz0_RW)/Fz0_RW;
    double Dx = (pDx1 + pDx2*dfz)*FN;
    double Ex = (pEx1 + pEx2*dfz+pEx3*pow(dfz,2))*(1 - pEx4*sgn(slip));
    double Kxka = FN*(pKx1 + pKx2*dfz)*exp(pKx3*dfz);
    double Bx = Kxka/(Cx*Dx);
    double Fx0 = Dx*sin(Cx*atan(Bx*slip - Ex*(Bx*slip - atan(Bx*slip))));
    double Bxal = rBx1*cos(atan(rBx2*slip));
    double slipAnglePT1 = contact->getx()(0);
    double FLo = cos(Cxal*atan(Bxal*slipAnglePT1))*Fx0;
    double phi = asin(static_cast<Tyre*>(contact->getContour(1))->getFrame()->getOrientation().col(1).T()*contact->getContourFrame(0)->getOrientation().col(2));
    double Kyalr = pKy1*Fz0_RW*sin(pKy2*atan(FN/((pKy3+pKy4*pow(phi,2))*Fz0_RW)))/(1+pKy5*pow(phi,2));
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
    double Kyal = pKy1*Fz0_RW*sin(pKy2*atan(FN/((pKy3 + pKy4*pow(ga,2))*Fz0_RW)))/(1 + pKy5*pow(ga,2));
    By = Kyal/(Cy*Dy);
    Kyga = (pKy6 + pKy7*dfz)*FN;
    Bga = Kyga/(Cga*Dy);
    Fy0 = Dy*sin(Cy*atan(By*slipAnglePT1 - Ey*(By*slipAnglePT1 - atan(By*slipAnglePT1))) + Cga*atan(Bga*ga - Ega*(Bga*ga - atan(Bga*ga))));

    ga = phi;

    double SHr = (qHz3+qHz4*dfz)*ga;
    double Bt = (qBz1 + qBz2*dfz)*(1 + qBz5*abs(ga) + qBz6*pow(ga,2));
    double Dt = FN*(tyre->getCrownRadius()/Fz0_RW)*(qDz1 + qDz2*dfz)*(1 + qDz3*abs(ga)+qDz4*pow(ga,2));
    double Et = (qEz1+qEz2*dfz)*(1 + qEz5*ga*(2./M_PI)*atan(Bt*Ct*slipAnglePT1));
    double Br = qBz9 + qBz10*By*Cy;
    double Dr = FN*tyre->getCrownRadius()*((qDz8 + qDz9*dfz)*ga + (qDz10+qDz11*dfz)*ga*abs(ga))/sqrt(1 + pow(slipAnglePT1,2));

    double Fy = cos(Cyka*atan(Byka*slip))*Fy0;
    double lat = sqrt(pow(slipAnglePT1,2) + pow(Kxka*slip/Kyal,2))*sgn(slipAnglePT1);
    double lar = sqrt(pow(slipAnglePT1 + SHr,2) + pow(Kxka*slip/Kyal,2))*sgn(slipAnglePT1 + SHr);
    double Mzr = Dr*cos(atan(Br*lar));
    double M = Dt*cos(Ct*atan(Bt*lat - Et*(Bt*lat - atan(Bt*lat))))/sqrt(1 + pow(slipAnglePT1,2))*Fy - Mzr;

    contact->getGeneralizedForce(false)(0) = FLo;
    contact->getGeneralizedForce(false)(1) = FLa;
    contact->getGeneralizedForce(false)(2) = FN;
    contact->getGeneralizedForce(false)(3) = M;

    contact->getsRelax(false) = Kyalr*(9.694e-6 - 1.333*1e-8*contact->getForwardVelocity()(0) + 1.898e-9*pow(contact->getForwardVelocity()(0),2));
  }
}
