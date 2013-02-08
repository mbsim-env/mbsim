/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>
#include "trafo33RCM.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  /* Class PositionFunction */
  PositionFunction::PositionFunction(RevCardanPtr angle_, double l0_, const Vec3& pL_, const Vec3& pR_, double cL1_, double cR1_, double cL2_, double cR2_, const RowVec3& rRrLmH_) :
      angle(angle_), l0(l0_), pL(pL_), pR(pR_), cL1(cL1_), cR1(cR1_), cL2(cL2_), cR2(cR2_), rRrLmH(rRrLmH_) {
  }

  PositionFunction::~PositionFunction() {
  }

  Vec11 PositionFunction::operator()(const Vec11& pos, const void *) {
    Vec3 pS = pos(Range0x2());

    Vec3 nS = angle->computen(pS);
    Vec3 bS = angle->computeb(pS);
    RowVec3 ntilSH = angle->computentil(pS).T();
    RowVec3 btilSH = angle->computebtil(pS).T();
    double xibtil = btilSH * nS;
    double xintil = ntilSH * nS;
    double etabtil = btilSH * bS;
    double etantil = ntilSH * bS;

    Vec11 F;
    F(0) = pos(0) - 0.5 * (pR(0) + pL(0) - sin(pos(1)) * (pos(9) + pos(10)));
    F(1) = pos(1) - 0.5 * (pR(1) + pL(1) - pos(5) - pos(6));
    F(2) = pos(2) - 0.5 * (pR(2) + pL(2) - pos(9) - pos(10));
    F(3) = pos(6) - pos(5) - pR(1) + pL(1);
    F(4) = pos(10) - pos(9) - pR(2) + pL(2);
    F(5) = xintil * (pos(4) - pos(3)) + xibtil * (pos(8) - pos(7)) - rRrLmH * nS;
    F(6) = etantil * (pos(4) - pos(3)) + etabtil * (pos(8) - pos(7)) - rRrLmH * bS;
    F(7) = 3. * l0 * (pos(5) - pos(6)) / 64. + 7. * (pos(3) + pos(4)) / 16. - cL1 - cR1;
    F(8) = -3. * l0 * (pos(5) + pos(6)) / 128. + 17. * (pos(4) - pos(3)) / 64. + cL1 - cR1;
    F(9) = 3. * l0 * (pos(9) - pos(10)) / 64. + 7. * (pos(7) + pos(8)) / 16. - cL2 - cR2;
    F(10) = -3. * l0 * (pos(9) + pos(10)) / 128. + 17. * (pos(8) - pos(7)) / 64. + cL2 - cR2;
    return F;
  }
  /*******************************************************************/

  /* Class PositionJacobian */
  PositionJacobian::PositionJacobian(RevCardanPtr angle_, double l0_, const RowVec3 &rRrLmH_, const Mat3x11 &pSbE_) :
      angle(angle_), l0(l0_), rRrLmH(rRrLmH_), pSbE(pSbE_) {
  }

  PositionJacobian::~PositionJacobian() {
  }

  void PositionJacobian::operator()(const Vec11& pos,   SqrMat11 & SMRHS_Jac, const void *) {
    Vec3 pS = pos(Range0x2());

    Vec3 nS = angle->computen(pS);
    Vec3 bS = angle->computeb(pS);
    RowVec3 ntilSH = angle->computentil(pS).T();
    RowVec3 btilSH = angle->computebtil(pS).T();
    double xibtil = btilSH * nS;
    double xintil = ntilSH * nS;
    double etabtil = btilSH * bS;
    double etantil = ntilSH * bS;

    Mat3x11 nSbE = angle->computenq(pS) * pSbE;
    Mat3x11 bSbE = angle->computebq(pS) * pSbE;
    Mat11V ntilSbEH = (angle->computentilq(pS) * pSbE).T(); //TODO:_size okay??
    Mat11V btilSbEH = (angle->computebtilq(pS) * pSbE).T(); //TODO:_size okay??

    RowVec11 xibtilbE = (btilSbEH * nS).T() + btilSH * nSbE;
    RowVec11 xintilbE = (ntilSbEH * nS).T() + ntilSH * nSbE;
    RowVec11 etabtilbE = (btilSbEH * bS).T() + btilSH * bSbE;
    RowVec11 etantilbE = (ntilSbEH * bS).T() + ntilSH * bSbE;

    SMRHS_Jac(0, 0) = 1.;
    SMRHS_Jac(0, 1) = 0.5 * cos(pS(1)) * (pos(9) + pos(10));
    SMRHS_Jac(0, 9) = 0.5 * sin(pS(1));
    SMRHS_Jac(0, 10) = 0.5 * sin(pS(1));

    SMRHS_Jac(1, 1) = 1.;
    SMRHS_Jac(1, 5) = 0.5;
    SMRHS_Jac(1, 6) = 0.5;

    SMRHS_Jac(2, 2) = 1.;
    SMRHS_Jac(2, 9) = 0.5;
    SMRHS_Jac(2, 10) = 0.5;

    SMRHS_Jac(3, 5) = -1.;
    SMRHS_Jac(3, 6) = 1.;

    SMRHS_Jac(4, 9) = -1.;
    SMRHS_Jac(4, 10) = 1.;

    //TODO: Test set/add functions
    SMRHS_Jac.set(Index(5, 5), Index(0, 10), -rRrLmH * nSbE);
    SMRHS_Jac.add(Index(5, 5), Index(0, 10), xintilbE * (pos(4) - pos(3)) + xibtilbE * (pos(8) - pos(7)));
    SMRHS_Jac(5, 3) -= xintil;
    SMRHS_Jac(5, 4) += xintil;
    SMRHS_Jac(5, 7) -= xibtil;
    SMRHS_Jac(5, 8) += xibtil;

    SMRHS_Jac.set(Index(6, 6), Index(0, 10), -rRrLmH * bSbE);
    SMRHS_Jac.add(Index(6, 6), Index(0, 10), etantilbE * (pos(4) - pos(3)) + etabtilbE * (pos(8) - pos(7)));
    SMRHS_Jac(6, 3) -= etantil;
    SMRHS_Jac(6, 4) += etantil;
    SMRHS_Jac(6, 7) -= etabtil;
    SMRHS_Jac(6, 8) += etabtil;

    SMRHS_Jac(7, 3) = 7. / 16.;
    SMRHS_Jac(7, 4) = 7. / 16.;
    SMRHS_Jac(7, 5) = 3. * l0 / 64.;
    SMRHS_Jac(7, 6) = -3. * l0 / 64.;

    SMRHS_Jac(8, 3) = -17. / 64.;
    SMRHS_Jac(8, 4) = 17. / 64.;
    SMRHS_Jac(8, 5) = -3. * l0 / 128.;
    SMRHS_Jac(8, 6) = -3. * l0 / 128.;

    SMRHS_Jac(9, 7) = 7. / 16.;
    SMRHS_Jac(9, 8) = 7. / 16.;
    SMRHS_Jac(9, 9) = 3. * l0 / 64.;
    SMRHS_Jac(9, 10) = -3. * l0 / 64.;

    SMRHS_Jac(10, 7) = -17. / 64.;
    SMRHS_Jac(10, 8) = 17. / 64.;
    SMRHS_Jac(10, 9) = -3. * l0 / 128.;
    SMRHS_Jac(10, 10) = -3. * l0 / 128.;

  }
  /*******************************************************************/

  /* CLASS TRAFO33RCM */
  Trafo33RCM::Trafo33RCM(RevCardanPtr angle_, double l0_) :
      angle(angle_), l0(l0_), l0h2(l0 * l0), l0h3(l0h2 * l0), l0h4(l0h3 * l0), l0h5(l0h4 * l0), xstar(0.25 * l0), xstarh2(xstar * xstar), xstarh3(xstarh2 * xstar), epstil(0.), k0(0.), rS(), pS(), rRrLp(), rRrLmH(), be(), tS(), nS(), bS(), ntilS(), btilS(), nSH(), bSH(), ntilSH(), btilSH(), tSpS(), nSpS(), bSpS(), ntilSpS(), btilSpS(), xibtil(0.), xintil(0.), etabtil(0.), etantil(0.), SMRHS_Jac(), V(), drRdrLp(), drRdrLm(), pSbE(), SMRHS(), nSbE(), bSbE(), ntilSbE(), btilSbE(), xibtilbE(), xintilbE(), etabtilbE(), etantilbE(), beqG(), tSqG(), nSqG(), bSqG(), ntilSqG(), btilSqG(), xintilqG(), xibtilqG(), etantilqG(), etabtilqG(), JIG(), JIGt(), k0t(0.), qIt(), bet(), rSt(), pSt(), tSt(), nSt(), bSt(), tStH(), nStH(), bStH(), ntilStH(), btilStH(), tSpSt(), nSpSt(), bSpSt(), ntilSpSt(), btilSpSt(), xibtilt(0.), xintilt(0.), etabtilt(0.), etantilt(0.) {
    /* trafo initial value */
    SMRHS(0, 2) = -1.;
    SMRHS(0, 3) = 1.;
    SMRHS(1, 6) = -1.;
    SMRHS(1, 7) = 1.;

    SMRHS(4, 0) = 7. / 16.;
    SMRHS(4, 1) = 7. / 16.;
    SMRHS(4, 2) = 3. * l0 / 64.;
    SMRHS(4, 3) = -3. * l0 / 64.;
    SMRHS(5, 0) = -17. / 64.;
    SMRHS(5, 1) = 17. / 64.;
    SMRHS(5, 2) = -3. * l0 / 128.;
    SMRHS(5, 3) = -3. * l0 / 128.;
    SMRHS(6, 4) = 7. / 16.;
    SMRHS(6, 5) = 7. / 16.;
    SMRHS(6, 6) = 3. * l0 / 64.;
    SMRHS(6, 7) = -3. * l0 / 64.;
    SMRHS(7, 4) = -17. / 64.;
    SMRHS(7, 5) = 17. / 64.;
    SMRHS(7, 6) = -3. * l0 / 128.;
    SMRHS(7, 7) = -3. * l0 / 128.;

    /* trafo Jacobian */
    computeV();

    SMRHS_Jac(0, 0) = 1.;
    SMRHS_Jac(1, 1) = 1.;
    SMRHS_Jac(1, 5) = 0.5;
    SMRHS_Jac(1, 6) = 0.5;
    SMRHS_Jac(2, 2) = 1.;
    SMRHS_Jac(2, 9) = 0.5;
    SMRHS_Jac(2, 10) = 0.5;
    SMRHS_Jac(3, 6) = 1.;
    SMRHS_Jac(3, 5) = -1.;
    SMRHS_Jac(4, 10) = 1.;
    SMRHS_Jac(4, 9) = -1.;
    //TODO: check values
    SMRHS_Jac.set(Index(7, 7), Index(3, 6), 2. * xstarh2 * (V(Index(1, 1), Index(0, 3)) * xstarh2 + V(Index(3, 3), Index(0, 3))));
    SMRHS_Jac.set(Index(8, 8), Index(3, 6), 2. * xstarh3 * (V(Index(0, 0), Index(0, 3)) * xstarh2 + V(Index(2, 2), Index(0, 3))));
    SMRHS_Jac.set(Index(9, 9), Index(7, 10), 2. * xstarh2 * (V(Index(1, 1), Index(0, 3)) * xstarh2 + V(Index(3, 3), Index(0, 3))));
    SMRHS_Jac.set(Index(10, 10), Index(7, 10), 2. * xstarh3 * (V(Index(0, 0), Index(0, 3)) * xstarh2 + V(Index(2, 2), Index(0, 3))));

    SMRHS_Jac(0, 14) = 0.5;
    SMRHS_Jac(0, 24) = 0.5;
    SMRHS_Jac(1, 15) = 0.5;
    SMRHS_Jac(1, 25) = 0.5;
    SMRHS_Jac(2, 16) = 0.5;
    SMRHS_Jac(2, 26) = 0.5;
    SMRHS_Jac(3, 15) = -1.;
    SMRHS_Jac(3, 25) = 1.;
    SMRHS_Jac(4, 16) = -1.;
    SMRHS_Jac(4, 26) = 1.;
    SMRHS_Jac(7, 17) = 1.;
    SMRHS_Jac(7, 18) = 1.;
    SMRHS_Jac(8, 17) = -1.;
    SMRHS_Jac(8, 18) = 1.;
    SMRHS_Jac(9, 19) = 1.;
    SMRHS_Jac(9, 20) = 1.;
    SMRHS_Jac(10, 19) = -1.;
    SMRHS_Jac(10, 20) = 1.;

    computedrRdrL();

    pSbE(0, 0) = 1.;
    pSbE(1, 1) = 1.;
    pSbE(2, 2) = 1.;
  }

  Trafo33RCM::~Trafo33RCM() {
  }

  void Trafo33RCM::computeprelim(const Vec16& qG) {
    rRrLmH = (qG(Range10x12()) - qG(Range0x2())).T();
  }

  Vec Trafo33RCM::computes0(const Vec16& qG) {

    Vec3 pM = 0.5 * (qG(Range3x5()) + qG(Range13x15()));

    Vec3 tM = angle->computet(pM);
    Vec3 nM = angle->computen(pM);
    Vec3 bM = angle->computeb(pM);
    Vec nMtil = angle->computentil(pM);
    Vec bMtil = angle->computebtil(pM);
    SqrMat3 nMpM = angle->computenq(pM);
    SqrMat3 bMpM = angle->computebq(pM);

    double xinMtil = nM.T() * nMtil;
    double xibMtil = nM.T() * bMtil;
    double etanMtil = bM.T() * nMtil;
    double etabMtil = bM.T() * bMtil;

    RowVec3 rRrLmnMpM = 0.5 * rRrLmH * nMpM;
    SMRHS(2, 0) = -xinMtil;
    SMRHS(2, 1) = xinMtil;
    SMRHS(2, 4) = -xibMtil;
    SMRHS(2, 5) = xibMtil;
    SMRHS(2, 2) = rRrLmnMpM(1);
    SMRHS(2, 3) = rRrLmnMpM(1);
    SMRHS(2, 6) = rRrLmnMpM(0) * sin(pM(1)) + rRrLmnMpM(2);
    SMRHS(2, 7) = rRrLmnMpM(0) * sin(pM(1)) + rRrLmnMpM(2);

    RowVec3 rRrLmbMpM = 0.5 * rRrLmH * bMpM;
    SMRHS(3, 0) = -etanMtil;
    SMRHS(3, 1) = etanMtil;
    SMRHS(3, 4) = -etabMtil;
    SMRHS(3, 5) = etabMtil;
    SMRHS(3, 2) = rRrLmbMpM(1);
    SMRHS(3, 3) = rRrLmbMpM(1);
    SMRHS(3, 6) = rRrLmbMpM(0) * sin(pM(1)) + rRrLmbMpM(2);
    SMRHS(3, 7) = rRrLmbMpM(0) * sin(pM(1)) + rRrLmbMpM(2);

    SMRHS(0, 8) = qG(14) - qG(4);
    SMRHS(1, 8) = qG(15) - qG(5);
    SMRHS(2, 8) = rRrLmH * nM;
    SMRHS(3, 8) = rRrLmH * bM;
    SMRHS(4, 8) = qG(6) + qG(7);
    SMRHS(5, 8) = qG(7) - qG(6);
    SMRHS(6, 8) = qG(8) + qG(9);
    SMRHS(7, 8) = qG(9) - qG(8);

    //TODO: check slvLU here
    Vec sol = slvLU(static_cast<SqrMat>(SMRHS(Index(0, 7), Index(0, 7))), static_cast<Vec>(SMRHS(Index(0, 7), Index(8, 8))));
    Vec11 s0(NONINIT);
    s0.set(Index(3, 10), sol);

    s0(1) = 0.5 * ((qG(4) + qG(14)) - (sol(2) + sol(3)));
    s0(2) = 0.5 * ((qG(5) + qG(15)) - (sol(6) + sol(7)));
    s0(0) = 0.5 * (qG(3) + qG(13) - sin(s0(1)) * (sol(6) + sol(7)));

    return s0;
  }

  void Trafo33RCM::computebe(const Vec16& qG) {
    // REQUIRED computeprelim()

    Vec3 pL = qG(Range3x5());
    Vec3 pR = qG(Range13x15());

    PositionFunction fun(angle, l0, pL, pR, qG(6), qG(7), qG(8), qG(9), rRrLmH); // (object itself no reference)
    PositionJacobian jac(angle, l0, rRrLmH, pSbE);
    GlobalResidualCriteriaFunction<Fixed<11>, double> crit;
    StandardDampingFunction<Fixed<11>, double > damp;
    MultiDimensionalNewtonMethod<Fixed<11>, double> rf;
    rf.setFunction(&fun);
    rf.setJacobianFunction(&jac);
    rf.setCriteriaFunction(&crit);
    rf.setDampingFunction(&damp);
    rf.setMaximumNumberOfIterations(10);

    if (nrm2(be) < MBSim::epsroot()) {
      Vec11 s0 = computes0(qG); // initial value

      be = rf.solve(s0);
    }
    else
      be = rf.solve(be);

    if (rf.getInfo() != 0) {
      throw MBSim::MBSimError("ERROR (TRAFO33RCM:computebe): No convergence of Newton method during bending correction.");
    }
  }

  void Trafo33RCM::computeCOSY() {
    // REQUIRED computebe()

    pS = be(Range0x2());
    tS = angle->computet(pS);
    nS = angle->computen(pS);
    bS = angle->computeb(pS);
    ntilS = angle->computentil(pS);
    btilS = angle->computebtil(pS);
    nSH = nS.T();
    bSH = bS.T();
    ntilSH = ntilS.T();
    btilSH = btilS.T();
    tSpS = angle->computetq(pS);
    nSpS = angle->computenq(pS);
    bSpS = angle->computebq(pS);
    ntilSpS = angle->computentilq(pS);
    btilSpS = angle->computebtilq(pS);

    xibtil = nSH * btilS;
    xintil = nSH * ntilS;
    etabtil = bSH * btilS;
    etantil = bSH * ntilS;
  }

  void Trafo33RCM::computerSepstk0(const Vec16& qG) {
    // REQUIRED computeCOSY()

    Vec3 rRrLp = qG(Range10x12()) + qG(Range0x2());

    rS = 0.5 * (rRrLp - (xintil * (be(3) + be(4)) + xibtil * (be(7) + be(8))) * nS - (etantil * (be(3) + be(4)) + etabtil * (be(7) + be(8))) * bS);
    epstil = rRrLmH * tS / l0 - 1.;
    k0 = (qG(13) - qG(3) - sin(pS(1)) * (be(10) - be(9))) / l0;
  }

  void Trafo33RCM::computeqI(const Vec16& qG) {
    computeprelim(qG);
    computebe(qG);
    computeCOSY();
    computerSepstk0(qG);
  }

  void Trafo33RCM::computedrRdrL() {
    drRdrLp(0, 0) = 1.;
    drRdrLp(1, 1) = 1.;
    drRdrLp(2, 2) = 1.;
    drRdrLp(0, 10) = 1.;
    drRdrLp(1, 11) = 1.;
    drRdrLp(2, 12) = 1.;

    drRdrLm(0, 0) = -1.;
    drRdrLm(1, 1) = -1.;
    drRdrLm(2, 2) = -1.;
    drRdrLm(0, 10) = 1.;
    drRdrLm(1, 11) = 1.;
    drRdrLm(2, 12) = 1.;
  }

  void Trafo33RCM::computeV() {
    V(0, 0) = 24. / l0h5;
    V(1, 0) = -8. / l0h4;
    V(2, 0) = -10. / l0h3;
    V(3, 0) = 4. / l0h2;

    V(0, 1) = -24. / l0h5;
    V(1, 1) = -8. / l0h4;
    V(2, 1) = 10. / l0h3;
    V(3, 1) = 4. / l0h2;

    V(0, 2) = 4. / l0h4;
    V(1, 2) = -2. / l0h3;
    V(2, 2) = -1. / l0h2;
    V(3, 2) = 1. / (2. * l0);

    V(0, 3) = 4. / l0h4;
    V(1, 3) = 2. / l0h3;
    V(2, 3) = -1. / l0h2;
    V(3, 3) = -1. / (2. * l0);
  }

  void Trafo33RCM::computebeqG() {
    // REQUIRED computeqI()

    nSbE = nSpS * pSbE;
    bSbE = bSpS * pSbE;
    ntilSbE = ntilSpS * pSbE;
    btilSbE = btilSpS * pSbE;

    xibtilbE = nSH * btilSbE + btilSH * nSbE;
    xintilbE = nSH * ntilSbE + ntilSH * nSbE;
    etabtilbE = bSH * btilSbE + btilSH * bSbE;
    etantilbE = bSH * ntilSbE + ntilSH * bSbE;

    SMRHS_Jac(0, 1) = 0.5 * cos(pS(1)) * (be(9) + be(10));
    SMRHS_Jac(0, 9) = 0.5 * sin(pS(1));
    SMRHS_Jac(0, 10) = 0.5 * sin(pS(1));
    SMRHS_Jac.set(Index(5, 5), Index(0, 10), -rRrLmH * nSbE);
    SMRHS_Jac.add(Index(5, 5), Index(0, 10), xintilbE * (be(4) - be(3)) + xibtilbE * (be(8) - be(7)));
    SMRHS_Jac(5, 4) += xintil;
    SMRHS_Jac(5, 3) -= xintil;
    SMRHS_Jac(5, 8) += xibtil;
    SMRHS_Jac(5, 7) -= xibtil;

    SMRHS_Jac.set(Index(6, 6), Index(0, 10), -rRrLmH * bSbE);
    SMRHS_Jac.add(Index(6, 6), Index(0, 10), etantilbE * (be(4) - be(3)) + etabtilbE * (be(8) - be(7)));
    SMRHS_Jac(6, 4) += etantil;
    SMRHS_Jac(6, 3) -= etantil;
    SMRHS_Jac(6, 8) += etabtil;
    SMRHS_Jac(6, 7) -= etabtil;

    SMRHS_Jac.set(Index(5, 5), Index(11, 26), nSH * drRdrLm);
    SMRHS_Jac.set(Index(6, 6), Index(11, 26), bSH * drRdrLm);

    //TODO: Is this done every timestep --> use different solver
    beqG = slvLU(static_cast<SqrMat>(SMRHS_Jac(Index(0, 10), Index(0, 10))), static_cast<Mat>(SMRHS_Jac(Index(0, 10), Index(11, 26)))); //TODO??
  }

  void Trafo33RCM::computeCOSYqG() {
    // REQUIRED computebeqG()

    tSqG = tSpS * beqG(Index(0, 2), Index(0, 15));
    nSqG = nSpS * beqG(Index(0, 2), Index(0, 15));
    bSqG = bSpS * beqG(Index(0, 2), Index(0, 15));
    ntilSqG = ntilSpS * beqG(Index(0, 2), Index(0, 15));
    btilSqG = btilSpS * beqG(Index(0, 2), Index(0, 15));

    xintilqG = nSH * ntilSqG + ntilSH * nSqG;
    xibtilqG = nSH * btilSqG + btilSH * nSqG;
    etantilqG = bSH * ntilSqG + ntilSH * bSqG;
    etabtilqG = bSH * btilSqG + btilSH * bSqG;
  }

  void Trafo33RCM::computeJIG(const Vec16& qG) {
    computeqI(qG);
    computebeqG();
    computeCOSYqG();

    RowVec3 tSH = tS.T();

    JIG.set(Index(0, 2), Index(0, 15), drRdrLp);
    /*Scheint zu passen;
     cout << nS << endl;
     cout << xintilqG << endl;
     cout << xibtilqG << endl;
     cout << be << endl;
     cout << xintil << endl;
     cout << xibtil << endl;
     cout << beqG << endl;

     cout << beqG(Index(3, 3), Index(0, 15)) << endl;
     cout << beqG(Index(4, 4), Index(0, 15)) << endl;
     cout << beqG(Index(7, 7), Index(0, 15)) << endl;
     cout << beqG(Index(8, 8), Index(0, 15)) << endl;*/

    JIG.add(Index(0, 2), Index(0, 15), -(nS * (xintilqG * (be(3) + be(4)) + xibtilqG * (be(7) + be(8)) + xintil * (beqG(Index(3, 3), Index(0, 15)) + beqG(Index(4, 4), Index(0, 15))) + xibtil * (beqG(Index(7, 7), Index(0, 15)) + beqG(Index(8, 8), Index(0, 15))))));
    JIG.add(Index(0, 2), Index(0, 15), -((xintil * (be(3) + be(4)) + xibtil * (be(7) + be(8))) * nSqG));
    JIG.add(Index(0, 2), Index(0, 15), -(bS * (etantilqG * (be(3) + be(4)) + etabtilqG * (be(7) + be(8)) + etantil * (beqG(Index(3, 3), Index(0, 15)) + beqG(Index(4, 4), Index(0, 15))) + etabtil * (beqG(Index(7, 7), Index(0, 15)) + beqG(Index(8, 8), Index(0, 15))))));
    JIG.add(Index(0, 2), Index(0, 15), -((etantil * (be(3) + be(4)) + etabtil * (be(7) + be(8))) * bSqG));
    JIG.mult(Index(0, 2), Index(0, 15), 0.5);
    JIG.set(Index(3, 5), Index(0, 15), beqG(Index(0, 2), Index(0, 15)));
    JIG.set(Index(6, 6), Index(0, 15), (tSH * drRdrLm + rRrLmH * tSqG) / l0);
    JIG.set(Index(7, 14), Index(0, 15), beqG(Index(3, 10), Index(0, 15)));
    JIG.set(Index(15, 15), Index(0, 15), -sin(pS(1)) * (beqG(Index(10, 10), Index(0, 15)) - beqG(Index(9, 9), Index(0, 15))) - (be(10) - be(9)) * cos(pS(1)) * beqG(Index(1, 1), Index(0, 15)));
    JIG(15, 3) -= 1.;
    JIG(15, 13) += 1.;
    JIG.div(Index(15, 15), Index(0, 15), l0);
  }

  void Trafo33RCM::computezI(const Vec16& qG, const Vec16& qGt) {
    computeJIG(qG);

    qIt = JIG * qGt;
    rSt = qIt(Range0x2());
    pSt = qIt(Range3x5());
    epstilt = qIt(6);
    bet.set(Range0x2(), pSt(Range0x2()));
    bet.set(Range3x10(), qIt(Range7x14()));
    k0t = qIt(15);
  }

  void Trafo33RCM::computeCOSYt(const Vec16& qG, const Vec16& qGt) {
    computezI(qG, qGt);

    tSt = tSpS * pSt;
    nSt = nSpS * pSt;
    bSt = bSpS * pSt;
    Vec3 ntilSt = ntilSpS * pSt;
    Vec3 btilSt = btilSpS * pSt;

    nStH = nSt.T();
    bStH = bSt.T();
    ntilStH = ntilSt.T();
    btilStH = btilSt.T();

    tSpSt = angle->computetqt(pS, pSt);
    nSpSt = angle->computenqt(pS, pSt);
    bSpSt = angle->computebqt(pS, pSt);
    ntilSpSt = angle->computentilqt(pS, pSt);
    btilSpSt = angle->computebtilqt(pS, pSt);

    xibtilt = nStH * btilS + btilStH * nS;
    xintilt = nStH * ntilS + ntilStH * nS;
    etabtilt = bStH * btilS + btilStH * bS;
    etantilt = bStH * ntilS + ntilStH * bS;
  }

  void Trafo33RCM::computeJIGt(const Vec16& qGt) {
    // REQUIRED computeCOSYt()

    RowVec3 rRrLtmH = (qGt(Range10x12()) - qGt(Range0x2())).T();
    RowVec3 tStH = tSt.T();

    Mat3x11 nSbEt = nSpSt * pSbE;
    Mat3x11 bSbEt = bSpSt * pSbE;
    Mat3x11 ntilSbEt = ntilSpSt * pSbE;
    Mat3x11 btilSbEt = btilSpSt * pSbE;

    RowVec11 xibtilbEt = nStH * btilSbE + btilStH * nSbE + nSH * btilSbEt + btilSH * nSbEt;
    RowVec11 xintilbEt = nStH * ntilSbE + ntilStH * nSbE + nSH * ntilSbEt + ntilSH * nSbEt;
    RowVec11 etabtilbEt = bStH * btilSbE + btilStH * bSbE + bSH * btilSbEt + btilSH * bSbEt;
    RowVec11 etantilbEt = bStH * ntilSbE + ntilStH * bSbE + bSH * ntilSbEt + ntilSH * bSbEt;

    Mat3V tSqGt = tSpSt * beqG(Index(0, 2), Index(0, 15)) + tSpS * JIGt(Index(3, 5), Index(0, 15));
    Mat3V nSqGt = nSpSt * beqG(Index(0, 2), Index(0, 15)) + nSpS * JIGt(Index(3, 5), Index(0, 15));
    Mat3V bSqGt = bSpSt * beqG(Index(0, 2), Index(0, 15)) + bSpS * JIGt(Index(3, 5), Index(0, 15));
    Mat3V ntilSqGt = ntilSpSt * beqG(Index(0, 2), Index(0, 15)) + ntilSpS * JIGt(Index(3, 5), Index(0, 15));
    Mat3V btilSqGt = btilSpSt * beqG(Index(0, 2), Index(0, 15)) + btilSpS * JIGt(Index(3, 5), Index(0, 15));

    RowVec16 xibtilqGt = nStH * btilSqG + btilStH * nSqG + nSH * btilSqGt + btilSH * nSqGt;
    RowVec16 xintilqGt = nStH * ntilSqG + ntilStH * nSqG + nSH * ntilSqGt + ntilSH * nSqGt;
    RowVec16 etabtilqGt = bStH * btilSqG + btilStH * bSqG + bSH * btilSqGt + btilSH * bSqGt;
    RowVec16 etantilqGt = bStH * ntilSqG + ntilStH * bSqG + bSH * ntilSqGt + ntilSH * bSqGt;

    SqrMat11 SMt;
    SMt(0, 1) = 0.5 * (-sin(pS(1)) * pSt(1) * (be(9) + be(10)) + cos(pS(1)) * (bet(9) + bet(10)));
    SMt(0, 9) = 0.5 * cos(pS(1)) * pSt(1);
    SMt(0, 10) = 0.5 * cos(pS(1)) * pSt(1);

    SMt.set(Index(5, 5), Index(0, 10), -rRrLtmH * nSbE - rRrLmH * nSbEt);
    SMt.add(Index(5, 5), Index(0, 10), xintilbEt * (be(4) - be(3)) + xibtilbEt * (be(8) - be(7)));
    SMt.add(Index(5, 5), Index(0, 10), xintilbE * (bet(4) - bet(3)) + xibtilbE * (bet(8) - bet(7)));
    SMt(5, 4) += xintilt;
    SMt(5, 3) -= xintilt;
    SMt(5, 8) += xibtilt;
    SMt(5, 7) -= xibtilt;

    SMt.set(Index(6, 6), Index(0, 10), -rRrLtmH * bSbE - rRrLmH * bSbEt);
    SMt.add(Index(6, 6), Index(0, 10), etantilbEt * (be(4) - be(3)) + etabtilbEt * (be(8) - be(7)));
    SMt.add(Index(6, 6), Index(0, 10), etantilbE * (bet(4) - bet(3)) + etabtilbE * (bet(8) - bet(7)));
    SMt(6, 4) += etantilt;
    SMt(6, 3) -= etantilt;
    SMt(6, 8) += etabtilt;
    SMt(6, 7) -= etabtilt;

    MatV16 RHS_JIGt = -SMt * beqG;
    RHS_JIGt.add(Index(5, 5), Index(0, 15), nStH * drRdrLm);
    RHS_JIGt.add(Index(6, 6), Index(0, 15), bStH * drRdrLm);

    Mat beqGt = slvLU(static_cast<SqrMat>(SMRHS_Jac(Index(0, 10), Index(0, 10))), static_cast<Mat>(RHS_JIGt)); //TODO??

    JIGt.set(Index(3, 5), Index(0, 15), beqGt(Index(0, 2), Index(0, 15)));
    JIGt.set(Index(7, 14), Index(0, 15), beqGt(Index(3, 10), Index(0, 15)));

    JIGt.set(Index(6, 6), Index(0, 15), (tStH * drRdrLm + rRrLtmH * tSqG + rRrLmH * tSqGt) / l0);

    JIGt.set(Index(0, 2), Index(0, 15), -nSt * (xintilqG * (be(3) + be(4)) + xibtilqG * (be(7) + be(8)) + xintil * (beqG(Index(3, 3), Index(0, 15)) + beqG(Index(4, 4), Index(0, 15))) + xibtil * (beqG(Index(7, 7), Index(0, 15)) + beqG(Index(8, 8), Index(0, 15)))));
    JIGt.add(Index(0, 2), Index(0, 15), -(nS * (xintilqGt * (be(3) + be(4)) + xibtilqGt * (be(7) + be(8)) + xintilt * (beqG(Index(3, 3), Index(0, 15)) + beqG(Index(4, 4), Index(0, 15))) + xibtilt * (beqG(Index(7, 7), Index(0, 15)) + beqG(Index(8, 8), Index(0, 15))))));
    JIGt.add(Index(0, 2), Index(0, 15), -(nS * (xintilqG * (bet(3) + bet(4)) + xibtilqG * (bet(7) + bet(8)) + xintil * (beqGt(Index(3, 3), Index(0, 15)) + beqGt(Index(4, 4), Index(0, 15))) + xibtil * (beqGt(Index(7, 7), Index(0, 15)) + beqGt(Index(8, 8), Index(0, 15))))));

    JIGt.add(Index(0, 2), Index(0, 15), -((xintil * (be(3) + be(4)) + xibtil * (be(7) + be(8))) * nSqGt));
    JIGt.add(Index(0, 2), Index(0, 15), -((xintilt * (be(3) + be(4)) + xibtilt * (be(7) + be(8))) * nSqG));
    JIGt.add(Index(0, 2), Index(0, 15), -((xintil * (bet(3) + bet(4)) + xibtil * (bet(7) + bet(8))) * nSqG));

    JIGt.add(Index(0, 2), Index(0, 15), -(bSt * (etantilqG * (be(3) + be(4)) + etabtilqG * (be(7) + be(8)) + etantil * (beqG(Index(3, 3), Index(0, 15)) + beqG(Index(4, 4), Index(0, 15))) + etabtil * (beqG(Index(7, 7), Index(0, 15)) + beqG(Index(8, 8), Index(0, 15))))));
    JIGt.add(Index(0, 2), Index(0, 15), -(bS * (etantilqGt * (be(3) + be(4)) + etabtilqGt * (be(7) + be(8)) + etantilt * (beqG(Index(3, 3), Index(0, 15)) + beqG(Index(4, 4), Index(0, 15))) + etabtilt * (beqG(Index(7, 7), Index(0, 15)) + beqG(Index(8, 8), Index(0, 15))))));
    JIGt.add(Index(0, 2), Index(0, 15), -(bS * (etantilqG * (bet(3) + bet(4)) + etabtilqG * (bet(7) + bet(8)) + etantil * (beqGt(Index(3, 3), Index(0, 15)) + beqGt(Index(4, 4), Index(0, 15))) + etabtil * (beqGt(Index(7, 7), Index(0, 15)) + beqGt(Index(8, 8), Index(0, 15))))));

    JIGt.add(Index(0, 2), Index(0, 15), -((etantil * (be(3) + be(4)) + etabtil * (be(7) + be(8))) * bSqGt));
    JIGt.add(Index(0, 2), Index(0, 15), -((etantilt * (be(3) + be(4)) + etabtilt * (be(7) + be(8))) * bSqG));
    JIGt.add(Index(0, 2), Index(0, 15), -((etantil * (bet(3) + bet(4)) + etabtil * (bet(7) + bet(8))) * bSqG));

    JIGt.mult(Index(0, 2), Index(0, 15), 0.5);

    JIGt.set(Index(15, 15), Index(0, 15), -cos(pS(1)) * pSt(1) * (beqG(Index(10, 10), Index(0, 15)) - beqG(Index(9, 9), Index(0, 15))) - ((bet(10) - bet(9)) * cos(pS(1)) - (be(10) - be(9)) * sin(pS(1)) * pSt(1)) * beqG(Index(1, 1), Index(0, 15)));
    JIGt.add(Index(15, 15), Index(0, 15), -sin(pS(1)) * (beqGt(Index(10, 10), Index(0, 15)) - beqGt(Index(9, 9), Index(0, 15))) - (be(10) - be(9)) * cos(pS(1)) * beqGt(Index(1, 1), Index(0, 15)));
    JIGt.div(Index(15, 15), Index(0, 15), l0);
  }

  void Trafo33RCM::computeTrafo(const Vec16& qG, const Vec16& qGt) {
    computeCOSYt(qG, qGt);
    computeJIGt(qGt);
  }
/*******************************************************************/

}

