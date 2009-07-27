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
 * Contact: schneidm@users.berlios.de
 */

#include "hydline_galerkin.h"
#include "environment.h"
#include "mbsim/utils/ansatz_functions.h"
#include "mbsim/utils/utils.h"

#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace fmatvec;

namespace MBSim {

  HydLineGalerkin::HydLineGalerkin(const string &name) : HydLineAbstract(name), mdim(0), plotdim(0), g(0), E(0), k(0), WInt(Vec(0)), wA(Vec(0)), wE(Vec(0)), lambda(0), MatIntWWT(SymMat(0)), MatIntWSWST(SymMat(0)), K(SymMat(0)), D(SymMat(0)), N(SymMat(0)), Omega(SymMat(0)), phi(SqrMat(0,0)), ansatz(NULL), plotVecW(Mat(0,0)), plotVecWS(Mat(0,0)), QIn(1), QOut(1), Flow2D(false), nAnsatz(0), p0(0), Q0(0), fracAir(0), delta_h(0), DLehr(0), relPlotPoints(Vec(0)) {
  }

  HydLineGalerkin::~HydLineGalerkin() {
  }

  void HydLineGalerkin::setAnsatzFunction(AnsatzTypes method_, int nAnsatz_) {
    if (l<=1e-4) {
      cout << "set length first!" << endl;
      throw(123);
    }
    switch (method_) {
      case BSplineOrd4:
        ansatz = new ansatz_function_BSplineOrd4(nAnsatz_, l);
        break;
      case BSplineOrd3:
        ansatz = new ansatz_function_BSplineOrd3(nAnsatz_, l);
        break;
      case Polynom:
        ansatz = new ansatz_function_polynom(nAnsatz_, l);
        break;
      case Harmonic:
        ansatz = new ansatz_function_harmonic(nAnsatz_, l);
        break;
    }

    mdim=ansatz->dim();
    WInt = ansatz->VecIntW();
    wA = ansatz->VecW0();
    wE = ansatz->VecWL();
    MatIntWWT=ansatz->MatIntWWT();
    MatIntWSWST=ansatz->MatIntWSWST();
  };

  void HydLineGalerkin::calcqSize() {
    qSize=mdim;
  }

  void HydLineGalerkin::calcuSize(int j) {
    uSize[j]=mdim;
  }

  void HydLineGalerkin::init() {
    HydLineAbstract::init();

    double nu=HydraulicEnvironment::getInstance()->getNu();
    g=0; // TODO consider gravity
    E=HydraulicEnvironment::getInstance()->getE(p0, fracAir);

    k=rho*g*delta_h/l;

    MFac=Area*rho*MatIntWWT;
    K=Area*E*MatIntWSWST;

    phi.resize(mdim, mdim);
    lambda.resize(mdim);
    if (eigvec(K, MFac, phi, lambda)) {
      cout << getName() << ": Fehler bei Eigenvektorberechnung!" << endl;
      throw 821;
    }

    Omega.resize(mdim, INIT, 0);
    for (int i=1; i<mdim; i++) // analytische Loesung unabhaengig vom Ansatztyp --> omega(0)=0
      Omega(i,i)=sqrt(lambda(i));

    N.resize(mdim, INIT, 0);
    if (Flow2D) {
      for (int i=0; i<mdim; i++) {
        double kH=sqrt(Omega(i,i)/nu)*sqrt(Area/M_PI);
        N(i,i)=(kH<5.)?1.+0.0024756*pow(kH, 3.0253322):0.44732718+0.175*kH;
      }
    }

    Mat DTmp(mdim, mdim, INIT, 0);
    if (!Flow2D)
      DTmp=8*rho*M_PI*nu*MatIntWWT*SymMat(mdim, EYE);
    else
      DTmp=8*rho*M_PI*nu*MatIntWWT*phi*N*inv(phi);
    if (DLehr>0)
      DTmp += 2.*DLehr*inv(trans(phi))*Omega*trans(phi)*MFac;
    D.resize(mdim, INIT, 0);
    for (int i=0; i<mdim; i++)
      for (int j=0; j<mdim; j++)
        D(i,j)=DTmp(i,j);

    plotdim=relPlotPoints.size();
    plotVecW.resize(mdim, plotdim);
    plotVecWS.resize(mdim, plotdim);
    for (int i=0; i<plotdim; i++) {
      plotVecW.col(i)=ansatz->VecW(relPlotPoints(i));
      plotVecWS.col(i)=ansatz->VecWS(relPlotPoints(i));
    }
    delete ansatz;

    setInitialGeneralizedVelocity(inv(MatIntWWT)*WInt*Q0); 
  }

  void HydLineGalerkin::updateT(double t) {
    T=SqrMat(mdim, EYE);
  }

  void HydLineGalerkin::updateM(double t) {
    M=MFac;
  }

  void HydLineGalerkin::updateStateDependentVariables(double t) {
    QIn(0)=Area*trans(wA)*u;
    QOut(0)=-Area*trans(wE)*u;
  }

  void HydLineGalerkin::updateh(double t) {
    HydLineAbstract::updateh(t);
    h = (-k*WInt + p0*(wE-wA))*Area - D*u - K*q;
  }

  void HydLineGalerkin::plot(double t, double dt) {
    for (int i=0; i<plotdim; i++)
      plotVector.push_back(Area*trans(u)*plotVecW.col(i)*6e4);
    for (int i=0; i<plotdim; i++)
      plotVector.push_back((-E*trans(q)*plotVecWS.col(i)+p0)*1e-5);
    HydLineAbstract::plot(t,dt);
  }

  void HydLineGalerkin::initPlot() {
    for (int i=0; i<plotdim; i++)
      plotColumns.push_back("Q(x="+numtostr(relPlotPoints(i)*l)+") [l/min]");
    for (int i=0; i<plotdim; i++)
      plotColumns.push_back("p(x="+numtostr(relPlotPoints(i)*l)+") [bar]");
    HydLineAbstract::initPlot();
  }

  //  void HydLineGalerkin::plotParameters() {
  //    HydLinkInertiaAbstract::plotParameters();
  //    parafile << "mdim=" << mdim << endl;
  //    parafile << "rho=" << rho << endl;
  //    parafile << "g=" << g << endl;
  //    parafile << "E=" << E << endl;
  //    parafile << "k=" << k << endl;
  //    parafile << "WInt=" << WInt << endl;
  //    parafile << "wA=" << wA << endl;
  //    parafile << "wE=" << wE << endl;
  //    parafile << "MatIntWWT=" << MatIntWWT << endl;
  //    parafile << "MatIntWSWST=" << MatIntWSWST << endl;
  //    parafile << "M=" << M << endl;
  //    parafile << "K=" << K << endl;
  //    parafile << "D=" << D << endl;
  //    parafile << "lambda=" << lambda << endl;
  //    parafile << "Omega=" << Omega << endl;
  //    parafile << "phi=" << phi << endl;
  //    parafile << "N=" << N << endl;
  //    parafile << "plotVecW=" << plotVecW << endl;
  //    parafile << "plotVecWS=" << plotVecWS << endl;
  //  }

}
