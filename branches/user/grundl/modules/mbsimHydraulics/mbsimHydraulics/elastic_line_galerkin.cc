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
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsimHydraulics/elastic_line_galerkin.h"
#include "mbsimHydraulics/objectfactory.h"
#include "environment.h"
#include "mbsim/utils/ansatz_functions.h"
#include "mbsim/utils/utils.h"

#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimHydraulics {

  ElasticLineGalerkin::ElasticLineGalerkin(const string &name) : HLine(name), mdim(0), plotdim(0), g(0), E(0), k(0), WInt(), wA(), wE(), lambda(0), MatIntWWT(), MatIntWSWST(), K(), D(), N(), Omega(), phi(), ansatz(NULL), plotVecW(), plotVecWS(), QIn(1), QOut(1), l(0), d(0), Area(0), Flow2D(false), nAnsatz(0), p0(0), Q0(0), fracAir(0), delta_h(0), DLehr(0), relPlotPoints() {
  }

  void ElasticLineGalerkin::setAnsatzFunction(AnsatzTypes method_, int nAnsatz_) {
    if (l<=1e-4)
      throw MBSimError("set length first");
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
  }

  void ElasticLineGalerkin::init(InitStage stage) {
    if (stage==MBSim::preInit) {
      HLine::init(stage);
      Area=M_PI*d*d/4.;
      if (direction.size()>0)
        g=trans(((DynamicSystem*)parent)->getFrame("I")->getOrientation()*MBSimEnvironment::getInstance()->getAccelerationOfGravity())*direction;
      else
        g=0;
      double E0=HydraulicEnvironment::getInstance()->getBasicBulkModulus();
      double kappa=HydraulicEnvironment::getInstance()->getKappa();
      double pinf=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
      OilBulkModulus bulkModulus(name, E0, pinf, kappa, fracAir);
      E=bulkModulus(p0);
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      k=rho*g*delta_h/l;
      MFac=Area*rho*MatIntWWT;
      K=Area*E*MatIntWSWST;
    }
    else if (stage==MBSim::resize) {
      HLine::init(stage);
      double nu=HydraulicEnvironment::getInstance()->getKinematicViscosity();
      double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      phi.resize(mdim);
      lambda.resize(mdim);
      
      Jacobian.resize(mdim, mdim, INIT, 0);
      for (int i=0; i<mdim; i++)
        Jacobian(i, i)=1.;
      
      if (eigvec(K, MFac, phi, lambda))
        throw MBSimError(name+": Fehler bei Eigenvektorberechnung!");
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
    }
    else if (stage==MBSim::plot) {
      if (relPlotPoints.size()>0) {
        setPlotFeature(globalPosition, enabled);
        setPlotFeature(globalVelocity, enabled);
      }
      updatePlotFeatures();
      if(getPlotFeature(plotRecursive)==enabled) {
        plotdim=relPlotPoints.size();
        plotVecW.resize(mdim, plotdim);
        plotVecWS.resize(mdim, plotdim);
        for (int i=0; i<plotdim; i++) {
          plotVecW.col(i)=ansatz->VecW(relPlotPoints(i));
          plotVecWS.col(i)=ansatz->VecWS(relPlotPoints(i));
        }
        delete ansatz;
      if (getPlotFeature(globalVelocity)==enabled)
        for (int i=0; i<plotdim; i++)
          plotColumns.push_back("Q(x="+numtostr(relPlotPoints(i)*l)+") [l/min]");
      if (getPlotFeature(globalPosition)==enabled)
        for (int i=0; i<plotdim; i++)
          plotColumns.push_back("p(x="+numtostr(relPlotPoints(i)*l)+") [bar]");
        HLine::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      HLine::init(stage);
      setInitialGeneralizedVelocity(inv(MatIntWWT)*WInt*Q0); 
      //      plotParameters();
    }
    else
      HLine::init(stage);

  }

  void ElasticLineGalerkin::updateT(double t) {
    T=SqrMat(mdim, EYE);
  }

  void ElasticLineGalerkin::updateM(double t, int j) {
    M[j]=MFac;
  }

  void ElasticLineGalerkin::updateStateDependentVariables(double t) {
    QIn(0)=Area*trans(wA)*u;
    QOut(0)=-Area*trans(wE)*u;
  }

  void ElasticLineGalerkin::updateh(double t, int j) {
    HLine::updateh(t);
    h[j] = (-k*WInt + p0*(wE-wA))*Area - D*u - K*q;
  }

  void ElasticLineGalerkin::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      if (getPlotFeature(globalVelocity)==enabled)
        for (int i=0; i<plotdim; i++)
          plotVector.push_back(Area*trans(u)*plotVecW.col(i)*6e4);
      if (getPlotFeature(globalPosition)==enabled)
        for (int i=0; i<plotdim; i++)
          plotVector.push_back((-E*trans(q)*plotVecWS.col(i)+p0)*1e-5);
      HLine::plot(t,dt);
    }
  }

  void ElasticLineGalerkin::plotParameters() {
    cout << "mdim=" << mdim << endl;
    cout << "g=" << g << endl;
    cout << "E=" << E << endl;
    cout << "k=" << k << endl;
    cout << "WInt=" << WInt << endl;
    cout << "wA=" << wA << endl;
    cout << "wE=" << wE << endl;
    cout << "MatIntWWT=" << MatIntWWT << endl;
    cout << "MatIntWSWST=" << MatIntWSWST << endl;
    cout << "M=" << MFac << endl;
    cout << "K=" << K << endl;
    cout << "D=" << D << endl;
    cout << "lambda=" << lambda << endl;
    cout << "Omega=" << Omega << endl;
    cout << "phi=" << phi << endl;
    cout << "N=" << N << endl;
    cout << "plotVecW=" << plotVecW << endl;
    cout << "plotVecWS=" << plotVecWS << endl;
  }

  void ElasticLineGalerkin::initializeUsingXML(TiXmlElement * element) {
    HLine::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"initialPressure");
    setp0(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fracAir");
    setFracAir(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"heightDifference");
    setdh(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"dLehr");
    setDLehr(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setDiameter(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLength(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"AnsatzFunction");
    TiXmlElement * ee = e->FirstChildElement();
    if (ee->ValueStr()==MBSIMHYDRAULICSNS"BSplineOrder3")
      setAnsatzFunction(BSplineOrd3, getInt(ee->NextSiblingElement()));
    else if (ee->ValueStr()==MBSIMHYDRAULICSNS"BSplineOrder4")
      setAnsatzFunction(BSplineOrd4, getInt(ee->NextSiblingElement()));
    else if (ee->ValueStr()==MBSIMHYDRAULICSNS"Polynom")
      setAnsatzFunction(Polynom, getInt(ee->NextSiblingElement()));
    else if (ee->ValueStr()==MBSIMHYDRAULICSNS"Harmonic")
      setAnsatzFunction(Harmonic, getInt(ee->NextSiblingElement()));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"flow2d");
    if (e)
      setFlow2D(true);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"relativePlotPoints");
    setRelativePlotPoints(getVec(e));
  }

}
