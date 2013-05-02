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
#include "mbsimHydraulics/elastic_line_variational.h"
#include "mbsimHydraulics/objectfactory.h"
#include "environment.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

#include "mbsimHydraulics/hnode.h"

#include "mbsim/dynamic_system_solver.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimHydraulics {

  ElasticLineVariational::ElasticLineVariational(const string &name) : HLine(name), p0(0), fracAir(0), r(0), l(0), relPlotPoints(0), window_function_type(BlackmanHarris), n(9), QIn(1), QOut(1), wO(0), wI(0), hq(0), hu(0), hp0(0), cu(0), y(0), Tlocal(0,0), relPlot(0,0), Mlocal(0) {
  }

  void ElasticLineVariational::init(InitStage stage) {
    if (stage==MBSim::resize) {
      HLine::init(stage);

      const double E0=HydraulicEnvironment::getInstance()->getBasicBulkModulus();
      const double kappa=HydraulicEnvironment::getInstance()->getKappa();
      const double pinf=HydraulicEnvironment::getInstance()->getEnvironmentPressure();
      OilBulkModulus bulkModulus(name, E0, pinf, kappa, fracAir);
      const double E=bulkModulus(p0);
      const double rho=HydraulicEnvironment::getInstance()->getSpecificMass();
      const double nu=HydraulicEnvironment::getInstance()->getKinematicViscosity();

      const double speedOfSound=sqrt(E/rho);
      const double waveTime=l/speedOfSound;
      const double epsilon=8*nu*waveTime/r/r;
      const double Z0=rho*speedOfSound/(M_PI*r*r);

      SqrMat AGes(2*n-1, INIT, 0);
      Mat BGes(2*n-1, 2, INIT, 0);
      Mat CGes(2+n, 2*n-1, INIT, 0);
      AGes(0,0)=-epsilon/waveTime; 
      BGes(0,0)=1.; 
      CGes(0,0)=2./waveTime; 
      CGes(2,0)=2./waveTime;
      for (int i=0; i<n-2; i+=2) {
        const double e0=sqrt((i+1)*M_PI*epsilon)/2.+7.*epsilon/16.;
        const double e1=sqrt((i+2)*M_PI*epsilon)/2.+7.*epsilon/16.;
        const double omega0=(i+1)*M_PI-sqrt((i+1)*M_PI*epsilon)/4.+epsilon/16.;
        const double omega1=(i+2)*M_PI-sqrt((i+2)*M_PI*epsilon)/4.+epsilon/16.;
        double k0; 
        double k1;
        switch (window_function_type) {
          case None:
            k0=1.;
            k1=1.;
            break;
          case Hann:
            k0=cos((i+1)*M_PI/double(n)/2.)*cos((i+1)*M_PI/double(n)/2.);
            k1=cos((i+2)*M_PI/double(n)/2.)*cos((i+2)*M_PI/double(n)/2.);
            break;
          case Hamming:
            k0=.54+.46*cos((i+1)*M_PI/double(n));
            k1=.54+.46*cos((i+2)*M_PI/double(n));
            break;
          case Riemann:
            k0= n/((i+1)*M_PI)*sin((i+1)*M_PI/double(n));
            k1= n/((i+2)*M_PI)*sin((i+2)*M_PI/double(n));
            break;
          case BlackmanHarris:
            k0=.42323+.49755*cos((i+1)*M_PI/double(n))+.07922*cos(2*(i+1)*M_PI/double(n));
            k1=.42323+.49755*cos((i+2)*M_PI/double(n))+.07922*cos(2*(i+2)*M_PI/double(n));
            break;
          default:
            k0=1.;
            k1=1.;
        }

        // symetric flow tranfer functions
        AGes(2*i+1, 2*i+2)=1.; 
        AGes(2*i+2, 2*i+1)=-(omega0*omega0)/waveTime/waveTime; 
        AGes(2*i+2, 2*i+2)=-e0/waveTime; 
        BGes(2*i+2, 1)=1.; 
        CGes(1, 2*i+2)=4.*k0/waveTime; 
        CGes(i+3, 2*i+2)=4.*k0/waveTime;

        // asymetric flow tranfer function
        AGes(2*i+3, 2*i+4)=1.; 
        AGes(2*i+4, 2*i+3)=-(omega1*omega1)/waveTime/waveTime; 
        AGes(2*i+4, 2*i+4)=-e1/waveTime; 
        BGes(2*i+4, 0)=1.; 
        CGes(0, 2*i+4)=4.*k1/waveTime; 
        CGes(i+4, 2*i+4)=4.*k1/waveTime;
      }

      const Vec b1Ges=BGes.col(0).copy();
      const Vec b2Ges=BGes.col(1).copy();
      BGes.col(0)=.5*(b1Ges+b2Ges);
      BGes.col(1)=.5*(-b1Ges+b2Ges);

      const RowVec c1Ges=CGes.row(0).copy();
      const RowVec c2Ges=CGes.row(1).copy();
      CGes.row(0)=(c1Ges+c2Ges)/Z0;
      CGes.row(1)=(-c1Ges+c2Ges)/Z0;

      // resort
      SqrMat AA(2*n-1, INIT, 0);
      Mat BB(2*n-1, 2, INIT, 0);
      Mat CC(2+n, 2*n-1, INIT, 0);
      AA(n-1, n-1)=AGes(0,0);
      BB.row(n-1)=BGes.row(0);
      CC.col(n-1)=CGes.col(0);
      for (int i=0; i<n-1; i++) {
        AA(i, n+i)=AGes(2*i+1, 2*i+2);
        AA(n+i, i)=AGes(2*i+2, 2*i+1);
        AA(n+i, n+i)=AGes(2*i+2, 2*i+2);
        BB.row(n+i)=BGes.row(2*i+2);
        CC.col(n+i)=CGes.col(2*i+2);
      }
      AGes=AA;
      BGes=BB;
      CGes=CC;

      // adapt system, such B=C(0:1,:)'
      SqrMat FF(2*n-1, EYE);
      for (int i=n-1; i<(2*n-1); i++)
        FF(i,i)=sqrt(BGes(i, 0)/CGes(0, i));
      AGes=inv(FF)*AGes*FF;
      BGes=inv(FF)*BGes;
      CGes=CGes*FF;

      assert(nrm2(trans(CGes.row(0))-BGes.col(0))<epsroot());
      assert(nrm2(trans(CGes.row(1))-BGes.col(1))<epsroot());

      Tlocal.resize(n-1, n, INIT, 0);
      for (int i=0; i<n-1; i++)
        Tlocal(i, i+1)=AGes(i, n+i);

      Mlocal.resize(n, EYE);

      hq.resize(n-1, INIT, 0);
      for (int i=0; i<n-1; i++)
        hq(i)=AGes(n+i, i);

      hu.resize(n, INIT, 0);
      for (int i=0; i<n; i++)
        hu(i)=AGes(n-1+i, n-1+i);

      wO.resize(n, INIT, 0); // p0 (line added as outflow to a node, factor at x=0)
      for (int i=0; i<n; i++)
        wO(i)=CGes(0, n-1+i);

      wI.resize(n, INIT, 0); // p1 (line added as inflow to a node, factor at x=L)
      for (int i=0; i<n; i++)
        wI(i)=CGes(1, n-1+i);

      cu.resize(n, INIT, 0);
      for (int i=0; i<n; i++)
        cu(i)=CGes(2+i, n-1+i);

      hp0=(-wO-wI)*p0;

      Jacobian.resize(n, n, INIT, 0);
      for (int i=0; i<n; i++)
        Jacobian(i, i)=1.;

      y.resize(n, INIT, 0);

      relPlot.resize(n, relPlotPoints.size());
      for (int j=0; j<relPlotPoints.size(); j++)
        for (int i=0; i<n; i++)
          relPlot(i, j)=sin(i*M_PI*relPlotPoints(j));
    }
    else if (stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        plotColumns.push_back("QIn [l/min]");
        plotColumns.push_back("QOut [l/min]");
        for (int i=0; i<relPlotPoints.size(); i++)
          plotColumns.push_back("p(x="+numtostr(relPlotPoints(i)*l)+") [bar]");
        if(getPlotFeature(state)==enabled)
          for (int i=0; i<y.size(); i++)
            plotColumns.push_back("y("+numtostr(i)+")");
        HLine::init(stage);
      }
    }
    else if (stage==MBSim::unknownStage) {
      HLine::init(stage);
      if (printStateSpace)
        doPrintStateSpace();
    }
    else
      HLine::init(stage);
  }

  void ElasticLineVariational::updateStateDependentVariables(double t) {
    QIn(0)=trans(wO)*u;
    QOut(0)=trans(wI)*u;
    for (int i=0; i<n; i++)
      y(i)=cu(i)*u(i);
  }

  void ElasticLineVariational::updateh(double t, int j) {
    HLine::updateh(t);
    h[j]=hp0.copy();
    for (int i=1; i<n; i++)
      h[j](i)+=hq(i-1)*q(i-1);
    for (int i=0; i<n; i++)
      h[j](i)+=hu(i)*u(i);
  }

  void ElasticLineVariational::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(QIn(0)*6e4);
      plotVector.push_back(QOut(0)*6e4);
      for (int i=0; i<relPlotPoints.size(); i++)
        plotVector.push_back((nFrom->getla()(0)*(1-relPlotPoints(i))+nTo->getla()(0)*relPlotPoints(i)+trans(y)*relPlot.col(i))*1e-5);
      if(getPlotFeature(state)==enabled)
        for (int i=0; i<y.size(); i++)
          plotVector.push_back(y(i));
      HLine::plot(t,dt);
    }
  }

  void ElasticLineVariational::initializeUsingXML(TiXmlElement * element) {
    Object::initializeUsingXML(element);
    TiXmlElement * e;
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"initialPressure");
    setp0(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"fracAir");
    setFracAir(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"diameter");
    setDiameter(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"length");
    setLength(getDouble(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"relativePlotPoints");
    if (e)
      setRelativePlotPoints(getVec(e));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"numberOfAnsatzFunctions");
    setNumberOfAnsatzFunctions((unsigned int)(getInt(e)));
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"windowNone");
    if (e)
      setWindowFunction(None);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"windowHann");
    if (e)
      setWindowFunction(Hann);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"windowHamming");
    if (e)
      setWindowFunction(Hamming);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"windowRiemann");
    if (e)
      setWindowFunction(Riemann);
    e=element->FirstChildElement(MBSIMHYDRAULICSNS"windowBlackmanHarris");
    if (e)
      setWindowFunction(BlackmanHarris);
    printLineStateSpace(element->FirstChildElement(MBSIMHYDRAULICSNS"printStateSpace"));
  }

  void ElasticLineVariational::doPrintStateSpace() {

    string lname=name;
    while (lname.find("/")!=string::npos)
      lname.replace(lname.find_first_of("/"), 1, "_");
    ofstream s(("stateSpace_" + lname + ".out").c_str()); 

    s.precision(numeric_limits<double>::digits10+1);
    s.setf(ios::scientific); 

    s << "Window=" << window_function_type << endl;

    s << "qd=T*u" << endl;
    s << "ud=h_p0 + hq*q + hu*u + wO*p0 + wI*p1" << endl;

    s << "mbsim_T=[";
    for (int i=0; i<n-1; i++)
      s << Tlocal(i, i+1) << ", ";
    s << "]; " << endl;
    s << "mbsim_p0=[";
    for (int i=0; i<hp0.size(); i++)
      s << hp0(i) << ", ";
    s << "]; " << endl;
    s << "mbsim_hq=[";
    for (int i=0; i<hq.size(); i++)
      s << hq(i) << ", ";
    s << "]; " << endl;
    s << "mbsim_hu=[";
    for (int i=0; i<hu.size(); i++)
      s << hu(i) << ", ";
    s << "]; " << endl;
    s << "mbsim_wO=[";
    for (int i=0; i<wO.size(); i++)
      s << wO(i) << ", ";
    s << "]; " << endl;
    s << "mbsim_wI=[";
    for (int i=0; i<wI.size(); i++)
      s << wI(i) << ", ";
    s << "]; " << endl;

    s.close();

  }

}
