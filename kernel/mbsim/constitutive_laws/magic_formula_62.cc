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
#include "magic_formula_62.h"
#include "mbsim/links/tyre_contact.h"
#include "mbsim/contours/tyre.h"
#include "mbsim/frames/contour_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, MagicFormula62)

  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  void MagicFormula62::importData() {
    ifstream file(inputDataFile);
    if(file.is_open()) {
      string str, line, name;
      do {
	getline(file,line);
	size_t found = line.find("[LONGITUDINAL_COEFFICIENTS]");
	if(found!=string::npos) {
	  string value[26];
	  for(int i=0; i<26; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  PCX1 = stod(value[0]);
	  PDX1 = stod(value[1]);
	  PDX2 = stod(value[2]);
	  PDX3 = stod(value[3]);
	  PEX1 = stod(value[4]);
	  PEX2 = stod(value[5]);
	  PEX3 = stod(value[6]);
	  PEX4 = stod(value[7]);
	  PKX1 = stod(value[8]);
	  PKX2 = stod(value[9]);
	  PKX3 = stod(value[10]);
	  PHX1 = stod(value[11]);
	  PHX2 = stod(value[12]);
	  PVX1 = stod(value[13]);
	  PVX2 = stod(value[14]);
	  PPX1 = stod(value[15]);
	  PPX2 = stod(value[16]);
	  PPX3 = stod(value[17]);
	  PPX4 = stod(value[18]);
	  RBX1 = stod(value[19]);
	  RBX2 = stod(value[20]);
	  RBX3 = stod(value[21]);
	  RCX1 = stod(value[22]);
	  REX1 = stod(value[23]);
	  REX2 = stod(value[24]);
	  RHX1 = stod(value[25]);
	}
	found = line.find("[OVERTURNING_COEFFICIENTS]");
	if(found!=string::npos) {
	  string value[15];
	  for(int i=0; i<15; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  QSX1  = stod(value[0]);
	  QSX2  = stod(value[1]);
	  QSX3  = stod(value[2]);
	  QSX4  = stod(value[3]);
	  QSX5  = stod(value[4]);
	  QSX6  = stod(value[5]);
	  QSX7  = stod(value[6]);
	  QSX8  = stod(value[7]);
	  QSX9  = stod(value[8]);
	  QSX10 = stod(value[9]);
	  QSX11 = stod(value[10]);
	  QSX12 = stod(value[11]);
	  QSX13 = stod(value[12]);
	  QSX14 = stod(value[13]);
	  PPMX1 = stod(value[14]);
	}
	found = line.find("[LATERAL_COEFFICIENTS]");
	if(found!=string::npos) {
	  string value[42];
	  for(int i=0; i<42; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  PCY1 = stod(value[0]);
	  PDY1 = stod(value[1]);
	  PDY2 = stod(value[2]);
	  PDY3 = stod(value[3]);
	  PEY1 = stod(value[4]);
	  PEY2 = stod(value[5]);
	  PEY3 = stod(value[6]);
	  PEY4 = stod(value[7]);
	  PEY5 = stod(value[8]);
	  PKY1 = stod(value[9]);
	  PKY2 = stod(value[10]);
	  PKY3 = stod(value[11]);
	  PKY4 = stod(value[12]);
	  PKY5 = stod(value[13]);
	  PKY6 = stod(value[14]);
	  PKY7 = stod(value[15]);
	  PHY1 = stod(value[16]);
	  PHY2 = stod(value[17]);
	  PVY1 = stod(value[18]);
	  PVY2 = stod(value[19]);
	  PVY3 = stod(value[20]);
	  PVY4 = stod(value[21]);
	  PPY1 = stod(value[22]);
	  PPY2 = stod(value[23]);
	  PPY3 = stod(value[24]);
	  PPY4 = stod(value[25]);
	  PPY5 = stod(value[26]);
	  RBY1 = stod(value[27]);
	  RBY2 = stod(value[28]);
	  RBY3 = stod(value[29]);
	  RBY4 = stod(value[30]);
	  RCY1 = stod(value[31]);
	  REY1 = stod(value[32]);
	  REY2 = stod(value[33]);
	  RHY1 = stod(value[34]);
	  RHY2 = stod(value[35]);
	  RVY1 = stod(value[36]);
	  RVY2 = stod(value[37]);
	  RVY3 = stod(value[38]);
	  RVY4 = stod(value[39]);
	  RVY5 = stod(value[40]);
	  RVY6 = stod(value[41]);
	}
	found = line.find("[ROLLING_COEFFICIENTS]");
	if(found!=string::npos) {
	  string value[8];
	  for(int i=0; i<8; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  QSY1 = stod(value[0]);
	  QSY2 = stod(value[1]);
	  QSY3 = stod(value[2]);
	  QSY4 = stod(value[3]);
	  QSY5 = stod(value[4]);
	  QSY6 = stod(value[5]);
	  QSY7 = stod(value[6]);
	  QSY8 = stod(value[7]);
	}
	found = line.find("[ALIGNING_COEFFICIENTS]");
	if(found!=string::npos) {
	  string value[33];
	  for(int i=0; i<33; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  QBZ1  = stod(value[0]);
	  QBZ2  = stod(value[1]);
	  QBZ3  = stod(value[2]);
	  QBZ4  = stod(value[3]);
	  QBZ5  = stod(value[4]);
	  QBZ9  = stod(value[5]);
	  QBZ10 = stod(value[6]);
	  QCZ1  = stod(value[7]);
	  QDZ1  = stod(value[8]);
	  QDZ2  = stod(value[9]);
	  QDZ3  = stod(value[10]);
	  QDZ4  = stod(value[11]);
	  QDZ6  = stod(value[12]);
	  QDZ7  = stod(value[13]);
	  QDZ8  = stod(value[14]);
	  QDZ9  = stod(value[15]);
	  QDZ10 = stod(value[16]);
	  QDZ11 = stod(value[17]);
	  QEZ1  = stod(value[18]);
	  QEZ2  = stod(value[19]);
	  QEZ3  = stod(value[20]);
	  QEZ4  = stod(value[21]);
	  QEZ5  = stod(value[22]);
	  QHZ1  = stod(value[23]);
	  QHZ2  = stod(value[24]);
	  QHZ3  = stod(value[25]);
	  QHZ4  = stod(value[26]);
	  PPZ1  = stod(value[27]);
	  PPZ2  = stod(value[28]);
	  SSZ1  = stod(value[29]);
	  SSZ2  = stod(value[30]);
	  SSZ3  = stod(value[31]);
	  SSZ4  = stod(value[32]);
	}
	found = line.find("[TURNSLIP_COEFFICIENTS]");
	if(found!=string::npos) {
	  string value[19];
	  for(int i=0; i<19; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  PDXP1 = stod(value[0]);
	  PDXP2 = stod(value[1]);
	  PDXP3 = stod(value[2]);
	  PKYP1 = stod(value[3]);
	  PDYP1 = stod(value[4]);
	  PDYP2 = stod(value[5]);
	  PDYP3 = stod(value[6]);
	  PDYP4 = stod(value[7]);
	  PHYP1 = stod(value[8]);
	  PHYP2 = stod(value[9]);
	  PHYP3 = stod(value[10]);
	  PHYP4 = stod(value[11]);
	  PECP1 = stod(value[12]);
	  PECP2 = stod(value[13]);
	  QDTP1 = stod(value[14]);
	  QCRP1 = stod(value[15]);
	  QCRP2 = stod(value[16]);
	  QBRP1 = stod(value[17]);
	  QDRP1 = stod(value[18]);
	}
      } while(not file.eof());
    }
    file.close();
  }

  void MagicFormula62::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(inputDataFile.empty()) 
	throwError("(MagicFormula62::init): Input data file must be defined.");
      importData();
    }
    TyreModel::init(stage, config);
  }

  void MagicFormula62::initPlot(vector<string> &plotColumns) {
    plotColumns.emplace_back("camber angle");
    plotColumns.emplace_back("rolling velocity");
    plotColumns.emplace_back("spin component of longitudinal velocity");
    plotColumns.emplace_back("longitudinal slip");
    plotColumns.emplace_back("cornering stiffness");
    plotColumns.emplace_back("relaxation length");
    plotColumns.emplace_back("slip angle");
    plotColumns.emplace_back("scrub radius");
  }

  void MagicFormula62::plot(vector<double> &plotVector) {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    plotVector.push_back(phi);
    plotVector.push_back(vRoll);
    plotVector.push_back(RvSx-vRoll);
    plotVector.push_back(slip);
    plotVector.push_back(Kyal);
    plotVector.push_back(sRelax);
    plotVector.push_back(slipAnglePT1);
    plotVector.push_back(rScrub);
  }

  void MagicFormula62::initializeUsingXML(DOMElement *element) {
    TyreModel::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"inputDataFileName");
    string str = X()%E(e)->getFirstTextChild()->getData();
    setInputDataFile(E(e)->convertPath(str.substr(1,str.length()-2)).string());
    e=E(element)->getFirstElementChildNamed(MBSIM%"cz");
    setcz(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"dz");
    setdz(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"Fz0");
    setFz0(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"dpi");
    setdpi(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"epsx");
    setepsx(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"epsk");
    setepsk(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"c1Rel");
    setc1Rel(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"c2Rel");
    setc2Rel(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"c3Rel");
    setc3Rel(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLongitudinalForce");
    if(e) setScaleFactorForLongitudinalForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLateralForce");
    if(e) setScaleFactorForLateralForce(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForAligningMoment");
    if(e) setScaleFactorForAligningMoment(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLongitudinalFricitionCoefficient");
    if(e) setScaleFactorForLongitudinalFricitionCoefficient(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLateralFricitionCoefficient");
    if(e) setScaleFactorForLateralFricitionCoefficient(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLongitudinalSlipStiffness");
    if(e) setScaleFactorForLongitudinalSlipStiffness(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForCorneringStiffness");
    if(e) setScaleFactorForCorneringStiffness(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForCamberStiffness");
    if(e) setScaleFactorForCamberStiffness(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForResidualTorque");
    if(e) setScaleFactorForResidualTorque(E(e)->getText<double>());
  }

  void MagicFormula62::updateGeneralizedForces() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double FLo, FLa, M;
    double FN = -cz*contact->evalGeneralizedRelativePosition()(0)-dz*contact->evalGeneralizedRelativeVelocity()(2);
    if(FN>0) {
      if(FN<1) FN = 1;
      RvSx = contact->getContourFrame(1)->evalOrientation().col(0).T()*contact->getContourFrame(1)->evalVelocity();
      vRoll = contact->evalForwardVelocity()(0);
      slip = -RvSx/vRoll;
      slipAnglePT1 = contact->getx()(0);
      phi = asin(tyre->getFrame()->getOrientation().col(1).T()*contact->getContourFrame(0)->getOrientation().col(2));
      double rCrown = tyre->getUnloadedRadius() - tyre->getRimRadius() + contact->getGeneralizedRelativePosition()(0);

      double dfz = (FN - Fz0)/Fz0;

      double Kxka = FN*(PKX1+PKX2*dfz)*exp(PKX3*dfz)*(1+PPX1*dpi+PPX2*pow(dpi,2))*sfkx;
      double muex = (PDX1+PDX2*dfz)*(1+PPX3*dpi+PPX4*pow(dpi,2))*(1-PDX3*pow(phi,2))*sfmux;
      double Cx = PCX1;
      double Dx = muex*FN;
      double Bx = Kxka/(Cx*Dx+epsx);
      double Ex =(PEX1+PEX2*dfz+PEX3*pow(dfz,2))*(1-PEX4*sgn(slip));
      double Svx = FN*(PVX1+PVX2*dfz);
      double Shx = PHX1+PHX2*dfz;
      double kx = slip + Shx;
      double Fx0 = Dx*sin(Cx*atan(Bx*kx-Ex*(Bx*kx-atan(Bx*kx)))) + Svx;
      double Bxa = (RBX1+RBX3*pow(phi,2))*cos(atan(RBX2*slip));
      double Cxa = RCX1;
      double Exa = REX1 + REX2*dfz;
      double Shxa = RHX1;
      double as = slipAnglePT1 + Shxa;
      double Gxa0 = cos(Cxa*atan(Bxa*Shxa-Exa*(Bxa*Shxa-atan(Bxa*Shxa))));
      double Gxa = cos(Cxa*atan(Bxa*as-Exa*(Bxa*as-atan(Bxa*as))))/Gxa0;
      FLo = Gxa*Fx0;

      double gas = phi;
      double Kyga = FN*(PKY6+PKY7*dfz)*(1+PPY5*dpi)*sfkg;
      Kyal = sfky*PKY1*Fz0*(1+PPY1*dpi)*(1-PKY3*abs(gas))*sin(PKY4*atan(FN/Fz0/((PKY2+PKY5*pow(gas,2))*(1+PPY2*dpi))));
      double muey = (PDY1+PDY2*dfz)*(1+PPY3*dpi+PPY4*pow(dpi,2))*(1-PDY3*pow(gas,2))*sfmuy;
      double Svyga = FN*(PVY3+PVY4*dfz)*gas;
      double Svy = FN*(PVY1+PVY2*dfz) + Svyga;
      double Shy = (PHY1+PHY2*dfz)+(Kyga*gas-Svyga)/(Kyal+epsk);
      double ay = slipAnglePT1 + Shy;
      double Cy = PCY1;
      double Dy = muey*FN;
      double Ey = (PEY1+PEY2*dfz)*(1+PEY5*pow(gas,2)-(PEY3+PEY4*gas)*sgn(ay));
      double By = Kyal/(Cy*Dy+epsk);
      double Fy0 = Dy*sin(Cy*atan(By*ay-Ey*(By*ay-atan(By*ay)))) + Svy;
      double Byk = (RBY1+RBY4*pow(gas,2))*cos(atan(RBY2*(slipAnglePT1-RBY3)));
      double Cyk = RCY1;
      double Dvyk = muey*FN*(RVY1+RVY2*dfz+RVY3*gas)*cos(atan(RVY4*slipAnglePT1));
      double Eyk = REY1 + REY2*dfz;
      double Svyk = Dvyk*sin(RVY5*atan(RVY6*slip));
      double Shyk = RHY1+RHY2*dfz;
      double ks = slip + Shyk;
      double Gyk0 = cos(Cyk*atan(Byk*Shyk-Eyk*(Byk*Shyk-atan(Byk*Shyk))));
      double Gyk = cos(Cyk*atan(Byk*ks-Eyk*(Byk*ks-atan(Byk*ks))))/Gyk0;
      FLa = Gyk*Fy0 + Svyk;

      gas = sin(phi);
      double Kyal0 = PKY1*Fz0*(1+PPY1*dpi)*sin(PKY4*atan(FN/Fz0/(PKY2*(1+PPY2*dpi))))*sfky;
      double muey0 = (PDY1+PDY2*dfz)*(1+PPY3*dpi+PPY4*pow(dpi,2))*sfmuy;
      double Svyga0 = 0;
      double Svy0 = FN*(PVY1+PVY2*dfz) + Svyga0;
      double Shy0 = (PHY1+PHY2*dfz)-Svyga0/(Kyal0+epsk);
      double ay0 = slipAnglePT1 + Shy0;
      double Cy0 = PCY1;
      double Dy0 = muey0*FN;
      double Ey0 = (PEY1+PEY2*dfz)*(1-PEY3*sgn(ay0));
      double By0 = Kyal0/(Cy0*Dy0+epsk);
      Fy0 = Dy0*sin(Cy0*atan(By0*ay0-Ey0*(By0*ay0-atan(By0*ay0)))) + Svy0;
      double Byk0 = RBY1*cos(atan(RBY2*(slipAnglePT1-RBY3)));
      double Cyk0 = RCY1;
      //double Dvyk0 = muey0*FN*(RVY1+RVY2*dfz)*cos(atan(RVY4*slipAnglePT1)); // TODO
      double Eyk0 = REY1 + REY2*dfz;
      //double Svyk0 = Dvyk0*sin(RVY5*atan(RVY6*ka)); // TODO
      double Shyk0 = RHY1+RHY2*dfz;
      double ks0 = slip + Shyk0;
      double Gyk00 = cos(Cyk0*atan(Byk0*Shyk0-Eyk0*(Byk0*Shyk0-atan(Byk0*Shyk0))));
      Gyk0 = cos(Cyk0*atan(Byk0*ks0-Eyk0*(Byk0*ks0-atan(Byk0*ks0))))/Gyk00;
      double Kyals = Kyal+epsk;
      Svyga = FN*(PVY3+PVY4*dfz)*gas;
      Svy = FN*(PVY1+PVY2*dfz) + Svyga;
      Shy = (PHY1+PHY2*dfz)+(Kyga*gas-Svyga)/Kyals;
      double Sht = QHZ1+QHZ2*dfz+(QHZ3+QHZ4*dfz)*gas;
      double Shf = Shy + Svy/Kyals;
      double alt = slipAnglePT1+Sht;
      double alr = slipAnglePT1+Shf;
      double Bt = (QBZ1+QBZ2*dfz+QBZ3*pow(dfz,2))*(1+QBZ5*abs(gas)+QBZ4*gas);
      double Ct = QCZ1;
      double Dt0 = FN*rCrown/Fz0*(QDZ1 + QDZ2*dfz)*(1-PPZ1*dpi);
      double Dt = Dt0*(1+QDZ3*abs(gas)+QDZ4*pow(gas,2));
      double Et = (QEZ1+QEZ2*dfz+QEZ3*pow(dfz,2))*(1+(QEZ4+QEZ5*gas)*2./M_PI*atan(Bt*Ct*alt));
      muey = (PDY1+PDY2*dfz)*(1+PPY3*dpi+PPY4*pow(dpi,2))*(1-PDY3*pow(gas,2))*sfmuy;
      Dy = muey*FN;
      double Br = (QBZ9+QBZ10*By*Cy);
      double Cr = 1;
      double Dr = FN*rCrown*((QDZ6+QDZ7*dfz)+((QDZ8+QDZ9*dfz)*(1+PPZ2*dpi)+(QDZ10+QDZ11*dfz)*fabs(gas))*gas)*sfkm;
      //double Mzt0 =-Dt*cos(Ct*atan(Bt*alt-Et*(Bt*alt-atan(Bt*alt))))*Fy0; // TODO
      //double Mzr0 = Dr*cos(Cr*atan(Br*alr)); // TODO
      //double Mz0  = Mzt0+Mzr0; // TODO
      double lat = sqrt(pow(tan(alt),2)+pow(Kxka*slip/Kyal,2))*sgn(alt);
      double lar = sqrt(pow(tan(alr),2)+pow(Kxka*slip/Kyal,2))*sgn(alr);
      double t = Dt*cos(Ct*atan(Bt*lat-Et*(Bt*lat-atan(Bt*lat))));
      double Fys = Gyk0*Fy0;
      double Mzs = -t*Fys;
      double Mzr = Dr*cos(Cr*atan(Br*lar));
      M = Mzs+Mzr;

      sRelax = Kyal*(c1Rel + c2Rel*contact->getForwardVelocity()(0) + c3Rel*pow(contact->getForwardVelocity()(0),2));

      rScrub = rCrown*sin(phi);
    }
    else {
      FN = 0;
      FLo = 0;
      FLa = 0;
      M = 0;
      phi = 0;
      vRoll = 0;
      RvSx = 0;
      slip = 0;
      Kyal = 0;
      sRelax = 1;
      slipAnglePT1 = 0;
      rScrub = 0;
    }

    contact->getGeneralizedForce(false)(0) = sfFLo*FLo;
    contact->getGeneralizedForce(false)(1) = sfFLa*FLa;
    contact->getGeneralizedForce(false)(2) = FN;
    contact->getGeneralizedForce(false)(3) = sfM*M;

    contact->getsRelax(false) = 1;
  }

  VecV MagicFormula62::getData() const {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    VecV data(8,NONINIT);
    data(0) = phi;
    data(1) = vRoll;
    data(2) = RvSx-vRoll;
    data(3) = slip;
    data(4) = Kyal;
    data(5) = sRelax;
    data(6) = slipAnglePT1;
    data(7) = rScrub;
    return data;
  }

}
