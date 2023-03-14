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
	size_t found = line.find("[MODEL]");
	if(found!=string::npos) {
	  string value[5];
	  for(int i=0; i<5; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  v0 = stod(value[2]);
	}
	found = line.find("[DIMENSION]");
	if(found!=string::npos) {
	  string value[5];
	  for(int i=0; i<5; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  R0 = stod(value[0]);
	  rRim = stod(value[2]);
	}
	found = line.find("[OPERATING_CONDITIONS]");
	if(found!=string::npos) {
	  string value[2];
	  for(int i=0; i<2; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  if(p<0) p = stod(value[0]);
	  if(p0<0) p0 = stod(value[1]);
	}
	found = line.find("[VERTICAL]");
	if(found!=string::npos) {
	  string value[25];
	  for(int i=0; i<25; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  if(Fz0<0) Fz0 = stod(value[0]);
	  if(cz<0) cz = stod(value[1]);
	  if(dz<0) dz = stod(value[2]);
	}
	found = line.find("[STRUCTURAL]");
	if(found!=string::npos) {
	  string value[22];
	  for(int i=0; i<22; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  Cx0 = stod(value[0]);
	  Cy0 = stod(value[1]);
	  PCFX1 = stod(value[15]);
	  PCFX2 = stod(value[16]);
	  PCFX3 = stod(value[17]);
	  PCFY1 = stod(value[18]);
	  PCFY2 = stod(value[19]);
	  PCFY3 = stod(value[20]);
	  PCMZ1 = stod(value[21]);
	}
	found = line.find("[SCALING_COEFFICIENTS]");
	if(found!=string::npos) {
	  string value[25];
	  for(int i=0; i<25; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  LFZ0 = stod(value[0]);
	  LCX = stod(value[1]);
	  if(LMUX<0) LMUX = stod(value[2]);
	  LEX = stod(value[3]);
	  if(LKX<0) LKX = stod(value[4]);
	  LHX = stod(value[5]);
	  LVX = stod(value[6]);
	  LCY = stod(value[7]);
	  if(LMUY<0) LMUY = stod(value[8]);
	  LEY = stod(value[9]);
	  if(LKY<0) LKY = stod(value[10]);
	  if(LKYC<0) LKYC = stod(value[11]);
	  if(LKZC<0) LKZC = stod(value[12]);
	  LHY = stod(value[13]);
	  LVY = stod(value[14]);
	  LTR = stod(value[15]);
	  LRES = stod(value[16]);
	  LXAL = stod(value[17]);
	  LYKA = stod(value[18]);
	  LVYKA = stod(value[19]);
	  LS = stod(value[20]);
	  LMX = stod(value[21]);
	  LVMX = stod(value[22]);
	  LMY = stod(value[23]);
	}
	found = line.find("[LONGITUDINAL_COEFFICIENTS]");
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
    else
      throw runtime_error("(MagicFormula62::init): Input data file does not exist.");
    file.close();
  }

  void MagicFormula62::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(inputDataFile.empty()) 
	throw runtime_error("(MagicFormula62::init): Input data file must be defined.");
      importData();
    }
    else if(stage==unknownStage) {
      TyreContact *contact = static_cast<TyreContact*>(parent);
      Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
      dpi = (p-p0)/p0;
      constsix = six>=0;
      constsiy = siy>=0;
      if(mck) rRim = tyre->getRimRadius();
      if(fabs(tyre->getUnloadedRadius()-R0)>1e-13)
	msg(Warn) << "Unloaded radius of " << tyre->getPath() << " (" << tyre->getUnloadedRadius() << ") is different to unloaded radius of " << inputDataFile << " (" << R0 << ")." << endl;
//      if(fabs(tyre->getRimRadius()-rRim)>1e-13)
//	msg(Warn) << "Rim radius of " << tyre->getPath() << " (" << tyre->getRimRadius() << ") is different to rim radius of " << inputDataFile << " (" << rRim << ")." << endl;
    }
    TyreModel::init(stage, config);
  }

  void MagicFormula62::initPlot(vector<string> &plotColumns) {
    plotColumns.emplace_back("camber angle");
    plotColumns.emplace_back("rolling velocity");
    plotColumns.emplace_back("spin component of longitudinal velocity");
    plotColumns.emplace_back("longitudinal slip");
    plotColumns.emplace_back("cornering stiffness");
    plotColumns.emplace_back("relaxation length for longitudinal slip");
    plotColumns.emplace_back("relaxation length for sideslip");
    plotColumns.emplace_back("slip angle");
    plotColumns.emplace_back("scrub radius");
  }

  void MagicFormula62::plot(vector<double> &plotVector) {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    plotVector.push_back(ga);
    plotVector.push_back(vx);
    plotVector.push_back(vsx-vx);
    plotVector.push_back(ka);
    plotVector.push_back(Kyal);
    plotVector.push_back(six);
    plotVector.push_back(siy);
    plotVector.push_back(alF);
    plotVector.push_back(Rs);
  }

  void MagicFormula62::initializeUsingXML(DOMElement *element) {
    TyreModel::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"inputDataFileName");
    string str = X()%E(e)->getFirstTextChild()->getData();
    setInputDataFile(E(e)->convertPath(str.substr(1,str.length()-2)).string());
    e=E(element)->getFirstElementChildNamed(MBSIM%"motorcycleKinematics");
    if(e) setMotorcycleKinematics(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"inflationPressure");
    if(e) setInflationPressure(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"verticalStiffness");
    if(e) setVerticalStiffness(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"verticalDamping");
    if(e) setVerticalDamping(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relaxationLengthForLongitudinalSlip");
    if(e) setRelaxationLengthForLongitudinalSlip(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"relaxationLengthForSideslip");
    if(e) setRelaxationLengthForSideslip(E(e)->getText<double>());
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForLateralForceCamberStiffness");
    if(e) setScaleFactorForCamberStiffness(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForAligningMomentCamberStiffness");
    if(e) setScaleFactorForResidualTorque(E(e)->getText<double>());
  }

  int MagicFormula62::getxSize() const {
    return (six!=0) + (siy!=0);
  }

  void MagicFormula62::updatexd() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    int i=0;
    if(six!=0) {
      contact->getxd(false)(i) = (-vsx - contact->getx()(i)*vx)/six;
      i++;
    }
    if(siy!=0)
      contact->getxd(false)(i) = (atan(vsy/vx) - contact->getx()(i))*vx/siy; // original MF62: (vsy - contact->getx()(0)*vx)/sigy
  }

  void MagicFormula62::updateGeneralizedForces() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double Fx, Fy, Mx, My, Mz;
    Fz0 *= LFZ0;
    double Fz = -cz*contact->evalGeneralizedRelativePosition()(0)-dz*contact->evalGeneralizedRelativeVelocity()(2);
    vsx = contact->getContourFrame(1)->evalOrientation().col(0).T()*contact->getContourFrame(1)->evalVelocity();
    vsy = contact->getContourFrame(1)->getOrientation().col(1).T()*contact->getContourFrame(1)->getVelocity();
    vx = contact->evalForwardVelocity()(0);
    if(Fz>0) {
      if(Fz<1) Fz = 1;
      double dfz = (Fz-Fz0)/Fz0;
      int i = 0;
      double alM = atan(vsy/vx);
      ka = six!=0 ? contact->getx()(i++) : -vsx/vx;
      alF = siy!=0 ? contact->getx()(i) : alM;
      ga = asin(tyre->getFrame()->getOrientation().col(1).T()*contact->getContourFrame(0)->getOrientation().col(2));

      double Kxka = Fz*(PKX1+PKX2*dfz)*exp(PKX3*dfz)*(1+PPX1*dpi+PPX2*pow(dpi,2))*LKX;
      double muex = (PDX1+PDX2*dfz)*(1+PPX3*dpi+PPX4*pow(dpi,2))*(1-PDX3*pow(ga,2))*LMUX;
      double Cx = PCX1*LCX;
      double Dx = muex*Fz*zeta1;
      double Bx = Kxka/(Cx*Dx);
      double Svx = Fz*(PVX1+PVX2*dfz)*LVX*LMUX*zeta1;
      double Shx = (PHX1+PHX2*dfz)*LHX;
      double kx = ka+Shx;
      double Bxa = (RBX1+RBX3*pow(ga,2))*cos(atan(RBX2*ka))*LXAL;
      double Cxa = RCX1;
      double Exa = REX1+REX2*dfz;
      double Shxa = RHX1;
      double as = alF+Shxa;
      double Gxa0 = cos(Cxa*atan(Bxa*Shxa-Exa*(Bxa*Shxa-atan(Bxa*Shxa))));
      double Gxa = cos(Cxa*atan(Bxa*as-Exa*(Bxa*as-atan(Bxa*as))))/Gxa0;
      double Ex =(PEX1+PEX2*dfz+PEX3*pow(dfz,2))*(1-PEX4*sgn(kx))*LEX; // TODO <=1 prüfen?
      double Fx0 = Dx*sin(Cx*atan(Bx*kx-Ex*(Bx*kx-atan(Bx*kx))))+Svx;
      Fx = Gxa*Fx0*LFX;
      // TODO Schalter für combined slip und turn slip

      double Kyga0 = Fz*(PKY6+PKY7*dfz)*(1+PPY5*dpi)*LKYC;
      Kyal = PKY1*Fz0*(1+PPY1*dpi)*(1-PKY3*abs(ga))*sin(PKY4*atan(Fz/Fz0/((PKY2+PKY5*pow(ga,2))*(1+PPY2*dpi))))*LKY*zeta3;
      double muey = (PDY1+PDY2*dfz)*(1+PPY3*dpi+PPY4*pow(dpi,2))*(1-PDY3*pow(ga,2))*LMUY;
      double Svy0 = Fz*(PVY1+PVY2*dfz)*LVY*LMUY;
      double Svyga = Fz*(PVY3+PVY4*dfz)*ga*LKYC*LMUY*zeta2;
      double Svy = Svy0*zeta2+Svyga;
      double Shy0 = (PHY1+PHY2*dfz)*LHY;
      double Shyga = (Kyga0*ga-Svyga)/Kyal*zeta0+zeta4-1;
      double Shy = Shy0+Shyga;
      double ay = alF+Shy;
      double Cy = PCY1*LCY;
      double Dy = muey*Fz*zeta2;
      double Ey = (PEY1+PEY2*dfz)*(1+PEY5*pow(ga,2)-(PEY3+PEY4*ga)*sgn(ay))*LEY; // <=1 prüfen?
      double By = Kyal/(Cy*Dy);
      double Byk = (RBY1+RBY4*pow(ga,2))*cos(atan(RBY2*(alF-RBY3)))*LYKA;
      double Cyk = RCY1;
      double Dvyk = muey*Fz*(RVY1+RVY2*dfz+RVY3*ga)*cos(atan(RVY4*alF))*zeta2;
      double Eyk = REY1+REY2*dfz;
      double Svyk = Dvyk*sin(RVY5*atan(RVY6*ka))*LVYKA;
      double Shyk = RHY1+RHY2*dfz;
      double ks = ka+Shyk;
      double Gyk0 = cos(Cyk*atan(Byk*Shyk-Eyk*(Byk*Shyk-atan(Byk*Shyk))));
      double Gyk = cos(Cyk*atan(Byk*ks-Eyk*(Byk*ks-atan(Byk*ks))))/Gyk0;
      double Fy0 = Dy*sin(Cy*atan(By*ay-Ey*(By*ay-atan(By*ay))))+Svy;
      Fy = (Gyk*Fy0+Svyk)*LFY;
      // TODO Schalter für combined slip und turn slip

      Mx = mck?0:R0*Fz*LMX*(QSX1*LVMX-QSX2*ga*(1+PPMX1*dpi)+QSX3*Fy/Fz0+QSX4*cos(QSX5*atan(pow(QSX6*Fz/Fz0,2)))*sin(QSX7*ga+QSX8*atan(QSX9*Fy/Fz0))+QSX10*atan(QSX11*Fz/Fz0)*ga) + R0*LMX*(Fy*(QSX13+QSX14*fabs(ga))-Fz*QSX12*ga*fabs(ga));

      My = -R0*Fz0*LMY*(QSY1+QSY2*Fx/Fz0+QSY3*fabs(vx/v0)+QSY4*pow(vx/v0,4)+(QSY5+QSY6*Fz/Fz0)*pow(ga,2))*pow(Fz/Fz0,QSY7)*pow(p/p0,QSY8);

      double Kyal0 = PKY1*Fz0*(1+PPY1*dpi)*sin(PKY4*atan(Fz/Fz0/(PKY2*(1+PPY2*dpi))))*LKY*zeta3;
      double muey0 = (PDY1+PDY2*dfz)*(1+PPY3*dpi+PPY4*pow(dpi,2))*LMUY;
      double Shyga0 = zeta4-1;
      Shy0 = Shy0+Shyga0;
      double ay0 = alF+Shy0;
      double Dy0 = muey0*Fz*zeta2;
      double Ey0 = (PEY1+PEY2*dfz)*(1-PEY3*sgn(ay0))*LEY;
      double By0 = Kyal0/(Cy*Dy0);
      double Byk0 = RBY1*cos(atan(RBY2*(alF-RBY3)))*LYKA;
      double Sht = QHZ1+QHZ2*dfz+(QHZ3+QHZ4*dfz)*ga;
      double Shf = Shy+Svy/Kyal;
      double alt = alF+Sht;
      double alr = alF+Shf;
      double Bt = (QBZ1+QBZ2*dfz+QBZ3*pow(dfz,2))*(1+QBZ4*ga+QBZ5*abs(ga))*LKY/LMUY;
      double Ct = QCZ1;
      double Dt0 = Fz*R0/Fz0*(QDZ1+QDZ2*dfz)*(1-PPZ1*dpi);
      double Dt = Dt0*(1+QDZ3*ga+QDZ4*pow(ga,2))*LTR*zeta5;
      double Et = (QEZ1+QEZ2*dfz+QEZ3*pow(dfz,2))*(1+(QEZ4+QEZ5*ga)*2./M_PI*atan(Bt*Ct*alt));
      double Br = (QBZ9*LKY/LMUY+QBZ10*By*Cy)*zeta6;
      double Dr = Fz*R0*LMUY*cos(alM)*((QDZ6+QDZ7*dfz)*LRES*zeta2+((QDZ8+QDZ9*dfz)*(1+PPZ2*dpi)+(QDZ10+QDZ11*dfz)*fabs(ga))*ga*LKZC*zeta0)-zeta8+1;
      double lat = atan(sqrt(pow(tan(alt),2)+pow(Kxka*ka/Kyal,2)))*sgn(alt);
      double lar = atan(sqrt(pow(tan(alr),2)+pow(Kxka*ka/Kyal,2)))*sgn(alr);
      double t = Dt*cos(Ct*atan(Bt*lat-Et*(Bt*lat-atan(Bt*lat))))*cos(alM);
      double Gyk00 = cos(Cyk*atan(Byk0*Shyk-Eyk*(Byk0*Shyk-atan(Byk0*Shyk))));
      Gyk0 = cos(Cyk*atan(Byk0*ks-Eyk*(Byk0*ks-atan(Byk0*ks))))/Gyk00;
      Fy0 = Dy0*sin(Cy*atan(By0*ay0-Ey0*(By0*ay0-atan(By0*ay0))))+Svy0;
      double Fys = Gyk0*Fy0;
      double Mzs = -t*Fys;
      double Mzr = Dr*cos(zeta7*atan(Br*lar));
      double s = (SSZ1+SSZ2*(Fy/Fz0)+(SSZ3+SSZ4*dfz)*ga)*R0*LS;
      Mz = (Mzs+Mzr+(mck?0:s*Fx))*LMZ;

      Rs = (tyre->getUnloadedRadius()-tyre->getRimRadius()+contact->getGeneralizedRelativePosition()(0))*sin(ga);

      double CX = Cx0*(1+PCFX1*dfz+PCFX2*pow(dfz,2))*(1+PCFX3*dpi);
      double CY = Cy0*(1+PCFY1*dfz+PCFY2*pow(dfz,2))*(1+PCFY3*dpi);
      if(not constsix) six = fabs(Kxka/CX);
      if(not constsiy) siy = fabs(Kyal0/CY);
    }
    else {
      Fz = 0;
      Fx = 0;
      Fy = 0;
      Mx = 0;
      My = 0;
      Mz = 0;
      ga = 0;
      ka = 0;
      Kyal = 0;
      alF = 0;
      Rs = 0;
    }

    // TODO Fz gemäß S. 22
    // TODO MF Swift
    contact->getGeneralizedForce(false)(0) = Fx;
    contact->getGeneralizedForce(false)(1) = Fy;
    contact->getGeneralizedForce(false)(2) = Fz;
    contact->getGeneralizedForce(false)(3) = Mx;
    contact->getGeneralizedForce(false)(4) = My;
    contact->getGeneralizedForce(false)(5) = Mz;
  }

  VecV MagicFormula62::getData() const {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    VecV data(9,NONINIT);
    data(0) = ga;
    data(1) = vx;
    data(2) = vsx-vx;
    data(3) = ka;
    data(4) = Kyal;
    data(5) = six;
    data(6) = siy;
    data(7) = alF;
    data(8) = Rs;
    return data;
  }

  double MagicFormula62::getRadius() const  {
    return R0;
  }

}
