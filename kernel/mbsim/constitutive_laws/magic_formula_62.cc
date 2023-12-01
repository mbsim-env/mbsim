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

  MagicFormula62::~MagicFormula62() {
    delete slipPoint[0];
    delete slipPoint[1];
  }

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
	  string value;
	  file >> name >> str >> value;
	  FITTYP = stoi(value);
	  getline(file,line);
	  file >> name >> str;
	  if(str.size()>1)
	    TYRESIDE = str.substr(2,str.size()-3);
	  else {
	    file >> value;
	    TYRESIDE = value.substr(1,value.size()-2);
	  }
	  getline(file,line);
	  file >> name >> str >> value;
	  v0 = stod(value);
	}
	found = line.find("[DIMENSION]");
	if(found!=string::npos) {
	  string value[5];
	  for(int i=0; i<5; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  R0 = stod(value[0]);
	  w = stod(value[1]);
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
	  p0 = stod(value[1]);
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
	  MC_CONTOUR_A = stod(value[3]);
	  MC_CONTOUR_B = stod(value[4]);
	  BREFF = stod(value[5]);
	  DREFF = stod(value[6]);
	  FREFF = stod(value[7]);
	  Q_RE0 = stod(value[8]);
	  Q_V1 = stod(value[9]);
	  Q_V2 = stod(value[10]);
	  Q_FZ2 = stod(value[11]);
	  Q_FCX = stod(value[12]);
	  Q_FCY = stod(value[13]);
	  Q_CAM = stod(value[14]);
	  PFZ1 = stod(value[15]);
	  Q_FCY2 = stod(value[16]);
	  Q_CAM1 = stod(value[17]);
	  Q_CAM2 = stod(value[18]);
	  Q_CAM3 = stod(value[19]);
	  Q_FYS1 = stod(value[20]);
	  Q_FYS2 = stod(value[21]);
	  Q_FYS3 = stod(value[22]);
	  rhobtm = stod(value[23]);
	  czbtm = stod(value[24]);
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
	found = line.find("[CONTACT_PATCH]");
	if(found!=string::npos) {
	  string value[4];
	  for(int i=0; i<4; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  Q_RB1 = stod(value[2]);
	  Q_RB2 = stod(value[3]);
	}
	found = line.find("[INFLATION_PRESSURE_RANGE]");
	if(found!=string::npos) {
	  string value[2];
	  for(int i=0; i<2; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  PRESMIN = stod(value[0]);
	  PRESMAX = stod(value[1]);
	}
	found = line.find("[VERTICAL_FORCE_RANGE]");
	if(found!=string::npos) {
	  string value[2];
	  for(int i=0; i<2; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  FZMIN = stod(value[0]);
	  FZMAX = stod(value[1]);
	}
	found = line.find("[LONG_SLIP_RANGE]");
	if(found!=string::npos) {
	  string value[2];
	  for(int i=0; i<2; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  KPUMIN = stod(value[0]);
	  KPUMAX = stod(value[1]);
	}
	found = line.find("SLIP_ANGLE_RANGE]");
	if(found!=string::npos) {
	  string value[2];
	  for(int i=0; i<2; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  ALPMIN = stod(value[0]);
	  ALPMAX = stod(value[1]);
	}
	found = line.find("[INCLINATION_ANGLE_RANGE]");
	if(found!=string::npos) {
	  string value[2];
	  for(int i=0; i<2; i++) {
	    file >> name >> str >> value[i];
	    getline(file,line);
	  }
	  CAMMIN = stod(value[0]);
	  CAMMAX = stod(value[1]);
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
	  if(LS<0) LS = stod(value[20]);
	  if(LMX<0) LMX = stod(value[21]);
	  LVMX = stod(value[22]);
	  if(LMY<0) LMY = stod(value[23]);
	  LMP = stod(value[24]);
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
	throw runtime_error("(MagicFormula62::init): input data file must be defined.");
      importData();
    }
    if(stage==preInit) {
      if(tyreSide==unknown)
        throwError("(MagicFormula62::init): tyre side unknown");
      else if((tyreSide==left and (TYRESIDE[0]=='r' or TYRESIDE[0]=='R')) or (tyreSide==right and (TYRESIDE[0]=='l' or TYRESIDE[0]=='L')))
	mirroring = true;
    }
    else if(stage==unknownStage) {
      TyreContact *contact = static_cast<TyreContact*>(parent);
      Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
      if(p > PRESMAX)
	p = PRESMAX;
      else if(p < PRESMIN)
	p = PRESMIN;
      dpi = (p-p0)/p0;
      constsix = six>=0;
      constsiy = siy>=0;
      if(abs(tyre->getRadius()-R0)>1e-6)
	msg(Warn) << "Unloaded radius of " << tyre->getPath() << " (" << tyre->getRadius() << ") is different to unloaded radius of " << inputDataFile << " (" << R0 << ")." << endl;
//      if(MC_CONTOUR_A > 0 and abs(tyre->getContourParameters()(0)-MC_CONTOUR_A*w)>1e-6)
//	msg(Warn) << "Contour parameter A of " << tyre->getPath() << " (" << tyre->getContourParameters()(0) << ") is different to ellipse parameter A of " << inputDataFile << " (" << MC_CONTOUR_A*w << ")." << endl;
//      if(MC_CONTOUR_B > 0 and abs(tyre->getContourParameters()(1)-MC_CONTOUR_B*w)>1e-6)
//	msg(Warn) << "Contour parameter B of " << tyre->getPath() << " (" << tyre->getContourParameters()(1) << ") is different to ellipse parameter B of " << inputDataFile << " (" << MC_CONTOUR_B*w << ")." << endl;
      slipPoint[0] = contact->getContour(0)->createContourFrame("S0");
      slipPoint[1] = contact->getContour(1)->createContourFrame("S1");
      slipPoint[0]->setParent(this);
      slipPoint[1]->setParent(this);
      slipPoint[0]->sethSize(contact->getContour(0)->gethSize(0), 0);
      slipPoint[0]->sethSize(contact->getContour(0)->gethSize(1), 1);
      slipPoint[1]->sethSize(contact->getContour(1)->gethSize(0), 0);
      slipPoint[1]->sethSize(contact->getContour(1)->gethSize(1), 1);
      slipPoint[0]->init(stage, config);
      slipPoint[1]->init(stage, config);
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
    plotColumns.emplace_back("deflection");
    if(ts) {
      plotColumns.emplace_back("turn slip");
      plotColumns.emplace_back("spin");
    }
  }

  void MagicFormula62::plot(vector<double> &plotVector) {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    plotVector.push_back(ga);
    plotVector.push_back(vcx);
    plotVector.push_back(vsx-vcx);
    plotVector.push_back(ka);
    plotVector.push_back(Kyal);
    plotVector.push_back(six);
    plotVector.push_back(siy);
    plotVector.push_back(alF);
    plotVector.push_back(rhoz);
    if(ts) {
      plotVector.push_back(phit);
      plotVector.push_back(phiF);
    }
  }

  void MagicFormula62::initializeUsingXML(DOMElement *element) {
    TyreModel::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"inputDataFileName");
    string str = X()%E(e)->getFirstTextChild()->getData();
    setInputDataFile(E(e)->convertPath(str.substr(1,str.length()-2)).string());
    e=E(element)->getFirstElementChildNamed(MBSIM%"tyreSide");
    if(e) {
      string tyreSideStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(tyreSideStr=="left") tyreSide=left;
      else if(tyreSideStr=="right") tyreSide=right;
      else tyreSide=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"motorcycleKinematics");
    if(e) setMotorcycleKinematics(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"contactPointTransformation");
    if(e) setContactPointTransformation(E(e)->getText<bool>());
//    e=E(element)->getFirstElementChildNamed(MBSIM%"coordinateTransformation");
//    if(e) setCoordinateTransformation(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"turnSlip");
    if(e) setTurnSlip(E(e)->getText<bool>());
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForOverturningMoment");
    if(e) setScaleFactorForOverturningMoment(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorRollingResistanceMoment");
    if(e) setScaleFactorForRollingResistanceMoment(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForAligningMoment");
    if(e) setScaleFactorForAligningMoment(E(e)->getText<double>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"scaleFactorForMomentArmOfLongitudinalForce");
    if(e) setScaleFactorForMomentArmOfLongitudinalForce(E(e)->getText<double>());
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
      contact->getxd(false)(i) = (-vsx - contact->getx()(i)*vcx)/six;
      i++;
    }
    if(siy!=0)
      contact->getxd(false)(i) = (atan(vcy/vcx) - contact->getx()(i))*vcx/siy; // original MF62: (vsy - contact->getx()(0)*vx)/sigy
  }

  VecV MagicFormula62::getContourParameters() const {
    VecV cp(2,NONINIT);
    cp(0) = MC_CONTOUR_A*w;
    cp(1) = MC_CONTOUR_B*w;
    return cp;
  }

  double MagicFormula62::evalFreeRadius() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double Om = tyre->getFrame()->evalOrientation().col(1).T()*tyre->getFrame()->evalAngularVelocity();
    return R0*(Q_RE0+Q_V1*pow(Om*R0/v0,2));
  }

  void MagicFormula62::updateGeneralizedForces() {
    TyreContact *contact = static_cast<TyreContact*>(parent);
    Tyre *tyre = static_cast<Tyre*>(contact->getContour(1));
    double Fx, Fy, Fz, Mx, My, Mz;
    Fz0 *= LFZ0;

    ga = asin(tyre->getFrame()->evalOrientation().col(1).T()*contact->getContourFrame(0)->evalOrientation().col(2));
    if(abs(ga) > CAMMAX)
      ga = sgn(ga)*CAMMAX;
    else if(abs(ga) < CAMMIN)
      ga = sgn(ga)*CAMMIN;

    vx = contact->getContourFrame(0)->getOrientation().col(0).T()*tyre->getFrame()->evalVelocity();
    vcx = max(1.,abs(contact->evalForwardVelocity()(0)));
    vcy = contact->getForwardVelocity()(1);
    vc = sqrt(pow(vcx,2)+pow(vcy,2));
    double Om = tyre->getFrame()->getOrientation().col(1).T()*tyre->getFrame()->evalAngularVelocity();
    double ROm = R0*(Q_RE0+Q_V1*pow(Om*R0/v0,2));
    double Cz = cz*(1+PFZ1*dpi);
    double fcorr = (1-Q_CAM*abs(ga))*(1+Q_V2*R0/v0*abs(Om))*(1+PFZ1*dpi);
    double Q_FZ1 = sqrt(pow(cz*R0/Fz0,2)-4*Q_FZ2);
    double Re;
    if(mck) {
      rhoz = -contact->evalGeneralizedRelativePosition()(0);
      if(rhoz<0) rhoz = 0;
      Fz = fcorr*(Q_FZ1*rhoz/R0+Q_FZ2*pow(rhoz/R0,2))*Fz0 - dz*contact->evalGeneralizedRelativeVelocity()(2);
//      double rhoze = Fz0/Cz*(DREFF*atan(BREFF*Cz/Fz0*rhoz)+FREFF*Cz/Fz0*rhoz); Pacejka
      double rhoze = Fz0/Cz*(DREFF*atan(BREFF*Fz/Fz0)+FREFF*Fz/Fz0); // Manual
      double rC = MC_CONTOUR_A*w;
      Re = ROm - rC + (rC-rhoze)*cos(ga);
    }
    else {
      Vec WrWC = contact->getContourFrame(1)->getPosition()-tyre->getFrame()->getPosition();
      double Rl = nrm2(WrWC);
      double rhozfr = ROm-Rl;
      if(rhozfr<0) rhozfr = 0;
      if(MC_CONTOUR_A>0 and MC_CONTOUR_B>0) {
	rhoz = rhozfr*cos(ga) + MC_CONTOUR_A*w*(1-cos(ga)); // = (ROm-rC)*cos(ga)+rC-Rl*cos(ga);
      } else {
	double rhozg = 0;
	// double a = R0*(Q_RA2*rhozf/R0 + Q_RA1*sqrt(rhozf/R0));
	double b = w*(Q_RB2*rhozfr/R0 + Q_RB1*pow(rhozfr/R0,1./3));
	double rtw = 2*b; // There is no equation for rtw in the manual, thus we use rtw = 2*b
	if(((Q_CAM1*ROm+Q_CAM2*pow(ROm,2))*ga)>0)
	  rhozg = pow((Q_CAM1*Rl+Q_CAM2*pow(Rl,2))*ga,2)*(rtw/8*abs(tan(ga)))/pow((Q_CAM1*ROm+Q_CAM2*pow(ROm,2))*ga,2)-(Q_CAM3*rhozfr*abs(ga));
	rhoz = rhozfr+rhozg;
      }
      if(rhoz<0) rhoz = 0;
//      double SFyg = (Q_FYS1*Q_FYS2*Rl/ROm+Q_FYS3*pow(Rl/ROm,2))*ga;
//      double fcorr = (1+Q_V2*R0/v0*abs(Om)-pow(Q_FCX*Fx/Fz0,2)-pow(pow(rhoz/R0,Q_FCY2)*Q_FCY*(Fy-SFyg)/Fz0,2))*(1+PFZ1*dpi); // the dependence on the longitudinal and lateral force is neglected
      Fz = fcorr*(Q_FZ1*rhoz/R0+Q_FZ2*pow(rhoz/R0,2))*Fz0;
      double Fzbtm = czbtm*(rRim+rhobtm-Rl);
      if(Fzbtm>Fz) Fz = Fzbtm;
      Fz -= dz*contact->evalGeneralizedRelativeVelocity()(2)/cos(ga);
//      double rhoze = Fz0/Cz*(DREFF*atan(BREFF*Cz/Fz0*rhoz)+FREFF*Cz/Fz0*rhoz); Pacejka
      double rhoze = Fz0/Cz*(DREFF*atan(BREFF*Fz/Fz0)+FREFF*Fz/Fz0); // Manual
      if(MC_CONTOUR_A>0 and MC_CONTOUR_B>0) {
	double rC = MC_CONTOUR_A*w;
	Re = ROm - rC + (rC-rhoze)*cos(ga);
      }
      else
	Re = ROm-rhoze;
    }
    Vec3 n = contact->getContourFrame(1)->getOrientation().col(2)*cos(ga) - contact->getContourFrame(1)->getOrientation().col(1)*sin(ga);
    slipPoint[0]->setPosition(tyre->getFrame()->getPosition()-Re*n);
    slipPoint[1]->setPosition(slipPoint[0]->getPosition());
    slipPoint[0]->setOrientation(contact->getContourFrame(0)->getOrientation());
    slipPoint[1]->setOrientation(contact->getContourFrame(1)->getOrientation());
    Vec3 WvD = slipPoint[1]->evalVelocity() - slipPoint[0]->evalVelocity();
    vsx = slipPoint[0]->getOrientation().col(0).T()*WvD;

    if(Fz>0) {
      if(Fz > FZMAX)
	Fz = FZMAX;
      else if(Fz < FZMIN) {
        LFM = Fz/FZMIN;
	Fz = FZMIN;
      }
      double dfz = (Fz-Fz0)/Fz0;
      int i = 0;
      double alM = atan(vcy/vcx);
      if(abs(alM) > ALPMAX)
	alM = sgn(alM)*ALPMAX;
      else if(abs(alM) < ALPMIN)
	alM = sgn(alM)*ALPMIN;
      ka = six!=0 ? contact->getx()(i++) : -vsx/vcx;
      if(abs(ka) > KPUMAX)
	ka = sgn(ka)*KPUMAX;
      else if(abs(ka) < KPUMIN)
	ka = sgn(ka)*KPUMIN;
      alF = siy!=0 ? contact->getx()(i) : alM;
      if(abs(alF) > ALPMAX)
	alF = sgn(alF)*ALPMAX;
      else if(abs(alF) < ALPMIN)
	alF = sgn(alF)*ALPMIN;

      if(mirroring) {
	alF = -alF;
	ga = -ga;
      }

      if(ts) {
	double Om = tyre->getFrame()->evalOrientation().col(1).T()*tyre->getFrame()->evalAngularVelocity();
	double psid = contact->getContourFrame(1)->evalOrientation().col(2).T()*tyre->getFrame()->getAngularVelocity();
	epsga = PECP1*(1+PECP2*dfz);
	phit = -psid/vc;
	phiF = -1./vc*(psid+(1-epsga)*Om*sin(ga));
	double Bxphi = PDXP1*(1+PDXP2*dfz)*cos(atan(PDXP3*ka));
	zeta1 = cos(atan(Bxphi*R0*phiF));
      }
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

      if(ts) {
	zeta0 = 0;
	double Byphi = PDYP1*(1+PDYP2*dfz)*cos(atan(PDYP3*tan(alF)));
	zeta2 = cos(atan(Byphi*(R0*abs(phiF)+PDYP4*sqrt(R0*abs(phiF)))));
	zeta3 = cos(atan(PKYP1*pow(R0*phiF,2)));
      }
      double Kyga0 = Fz*(PKY6+PKY7*dfz)*(1+PPY5*dpi)*LKYC;
      Kyal = PKY1*Fz0*(1+PPY1*dpi)*(1-PKY3*abs(ga))*sin(PKY4*atan(Fz/Fz0/((PKY2+PKY5*pow(ga,2))*(1+PPY2*dpi))))*LKY*zeta3;
      double muey = (PDY1+PDY2*dfz)*(1+PPY3*dpi+PPY4*pow(dpi,2))*(1-PDY3*pow(ga,2))*LMUY;
      double Svy0 = Fz*(PVY1+PVY2*dfz)*LVY*LMUY;
      double Svyga = Fz*(PVY3+PVY4*dfz)*ga*LKYC*LMUY*zeta2;
      if(ts) {
	double Kyal0 = PKY1*Fz0*(1+PPY1*dpi)*sin(PKY4*atan(Fz/Fz0/(PKY2*(1+PPY2*dpi))))*LKY;
	double Chyphi = PHYP1;
	double Dhyphi = PHYP2+PHYP3*dfz;
	double Ehyphi = PHYP4;
	double KyRphi0 = Kyga0/(1-epsga);
	double Bhyphi = -KyRphi0/(Chyphi*Dhyphi*Kyal0);
	double Shyphi = Dhyphi*sin(Chyphi*atan(Bhyphi*R0*phiF-Ehyphi*(Bhyphi*R0*phiF-atan(Bhyphi*R0*phiF))));
	zeta4 = 1+Shyphi-Svyga/Kyal;
      }
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

      Mx = R0*Fz*LMX*(QSX1*LVMX-QSX2*ga*(1+PPMX1*dpi)+QSX3*Fy/Fz0+QSX4*cos(QSX5*atan(pow(QSX6*Fz/Fz0,2)))*sin(QSX7*ga+QSX8*atan(QSX9*Fy/Fz0))+QSX10*atan(QSX11*Fz/Fz0)*ga) + R0*LMX*(Fy*(QSX13+QSX14*abs(ga))-Fz*QSX12*ga*abs(ga));

      My = -R0*Fz0*LMY*(QSY1+QSY2*Fx/Fz0+QSY3*abs(vx/v0)+QSY4*pow(vx/v0,4)+(QSY5+QSY6*Fz/Fz0)*pow(ga,2))*pow(Fz/Fz0,QSY7)*pow(p/p0,QSY8);

      if(ts) {
	double phiM = phiF;
	zeta5 = cos(atan(QDTP1*R0*phiM));
	zeta6 = cos(atan(QBRP1*R0*phiM));
	double Mzphiinf = QCRP1*abs(muey)*R0*Fz*sqrt(Fz/Fz0)*LMP;
	double Mzphi90 = Mzphiinf*2./M_PI*atan(QCRP2*R0*abs(phit))*Gyk;
	double Cdrphi = QDRP1;
	double Ddrphi = Mzphiinf/sin(0.5*M_PI*Cdrphi);
	double Kzgar0 = Fz*R0*(QDZ8+QDZ9*dfz+(QDZ10+QDZ11*dfz)*abs(ga))*LKZC;
	double Bdrphi = Kzgar0/(Cdrphi*Ddrphi*(1-epsga));
	double Drphi = Ddrphi*sin(Cdrphi*atan(Bdrphi*R0*phiM));
	zeta7 = 2./M_PI*acos(Mzphi90/abs(Drphi));
	zeta8 = 1+Drphi;
      }
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
      double Dr = Fz*R0*LMUY*cos(alM)*((QDZ6+QDZ7*dfz)*LRES*zeta2+((QDZ8+QDZ9*dfz)*(1+PPZ2*dpi)+(QDZ10+QDZ11*dfz)*abs(ga))*ga*LKZC*zeta0)-zeta8+1;
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
      Mz = (Mzs+Mzr+s*Fx)*LMZ;

      double CX = Cx0*(1+PCFX1*dfz+PCFX2*pow(dfz,2))*(1+PCFX3*dpi);
      double CY = Cy0*(1+PCFY1*dfz+PCFY2*pow(dfz,2))*(1+PCFY3*dpi);
      if(not constsix) six = abs(Kxka/CX);
      if(not constsiy) siy = abs(Kyal0/CY);
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
    }

    if(mck and contactPointTransformation) {
      double dy = (MC_CONTOUR_A*w-rhoz)*tan(ga);
      Mx += dy*Fz;
      Mz -= dy*Fx;
    }
//    if(coordinateTransformation) {
//      double My_ = My*cos(ga) - Mz*sin(ga);
//      double Mz_ = My*sin(ga) + Mz*cos(ga);
//      My = My_;
//      Mz = Mz_;
//    }

    if(mirroring) {
      alF = -alF;
      ga = -ga;
      Fy = -Fy;
      Mx = -Mx;
      Mz = -Mz;
    }

    contact->getGeneralizedForce(false)(0) = LFM*Fx;
    contact->getGeneralizedForce(false)(1) = LFM*Fy;
    contact->getGeneralizedForce(false)(2) = LFM*Fz;
    contact->getGeneralizedForce(false)(3) = LFM*Mx;
    contact->getGeneralizedForce(false)(4) = LFM*My;
    contact->getGeneralizedForce(false)(5) = LFM*Mz;

  }

  VecV MagicFormula62::getData() const {
    static_cast<TyreContact*>(parent)->evalGeneralizedForce(); // Enforce variables to be up to date
    VecV data(9,NONINIT);
    data(0) = ga;
    data(1) = vcx;
    data(2) = vsx-vcx;
    data(3) = ka;
    data(4) = Kyal;
    data(5) = six;
    data(6) = siy;
    data(7) = alF;
    return data;
  }

  void MagicFormula62::resetUpToDate() {
    TyreModel::resetUpToDate();
    slipPoint[0]->resetUpToDate();
    slipPoint[1]->resetUpToDate();
  }

}
