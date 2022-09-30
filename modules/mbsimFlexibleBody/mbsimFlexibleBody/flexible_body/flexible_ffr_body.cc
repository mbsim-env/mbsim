/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "flexible_ffr_body.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FlexibleFfrBody)

  void FlexibleFfrBody::setNodeNumbers(const vector<int> &n) {
    for(size_t i=0; i<n.size(); i++)
      nodeMap[n[i]] = i;
  }

  void FlexibleFfrBody::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(not(De0.size()) and mDamping.size()) {
	if(mDamping.size()!=(int)Pdm.cols())
	  throwError(string("(FlexibleFfrBody::init): size of modal damping does not match, must be ") + to_string(Pdm.cols()) +
		", but is " + to_string(mDamping.size()) + ".");
	SquareMatrix<Ref,double> V;
	Vector<Ref,double> w;
	eigvec(Ke0,SymMat(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]),V,w);
	Pdm <<= Pdm*V;
	for(int i=0; i<3; i++) {
	   rPdm[i] <<= rPdm[i]*V;
	  for(int j=0; j<3; j++)
	    PPdm[i][j] <<= V.T()*PPdm[i][j]*V;
	}
	Ke0 <<= JTMJ(Ke0,V);
	for(size_t i=0; i<Phi.size(); i++)
	  Phi[i] <<= Phi[i]*V;
	for(size_t i=0; i<Psi.size(); i++)
	  Psi[i] <<= Psi[i]*V;
	for(size_t i=0; i<sigmahel.size(); i++)
	  sigmahel[i] <<= sigmahel[i]*V;
	De0.resize(V.cols(),INIT,0);
	for(int i=0; i<De0.size(); i++)
	  De0(i,i) = 2*sqrt((PPdm[0][0](i,i)+PPdm[1][1](i,i)+PPdm[2][2](i,i))*Ke0(i,i))*mDamping(i);
      }
    }
    GenericFlexibleFfrBody::init(stage, config);
  }

  void FlexibleFfrBody::initializeUsingXML(DOMElement *element) {
    GenericFlexibleFfrBody::initializeUsingXML(element);

    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"mass");
    setMass(E(e)->getText<double>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"positionIntegral");
    setPositionIntegral(E(e)->getText<Vec3>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"positionPositionIntegral");
    setPositionPositionIntegral(E(e)->getText<SymMat3>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeFunctionIntegral");
    if(e) setShapeFunctionIntegral(E(e)->getText<Mat3xV>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"positionShapeFunctionIntegralArray");
    if(e) setPositionShapeFunctionIntegral(getCellArray1D<Mat3xV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"positionShapeFunctionIntegral");
      if(e) setPositionShapeFunctionIntegral(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeFunctionShapeFunctionIntegralArray");
    if(e) setShapeFunctionShapeFunctionIntegral(getCellArray2D<SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeFunctionShapeFunctionIntegral");
      if(e) setShapeFunctionShapeFunctionIntegral(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"stiffnessMatrix");
    if(e) setStiffnessMatrix(E(e)->getText<SymMat>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"dampingMatrix");
    if(e) setDampingMatrix(E(e)->getText<SymMat>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"proportionalDamping");
    if(e) setProportionalDamping(E(e)->getText<Vec>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"modalDamping");
    if(e) setModalDamping(MBXMLUtils::E(e)->getText<VecV>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrderArray");
    if(e) setNonlinearStiffnessMatrixOfFirstOrder(getCellArray1D<SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrder");
      if(e) setNonlinearStiffnessMatrixOfFirstOrder(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrderArray");
    if(e) setNonlinearStiffnessMatrixOfSecondOrder(getCellArray2D<SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrder");
      if(e) setNonlinearStiffnessMatrixOfSecondOrder(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"initialStressIntegral");
    if(e) setInitialStressIntegral(E(e)->getText<Vec>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearInitialStressIntegral");
    if(e) setNonlinearInitialStressIntegral(E(e)->getText<SqrMat>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAccelerationArray");
    if(e) setGeometricStiffnessMatrixDueToAcceleration(getCellArray1D<SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAcceleration");
      if(e) setGeometricStiffnessMatrixDueToAcceleration(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAccelerationArray");
    if(e) setGeometricStiffnessMatrixDueToAngularAcceleration(getCellArray1D<SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAcceleration");
      if(e) setGeometricStiffnessMatrixDueToAngularAcceleration(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocityArray");
    if(e) setGeometricStiffnessMatrixDueToAngularVelocity(getCellArray1D<SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocity");
      if(e) setGeometricStiffnessMatrixDueToAngularVelocity(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumbers");
    if(e) setNodeNumbers(E(e)->getText<vector<int>>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativePositionArray");
    if(e) setNodalRelativePosition(getCellArray1D<Vec3>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativePosition");
      if(e) setNodalRelativePosition(E(e)->getText<Vec>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativeOrientationArray");
    if(e) setNodalRelativeOrientation(getCellArray1D<SqrMat3>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativeOrientationArray");
      if(e) setNodalRelativeOrientation(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfTranslationArray");
    if(e) setNodalShapeMatrixOfTranslation(getCellArray1D<Mat3xV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfTranslation");
      if(e) setNodalShapeMatrixOfTranslation(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfRotationArray");
    if(e) setNodalShapeMatrixOfRotation(getCellArray1D<Mat3xV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfRotation");
      if(e) setNodalShapeMatrixOfRotation(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalStressMatrixArray");
    if(e) setNodalStressMatrix(getCellArray1D<Matrix<General, Fixed<6>, Var, double>>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalStressMatrix");
      if(e) setNodalStressMatrix(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalNonlinearStressMatrixArray");
    if(e) setNodalNonlinearStressMatrix(getCellArray2D<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalNonlinearStressMatrix");
      if(e) setNodalNonlinearStressMatrix(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalInitialStressArray");
    if(e) setNodalInitialStress(getCellArray1D<fmatvec::Vector<fmatvec::Fixed<6>, double>>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalInitialStress");
      if(e) setNodalInitialStress(E(e)->getText<Vec>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToForceArray");
    if(e) setNodalGeometricStiffnessMatrixDueToForce(getCellArray2D<fmatvec::SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToForce");
      if(e) setNodalGeometricStiffnessMatrixDueToForce(E(e)->getText<Mat>());
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToMomentArray");
    if(e) setNodalGeometricStiffnessMatrixDueToMoment(getCellArray2D<fmatvec::SqrMatV>(e));
    else {
      e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToMoment");
      if(e) setNodalGeometricStiffnessMatrixDueToMoment(E(e)->getText<Mat>());
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"openMBVFlexibleBody");
    if(e) {
      openMBVBody=OpenMBV::ObjectFactory::create<OpenMBV::FlexibleBody>(e->getFirstElementChild());
      openMBVBody->initializeUsingXML(e->getFirstElementChild());
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"openMBVColorRepresentation");
    if(e) {
      string colorRepresentationStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(colorRepresentationStr=="none") ombvColorRepresentation=OpenMBVFlexibleBody::none;
      else if(colorRepresentationStr=="xDisplacement") ombvColorRepresentation=OpenMBVFlexibleBody::xDisplacement;
      else if(colorRepresentationStr=="yDisplacement") ombvColorRepresentation=OpenMBVFlexibleBody::yDisplacement;
      else if(colorRepresentationStr=="zDisplacement") ombvColorRepresentation=OpenMBVFlexibleBody::zDisplacement;
      else if(colorRepresentationStr=="totalDisplacement") ombvColorRepresentation=OpenMBVFlexibleBody::totalDisplacement;
      else if(colorRepresentationStr=="xxStress") ombvColorRepresentation=OpenMBVFlexibleBody::xxStress;
      else if(colorRepresentationStr=="yyStress") ombvColorRepresentation=OpenMBVFlexibleBody::yyStress;
      else if(colorRepresentationStr=="zzStress") ombvColorRepresentation=OpenMBVFlexibleBody::zzStress;
      else if(colorRepresentationStr=="xyStress") ombvColorRepresentation=OpenMBVFlexibleBody::xyStress;
      else if(colorRepresentationStr=="yzStress") ombvColorRepresentation=OpenMBVFlexibleBody::yzStress;
      else if(colorRepresentationStr=="zxStress") ombvColorRepresentation=OpenMBVFlexibleBody::zxStress;
      else if(colorRepresentationStr=="equivalentStress") ombvColorRepresentation=OpenMBVFlexibleBody::equivalentStress;
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"plotNodeNumbers");
    if(e) setPlotNodeNumbers(E(e)->getText<VecVI>());
  }

  void FlexibleFfrBody::setOpenMBVFlexibleBody(const std::shared_ptr<OpenMBV::FlexibleBody> &body) {
    openMBVBody = body;
  }

}
