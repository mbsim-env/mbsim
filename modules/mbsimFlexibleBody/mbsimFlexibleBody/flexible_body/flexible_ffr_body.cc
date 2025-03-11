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

  void FlexibleFfrBody::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(mDamping.size()) {
	if(mDamping.size()!=Pdm.cols())
	  throwError(string("(GenericFlexibleFfrBody::init): size of modal damping does not match, must be ") + to_string(Pdm.cols()) +
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
	for(auto & i : Phi)
	  i <<= i*V;
	for(auto & i : Psi)
	  i <<= i*V;
	for(auto & i : sigmahel)
	  i <<= i*V;
	De0.resize(V.cols(),INIT,0);
	for(int i=0; i<De0.size(); i++)
	  De0(i,i) = 2*sqrt((PPdm[0][0](i,i)+PPdm[1][1](i,i)+PPdm[2][2](i,i))*Ke0(i,i))*mDamping(i);
      }
      else if(beta.e(0)>0 or beta.e(1)>0)
	De0 <<= beta.e(0)*SymMatV(PPdm[0][0]+PPdm[1][1]+PPdm[2][2]) + beta.e(1)*Ke0;
      else if(not(De0.size()))
	De0.resize(Ke0.size(),INIT,0);
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
    if(e) setPositionShapeFunctionIntegralArray(getCellArray1D<Mat3xV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeFunctionShapeFunctionIntegralArray");
    if(e) setShapeFunctionShapeFunctionIntegralArray(getCellArray2D<SqrMatV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"stiffnessMatrix");
    if(e) setStiffnessMatrix(E(e)->getText<SymMat>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"dampingMatrix");
    if(e) setDampingMatrix(E(e)->getText<SymMat>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"modalDamping");
    if(e) setModalDamping(MBXMLUtils::E(e)->getText<VecV>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"proportionalDamping");
    if(e) setProportionalDamping(E(e)->getText<Vec>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrderArray");
    if(e) setNonlinearStiffnessMatrixOfFirstOrderArray(getCellArray1D<SqrMatV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrderArray");
    if(e) setNonlinearStiffnessMatrixOfSecondOrderArray(getCellArray2D<SqrMatV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"initialStressIntegral");
    if(e) setInitialStressIntegral(E(e)->getText<Vec>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearInitialStressIntegral");
    if(e) setNonlinearInitialStressIntegral(E(e)->getText<SqrMat>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAccelerationArray");
    if(e) setGeometricStiffnessMatrixDueToAccelerationArray(getCellArray1D<SqrMatV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAccelerationArray");
    if(e) setGeometricStiffnessMatrixDueToAngularAccelerationArray(getCellArray1D<SqrMatV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocityArray");
    if(e) setGeometricStiffnessMatrixDueToAngularVelocityArray(getCellArray1D<SqrMatV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumbers");
    if(e) setNodeNumbers(E(e)->getText<vector<int>>());

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativePositionArray");
    if(e) setNodalRelativePositionArray(getCellArray1D<Vec3>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativeOrientationArray");
    if(e) setNodalRelativeOrientationArray(getCellArray1D<SqrMat3>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfTranslationArray");
    if(e) setNodalShapeMatrixOfTranslationArray(getCellArray1D<Mat3xV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfRotationArray");
    if(e) setNodalShapeMatrixOfRotationArray(getCellArray1D<Mat3xV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalStressMatrixArray");
    if(e) setNodalStressMatrixArray(getCellArray1D<Matrix<General, Fixed<6>, Var, double>>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalNonlinearStressMatrixArray");
    if(e) setNodalNonlinearStressMatrixArray(getCellArray2D<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double>>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalInitialStressArray");
    if(e) setNodalInitialStressArray(getCellArray1D<fmatvec::Vector<fmatvec::Fixed<6>, double>>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToForceArray");
    if(e) setNodalGeometricStiffnessMatrixDueToForceArray(getCellArray2D<fmatvec::SqrMatV>(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalGeometricStiffnessMatrixDueToMomentArray");
    if(e) setNodalGeometricStiffnessMatrixDueToMomentArray(getCellArray2D<fmatvec::SqrMatV>(e));

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"openMBVFlexibleBody");
    if(e) {
      openMBVBody=OpenMBV::ObjectFactory::create<OpenMBV::FlexibleBody>(e->getFirstElementChild());
      openMBVBody->initializeUsingXML(e->getFirstElementChild());
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"openMBVNodeNumbers");
    if(e) setOpenMBVNodeNumbers(E(e)->getText<vector<int>>());

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
    if(e) setPlotNodeNumbers(E(e)->getText<vector<int>>());
  }

  void FlexibleFfrBody::setOpenMBVFlexibleBody(const std::shared_ptr<OpenMBV::FlexibleBody> &body) {
    openMBVBody = body;
  }

}
