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
#include "flexible_body_ffr.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/links/joint.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/objectfactory.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/rotation_about_axes_xyz.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zyx.h"
#include "mbsim/functions/kinematics/rotation_about_axes_xyz_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zyx_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_xyz_transformed_mapping.h"
#include "mbsim/functions/kinematics/rotation_about_axes_zxz_transformed_mapping.h"
#include "mbsimFlexibleBody/namespace.h"
#include "mbsimFlexibleBody/utils/openmbv_utils.h"
#include <openmbvcppinterface/rigidbody.h>
#include <openmbvcppinterface/invisiblebody.h>
#include <openmbvcppinterface/objectfactory.h>
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/dynamicindexedfaceset.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  Range<Var,Var> i02(0,2);

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FlexibleBodyFFR)

  FlexibleBodyFFR::FlexibleBodyFFR(const string &name) : NodeBasedBody(name), m(0), Id(Eye()), ne(0), coordinateTransformation(true), APK(EYE), fTR(0), fPrPK(0), fAPK(0), frameForJacobianOfRotation(0), translationDependentRotation(false), constJT(false), constJR(false), constjT(false), constjR(false), updPjb(true), updGC(true), updT(true), updMb(true), bodyFixedRepresentationOfAngularVelocity(false) {

    updKJ[0] = true;
    updKJ[1] = true;

    K=new Frame("K");
    NodeBasedBody::addFrame(K);

    updateJacobians_[0] = &FlexibleBodyFFR::updateJacobians0;
    updateJacobians_[1] = &FlexibleBodyFFR::updateJacobians1;
    updateKJ_[0] = &FlexibleBodyFFR::updateKJ0;
    updateKJ_[1] = &FlexibleBodyFFR::updateKJ1;
  }

  FlexibleBodyFFR::~FlexibleBodyFFR() {
    if(fPrPK) { delete fPrPK; fPrPK=0; }
    if(fAPK) { delete fAPK; fAPK=0; }
    if(fTR) { delete fTR; fTR=0; }
  }

  void FlexibleBodyFFR::updateh(int index) {
    h[index] += evalKJ(index).T()*(evalhb() - evalMb()*evalKi());
  }

  void FlexibleBodyFFR::calcqSize() {
    NodeBasedBody::calcqSize();
    qSize = nq;
  }

  void FlexibleBodyFFR::calcuSize(int j) {
    NodeBasedBody::calcuSize(j);
    if(j==0)
      uSize[j] = nu[j];
    else
      uSize[j] = 6 + ne;
  }

  void FlexibleBodyFFR::determineSID() {
    Cr0.resize(ne,NONINIT);
    Gr0.resize(ne);
    Gr1.resize(ne,vector<SqrMat3>(ne));
    Ge.resize(ne);
    Oe0.resize(ne,NONINIT);
    Oe1.resize(6);
    vector<vector<SqrMatV> > Kom(3,vector<SqrMatV>(3));
    mmi0(0,0) = rrdm(1,1) + rrdm(2,2);
    mmi0(0,1) = -rrdm(1,0);
    mmi0(0,2) = -rrdm(2,0);
    mmi0(1,1) = rrdm(0,0) + rrdm(2,2);
    mmi0(1,2) = -rrdm(2,1);
    mmi0(2,2) = rrdm(0,0) + rrdm(1,1);
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        if(i!=j)
          Kom[i][j].resize() = PPdm[i][j];
      }
    }
    Kom[0][0].resize() = -PPdm[1][1]-PPdm[2][2];
    Kom[1][1].resize() = -PPdm[2][2]-PPdm[0][0];
    Kom[2][2].resize() = -PPdm[0][0]-PPdm[1][1];

    Me.resize(ne,NONINIT);
    mmi1.resize(ne);
    mmi2.resize(ne,vector<SqrMat3>(ne));

    for(int i=0; i<ne; i++) {
      mmi1[i](0,0) = 2.*(rPdm[1](1,i) + rPdm[2](2,i));
      mmi1[i](0,1) = -(rPdm[1](0,i) + rPdm[0](1,i));
      mmi1[i](0,2) = -(rPdm[2](0,i) + rPdm[0](2,i));
      mmi1[i](1,1) = 2.*(rPdm[0](0,i) + rPdm[2](2,i));
      mmi1[i](1,2) = -(rPdm[2](1,i) + rPdm[1](2,i));
      mmi1[i](2,2) = 2.*(rPdm[0](0,i) + rPdm[1](1,i));

      Oe0.e(i,0) = -rPdm[2](2,i) - rPdm[1](1,i);
      Oe0.e(i,1) = -rPdm[2](2,i) - rPdm[0](0,i);
      Oe0.e(i,2) = -rPdm[1](1,i) - rPdm[0](0,i);
      Oe0(i,3) = rPdm[0](1,i) + rPdm[1](0,i);
      Oe0(i,4) = rPdm[1](2,i) + rPdm[2](1,i);
      Oe0(i,5) = rPdm[2](0,i) + rPdm[0](2,i);

      Gr0[i](0,0) = 2.*(rPdm[2](2,i) + rPdm[1](1,i));
      Gr0[i](0,1) = -2.*rPdm[1](0,i);
      Gr0[i](0,2) = -2.*rPdm[2](0,i);
      Gr0[i](1,0) = -2.*rPdm[0](1,i);
      Gr0[i](1,1) = 2.*(rPdm[2](2,i) + rPdm[0](0,i));
      Gr0[i](1,2) = -2.*rPdm[2](1,i);
      Gr0[i](2,0) = -2.*rPdm[0](2,i);
      Gr0[i](2,1) = -2.*rPdm[1](2,i);
      Gr0[i](2,2) = 2.*(rPdm[1](1,i) + rPdm[0](0,i));

      Cr0.e(i,0) = rPdm[1](2,i) - rPdm[2](1,i);
      Cr0.e(i,1) = rPdm[2](0,i) - rPdm[0](2,i);
      Cr0.e(i,2) = rPdm[0](1,i) - rPdm[1](0,i);

      for(int j=i; j<ne; j++) {
        Me.ej(i,j) = PPdm[0][0].e(i,j) + PPdm[1][1].e(i,j) + PPdm[2][2].e(i,j);

        mmi2[i][j].e(0,0) = PPdm[1][1].e(i,j) + PPdm[2][2].e(i,j);
        mmi2[i][j].e(1,1) = PPdm[0][0].e(i,j) + PPdm[2][2].e(i,j);
        mmi2[i][j].e(2,2) = PPdm[0][0].e(i,j) + PPdm[1][1].e(i,j);
        mmi2[i][j].e(0,1) = -PPdm[1][0].e(i,j);
        mmi2[i][j].e(0,2) = -PPdm[2][0].e(i,j);
        mmi2[i][j].e(1,2) = -PPdm[2][1].e(i,j);
        mmi2[i][j].e(1,0) = -PPdm[0][1].e(i,j);
        mmi2[i][j].e(2,0) = -PPdm[0][2].e(i,j);
        mmi2[i][j].e(2,1) = -PPdm[1][2].e(i,j);
        mmi2[j][i] = mmi2[i][j].T();

        Gr1[i][j] = 2.*mmi2[i][j];
        Gr1[j][i] = Gr1[i][j].T();
      }
    }
    Ct0 = Pdm.T();
    for(unsigned int i=0; i<K0t.size(); i++)
      Ct1.push_back(K0t[i]);

    vector<SqrMatV> Kr(3);
    Kr[0].resize() = -PPdm[1][2] + PPdm[1][2].T();
    Kr[1].resize() = -PPdm[2][0] + PPdm[2][0].T();
    Kr[2].resize() = -PPdm[0][1] + PPdm[0][1].T();

    for(unsigned int i=0; i<Kr.size(); i++)
      Cr1.push_back(Kr[i]);
    for(unsigned int i=0; i<K0r.size(); i++)
      Cr1[i] += K0r[i];

    Ge.resize(3);
    for(int i=0; i<3; i++)
      Ge[i].resize() = 2.*Kr[i];

    for(int i=0; i<3; i++)
      Oe1[i].resize() = Kom[i][i];
    Oe1[3].resize() = Kom[0][1] + Kom[0][1].T();
    Oe1[4].resize() = Kom[1][2] + Kom[1][2].T();
    Oe1[5].resize() = Kom[2][0] + Kom[2][0].T();
    for(unsigned int i=0; i<K0om.size(); i++)
      Oe1[i] += K0om[i];

    if(not(De0.size()))
      De0 = beta.e(0)*Me + beta.e(1)*Ke0;

    if(Knl1.size()) {
      Ke1.resize(Knl1.size());
      if(Knl2.size()) Ke2.resize(Knl2.size(),vector<SqrMatV>(Knl2.size()));
      for(unsigned int i=0; i<Knl1.size(); i++) {
        Ke1[i].resize() = (Knl1[i].T() + 0.5*Knl1[i]);
        for(unsigned int j=0; j<Knl2.size(); j++)
          Ke2[i][j].resize() = 0.5*Knl2[i][j];
      }
    }
 }

  void FlexibleBodyFFR::prefillMassMatrix() {
    M_.resize(6+ne,NONINIT);
    for(int i=0; i<3; i++) {
      M_.ej(i,i)=m;
      for(int j=i+1; j<3; j++)
        M_.ej(i,j)=0;
    }
    for(int i=0; i<ne; i++) {
      for(int j=0; j<3; j++)
        M_.e(i+6,j) = Ct0.e(i,j);
      for(int j=i; j<ne; j++)
        M_.ej(i+6,j+6) = Me.ej(i,j);
    }
  }

  void FlexibleBodyFFR::init(InitStage stage) {
    if(stage==preInit) {
      ne = Pdm.cols();

      for(unsigned int k=0; k<contour.size(); k++) {
        RigidContour *contour_ = dynamic_cast<RigidContour*>(contour[k]);
        if(contour_ and not(contour_->getFrameOfReference()))
          contour_->setFrameOfReference(K);
      }

      NodeBasedBody::init(stage);

      int nqT=0, nqR=0, nuT=0, nuR=0;
      if(fPrPK) {
        nqT = fPrPK->getArg1Size();
        nuT = fPrPK->getArg1Size(); // TODO fTT->getArg1Size()
      }
      if(fAPK) {
        nqR = fAPK->getArg1Size();
        nuR = fAPK->getArg1Size(); // TODO fTR->getArg1Size()
      }
 
      if(translationDependentRotation) {
        assert(nqT == nqR);
        assert(nuT == nuR);
        nq = nqT + ne;
        nu[0] = nuT + ne;
        iqT = Range<Var,Var>(0,nqT+nqR-1);
        iqR = Range<Var,Var>(0,nqT+nqR-1);
        iqE = Range<Var,Var>(nqT+nqR,nq-1);
        iuT = Range<Var,Var>(0,nuT+nuR-1);
        iuR = Range<Var,Var>(0,nuT+nuR-1);
        iuE = Range<Var,Var>(nuT+nuR,nu[0]-1);
      }
      else {
        nq = nqT + nqR + ne;
        nu[0] = nuT + nuR + ne;
        iqT = Range<Var,Var>(0,nqT-1);
        iqR = Range<Var,Var>(nqT,nqT+nqR-1);
        iqE = Range<Var,Var>(nqT+nqR,nq-1);
        iuT = Range<Var,Var>(0,nuT-1);
        iuR = Range<Var,Var>(nuT,nqT+nqR-1);
        iuE = Range<Var,Var>(nuT+nuR,nu[0]-1);
      }

      nu[1] = 6 + ne;

      Ki.resize(6+ne);

      PJT[0].resize(nu[0]);
      PJR[0].resize(nu[0]);

      PJT[1].resize(nu[1]);
      PJR[1].resize(nu[1]);
      for(int i=0; i<3; i++)
	PJT[1](i,i) = 1;
      for(int i=3; i<6; i++)
	PJR[1](i-3,i) = 1;

      qRel.resize(nq);
      uRel.resize(nu[0]);
      TRel.resize(nq,nu[0],Eye());
      WJTrel.resize(nu[0]);
      WJRrel.resize(nu[0]);

      updNodalPos.resize(KrKP.size(),true);
      updNodalVel.resize(KrKP.size(),true);
      updNodalStress.resize(KrKP.size(),true);
      WrOP.resize(KrKP.size());
      WrRP.resize(KrKP.size());
      disp.resize(KrKP.size());
      Womrel.resize(KrKP.size());
      Wvrel.resize(KrKP.size());
      if(not ARP.size())
        ARP.resize(KrKP.size(),SqrMat3(Eye()));
      AWK.resize(ARP.size());
      if(not Psi.size())
        Psi.resize(Phi.size(),Mat3xV(ne));
      if(not sigma0.size())
        sigma0.resize(KrKP.size());
      sigma.resize(KrKP.size());

      updateM_ = &FlexibleBodyFFR::updateMNotConst;
      updateLLM_ = &FlexibleBodyFFR::updateLLMNotConst;
    }
    else if(stage==unknownStage) {
      NodeBasedBody::init(stage);

      KJ[0].resize(6+ne,hSize[0]);
      KJ[1].resize(6+ne,hSize[1]);
      for(int i=0; i<ne; i++) {
        KJ[0](6+i,hSize[0]-ne+i) = 1;
        KJ[1](6+i,hSize[1]-ne+i) = 1;
      }

      q.resize(qSize);
      u.resize(uSize[0]);

      if(Me.size()==0)
        determineSID();
      prefillMassMatrix();

      K->getJacobianOfTranslation(1,false) = PJT[1];
      K->getJacobianOfRotation(1,false) = PJR[1];

      StateDependentFunction<RotMat3> *Atmp = dynamic_cast<StateDependentFunction<RotMat3>*>(fAPK);
      if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesXYZ<VecV>*>(Atmp->getFunction())) {
        if(bodyFixedRepresentationOfAngularVelocity)
          fTR = new RotationAboutAxesXYZTransformedMapping<VecV>;
        else
          fTR = new RotationAboutAxesXYZMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
      }
      else if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesZXZ<VecV>*>(Atmp->getFunction())) {
        if(bodyFixedRepresentationOfAngularVelocity)
          fTR = new RotationAboutAxesZXZTransformedMapping<VecV>;
        else
          fTR = new RotationAboutAxesZXZMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
      }
      else if(Atmp and coordinateTransformation and dynamic_cast<RotationAboutAxesZYX<VecV>*>(Atmp->getFunction())) {
        if(bodyFixedRepresentationOfAngularVelocity)
          THROW_MBSIMERROR("(FlexibleBodyFFR::init): coordinate transformation not yet available for zyx-rotation");
        else
          fTR = new RotationAboutAxesZYXMapping<VecV>;
        fTR->setParent(this);
        constJR = true;
        constjR = true;
        PJRR = SqrMat3(EYE);
        PJR[0].set(i02,iuR,PJRR);
      }

      if(fPrPK) {
        if(fPrPK->constParDer1()) {
          constJT = true;
          PJTT = fPrPK->parDer1(qTRel,0);
          PJT[0].set(i02,iuT,PJTT);
        }
        if(fPrPK->constParDer2()) {
          constjT = true;
          PjhT = fPrPK->parDer2(qTRel,0);
        }
      }
      if(fAPK) {
        if(fAPK->constParDer1()) {
          constJR = true;
          PJRR = fTR?fAPK->parDer1(qRRel,0)*(*fTR)(qRRel):fAPK->parDer1(qRRel,0);
          PJR[0].set(i02,iuR,PJRR);
        }
        if(fAPK->constParDer2()) {
          constjR = true;
          PjhR = fAPK->parDer2(qRRel,0);
        }
      }

      if(bodyFixedRepresentationOfAngularVelocity) {
        frameForJacobianOfRotation = K;
        // TODO: do not invert generalized mass matrix in case of special
        // parametrisation
//        if(K == C && dynamic_cast<DynamicSystem*>(R->getParent())) {
//          if(fPrPK) {
//            fPrPK->updateJacobian(qRel(iqT),0);
//            PJT[0].set(i02,iuT,fPrPK->getJacobian());
//          }
//          if(fAPK) {
//            fAPK->updateJacobian(qRel(iqR),0);
//            PJR[0].set(i02,iuR,fAPK->getJacobian());
//          }
//          updateM_ = &FlexibleBodyFFR::updateMConst;
//          Mbuf = m*JTJ(PJT[0]) + JTMJ(SThetaS,PJR[0]);
//          LLM[0] = facLL(Mbuf);
//          facLLM_ = &FlexibleBodyFFR::facLLMConst;
//        }
      }
      else
        frameForJacobianOfRotation = R;

      T.init(Eye());
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
        if(getPlotFeature(notMinimalState)==enabled) {
          for(int i=0; i<nq; i++)
            plotColumns.push_back("qRel("+numtostr(i)+")");
          for(int i=0; i<nu[0]; i++)
            plotColumns.push_back("uRel("+numtostr(i)+")");
        }
        if(getPlotFeature(globalPosition)==enabled) {
          for(unsigned int i=0; i<WrOP.size(); i++)
            for(int j=0; j<3; j++)
              plotColumns.push_back("WrOP["+numtostr(int(i))+"]("+numtostr(j)+")");
        }
        if(dynamic_pointer_cast<OpenMBV::DynamicIndexedFaceSet>(openMBVBody)) {
          dynamic_pointer_cast<OpenMBV::DynamicIndexedFaceSet>(openMBVBody)->setNumberOfVertexPositions(ombvNodes.size());
          if(ombvIndices.size())
            dynamic_pointer_cast<OpenMBV::DynamicIndexedFaceSet>(openMBVBody)->setIndices(ombvIndices);
        }
        NodeBasedBody::init(stage);
      }
    }
    else
      NodeBasedBody::init(stage);
    if(fTR) fTR->init(stage);
    if(fPrPK) fPrPK->init(stage);
    if(fAPK) fAPK->init(stage);
  }

  void FlexibleBodyFFR::initz() {
    Object::initz();
    qRel>>q;
    uRel>>u;
    TRel>>T;
  }

  void FlexibleBodyFFR::setUpInverseKinetics() {
    InverseKineticsJoint *joint = new InverseKineticsJoint(string("Joint_")+R->getParent()->getName()+"_"+name);
    static_cast<DynamicSystem*>(parent)->addInverseKineticsLink(joint);
    joint->setForceDirection(Mat3xV(3,EYE));
    joint->setMomentDirection(Mat3xV(3,EYE));
    joint->connect(R,K);
    joint->setBody(this);
  }

  void FlexibleBodyFFR::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      if(getPlotFeature(notMinimalState)==enabled) {
        for(int i=0; i<nq; i++)
          plotVector.push_back(qRel.e(i));
        for(int i=0; i<nu[0]; i++)
          plotVector.push_back(uRel.e(i));
      }

      if(getPlotFeature(globalPosition)==enabled) {
        for(unsigned int i=0; i<WrOP.size(); i++) {
          Vec3 WrOP = evalNodalPosition(i);
          for(int j=0; j<3; j++)
            plotVector.push_back(WrOP(j));
        }
      }
      if(getPlotFeature(openMBV)==enabled) {
        if(dynamic_pointer_cast<OpenMBV::DynamicIndexedFaceSet>(openMBVBody)) {
          vector<double> data;
          data.push_back(getTime());
          for(unsigned int i=0; i<ombvNodes.size(); i++) {
            Vec3 WrOP = evalNodalPosition(ombvNodes[i]);
            Vector<Fixed<6>, double> sigma = evalNodalStress(ombvNodes[i]);
              for(int j=0; j<3; j++)
                data.push_back(WrOP(j));
              //data.push_back(nrm2(disp[ombvNodes[i]]));
              data.push_back(fabs(sigma(0)));
          }
          dynamic_pointer_cast<OpenMBV::DynamicIndexedFaceSet>(openMBVBody)->append(data);
        }
      }
      NodeBasedBody::plot();
    }
  }

  void FlexibleBodyFFR::updateMb() {
    Vec3 mc = rdm + Pdm*q(iqE);
    SqrMat3 mtc = tilde(mc);

    SymMat3 I1;
    SqrMat3 I2;
    for (int i=0; i<ne; i++) {
      I1 += mmi1[i]*q(iqE).e(i);
      for (int j=0; j<ne; j++)
        I2 += mmi2[i][j]*(q(iqE).e(i)*q(iqE).e(j));
    }

    SymMat3 I = mmi0 + I1 + SymMat3(I2);
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        M_.e(i+3,j+3) = I.e(i,j);
        M_.e(i+3,j) = mtc.e(i,j);
      }
    }

    MatVx3 Cr1_(ne,NONINIT);
    for(int i=0; i<3; i++)
      Cr1_.set(i,Cr1[i]*q(iqE));
    for(int i=0; i<ne; i++)
      for(int j=0; j<3; j++)
        M_.e(i+6,j+3) = Cr0.e(i,j) + Cr1_.e(i,j);

    MatVx3 Ct_ = Ct0;
    if(Ct1.size()) {
      MatVx3 Ct1_(ne,NONINIT);
      for(int i=0; i<3; i++)
        Ct1_.set(i,Ct1[i]*q(iqE));
      Ct_ += Ct1_;
      for(int i=0; i<ne; i++)
        for(int j=0; j<3; j++)
          M_.e(i+6,j) = Ct_.e(i,j);
    }

    Matrix<General,Var,Fixed<6>,double> Oe_(ne,NONINIT), Oe1_(ne,NONINIT);
    for(int i=0; i<6; i++)
      Oe1_.set(i,Oe1[i]*q(iqE));

    Oe_ = Oe0 + Oe1_;

    vector<SqrMat3> Gr1_(ne);
    SqrMat3 hom21;
    for(int i=0; i<ne; i++) {
      Gr1_[i].init(0);
      for(int j=0; j<ne; j++)
        Gr1_[i] += Gr1[j][i]*q(iqE).e(j);
      hom21 += (Gr0[i]+Gr1_[i])*u(iuE).e(i);
    }
    MatVx3 Ge_(ne,NONINIT);
    for(int i=0; i<3; i++)
      Ge_.set(i,Ge[i]*u(iuE));

    Vec3 om = K->evalOrientation().T()*K->evalAngularVelocity();
    Vector<Fixed<6>,double> omq;
    for(int i=0; i<3; i++)
      omq.e(i) = pow(om.e(i),2);
    omq.e(3) = om.e(0)*om.e(1); 
    omq.e(4) = om.e(1)*om.e(2); 
    omq.e(5) = om.e(0)*om.e(2);

    VecV hom(6+ne), hg(6+ne), he(6+ne);

    hom.set(RangeV(0,2),-crossProduct(om,crossProduct(om,mc))-2.*crossProduct(om,Ct_.T()*u(iuE)));
    hom.set(RangeV(3,5),-(hom21*om) - crossProduct(om,I*om));
    hom.set(RangeV(6,6+ne-1),-(Ge_*om) - Oe_*omq);

    Vec Kg = K->getOrientation().T()*(MBSimEnvironment::getInstance()->getAccelerationOfGravity());
    hg.set(RangeV(0,2),m*Kg);
    hg.set(RangeV(3,5),mtc*Kg);
    hg.set(RangeV(6,hg.size()-1),Ct_*Kg);

    SqrMatV Ke_ = SqrMatV(Ke0);
    for(unsigned int i=0; i<Ke1.size(); i++) {
      Ke_ += Ke1[i]*q(iqE).e(i);
      for(unsigned int j=0; j<Ke2.size(); j++)
        Ke_ += Ke2[i][j]*(q(iqE).e(i)*q(iqE).e(j));
    }

    VecV ke = Ke_*q(iqE) + De0*qd(iqE);

    if(ksigma0.size())
      ke += ksigma0;
    if(ksigma1.size())
      ke += ksigma1*q(iqE);

    he.set(RangeV(6,hg.size()-1),ke);

    h_ = hom + hg - he;

    updMb = false;
  }

  void FlexibleBodyFFR::updateqd() {
    qd(iqT) = uRel(iuT);
    if(fTR)
      qd(iqR) = (*fTR)(qRel(iuR))*uRel(iuR);
    else
      qd(iqR) = uRel(iuR);
    qd(iqE) = uRel(iuE);
  }

  void FlexibleBodyFFR::updatedq() {
    dq(iqT) = uRel(iuT)*getStepSize();
    if(fTR)
      dq(iqR) = (*fTR)(qRel(iuR))*uRel(iuR)*getStepSize();
    else
      dq(iqR) = uRel(iuR)*getStepSize();
    dq(iqE) = uRel(iuE)*getStepSize();
  }
  void FlexibleBodyFFR::updateT() {
    if(fTR) TRel(iqR,iuR) = (*fTR)(qRel(iuR));
  }

  void FlexibleBodyFFR::updateGeneralizedCoordinates() {
    qTRel = qRel(iqT);
    qRRel = qRel(iqR);
    uTRel = uRel(iuT);
    uRRel = uRel(iuR);
    updGC = false;
  }

 void FlexibleBodyFFR::updatePositions() {
    if(fPrPK) PrPK = (*fPrPK)(evalqTRel(),getTime());
    if(fAPK) APK = (*fAPK)(evalqRRel(),getTime());
    WrPK = R->evalOrientation()*PrPK;
    updPos = false;
  }

  void FlexibleBodyFFR::updateVelocities() {
    if(fPrPK) WvPKrel = R->evalOrientation()*(evalPJTT()*evaluTRel() + PjhT);
    if(fAPK) WomPK = frameForJacobianOfRotation->evalOrientation()*(evalPJRR()*evaluRRel() + PjhR);
    updVel = false;
  }

  void FlexibleBodyFFR::updateJacobians() {
    if(fPrPK) {
      if(!constJT) {
        PJTT = fPrPK->parDer1(evalqTRel(),getTime());
        PJT[0].set(i02,iuT,PJTT);
      }
      if(!constjT)
        PjhT = fPrPK->parDer2(evalqTRel(),getTime());
    }

    if(fAPK) {
      if(!constJR) {
        PJRR = fTR?fAPK->parDer1(evalqRRel(),getTime())*(*fTR)(evalqRRel()):fAPK->parDer1(evalqRRel(),getTime());
        PJR[0].set(i02,iuR,PJRR);
      }
      if(!constjR)
        PjhR = fAPK->parDer2(evalqRRel(),getTime());
    }
     updPJ = false;
  }

  void FlexibleBodyFFR::updateGyroscopicAccelerations() {
    VecV qdTRel = evaluTRel();
    VecV qdRRel = fTR ? (*fTR)(qRRel)*uRRel : uRRel;
    if(fPrPK) {
      if(not(constJT and constjT)) {
        PjbT = (fPrPK->parDer1DirDer1(qdTRel,qTRel,getTime())+fPrPK->parDer1ParDer2(qTRel,getTime()))*uTRel + fPrPK->parDer2DirDer1(qdTRel,qTRel,getTime()) + fPrPK->parDer2ParDer2(qTRel,getTime());
      }
    }
    if(fAPK) {
      if(not(constJR and constjR)) {
        if(fTR) {
          Mat3xV JRd = (fAPK->parDer1DirDer1(qdRRel,qRRel,getTime())+fAPK->parDer1ParDer2(qRRel,getTime()));
          MatV TRd = fTR->dirDer(qdRRel,qRRel);
          PjbR = JRd*qdRRel + fAPK->parDer1(qRRel,getTime())*TRd*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,getTime()) + fAPK->parDer2ParDer2(qRRel,getTime());
        }
        else
          PjbR = (fAPK->parDer1DirDer1(qdRRel,qRRel,getTime())+fAPK->parDer1ParDer2(qRRel,getTime()))*uRRel + fAPK->parDer2DirDer1(qdRRel,qRRel,getTime()) + fAPK->parDer2ParDer2(qRRel,getTime());
      }
    }
    updPjb = false;
  }

  void FlexibleBodyFFR::updateKJ0() {
    KJ[0].set(RangeV(0,2),RangeV(0,KJ[0].cols()-1),K->evalOrientation().T()*K->evalJacobianOfTranslation());
    KJ[0].set(RangeV(3,5),RangeV(0,KJ[0].cols()-1),K->getOrientation().T()*K->getJacobianOfRotation());
    Ki.set(RangeV(0,2),K->getOrientation().T()*K->evalGyroscopicAccelerationOfTranslation());
    Ki.set(RangeV(3,5),K->getOrientation().T()*K->getGyroscopicAccelerationOfRotation());
    updKJ[0] = false;
  }

  void FlexibleBodyFFR::updateKJ1() {
    KJ[1].set(RangeV(0,2),RangeV(0,KJ[1].cols()-1),K->evalOrientation().T()*K->evalJacobianOfTranslation(1));
    KJ[1].set(RangeV(3,5),RangeV(0,KJ[1].cols()-1),K->getOrientation().T()*K->getJacobianOfRotation(1));
    updKJ[1] = false;
  }

  void FlexibleBodyFFR::updatePositions(Frame *frame) {
    frame->setPosition(R->getPosition() + evalGlobalRelativePosition());
    frame->setOrientation(R->getOrientation()*APK); // APK already update to date
  }

 void FlexibleBodyFFR::updateVelocities(Frame *frame) {
    frame->setAngularVelocity(R->evalAngularVelocity() + evalGlobalRelativeAngularVelocity());
    frame->setVelocity(R->getVelocity() + WvPKrel + crossProduct(R->getAngularVelocity(),evalGlobalRelativePosition())); // WvPKrel already update to date
  }

  void FlexibleBodyFFR::updateAccelerations(Frame *frame) {
    frame->setAcceleration(K->evalJacobianOfTranslation()*evaludall() + K->evalGyroscopicAccelerationOfTranslation());
    frame->setAngularAcceleration(K->evalJacobianOfRotation()*udall + K->evalGyroscopicAccelerationOfRotation());
  }

  void FlexibleBodyFFR::updateJacobians0(Frame *frame) {
    frame->getJacobianOfTranslation(0,false).init(0);
    frame->getJacobianOfRotation(0,false).init(0);
    frame->getJacobianOfTranslation(0,false).set(i02,RangeV(0,R->gethSize()-1), R->evalJacobianOfTranslation() - tilde(evalGlobalRelativePosition())*R->evalJacobianOfRotation());
    frame->getJacobianOfRotation(0,false).set(i02,RangeV(0,R->gethSize()-1), R->evalJacobianOfRotation());
    frame->getJacobianOfTranslation(0,false).add(i02,RangeV(gethSize(0)-getuSize(0),gethSize(0)-1), R->evalOrientation()*evalPJT());
    frame->getJacobianOfRotation(0,false).add(i02,RangeV(gethSize(0)-getuSize(0),gethSize(0)-1), frameForJacobianOfRotation->evalOrientation()*PJR[0]);
  }

  void FlexibleBodyFFR::updateGyroscopicAccelerations(Frame *frame) {
    frame->setGyroscopicAccelerationOfTranslation(R->evalGyroscopicAccelerationOfTranslation() + crossProduct(R->evalGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition()) + R->evalOrientation()*evalPjbT() + crossProduct(R->evalAngularVelocity(), 2.*evalGlobalRelativeVelocity()+crossProduct(R->evalAngularVelocity(),evalGlobalRelativePosition())));
    frame->setGyroscopicAccelerationOfRotation(R->evalGyroscopicAccelerationOfRotation() + frameForJacobianOfRotation->evalOrientation()*PjbR + crossProduct(R->evalAngularVelocity(), evalGlobalRelativeAngularVelocity())); // PjbR already up to date
  }

  void FlexibleBodyFFR::resetUpToDate() {
    NodeBasedBody::resetUpToDate();
    updPjb = true;
    updGC = true;
    updT = true;
    updMb = true;
    updKJ[0] = true;
    updKJ[1] = true;
    for(unsigned int i=0; i<updNodalPos.size(); i++) {
      updNodalPos[i] = true;
      updNodalVel[i] = true;
      updNodalStress[i] = true;
    }
  }

  void FlexibleBodyFFR::updateqRef(const Vec& ref) {
    NodeBasedBody::updateqRef(ref);
    qRel>>q;
  }

  void FlexibleBodyFFR::updateuRef(const Vec& ref) {
    NodeBasedBody::updateuRef(ref);
    uRel>>u;
  }

  void FlexibleBodyFFR::updateudRef(const Vec& ref) {
    NodeBasedBody::updateudRef(ref);
  }

  void FlexibleBodyFFR::setPositionShapeFunctionIntegral(const MatV &rPdm_) {
    rPdm.resize(3);
    for(int i=0; i<3; i++)
      rPdm[i] = rPdm_(RangeV(3*i,3*i+2),RangeV(0,rPdm_.cols()-1));
  }

  void FlexibleBodyFFR::setShapeFunctionShapeFunctionIntegral(const MatV &PPdm_) {
    PPdm.resize(3, vector<SqrMatV>(3));
    int n = PPdm_.cols();
    int k = 0;
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        PPdm[i][j].resize() = PPdm_(RangeV(n*k,n*k+n-1),RangeV(0,n-1));
        k++;
      }
    }
  }

  void FlexibleBodyFFR::setNonlinearStiffnessMatrixOfFirstOrder(const MatV &Knl1_) {
    int n = Knl1_.cols();
    Knl1.resize(n);
    for(int i=0; i<n; i++)
      Knl1[i] = Knl1_(RangeV(n*i,n*i+n-1),RangeV(0,n-1));
  }

  void FlexibleBodyFFR::setNonlinearStiffnessMatrixOfSecondOrder(const MatV &Knl2_) {
    int n = Knl2_.cols();
    int k = 0;
    Knl2.resize(n);
    for(int i=0; i<n; i++) {
      Knl2[i].resize(n);
      for(int j=0; j<n; j++) {
        Knl2[i][j] = Knl2_(RangeV(n*k,n*k+n-1),RangeV(0,n-1));
        k++;
      }
    }
  }

  void FlexibleBodyFFR::setGeometricStiffnessMatrixDueToAcceleration(const MatV &K0t_) {
    int n = K0t_.cols();
    K0t.resize(3);
    for(int i=0; i<3; i++)
      K0t[i] = K0t_(RangeV(n*i,n*i+n-1),RangeV(0,n-1));
  }

  void FlexibleBodyFFR::setGeometricStiffnessMatrixDueToAngularAcceleration(const MatV &K0r_) {
    int n = K0r_.cols();
    K0r.resize(3);
    for(int i=0; i<3; i++)
      K0r[i] = K0r_(RangeV(n*i,n*i+n-1),RangeV(0,n-1));
  }

  void FlexibleBodyFFR::setGeometricStiffnessMatrixDueToAngularVelocity(const MatV &K0om_) {
    int n = K0om_.cols();
    K0om.resize(3);
    for(int i=0; i<3; i++)
      K0om[i] = K0om_(RangeV(n*i,n*i+n-1),RangeV(0,n-1));
  }

  void FlexibleBodyFFR::setNodalRelativePosition(const VecV &r) {
    KrKP.resize(r.size()/3);
    for(unsigned int i=0; i<KrKP.size(); i++)
      KrKP[i] = r(RangeV(3*i,3*i+2));
  }

  void FlexibleBodyFFR::setNodalRelativeOrientation(const MatVx3 &A) {
    ARP.resize(A.rows()/3);
    for(unsigned int i=0; i<ARP.size(); i++)
      ARP[i] = A(RangeV(3*i,3*i+2),RangeV(0,2));
  }

  void FlexibleBodyFFR::setNodalShapeMatrixOfTranslation(const MatV &Phi_) {
    Phi.resize(Phi_.rows()/3);
    for(unsigned int i=0; i<Phi.size(); i++)
      Phi[i] = Phi_(RangeV(3*i,3*i+2),RangeV(0,Phi_.cols()-1));
  }

  void FlexibleBodyFFR::setNodalShapeMatrixOfRotation(const MatV &Psi_) {
    Psi.resize(Psi_.rows()/3);
    for(unsigned int i=0; i<Psi.size(); i++)
      Psi[i] = Psi_(RangeV(3*i,3*i+2),RangeV(0,Psi_.cols()-1));
  }

  void FlexibleBodyFFR::setNodalStressMatrix(const MatV &sigmahel_) {
    sigmahel.resize(sigmahel_.rows()/6);
    for(unsigned int i=0; i<sigmahel.size(); i++)
      sigmahel[i] = sigmahel_(RangeV(6*i,6*i+5),RangeV(0,sigmahel_.cols()-1));
  }

  void FlexibleBodyFFR::setNodalNonlinearStressMatrix(const MatV &sigmahen_) {
    int n = sigmahen_.cols();
    int k = 0;
    sigmahen.resize(sigmahen_.rows()/6/n);
    for(unsigned int i=0; i<sigmahen.size(); i++) {
      sigmahen[i].resize(n);
      for(int j=0; j<n; j++) {
        sigmahen[i][j] = sigmahen_(RangeV(6*k,6*k+5),RangeV(0,n-1));
        k++;
      }
    }
  }

  void FlexibleBodyFFR::setNodalInitialStress(const VecV &sigma0_) {
    sigma0.resize(sigma0_.rows()/6);
    for(unsigned int i=0; i<sigma0.size(); i++)
      sigma0[i] = sigma0_(RangeV(6*i,6*i+5));
  }

  void FlexibleBodyFFR::setNodalGeometricStiffnessMatrixDueToForce(const MatV &K0F_) {
    int n = K0F_.cols();
    int k = 0;
    K0F.resize(K0F_.rows()/n/n);
    for(unsigned int i=0; i<K0F.size(); i++) {
      K0F.resize(n);
      for(int j=0; j<n; j++) {
        K0F[i][j] = K0F_(RangeV(n*k,n*k+n-1),RangeV(0,n-1));
        k++;
      }
    }
  }

  void FlexibleBodyFFR::setNodalGeometricStiffnessMatrixDueToMoment(const MatV &K0M_) {
    int n = K0M_.cols();
    int k = 0;
    K0M.resize(K0M_.rows()/n/n);
    for(unsigned int i=0; i<K0M.size(); i++) {
      K0M.resize(n);
      for(int j=0; j<n; j++) {
        K0M[i][j] = K0M_(RangeV(n*k,n*k+n-1),RangeV(0,n-1));
        k++;
      }
    }
  }

  void FlexibleBodyFFR::updateMConst() {
    M += Mbuf;
  }

  void FlexibleBodyFFR::updateMNotConst() {
    M += JTMJ(evalMb(),evalKJ());
  }

  void FlexibleBodyFFR::initializeUsingXML(DOMElement *element) {
    DOMElement *e;
    NodeBasedBody::initializeUsingXML(element);

    // frames
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"frames")->getFirstElementChild();
    while(e) {
      Frame *f=ObjectFactory::createAndInit<Frame>(e);
      addFrame(f);
      e=e->getNextElementSibling();
    }

    // contours
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"contours")->getFirstElementChild();
    while(e) {
      Contour *c=ObjectFactory::createAndInit<Contour>(e);
      addContour(c);
      e=e->getNextElementSibling();
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"mass");
    setMass(getDouble(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"positionIntegral");
    setPositionIntegral(getVec3(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"positionPositionIntegral");
    setPositionPositionIntegral(getSymMat3(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeFunctionIntegral");
    setShapeFunctionIntegral(getMat3xV(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"positionShapeFunctionIntegral");
    rPdm = vector<Mat3xV>(3);
    e=e->getFirstElementChild();
    if(E(e)->getTagName()==MBSIMFLEX%"ele") {
      for(int i=0; i<3; i++) {
        rPdm[i].resize() = getMat3xV(e);
        e=e->getNextElementSibling();
      }
    }
    else
      setPositionShapeFunctionIntegral(getMat(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"shapeFunctionShapeFunctionIntegral");
    PPdm = vector<vector<SqrMatV> >(3,vector<SqrMatV>(3));
    e=e->getFirstElementChild();
    if(E(e)->getTagName()==MBSIMFLEX%"row") {
      for(int i=0; i<3; i++) {
        DOMElement *ee=e->getFirstElementChild();
        for(int j=0; j<3; j++) {
          PPdm[i][j].resize() = getSqrMat(ee);
          ee=ee->getNextElementSibling();
        }
        e=e->getNextElementSibling();
      }
    }
    else
      setShapeFunctionShapeFunctionIntegral(getMat(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"stiffnessMatrix");
    setStiffnessMatrix(getSymMat(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"dampingMatrix");
    if(e) setDampingMatrix(getSymMat(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"proportionalDamping");
    if(e) setProportionalDamping(getVec(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfFirstOrder");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        while(e) {
          Knl1.push_back(getSqrMat(e));
          e=e->getNextElementSibling();
        }
      }
      else
        setNonlinearStiffnessMatrixOfFirstOrder(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearStiffnessMatrixOfSecondOrder");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"row") {
        while(e) {
          Knl2.push_back(vector<SqrMatV>());
          DOMElement *ee=e->getFirstElementChild();
          while(ee) {
            Knl2[Knl2.size()-1].push_back(getSqrMat(ee));
            ee=ee->getNextElementSibling();
          }
          e=e->getNextElementSibling();
        }
      }
      else
        setNonlinearStiffnessMatrixOfSecondOrder(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"initialStressIntegral");
    if(e) setInitialStressIntegral(getVec(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nonlinearInitialStressIntegral");
    if(e) setNonlinearInitialStressIntegral(getSqrMat(e));

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAcceleration");
    if(e) {
      K0t = vector<SqrMatV>(3);
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        for(int i=0; i<3; i++) {
          K0t[i].resize() = getSqrMat(e);
          e=e->getNextElementSibling();
        }
      }
      else
        setGeometricStiffnessMatrixDueToAcceleration(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularAcceleration");
    if(e) {
      K0r = vector<SqrMatV>(3);
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        for(int i=0; i<3; i++) {
          K0r[i].resize() = getSqrMat(e);
          e=e->getNextElementSibling();
        }
      }
      else
        setGeometricStiffnessMatrixDueToAngularAcceleration(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"geometricStiffnessMatrixDueToAngularVelocity");
    if(e) {
      K0om = vector<SqrMatV>(3);
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        for(int i=0; i<3; i++) {
          K0om[i].resize() = getSqrMat(e);
          e=e->getNextElementSibling();
        }
      }
      else
        setGeometricStiffnessMatrixDueToAngularVelocity(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativePosition");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        while(e) {
          KrKP.push_back(getVec3(e));
          e=e->getNextElementSibling();
        }
      }
      else
        setNodalRelativePosition(getVec(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalRelativeOrientation");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        while(e) {
          ARP.push_back(getSqrMat3(e));
          e=e->getNextElementSibling();
        }
      }
      else
        if(e) setNodalRelativeOrientation(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfTranslation");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        while(e) {
          Phi.push_back(getMat3xV(e));
          e=e->getNextElementSibling();
        }
      }
      else
        if(e) setNodalShapeMatrixOfTranslation(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalShapeMatrixOfRotation");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        while(e) {
          Psi.push_back(getMat3xV(e));
          e=e->getNextElementSibling();
        }
      }
      else
        if(e) setNodalShapeMatrixOfRotation(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalStressMatrix");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        while(e) {
          sigmahel.push_back(getMat(e));
          e=e->getNextElementSibling();
        }
      }
      else
        if(e) setNodalStressMatrix(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalNonlinearStressMatrix");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"row") {
        while(e) {
          sigmahen.push_back(vector<Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> >());
          DOMElement *ee=e->getFirstElementChild();
          while(ee) {
            sigmahen[sigmahen.size()-1].push_back(getMat(ee));
            ee=ee->getNextElementSibling();
          }
          e=e->getNextElementSibling();
        }
      }
      else
        setNodalNonlinearStressMatrix(getMat(e));
    }

    e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodalInitialStress");
    if(e) {
      e=e->getFirstElementChild();
      if(E(e)->getTagName()==MBSIMFLEX%"ele") {
        while(e) {
          sigma0.push_back(getVec3(e));
          e=e->getNextElementSibling();
        }
      }
      else
        setNodalInitialStress(getVec(e));
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"generalTranslation");
    if(e && e->getFirstElementChild()) {
      MBSim::Function<Vec3(VecV,double)> *trans=ObjectFactory::createAndInit<MBSim::Function<Vec3(VecV,double)> >(e->getFirstElementChild());
      setGeneralTranslation(trans);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"timeDependentTranslation");
    if(e && e->getFirstElementChild()) {
      MBSim::Function<Vec3(double)> *trans=ObjectFactory::createAndInit<MBSim::Function<Vec3(double)> >(e->getFirstElementChild());
      setTimeDependentTranslation(trans);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"stateDependentTranslation");
    if(e && e->getFirstElementChild()) {
      MBSim::Function<Vec3(VecV)> *trans=ObjectFactory::createAndInit<MBSim::Function<Vec3(VecV)> >(e->getFirstElementChild());
      setStateDependentTranslation(trans);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"generalRotation");
    if(e && e->getFirstElementChild()) {
      MBSim::Function<RotMat3(VecV,double)> *rot=ObjectFactory::createAndInit<MBSim::Function<RotMat3(VecV,double)> >(e->getFirstElementChild());
      setGeneralRotation(rot);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"timeDependentRotation");
    if(e && e->getFirstElementChild()) {
      MBSim::Function<RotMat3(double)> *rot=ObjectFactory::createAndInit<MBSim::Function<RotMat3(double)> >(e->getFirstElementChild());
      setTimeDependentRotation(rot);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"stateDependentRotation");
    if(e && e->getFirstElementChild()) {
      MBSim::Function<RotMat3(VecV)> *rot=ObjectFactory::createAndInit<MBSim::Function<RotMat3(VecV)> >(e->getFirstElementChild());
      setStateDependentRotation(rot);
    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"translationDependentRotation");
    if(e) translationDependentRotation = getBool(e);
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"coordinateTransformationForRotation");
    if(e) coordinateTransformation = getBool(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"bodyFixedRepresentationOfAngularVelocity");
    if(e) bodyFixedRepresentationOfAngularVelocity = getBool(e);

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      DOMElement *ee=E(e)->getFirstElementChildNamed(MBSIMFLEX%"nodes");
      Vec nodes;
      if(ee) nodes=getVec(ee);
      ombvNodes.resize(nodes.size());
      for(int i=0; i<nodes.size(); i++)
        ombvNodes[i] = static_cast<Index>(nodes(i))-1;
      ee=E(e)->getFirstElementChildNamed(MBSIMFLEX%"indices");
      Vec indices;
      if(ee) indices=getVec(ee);
      ombvIndices.resize(indices.size());
      for(int i=0; i<indices.size(); i++)
        ombvIndices[i] = static_cast<Index>(indices(i))-1;
      OpenMBVDynamicIndexedFaceSet ombv;
      openMBVBody=ombv.createOpenMBV(e);
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBVFrameK");
    if(e) {
//      if(!openMBVBody) setOpenMBVRigidBody(OpenMBV::ObjectFactory::create<OpenMBV::InvisibleBody>());
      OpenMBVFrame ombv;
      K->setOpenMBVFrame(ombv.createOpenMBV(e));
    }

    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"plotFeatureFrameK");
    while(e and E(e)->getTagName()==MBSIMFLEX%"plotFeatureFrameK") {
      PlotFeatureStatus status = initializePlotFeatureStatusUsingXML(e);
      PlotFeature feature = initializePlotFeatureUsingXML(e);
      K->setPlotFeature(feature, status);
      e=e->getNextElementSibling();
    }
  }

  void FlexibleBodyFFR::updateStresses(int j) {
    sigma[j] = sigma0[j];
    if(sigmahel[j].cols()) {
      Matrix<General, Fixed<6>, Var, double> sigmahe = sigmahel[j];
      for(unsigned int i=0; i<sigmahen.size(); i++)
        sigmahe += sigmahen[j][i]*q(iqE)(i);
      sigma[j] += sigmahe*q(iqE);
    }
    updNodalStress[j] = false;
  }

  void FlexibleBodyFFR::updatePositions(int i) {
    AWK[i] = K->evalOrientation()*ARP[i]*(Id+tilde(Psi[i]*q(iqE)));
    disp[i] = Phi[i]*q(iqE);
    WrRP[i] = K->getOrientation()*(KrKP[i]+disp[i]);
    WrOP[i] = K->getPosition() + WrRP[i];
    updNodalPos[i] = false;
  }

  void FlexibleBodyFFR::updateVelocities(int i) {
    Womrel[i] = K->evalOrientation()*(Psi[i]*u(iqE));
    Wvrel[i] = K->getOrientation()*(Phi[i]*u(iqE));
    updNodalVel[i] = false;
  }

  void FlexibleBodyFFR::updatePositions(NodeFrame* frame) {
    Index i = frame->getNodeNumber();
    frame->setPosition(evalNodalPosition(i));
    frame->setOrientation(AWK[i]);
 }

  void FlexibleBodyFFR::updateVelocities(NodeFrame* frame) {
    Index i = frame->getNodeNumber();
    frame->setAngularVelocity(K->evalAngularVelocity() + evalGlobalRelativeAngularVelocity(i));
    frame->setVelocity(K->getVelocity() + crossProduct(K->getAngularVelocity(), evalGlobalRelativePosition(i)) + Wvrel[i]);
  }

  void FlexibleBodyFFR::updateAccelerations(NodeFrame* frame) {
    Index i = frame->getNodeNumber();
    frame->setAngularAcceleration(K->evalAngularAcceleration() +  crossProduct(K->evalAngularVelocity(),evalGlobalRelativeAngularVelocity(i)) + K->evalOrientation()*(Psi[i]*ud(iqE)));
    frame->setAcceleration(K->getAcceleration() + crossProduct(K->getAngularAcceleration(), evalGlobalRelativePosition(i)) + crossProduct(K->getAngularVelocity(), crossProduct(K->getAngularVelocity(), evalGlobalRelativePosition(i))) + 2.*crossProduct(K->getAngularVelocity(), Wvrel[i]) + K->getOrientation()*(Phi[i]*ud(iqE)));
  }

  void FlexibleBodyFFR::updateJacobians(NodeFrame* frame, int j) {
    Index i = frame->getNodeNumber();
    Mat3xV Phi_ = Phi[i];
    if(K0F.size() and K0F[i].size()) {
      MatVx3 PhigeoT(ne,NONINIT);
      for(int k=0; k<3; k++)
        PhigeoT.set(k,K0F[i][k]*q);
      Phi_ += PhigeoT.T();
    }
    Mat3xV Psi_ = Psi[i];
    if(K0M.size() and K0M[i].size()) {
      MatVx3 PsigeoT(ne,NONINIT);
      for(int k=0; k<3; k++)
        PsigeoT.set(k,K0M[i][k]*q);
      Psi_ += PsigeoT.T();
    }
    frame->setJacobianOfRotation(K->getJacobianOfRotation(j));
    frame->getJacobianOfRotation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-ne,gethSize(j)-1),K->evalOrientation()*Psi_);
    frame->setJacobianOfTranslation(K->getJacobianOfTranslation(j) - tilde(evalGlobalRelativePosition(i))*K->getJacobianOfRotation(j));
    frame->getJacobianOfTranslation(j,false).add(RangeV(0,2),RangeV(gethSize(j)-ne,gethSize(j)-1),K->getOrientation()*Phi_);
  }

  void FlexibleBodyFFR::updateGyroscopicAccelerations(NodeFrame* frame) {
    Index i = frame->getNodeNumber();
    frame->setGyroscopicAccelerationOfRotation(K->evalGyroscopicAccelerationOfRotation() + crossProduct(K->evalAngularVelocity(),evalGlobalRelativeAngularVelocity(i)));
    frame->setGyroscopicAccelerationOfTranslation(K->getGyroscopicAccelerationOfTranslation() + crossProduct(K->getGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition(i)) + crossProduct(K->getAngularVelocity(),crossProduct(K->getAngularVelocity(),evalGlobalRelativePosition(i))) + 2.*crossProduct(K->getAngularVelocity(),Wvrel[i]));
  }

}
