/* Copyright (C) 2005-2006  Rainer Britz, Roland Zander
 
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
 * Contact:
 *    rzander@users.berlios.de
 *
 */
#include <config.h>
#define FMATVEC_DEEP_COPY
#include "body_flexible_1s_01_torsion.h"
#include "frame.h"
#include "dynamic_system_solver.h"
#include "contact_flexible.h"
#include "contact_rigid.h"
#include "contour.h"

namespace MBSim {

  BodyFlexible1s01Torsion::BodyFlexible1s01Torsion(const string &name)
    :BodyFlexible1s(name),
    E(0),rho(0),A(0),I(0),
    Wt(3,2), Wn(3), CrOC(3), CvC(3),
    WrON00(3), WrON0(3){ 

    }

  void BodyFlexible1s01Torsion::setNumberShapeFunctions(int n_) {
    n = n_;

    qSize = n;
    uSize = qSize;

    q0.resize(qSize);
    u0.resize(uSize);
  }

  void BodyFlexible1s01Torsion::init(InitStage stage) {
    if(stage==unknownStage) {
      BodyFlexible1s::init(stage);
      assert(0<n);

      JT = Mat(0,0);

      initMatrizes();
    }
    else
      BodyFlexible1s::init(stage);
  }


  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------
  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------
  void BodyFlexible1s01Torsion::initMatrizes()
  {
    //Massenmatrix
    K=SymMat(n,INIT,0.0);
    for(int i=1;i<=n;i++){
      for(int j =i; j<=n;j++){
	M(i-1,j-1)=1.0/(double (i+j-1));
	if(i!=1) K(i-1,j-1)=1.0/(double (i+j-3));
      }
    }
    LLM = facLL(M);
  }

  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //------------------------------------------------------------------------------
  void BodyFlexible1s01Torsion::updateh(double t)
  {
    h = -K*q;

    sumUpForceElements(t);
  }


  //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------

  void BodyFlexible1s01Torsion::updateStateDependentVariables(double t) {
    sTangent = Vec(0);

    WrON0 = WrON00; // + JT.col(2)*q(0); // verschiebung in z-Richtung
    updatePorts(t);
    //updateContours(t);
  }

  void BodyFlexible1s01Torsion::updatePorts(double t) {
    for(unsigned int i=0; i<frame.size(); i++) {
      double s=S_Port[i].alpha(0);
      /* Ermittlung der Position in Achsrichtung  ist konstant */
      frame[i]->setWrOP(WrON00 + Axis*s*l);
      /* Ermittlung der Geschwindigkeiten */
      frame[i]->setWvP (Vec(3,INIT,0)); 
      /* Winkelgeschwindigkeit der Welle
       * am Ort des Ports.
       * Transformierte der Jacobimatrix*generalisierte Geschwindigkeiten
       * Jacobimatrix entspricht den Ansatzfunktionen
       */
      frame[i]->setWomegaP(Axis*trans( computeJacobianMatrix(S_Port[i]) )*u); 
    }
  }

  Mat BodyFlexible1s01Torsion::computeJacobianMatrix(const ContourPointData &S_) {
    double s = S_.alpha(0);
    Mat Jacobian(n,1);

    for(int i =0;i<n;i++){
      if(i==0)
	Jacobian(i,0)=1;
      else
	Jacobian(i,0)=pow(s,i);
    }
    return Jacobian;  
  }

}

  //------------------------------------------------------------------------------
  // ENDE
