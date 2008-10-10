/* Copyright (C) 2004-2006  Martin Förg, Robert Huber

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
 *   mfoerg@users.berlios.de
 *
 */

#include<config.h>
#include<ctime>
#include<fstream>
#include "multi_body_system.h"
#include "fortran_wrapper.h"
#include "radau5dae_integrator.h"
#include "eps.h"

#ifndef NO_ISO_14882
using namespace std;
#endif

namespace MBSim {

  RADAU5DAEIntegrator::RADAU5DAEIntegrator() : aTol(1,INIT,1e-6), rTol(1,INIT,1e-6), dt0(0), maxSteps(0), dtMax(0), DAEIndex(2), useExternalJac(0)  {
    name = "RADAU4DAE";
  }

  double RADAU5DAEIntegrator::tPlot = 0;
  double RADAU5DAEIntegrator::dtOut = 0;
  Vec RADAU5DAEIntegrator::zInp;
  ofstream RADAU5DAEIntegrator::integPlot;
  double RADAU5DAEIntegrator::s0;
  double RADAU5DAEIntegrator::time = 0;
  bool RADAU5DAEIntegrator::output_;

  void RADAU5DAEIntegrator::setDAEIndex(int index) {	// Index 2 or 3 or Index 2 with Gear Gupta Leimkuhler Stabilisation (21)
    DAEIndex = index;
    if (!(DAEIndex == 21)) {
      assert(DAEIndex<4);
      assert(DAEIndex>1);
    }
  }

  void RADAU5DAEIntegrator::fdae(int* YSize, double* t, double* Y_, double* F_, double* rpar, int* ipar) {
    Vec Y(*YSize, Y_);
    Vec F(*YSize, F_);
    system->F_DAE(Y, F, *t, *ipar);	//ipar[0]: DAEIndex
  }

  void RADAU5DAEIntegrator::mdae(int* YSize, double* MasMat_, int* lMas, double* rpar, int* ipar) {
    Mat MasMat(*lMas,*YSize,MasMat_);
    int zSize = *(ipar+1);
    for (int i=0; i<zSize; i++) MasMat(0,i)=1.0;
    for (int i=zSize; i<*YSize; i++) MasMat(0,i) =0.0;
  } 

  void RADAU5DAEIntegrator::jac(int* YSize, double* t, double* Y_, double* jac_, int* ljac_, double* rpar, int* ipar) {
    Vec Y(*YSize, Y_);
    Mat Jacobian(*YSize, *YSize, jac_);
    system->JacF_DAE(*t, Y, Jacobian, *ipar);
  }
  void  RADAU5DAEIntegrator::plot(int* nr, double* told, double* t, double* z, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn) {

    while(*t >= tPlot) {
      for(int i=1; i<=*n; i++)
	zInp(i-1) = CONTR5(&i,&tPlot,cont,lrc);
      system->plotDAE(zInp, tPlot, *ipar);
      if(output_)
	cout << "   t = " <<  tPlot << ",\tdt = "<< *t-*told << "\r"<<flush;

      double s1 = clock();
      time += (s1-s0)/CLOCKS_PER_SEC;
      s0 = s1; 

      integPlot<< tPlot << " " << *t-*told << " " << time << endl;
      tPlot += dtOut;
    }
  }

  void RADAU5DAEIntegrator::integrate(MultiBodySystem& system_) {

    system            = &system_;
    int zSize         = system->getzSize();
    int laBiSize      = system->getlaBilateralSize();
    int YSize         = zSize + laBiSize;
    if (DAEIndex==21) YSize += laBiSize;
    double t = tStart;

    Vec Y(YSize,INIT,0.0);
    Vec z;
    z>> Y(0,zSize-1);

    if(z0.size())
      Y(0,zSize-1) = z0;
    else
      system->initz(z);

    assert(aTol.size() == rTol.size());

    int iTol;
    if(aTol.size() == 1) iTol = 0; // Skalar
    else { 
      iTol = 1; // Vektor
      assert (aTol.size() >= YSize);
    }

    int iout = 1; // Subroutine for output

    Vec rPar(1);
    int iPar[3]; 			// zur Uebergabe an Funktionen
    iPar[0] = DAEIndex;    
    iPar[1] = zSize;
    iPar[2] = YSize;
    int iJac  = 0;  			// 0: Jacobi Matrix dF/dY wird über finite Differenzen bestimmt; 1: über Subroutine
    if (useExternalJac) iJac = 1;
    int mlJac = YSize; 			// N: vollbesetzte Jacobi; <N: Jacobi hat Bandstruktur
    int muJac = 0; 			// oberer Bandweite der Jacobi; nicht von Bedeutung falls mlJac = N
    int iMas  = 1; 			// 0:M =EYE;	1:Subroutine fuer M: MAS(N,AM,LMAS,RPAR,IPAR)
    int mlMas = 0; 			// lower Bandwith
    int muMas = 0; 			// upper Bandwith

    int lJac = YSize; 			// zur Berechnung der Laenge des Working Space
    int lMas = mlMas + muMas +1;
    int le = YSize; 

    int lWork = (YSize*(lJac+lMas + 3*le + 12) +20 ); 
    int liWork = 3*YSize +20;
    Vector<int> iWork(liWork);
    Vec work(lWork);

    work(0) = macheps;  		// rounding unit
    if(dtMax) work(6) = dtMax;

    iWork(1)=maxSteps;					//Maximum Step Numbers
    //iWork(2)						//max number of Newton Iterations for the solution of the implizi system; def. 7 
    iWork(4) = system->getqSize() + system->getxSize(); // Dimension of Index 1 Variables (fuer ODE = N)
    iWork(5) = system->getuSize();			// Dimension of Index 2 Variables
    iWork(6) = laBiSize;				// Dimension of Index 3 Variables
    if (DAEIndex == 21) iWork(6) = 2*laBiSize;
    int idid;

    tPlot = t + dtPlot;
    dtOut = dtPlot;
    system->plot(z, t);

    zInp.resize(YSize);

    integPlot.open((system->getDirectoryName() + name + ".plt").c_str());

    integPlot << "#1 t [s]:" << endl; 
    integPlot << "#1 dt [s]:" << endl; 
    integPlot << "#1 calculation time [s]:" << endl; 

    cout.setf(ios::scientific, ios::floatfield);

    output_ = output;

    s0 = clock();
    cout << "DAE index ";
    if (DAEIndex==21) cout << "2 (GGL formulation)";
    else cout << DAEIndex;
    cout << " with ";
    if (useExternalJac) cout << "external jacobian." << endl;
    else cout << "internally computed jacobian." << endl;

    RADAU5(&YSize,fdae,&t,z(),&tEnd,&dt0,
	rTol(),aTol(),&iTol,
	jac,&iJac,&mlJac,&muJac,
	mdae,&iMas,&mlMas,&muMas,
	plot,&iout,
	work(),&lWork,iWork(),&liWork,rPar(),iPar,&idid);

    integPlot.close();

    ofstream integSum((system->getDirectoryName() + name + ".sum").c_str());
    integSum << "Integration time:  " << time << endl;
    integSum << "Integration steps: " << iWork(16)<< "  (total "<<iWork(15)<<"; "<<iWork(17)<<" rejected due to error test)"  << endl;
    integSum << "Function calls :   " << iWork(13) << "(without numerical evaluation of Jacobian)"<<endl;
    integSum << "Jacobian calls :   " << iWork(14) << endl;

    integSum.close();

    cout.unsetf (ios::scientific);

    cout << "Integration time:  " << time << endl;
    cout << "Integration steps: " << iWork(16)<< "  (total "<<iWork(15)<<"; "<<iWork(17)<<" rejected due to error test)"  << endl;
    cout << "Function calls :   " << iWork(13) << "(without numerical evaluation of Jacobian)"<<endl;
    cout << "Jacobian calls :   " << iWork(14) << endl;
    cout << endl;
  }

}
