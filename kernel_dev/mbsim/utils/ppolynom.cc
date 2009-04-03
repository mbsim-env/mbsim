/* Copyright (C) 2004-2006  Robert Huber
 
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
 *   rzander@users.berlios.de
 *
 */
#include <config.h>
#include "ppolynom.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  //   Info zu Splines / PP-Form (pieceweise Polonominal Form)
  //===========================================================

  // PP-Form 	
  //---------     besteht aus   - Mat coefs
  //                  	      - Vec breaks
  // In breaks sind die Eckpunkte der Intervalle der jeweiligen abschnittsweise definierten Polynome abgelegt
  // In coefs sind die Koeffizienten der Polynome definiert

  // Ansatz fuer Polynome:  c0* xloc^n + c1* xloc^(n-1) + c2* xloc^(n-2) + ... + cn
  //                  mit:  [c0 c1 c2 ... cn] i-ter Zeilenvektor aus coefs-Matrix 
  //                        breaks(i) << x << breaks(i+1) --> Legt i und somit Zeilenvektor der coefs Matrix fest
  //							--> xloc = x-breaks(i)  (lokales x des abschn.w. def. Poly.)
  //
  // Bsp. kubische Spline mit
  //	 breaks= [0; 0.3; 0.5] 		:erstes Polynom gilt fuer x aus Intervall 0 bis 0.3  -> xloc = x -0
  //	 coefs= [d1 c1 b1 a1; 		 dazugehoerige Koeffizienten: d1 c1 b1 a1   
  //               d2 c2 b2 a2]       	 --> Interpolationswert S(x) = a1 + b1*xloc + c1*xloc^2 + d1*xloc^3
  //      ausgewertet an Stelle x		
  //					:zweite Polynom gilt fuer x aus 0.3 bis 0.5     -> xloc = x - 0.3;
  //					--> Interpolationswert S(x) = a2 + b2*xloc + c2*xloc^2 + d2*xloc^3



  // kubische Spline
  // ---------------

  // Die gegebenen Werte (xi,fi) i=1..N werden durch N-1 abschnittsweise def. Polynome Si vom Grad 3 interpoliert,
  // so dass sie am Übergang zweimal stetig diffbar sind
  // Damit die Koeffizienten eindeutig bestimmbar sind sind noch ZWEI weitere Randbedingungen erforderlich.
  // Man unterschiedet:		1. Natuerliche Randbedingungen:  S''(x1) = S''(xN) = 0
  //				2. periodische Randbedingungen:  S'(x1) = S'(xN)    UND   S''(x1) = S''(xN)
  //								 Die zusaetzlich Forderung S(x1)=S(xN), muss durch 
  //								 die Eintraege im Vektor f sichergestellt werden.
  //                                                               --> dazu MUSS f(x1) = f(xN) gelten  

  // Bemerkung zu periodischen Randbedingungen: 
  // erster und letzter Wert von f muessen uebereinstimmen, damit sich glatte Kontur ergibt (z.B. Nockenkontur)
  void PPolynom::setXF(const Vec &x, const Vec &f, std::string InterpolationMethod) {

    bool noInterpolation=true;

    if (InterpolationMethod=="csplinePer") {
      calculateSplinePeriodic(x, f);   
      noInterpolation=false;
    }

    if (InterpolationMethod=="csplineNat") { 
      calculateSplineNatural(x, f);   
      noInterpolation=false;
    }

    if (noInterpolation) {
      cout <<"No valid Method to calculate pp-Form"<<endl;
      exit(-1); }

      index  = 0;
      nPoly  = coefs.rows();
      order  = coefs.cols()-1;
  }


  //=============================================================================================
  // Auswertung Splines 
  Vec PPolynom::operator()(double x){

    if (x>breaks(nPoly)){
      cout << "PPEVAL: x out of Range!   x= "  << x <<" Range= "<< breaks(nPoly) <<endl;
      throw 54;
    }
    if (x<breaks(0)){
      cout << "PPEVAL: x out of Range!   x= "  << x <<" Range= "<< breaks(0) <<endl;
      throw 54;
    }


    if (!(x>breaks(index) && x<breaks(index+1)))  {	// Pruefe ob gespeicherter Index noch aktuell
      index =0;
      while ( index < nPoly && breaks(index) <= x){	// Suchen des richtigen Index
	index++;
      }
      index--;
    }

    double dx = x - breaks(index);
    double yi;
    yi=coefs(index,0);
    for (int i=1;i<=order;i++){
      yi=yi*dx+coefs(index,i);
    }
    return Vec(1,INIT,yi);
  }

  //..............................................................................................................
  // Auswertung spline: Erste Ableitung nach Konturparameter
  Vec PPolynom::diff1(double x){

    if (x>breaks(nPoly)){
      cout << "PPEVAL_S: x out of Range!   x= "  << x <<" Range= "<< breaks(nPoly) <<endl;
      throw 54;
    }
    if (x<breaks(0)){
      cout << "PPEVAL_S: x out of Range!   x= "  << x <<" Range= "<< breaks(0) <<endl;
      throw 54;
    }


    if (!(x>breaks(index) && x<breaks(index+1))) {		// Pruefe ob gespeicherter Index noch aktuell
      index =0;
      while ( index < nPoly && breaks(index) <= x){	// Suchen des richtigen Index
	index++;
      }
      index--;
    }

    double dx = x - breaks(index);
    double yi;
    yi=coefs(index,0)*order;
    for (int i=1;i<order;i++){
      yi=yi*dx+coefs(index,i)*(order-i);
    }
    return Vec(1,INIT,yi);
  }



  //..............................................................................................................

  // Auswertung Spline: Zweite Ableitung nach Konturparameter
  Vec PPolynom::diff2(double x){

    if (x>breaks(nPoly)){
      cout << "PPEVAL_SS: x out of Range!   x= "  << x <<" Range= "<< breaks(nPoly) <<endl;
      throw 54;
    }
    if (x<breaks(0)){
      cout << "PPEVAL_SS: x out of Range!   x= "  << x <<" Range= "<< breaks(0) <<endl;
      throw 54;
    }


    if (!(x>breaks(index) && x<breaks(index+1))) {		// Pruefe ob gespeicherter Index noch aktuell
      index =0;
      while ( index < nPoly && breaks(index) <= x){	// Suchen des richtigen Index
	index++;
      }
      index--;
    }

    double dx = x - breaks(index);
    double yi;
    yi=coefs(index,0)*order*(order-1);
    for (int i=1;i<=(order-2);i++){
      yi=yi*dx+coefs(index,i)*(order-i)*(order-i-1);
    }
    return Vec(1,INIT,yi);
  }

  // ================================================================================================

  // Interpolationsmethoden

  //.................................................................................................

  // kubische Spline
  //==================


  void PPolynom::calculateSplinePeriodic(const Vec &x, const Vec &f) {
    // Randbedingungen Periodische Splines:	S(x1)  =   S(xN)  (dazu muss f(0)=f(end) gelten: keine RB sonder Forderung an f)
    // 					S'(x1) =   S'(xN)
    // 					S''(x1)=   S''(xN)
    //
    // Bemerkung: erster und letzter Wert von f muessen uebereinstimmen, damit sich glatte Kontur ergibt (z.B. Nockenkontur)

    double hi, hii, h1, hN_1;
    int i;
    int N;
    N= x.size();

    if(f(0)!=f(f.size()-1)) {
      cout << "PPolynom::calculateSplinePeriodic: f(0)"<<f(0)<<"!="<<f(f.size()-1)<<endl;
      throw 50;
    }

    SqrMat C(N-1,N-1,INIT,0.0);
    Vec rs(N-1,INIT,0.0);

    // Matrix C und Vektor rs des Gleichungssystems C*c=rs aufstellen

    for (i=0; i<N-3;i++) {
      hi =x(i+1) - x(i);
      hii = x(i+2)-x(i+1);
      C(i,i)  = hi;
      C(i,i+1)= 2*(hi+hii);
      C(i,i+2)= hii;
      rs(i) = 3*( (f(i+2)-f(i+1))/hii - (f(i+1)-f(i))/hi);
    }
    // vorletzte Zeile GS
    i=N-3;
    hi  = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i)  = hi;
    C(i,i+1)= 2*(hi+hii);
    C(i,0)= hii;
    rs(i) = 3*( (f(i+2)-f(i+1))/hii - (f(i+1)-f(i))/hi);

    // letzte Zeile des GS
    h1   = x(1)-x(0);
    hN_1 = x(N-1)-x(N-2);
    C(N-2,0)  = 2*(h1+hN_1);
    C(N-2,1)  = h1;
    C(N-2,N-2)= hN_1;
    rs(N-2) = 3*( (f(1)-f(0))/h1 - (f(0)-f(N-2))/hN_1);


    // Loesen des Gleichungssystems C*c = rs
    Vec c;
    Vec ctmp(N);
    c = slvLU(C,rs);
    ctmp(N-1)  = c(0);    	//CN = c1
    ctmp(0,N-2)= c;

    // Berechnung der ai bi di

    Vec d(N-1,INIT,0.0);
    Vec b(N-1,INIT,0.0);
    Vec a(N-1,INIT,0.0);



    for (i=0; i<N-1; i++) {
      hi  = x(i+1)-x(i);  
      a(i)= f(i);
      d(i)= (ctmp(i+1) - ctmp(i) ) / 3 / hi;
      b(i)= (f(i+1)-f(i)) / hi - (ctmp(i+1) + 2*ctmp(i) ) / 3 * hi;
    }

    breaks.resize(N);
    coefs.resize(N-1,4);
    breaks = x;
    coefs.col(0) = d;
    coefs.col(1) = c;
    coefs.col(2) = b;
    coefs.col(3) = a;
  }




  //......................................................................................................................

  void PPolynom::calculateSplineNatural(const Vec &x, const Vec &f) {
    // Natuerliche Randbedingungen:	S''(x1)  =   S''(xN) = 0;

    int N,i;
    N = x.size();

    SqrMat C(N-2,N-2,INIT,0.0);
    Vec rs(N-2,INIT,0.0);

    double hi,hii;

    i=0;   
    hi  = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i)   = 2*hi+2*hii;
    C(i,i+1) = hii;
    rs(i) = 3*(f(i+2)-f(i+1))/hii - 3*(f(i+1)-f(i))/hi;    

    i=(N-3);
    hi  = x(i+1)-x(i);
    hii = x(i+2)-x(i+1);
    C(i,i-1) = hi;
    C(i,i)   = 2*hii + 2*hi;
    rs(i) = 3*(f(i+2)-f(i+1))/hii - 3*(f(i+1)-f(i))/hi;

    for(i=1;i<N-3;i++) { 
      hi  = x(i+1)-x(i);
      hii = x(i+2)-x(i+1);
      C(i,i-1) = hi;
      C(i,i)   = 2*(hi+hii);
      C(i,i+1) = hii;
      rs(i) = 3*(f(i+2)-f(i+1))/hii - 3*(f(i+1)-f(i))/hi;
    }

    // Loesen des Gleichungssystems C*c = rs
    // C tridiagonal d.h nur Elemente die +- eins von Diag entfernt sind besetzt
    Mat C_rs(N-2,N-1,INIT,0.0);  

    C_rs(0,0,N-3,N-3) = C;			// C_rs=[C rs] fuer Gauss in eine Matirx schreiben
    C_rs.col(N-2) =rs;

    for(i=1; i<N-2; i++)  {                // C_rs  umformen so dass C obere Dreiecksmatrix C1 wird -> C1 rs1
      C_rs.row(i) = C_rs.row(i) - C_rs.row(i-1)*C_rs(i,i-1)/C_rs(i-1,i-1);
    }

    Vec rs1(N-2);
    Mat C1(N-2,N-2);

    rs1 = C_rs.col(N-2);
    C1  = C_rs(0,0,N-3,N-3);

    Vec c(N-2,INIT,0.0);
    double sum_ciCi;
    int ii;

    for (i=N-3;i>=0 ;i--) {  	// Rueckwaertseinsetzen
      sum_ciCi = 0; 
      for (ii=i+1; ii<=N-3; ii++) {
	sum_ciCi = sum_ciCi + C1(i,ii)*c(ii);
      }
      c(i)= (rs1(i) - sum_ciCi)/C1(i,i);
    }

    Vec c_solve;
    c_solve= slvLU(C,rs);	//TO DO vgl. mit solve Routine
    //cout<< "c-csolve" << c-c_solve<<endl;
    //cout<< "norm c-csolve" << nrm2(c-c_solve)<<endl;


    Vec ctmp(N,INIT,0.0);
    ctmp(1,N-2) = c; 		// c1=cN = 0 Natuerliche Splines c=[ 0; c; 0]


    //Berechnung der ai bi di
    Vec d(N-1,INIT,0.0);
    Vec b(N-1,INIT,0.0);
    Vec a(N-1,INIT,0.0);

    for (i=0; i<N-1; i++) {
      hi  = x(i+1)-x(i);  
      a(i)= f(i);
      d(i)= (ctmp(i+1) - ctmp(i) ) / 3 / hi;
      b(i)= (f(i+1)-f(i)) / hi - (ctmp(i+1) + 2*ctmp(i) ) / 3 * hi;
    }

    breaks.resize(N);
    coefs.resize(N-1,4);
    breaks = x;
    coefs.col(0) = d;
    coefs.col(1) = ctmp(0,N-2);
    coefs.col(2) = b;
    coefs.col(3) = a;
  }

}
