/* Copyright (C) 2006 Mathias Bachmayer
 *
 * Institute of Applied Mechanics
 * Technical University of Munich

 *
 * Contact:
 *   bachmayer@amm.mw.tum.de
 *
 */ 


#include <config.h>

#include "iostream"
#include "fmatvec.h"
#include "polynomutils.h"
using namespace::fmatvec;
using namespace::std;

double PolyNormierung(Vec &Polynom){
    double Normkonst;
    Normkonst=Polynom(Polynom.rows()-1);
    if (Normkonst!=0){Polynom=Polynom/Normkonst;} else {cout<<"Error in PolyNormierung - At least the highest koefficient in a Polynom Vektor has to be unequal to zero!!"; throw 33;} 
    return Normkonst;
}
int PolynomVis(Vec Polynom){
    int ordnung=Polynom.rows()-1;
    int ausgabe=0;
if (ordnung>-1)
{
    
    while(ordnung>0){
	
	if (Polynom(ordnung)!=0)
	{
	ausgabe=1;
	if (Polynom(ordnung)!=1) cout<<Polynom(ordnung);
	cout<<" x";
	if (ordnung>1) cout<<"^"<<ordnung;
	}
        if (Polynom(ordnung-1)>0) cout<<" + ";
	ordnung--;
    }
    if ((Polynom(0)!=0)||(ausgabe==0)) {cout<<Polynom(0)<<endl;}
}
    return 1;
}


Vec Pole2Koeff(Vec Pole){
    //Vec Pole enthaelt negative Nullstellen pn des Polynoms (s+p1)(s+p2)...(s+pn)
    // Funktion Pole2Koeff gibt die Koeffizienten des zugehörigen Polynoms
    // s^n+c_(n-1)*s^(n-1)+...+c_0 als Vektor zurück (Index 0=c_0 bis n+1=c_(n+1)=1)
    int ordnung,bitpos1;
    int i;
    unsigned short Polmaske;
    unsigned short Maxmaske;
    double koeff=0;
    Pole=-Pole;
    ordnung=Pole.rows();
    Vec VZWERG(ordnung+1);  
   for (i=0;i<=ordnung;i++)
   {
    Bitmaskinit(i,ordnung,Polmaske,Maxmaske);
    koeff=Produktmaskiert(Pole,Polmaske);
    while(Polmaske<Maxmaske)
    {
	NaechsteBitmaske(ordnung,Polmaske,1,bitpos1);
	koeff=Produktmaskiert(Pole,Polmaske)+koeff;
    }
   VZWERG(i)=koeff;
   }
   return VZWERG;
}

Vec PolynomDivision(Vec Zaehler, Vec Nenner, Vec &Erg){
    int Zordnung=Zaehler.rows()-1;
    int Nordnung=Nenner.rows()-1;
    int ergpos=Zordnung-Nordnung; 
    int i;
    Vec Rest;
if (ergpos>-1){
    Vec Zwerg(Erg.rows()); //ZwergVektor
    Vec Zaehlerreduziert(Zordnung);

    Zwerg(ergpos)=Zaehler(Zordnung)/Nenner(Nordnung); //Ergebniss der Div
   i=0;
   while(i<Zordnung){
  if (i<Nordnung){Zaehlerreduziert(Zordnung-1-i)=Zaehler(Zordnung-1-i)-Nenner(Nordnung-1-i)*Zwerg(ergpos);}
  else
  { Zaehlerreduziert(Zordnung-1-i)=Zaehler(Zordnung-1-i);}
   i++;	    
   }

  if (Zordnung>Nordnung){ 
      Rest=PolynomDivision(Zaehlerreduziert,Nenner,Erg);
      Erg=Erg+Zwerg;
     
  } else
  {
  Erg=Zwerg;
  Rest=Zaehlerreduziert;
  }
}
else
{
Vec Zwerg(Erg.rows());
Erg=Zwerg;
Rest=Zaehler;
}
return Rest;
}

