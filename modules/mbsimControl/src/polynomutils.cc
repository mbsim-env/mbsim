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

using namespace::fmatvec;
using namespace::std;


int Bitmaskinit(int order,int ordnung, unsigned short& Bitmask,unsigned short& Maxmask){
    //Konvention: eine Null in der Bitmaske bedeutet, entsprechender Pol ist zu
    //multiplizieren => int=0 Bitmask =0!
    Bitmask=0;
    Maxmask=0;
    int n=0;

    unsigned short MaxLSB;

    int delta;
    delta=ordnung-1;
    MaxLSB=0x01<<delta;
    while(n<order)    
    {
    n++;
    Bitmask=Bitmask<<1;
    Bitmask=Bitmask|0x01;
    Maxmask=Maxmask>>1;
    Maxmask=Maxmask|MaxLSB;
    }
    return 1;
}

double Produktmaskiert(Vec Pole, unsigned short Polmaske){
    double zwerg;
    int i=0;
    int max;
    unsigned short bit;
    bit=0x01;
    max=Pole.rows();
    zwerg=1;
    while(i<max){
	if ((bit&Polmaske)>0){} else {zwerg=zwerg*Pole(i);}
	bit=bit<<1;
	i++;
	}
    return zwerg;
}

int NaechsteBitmaske(int ordnung, unsigned short& Polmaske,int ntesBit,int &nBitpos){
    int bitpos,vorigebitpos;	
    int ngefunden;
    unsigned short NtesBitMaske;
    unsigned short DummieMaske;
    NtesBitMaske=0x01<<ordnung;
    bitpos=ordnung;
    ngefunden=0;
    while((ngefunden<ntesBit)&&(bitpos>-1))
    {
	NtesBitMaske=NtesBitMaske>>1;
	bitpos--;
	if ((Polmaske&NtesBitMaske)>0){ ngefunden++; }
    }
    
   if (bitpos<0){ cout<<"Fehler!!! polynomutils.h::NaechsteBitMaske es existiert in Polmaske "<<Polmaske<<" kein "<<ntesBit<<"tes Bit!";throw 5; }
   else
   {
       if ((((NtesBitMaske<<1)&Polmaske)==0)&&(bitpos<ordnung-1))
       {
         DummieMaske=~NtesBitMaske;
	 Polmaske=Polmaske&DummieMaske;
	 NtesBitMaske=NtesBitMaske<<1;
	 Polmaske=Polmaske|NtesBitMaske;
	 nBitpos=bitpos+1;
       }
       else {
       NaechsteBitmaske(ordnung,Polmaske,ntesBit+1,vorigebitpos);
       DummieMaske=~NtesBitMaske;
       Polmaske=Polmaske&DummieMaske;
       NtesBitMaske=0x01<<vorigebitpos+1;
       Polmaske=Polmaske|NtesBitMaske;
       nBitpos=vorigebitpos+1;
       }
   }
    return 1;
}


