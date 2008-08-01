/* Copyright (C) 2006 Mathias Bachmayer
 *
 * Institute of Applied Mechanics
 * Technical University of Munich

 *
 * Contact:
 *   bachmayer@amm.mw.tum.de
 *
 */ 


//**********************************************************************************
//*Polynomutils.h Header File                                                      *
//*                                                                                *
//* Polynomutils.h provides normally non user functions, which are                 *
//* required by polynome.h for polynomial computation.                             *
//*                                                                                *
//* AUTOR: BACHMAYER MATHIAS                                                       *
//* Date: 26.05.2006                                                               *
//* Last Reviewed by: -------                                                      *
//* Last Review date: 26.05.2006                                                   *
//**********************************************************************************
#ifndef _Polynomutils_
#define _Polynomutils_

#include "iostream"
#include "fmatvec.h"

using namespace::fmatvec;
using namespace::std;


int Bitmaskinit(int order,int ordnung, unsigned short& Bitmask,unsigned short& Maxmask);
double Produktmaskiert(Vec Pole, unsigned short Polmaske);
int NaechsteBitmaske(int ordnung, unsigned short& Polmaske,int ntesBit,int &nBitpos);

#endif

