 /* Copyright (C) 2006 Mathias Bachmayer
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
  * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 US

  *
  * Contact:
  *          mbachmayer@users.berlios.de
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

