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
//*Polynome.h Header File    - needs polynomutils.h                                *
//*                                                                                *
//*Polynome.h provides polynomial computation skills based on fmatvec Vectors.     *
//*A Polynome is represented by its coefficients. For instance the second order    *
//*Polynom 5*x^2+x+7 is represented eg. by a Vec Vector, where Vector(0)=7,        *
//*Vector(1)=1 and Vector(2)=5 - corresponding to the exponent of x.               *
//*										   *
//*Implemented Polynomial computation skills:                                      *
//* Polynom Division       : Vec PolynomDivision(Zaehler, Nenner, &Erg) - returns  * 
//*                            the remainder and expects a Ord(Zaehler)-1          *
//*                            dimensional result Vector Erg provided by the user. *
//* Multiplikation of Poles: Vec Pole2Koeff - returns the coefficients of          *  
//*                            Polynom for a set of given Poles.                   *
//* Norming                : double PolyNormierung - modifies a Polynom by norming *
//*                            its highest exponent's coefficient and returns the  *
//*                            old coefficient.                                    *
//* Cout Visualisation     : int PolynomVis - prints polynom into std out.         *
//*                                                                                *
//*                                                                                *
//* AUTOR: BACHMAYER MATHIAS     
//*
//* Date: 26.05.2006                                                               *
//* Last Reviewed by: -------                                                      *
//* Last Review date: 26.05.2006                                                   *
//**********************************************************************************

#ifndef _Polynome_
#define _Polynome_

#include "iostream"
#include "fmatvec.h"
#include "polynomutils.h"
using namespace::fmatvec;
using namespace::std;

double PolyNormierung(Vec &Polynom);
int PolynomVis(Vec Polynom);
Vec Pole2Koeff(Vec Pole);
    //Vec Pole enthaelt negative Nullstellen pn des Polynoms (s+p1)(s+p2)...(s+pn)
    // Funktion Pole2Koeff gibt die Koeffizienten des zugehörigen Polynoms
    // s^n+c_(n-1)*s^(n-1)+...+c_0 als Vektor zurück (Index 0=c_0 bis n+1=c_(n+1)=1)
Vec PolynomDivision(Vec Zaehler, Vec Nenner, Vec &Erg);
#endif
