/*=============================================================================
        File: global.h
     Purpose: Define and include some general definitions valid for all 
              matrix header files
    Revision: $Id: nurbs_global.h,v 1.2 2002/05/13 21:07:46 philosophil Exp $
  Created by: Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1998 Philippe Lavoie
 
	  This library is free software; you can redistribute it and/or
	  modify it under the terms of the GNU Library General Public
	  License as published by the Free Software Foundation; either
	  version 2 of the License, or (at your option) any later version.
 
	  This library is distributed in the hope that it will be useful,
	  but WITHOUT ANY WARRANTY; without even the implied warranty of
	  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	  Library General Public License for more details.
 
	  You should have received a copy of the GNU Library General Public
	  License along with this library; if not, write to the Free
	  Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
=============================================================================*/
#ifndef _Matrix_internals_h_
#define _Matrix_internals_h_

#include "matrix_global.h"

enum CoordinateType { coordX, coordY, coordZ } ;

#include <iostream>
#include <cstdlib>

/*!
  \class NurbsError internals.h nurbs/internals.h
  \brief The class thrown when an error happens inside the NURBS library

  This is the base class for all thrown classes inside the NURBS library.
  
  If VERBOSE_EXCEPTION is defined, an error message will be displayed
  when this exception is thrown.

  \author Philippe Lavoie
  \date 17 February 1999
*/
struct NurbsError {
  NurbsError() { print_debug(); }
  void print_debug(){
#ifdef VERBOSE_EXCEPTION
    print();
#else
    ;
#endif
  }
  virtual void print() { cerr << "NURBS error.\n" ; }
};

/*!
  \class NurbsInputError internals.h nurbs/internals.h
  \brief An input error class

  This class is thrown if at least one of the input parameter isn't
  suitable for the function throwing it.

  If mode is set to 1 it means that two values should have been
  equal, but are not. This is normally set when the size of a
  vector isn't equal to the size of another vector or that
  a matrix size isn't equal to another matrix's size.

  \author Philippe Lavoie
  \date 17 February 1999 
*/
struct NurbsInputError : public NurbsError {
  NurbsInputError(): mode(0),x(0),y(0) { print_debug() ; }
  NurbsInputError(int a, int b): mode(1),x(a),y(b) { print_debug() ; }
  virtual void print() { 
    if(mode==1) cerr << "The values " << x << " and " << y << " are not equal.\n" ; 
    else cerr << "An error in one of  the input parameter.\n" ; }
  int mode ;
  int x,y ;
};

/*!
  \class NurbsSizeError internals.h nurbs/internals.h
  \brief A NURBS size error

  There is a relationship between the number of control points,
  the number of knots and the degree of a curve or a surface.
  This class is thrown when 
  \a U.n \a \a = \a P.n \a + \a deg \a + \a 1 is false.
  
  \author Philippe Lavoie
  \date 17 February 1999 
*/
struct NurbsSizeError : public NurbsInputError {
  NurbsSizeError(int pnts, int knots, int deg) : p(pnts), k(knots), d(deg) { print_debug();}
  virtual void print() { cerr << " The number of knots (" << k << "), the number of control points ("<< p << ") and the degree ("<< d << ") are not compatible.\n" ; }
  int p,k,d ;
};

/*!
  \class NurbsComputationError internals.h nurbs/internals.h
  \brief Can't succesfully compute

  This class is thrown if a computation error occurs.
  
  \author Philippe Lavoie
  \date 17 February 1999 
*/
struct NurbsComputationError : public NurbsError {
  NurbsComputationError() { print_debug(); }
  virtual void print() { cerr << "Couldn't not succesfully perform the computation.\n" ; }
};

/*!
  \class NurbsWarning internals.h nurbs/internals.h
  \brief A warning class

  This class is thrown if a non-critical error occurs. The user
  can choose to ignore it but should probably fix it.
  
  \author Philippe Lavoie
  \date 17 February 1999 
*/
struct NurbsWarning : public NurbsError {
  NurbsWarning() { print_debug(); }
  virtual void print() { cerr << "A non-critical error occured.\n" ; }
};


#endif 

