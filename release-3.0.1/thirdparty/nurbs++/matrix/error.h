/*=============================================================================
        File: error.h
     Purpose:       
    Revision: $Id: error.h,v 1.3 2002/05/17 18:24:21 philosophil Exp $
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
#ifndef _ERROR_REP_HH_
#define _ERROR_REP_HH_

#include <stdlib.h>
#include "matrix_global.h"

#include <sstream>
typedef std::ostringstream ErrorStream;
/* the standard is now sstream and not strstream
#else
#include <strstream>
typedef std::ostrstream ErrorStream;
#endif
*/


/*!
 */
namespace PLib {

  struct MatrixErr { 
    MatrixErr() { print_debug(); }
    void print_debug() { 
#ifdef VERBOSE_EXCEPTION
      print();
#else
      ;
#endif
    }
    virtual void print() { cerr << "Matrix error.\n" ; }
  };
  
  struct MatrixInputError : public MatrixErr {
    MatrixInputError() { print_debug();}
    virtual void print(){
      cerr << "One of the input value is not in appropriate.\n";
    }
  };

  struct OutOfBound : public MatrixInputError {
    int i ;
    int s,e ;
    OutOfBound(int index, int from, int to): i(index), s(from), e(to) { print_debug(); }
    virtual void print() { 
      cerr << "Out of bound error, trying to access " << i << 
	" but the valid range is [ " << s << "," << e << "]\n" ; 
    }
  };
  
  struct OutOfBound2D : public MatrixInputError {
    int i,j ;
    int s_i,e_i ;
    int s_j,e_j ;
    OutOfBound2D(int I, int J, int fI, int tI, int fJ, int tJ): i(I), j(J), s_i(fI), e_i(tI), s_j(fJ), e_j(tJ) { print_debug(); } 
    virtual void print() { 
      cerr << "Out of bound error, trying to access (" << i << ',' << j <<
	") but the valid range is ([ " << s_i << "," << e_i << "], [" <<
	s_j << ',' << e_j << "])\n" ;
    }
    
  };
  
  struct WrongSize : public MatrixInputError {
    int s1,s2 ;
    WrongSize(int a, int b) : s1(a), s2(b) { print_debug();}
    virtual void print(){
      cerr << "The vector sizes  " << s1 << " and " << s2 << " are incompatible.\n" ;
    }
  }; 
  
  struct WrongSize2D : public MatrixInputError {
    int rows,cols ;
    int bad_rows, bad_cols ;
    WrongSize2D(int r, int c, int br, int bc) : rows(r), cols(c), bad_rows(br), bad_cols(bc) { print_debug();}
    virtual void print(){
      cerr << "The matrix sizes  (" << rows << " x " << cols << ") and (" << bad_rows << " x " << bad_cols << ") are incompatible.\n" ;
    }
  }; 
  
/*!
  \brief A class to print and handle error messages

  This class prints error messages in a standard way.
  \author Philippe Lavoie 
  \date 4 October 1996
*/
 class Error : public ErrorStream
 {
 private:
   char* prog;
   void report(const char *msg = NULL);
 public:
   Error(): ErrorStream(), prog(0) {}
   Error(const char *s);
   ~Error(){ if (prog) delete []prog ; }
   
   void warning(const char *msg = 0);
   void nonfatal(const char *msg = 0) { warning(msg); }
   void fatal(const char * = 0 );
   void memory(const void * = 0 );
   
 };

} // end namespace

#ifdef INCLUDE_TEMPLATE_SOURCE
#ifndef USING_VCC
#include "error.cpp"
#endif
#endif



#endif

