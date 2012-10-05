/*=============================================================================
        File: error.cpp
     Purpose: Implementation of general error reporting class
    Revision: $Id: error.cpp,v 1.7 2002/05/22 17:06:47 philosophil Exp $
  Created by:    Philippe Lavoie          (3 Oct, 1996)
 Modified by: 

 Copyright notice:
          Copyright (C) 1996-1997 Philippe Lavoie
 
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

#include "error.h"

//#include <stream.h>
#include <string.h>

/*!
 */
namespace PLib {

/*!
  \brief constructor 
  Description:
  \param title  the title of the routine where the error occurs
  \return 
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
#ifndef USING_VCC
Error::Error(const char *title) :  ErrorStream(), prog(0)
#else
Error::Error(const char *title) : prog(0)
#endif
{
  int l = strlen( title ) + 1;

  prog = new char[l];
  strcpy(prog, title);
#ifndef USING_VCC
  // Can't remember why I had the following before, and now gcc-3.0 complains.
  //  init (&__my_sb);
#endif
  clear() ;

}

/*!
  \brief reports an error message
  The usual call to this function is made through the 
               $\ll$ operator, {\em i.e.} 
	       \begin{verbatim}
	         Error error("routine name") ;
		 error << "An error occured " ;
		 error.report() ;
	       \end{verbatim}
  \param msg  the message to report
  \return 
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
void Error::report(const char *msg)
{
  if ( msg == 0 )		// message must have been sent via <<
    cerr << str() ;
  else
    cerr << msg;

  cerr << '\n';
#ifdef DEBUG_PLIB
  cerr << "\n\nThe program is now in an infinte loop. Press CTRL-c to exit.\n" ;
#endif
  
}

/*!
  \brief reports a warning error message
  The usual call to this function is made through the 
               $\ll$ operator, {\em i.e.} 
	       \begin{verbatim}
	         Error error("routine name") ;
		 error << "An error occured " ;
		 error.warning() ;
	       \end{verbatim}
  \param msg  the message to report
  \return 
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
void Error::warning(const char *msg)
{
  cerr << "\nRoutine: " << prog << "\nWarning: ";

  report( msg );

}

/*!
  \brief reports a fatal error
  Reports a fatal error. If the \verb.DEBUG_MATRIX. flag has been
               set then the routine starts an infinte loop. An exit(1) is
	       executed otherwise.

	       The usual call to this function is made through the 
               $\ll$ operator, {\em i.e.} 
	       \begin{verbatim}
	         Error error("routine name") ;
		 error << "A fatal error occured " ;
		 error.fatal() ;
	       \end{verbatim}

  \param msg  a message to report
  \return 
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
void Error::fatal(const char *msg)
{
  cerr << "\nRoutine: " << prog << "\nFatal error: ";
  
  report( msg );
#ifdef DEBUG_PLIB
  while(1){ ; }
#else
  exit(1);
#endif
}

/*!
  \brief reports a memory allocation error
  Reports a memory allocation error. If the DEBUG_PLIB
  flag has been set then the routine starts an infinte loop. 
  An exit(1) is executed otherwise.
	       
  The usual call to this function is made through the 
   operator>> , {\em i.e.} 
  \begin{verbatim}
	         Error error("variable name") ;
		 float* var = new float[bigSize] ;
		 error.memory(var) ;
		\end{verbatim}

  \param p  the pointer to test
  \return 
  \warning
  \author Philippe Lavoie 
  \date 24 January 1997
*/
void Error::memory(const void *p)
{
  if ( p == 0)
    {
      cerr << "\nRoutine: " << prog << " Memory allocation error\n";
#ifdef DEBUG_PLIB
      while (1) { ; }
#else
      exit(1);
#endif
    }  
}

}

#ifdef NO_IMPLICIT_TEMPLATES


#if GCC_VERSION >= 30000
template std::basic_ostream<char, std::char_traits<char> >& std::operator<< <char, std::char_traits<char> >(std::basic_ostream<char, std::char_traits<char> >&, std::_Setw);
#else

#endif



#endif


