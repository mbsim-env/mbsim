/*=============================================================================
        File: intccq.cpp
     Purpose:       
    Revision: $Id: intccq_.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#include "intccq.cpp"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

  template class ClassPO<float>;
  template class ClassPOvoid<float>;
  
  template void intccini(BasicArray<float> &w) ;
  template float intcc(ClassPO<float>* f, float a, float b, float eps, 
		       BasicArray<float> &w, float &err) ;
  template float integrate(ClassPO<float>* f, float a, float b, float eps, int n, float &err) ;
  template float intcc(ClassPOvoid<float>* f,void*, float a, float b, float eps, BasicArray<float> &w, float &err) ;
  template float integrate(ClassPOvoid<float>* f,void*, float a, float b, float eps, int n, float &err) ;
  template float intcc2(ClassPO<float>* f, float a, float b, float eps, 
			BasicArray<float> w, float &err) ;
  template float integrate2(ClassPO<float>* f, float a, float b, float eps, int n, float &err) ;
  template float intcc2(ClassPOvoid<float>* f,void*, float a, float b, float eps, BasicArray<float> w, float &err) ;
  template float integrate2(ClassPOvoid<float>* f,void*, float a, float b, float eps, int n, float &err) ;
  
  
  template class ClassPO<double>;
  template class ClassPOvoid<double>;
  
  template void intccini(BasicArray<double> &w) ;
  template double intcc(ClassPO<double>* f, double a, double b, double eps, 
			BasicArray<double> &w, double &err) ;
  template double integrate(ClassPO<double>* f, double a, double b, double eps, int n, double &err) ;
  template double intcc(ClassPOvoid<double>* f,void*, double a, double b, double eps, BasicArray<double> &w, double &err) ;
  template double integrate(ClassPOvoid<double>* f,void*, double a, double b, double eps, int n, double &err) ;
  template double intcc2(ClassPO<double>* f, double a, double b, double eps, 
			 BasicArray<double> w, double &err) ;
  template double integrate2(ClassPO<double>* f, double a, double b, double eps, int n, double &err) ;
  template double intcc2(ClassPOvoid<double>* f,void*, double a, double b, double eps, BasicArray<double> w, double &err) ;
  template double integrate2(ClassPOvoid<double>* f,void*, double a, double b, double eps, int n, double &err) ;
  
#endif

}
