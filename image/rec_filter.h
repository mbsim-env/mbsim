/*=============================================================================
        File: rec_filter.h
     Purpose:       
    Revision: $Id: rec_filter.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (18 February 1999)
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


#ifndef _Matrix_Recursive_Filters_h_
#define _Matrix_Recursive_Filters_h_

#include "barray2d.h"

/*!
 */
namespace PLib {

  struct Params{
    double a1,a2,a3,a4,a5,a6,a7,a8 ;
    double b1,b2 ;
    double c1,c2 ;
    double k ;
  };

  void toParams(double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double b1, double b2, double c1, double c2, double k, Params& params);  
  void fromParams(const Params& params, double &a1, double &a2, double &a3, double &a4, double &a5, double &a6, double &a7, double &a8, double &b1, double &b2, double &c1, double &c2, double &k);

  void generalRFx(const Params& params, const Basic2DArray<double> &x, Basic2DArray<double> &y);
  void generalRFy(const Params& params, const Basic2DArray<double> &r, Basic2DArray<double> &y);
  void generalRF(const Params& params, const Basic2DArray<double> &x, Basic2DArray<double> &y);

  void smooth2ndOrder(Params& params, double alpha);
  void xderiv2ndOrderSmooth(Params& params, double alpha);
  void yderiv2ndOrderSmooth(Params& params, double alpha);
  void xderiv2ndOrder(Params& params, double alpha);
  void yderiv2ndOrder(Params& params, double alpha);
  void smooth1stOrder(Params& params, double alpha, double k0);
  void LL1stOrder(Params& params, double alpha);
  
  void ubyteToDouble(const Basic2DArray<unsigned char>& img, Basic2DArray<double>& mat);
  void doubleToUbyte(const Basic2DArray<double>& mat, Basic2DArray<unsigned char>& img);

  inline double firstDivDiff(double x0, double f0, double x1, double f1){
    return (f1-f0)/(x1-x0) ;
  }

  inline double secondDivDiff(double x0, double f0, double x1, double f1, double x2, double f2){
    return (firstDivDiff(x1,f1,x2,f2)- firstDivDiff(x0,f0,x1,f1))/(x2-x0) ;
  }

  
  double quadInterp(double x, double x0, double f0, double x1, double f1, double x2, double f2);

  int findEdge(const Basic2DArray<double>& dx, const Basic2DArray<double>& dy, Basic2DArray<double> &edge, Basic2DArray<double>& gradN, double thresh);
  int findSubEdge(const Basic2DArray<double>& dx, const Basic2DArray<double>& dy, Basic2DArray<double> &edge, double thresh, const Basic2DArray<double>& in);


  template <class T>
    class RecursiveFilter {
    public:
      RecursiveFilter(const Basic2DArray<T>& in, Basic2DArray<T>& out);

      void compute_xderiv2ndOrderSmooth(double alpha);
      void compute_yderiv2ndOrderSmooth(double alpha);
      void compute_xderiv2ndOrder(double alpha);
      void compute_yderiv2ndOrder(double alpha);
      void compute_smooth1stOrder(double alpha, double k0);
      void compute_smooth1stOrder_x(double alpha, double k0);
      void compute_smooth1stOrder_y(double alpha, double k0);
      void compute_smooth2ndOrder(double alpha);
      void compute_smooth2ndOrder_x(double alpha);
      void compute_smooth2ndOrder_y(double alpha);
      void compute_LL1stOrder(double alpha);
      void compute_LL1stOrder_x(double alpha);
      void compute_LL1stOrder_y(double alpha);

    protected:
      Params params ; 
      const Basic2DArray<T> &input ;
      Basic2DArray<T>& output ;
      Basic2DArray<double> *input_, *output_ ;
    };


 template <class T> 
   inline void toDouble(const Basic2DArray<T>& a, Basic2DArray<double>& b){
   b.resize(a.rows(),a.cols());
   for(int i=a.rows()-1;i>=0;--i)
     for(int j=a.cols()-1;j>=0;--j)
       b(i,j) = double(a(i,j)) ;
 }

 template <class T> 
   inline void fromDouble(const Basic2DArray<double>& a, Basic2DArray<T>& b){
   b.resize(a.rows(),a.cols());
   for(int i=a.rows()-1;i>=0;--i)
     for(int j=a.cols()-1;j>=0;--j)
       b(i,j) = T(a(i,j)) ;
 }

 template <> 
   inline void toDouble(const Basic2DArray<double>& a, Basic2DArray<double>& b){
   if(&a == &b)
     return ;
   b.resize(a.rows(),a.cols());
   for(int i=a.rows()-1;i>=0;--i)
     for(int j=a.cols()-1;j>=0;--j)
       b(i,j) = double(a(i,j)) ;
 }

 template <> 
   inline void fromDouble(const Basic2DArray<double>& a, Basic2DArray<double>& b){
   if(&a == &b)
     return ;   
   b.resize(a.rows(),a.cols());
   for(int i=a.rows()-1;i>=0;--i)
     for(int j=a.cols()-1;j>=0;--j)
       b(i,j) = double(a(i,j)) ;
 }


}

#endif

