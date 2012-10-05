/*=============================================================================
        File: rec_filter.cpp
     Purpose:
    Revision: $Id: rec_filter.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (18 February 1999)
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

#ifndef REC_FILTER_SOURCE
#define REC_FILTER_SOURCE

#include "rec_filter.h"

/*!
 */
namespace PLib{

void toParams(double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double b1, double b2, double c1, double c2, double k, Params& params){
  params.a1 = a1 ;
  params.a2 = a2 ;
  params.a3 = a3 ;
  params.a4 = a4 ;
  params.a5 = a5 ;
  params.a6 = a6 ;
  params.a7 = a7 ;
  params.a8 = a8 ;
  params.b1 = b1 ;
  params.b2 = b2 ;
  params.c1 = c1 ;
  params.c2 = c2 ; 
  params.k = k ;
}

void fromParams(const Params& params, double &a1, double &a2, double &a3, double &a4, double &a5, double &a6, double &a7, double &a8, double &b1, double &b2, double &c1, double &c2, double &k){
  a1 = params.a1 ;
  a2 = params.a2 ;
  a3 = params.a3 ;
  a4 = params.a4 ;
  a5 = params.a5 ;
  a6 = params.a6 ;
  a7 = params.a7 ;
  a8 = params.a8 ;
  b1 = params.b1 ;
  b2 = params.b2 ;
  c1 = params.c1 ;
  c2 = params.c2 ; 
  k = params.k ;
}

void generalRFx(const Params& params, const Basic2DArray<double> &x, Basic2DArray<double> &y){
  Basic2DArray<double> y1,y2 ;
  y1.resize(x);
  y2.resize(x);
  y.resize(x);
  
  double a1,a2,a3,a4,a5,a6,a7,a8 ;
  double b1,b2 ;
  double c1,c2 ;
  double k ;
  
  fromParams(params,a1,a2,a3,a4,a5,a6,a7,a8,b1,b2,c1,c2,k) ;
  int i,j ;
  const int rows = x.rows() ;
  const int cols = x.cols() ; 
  
  for(i=0;i<rows-1;++i){
    y1(i,0) = a1*x(i,0) ;
    y1(i,1) = a1*x(i,1) + a2*x(i,0) + b1*y1(i,0) ;
    for(j=2;j<cols-1;++j){
      y1(i,j) = a1*x(i,j) + a2*x(i,j-1) + b1*y1(i,j-1) + b2*y1(i,j-2) ;
    }
  }

  for(i=rows-1;i>=0;--i){
    j = cols-1 ;
    y2(i,j) = 0 ;
    y(i,j) = c1*(y1(i,j)+y2(i,j));
    j = cols-2 ;
    y2(i,j) = a3*x(i,j+1) + b1*y2(i,j+1) ;
    y(i,j) = c1*(y1(i,j)+y2(i,j));
    for(j=cols-3;j>=0;--j){
      y2(i,j) = a3*x(i,j+1) + a4*x(i,j+2) + b1*y2(i,j+1) + b2*y2(i,j+2) ;
      y(i,j) = c1*(y1(i,j)+y2(i,j));
    }
  }  

}


void generalRFy(const Params& params, const Basic2DArray<double> &r, Basic2DArray<double> &y){
  Basic2DArray<double> y1,y2 ;
  y1.resize(r);
  y2.resize(r);
  y.resize(r);
  
  double a1,a2,a3,a4,a5,a6,a7,a8 ;
  double b1,b2 ;
  double c1,c2 ;
  double k ;
  
  fromParams(params,a1,a2,a3,a4,a5,a6,a7,a8,b1,b2,c1,c2,k) ;
  int i,j ;
  const int rows = r.rows() ;
  const int cols = r.cols() ; 
  
  for(j=0;j<cols-1;++j){
    y1(0,j) = a5*r(0,j) ;
    y1(1,j) = a5*r(1,j) + a6*r(0,j) + b1*y1(0,j);
    for(i=2;i<rows-1;++i){
      y1(i,j) = a5*r(i,j) + a6*r(i-1,j) + b1*y1(i-1,j) + b2*y1(i-2,j) ;
    }
  }

  for(j=cols-1;j>=0;--j){
    i = rows-1 ;
    y2(i,j) = 0 ;
    y(i,j) = c2*(y1(i,j)+y2(i,j));
    i = rows-2 ;
    y2(i,j) = a7*r(i+1,j) + b1*y2(i+1,j) ;
    y(i,j) = c2*(y1(i,j)+y2(i,j));    
    for(i=rows-3;i>=0;--i){
      y2(i,j) = a7*r(i+1,j) + a8*r(i+2,j) + b1*y2(i+1,j) + b2*y2(i+2,j) ;
      y(i,j) = c2*(y1(i,j)+y2(i,j));
    }
  }
}



void generalRF(const Params& params, const Basic2DArray<double> &x, Basic2DArray<double> &y){
  Basic2DArray<double> y1,y2,r ;
  y1.resize(x);
  y2.resize(x);
  r.resize(x) ;
  y.resize(x);
  
  double a1,a2,a3,a4,a5,a6,a7,a8 ;
  double b1,b2 ;
  double c1,c2 ;
  double k ;
  
  fromParams(params,a1,a2,a3,a4,a5,a6,a7,a8,b1,b2,c1,c2,k) ;

  int i,j ;
  const int rows = x.rows() ;
  const int cols = x.cols() ; 
  
  for(i=0;i<rows-1;++i){
    y1(i,0) = a1*x(i,0) ;
    y1(i,1) = a1*x(i,1) + a2*x(i,0) + b1*y1(i,0) ;
    for(j=2;j<cols-1;++j){
      y1(i,j) = a1*x(i,j) + a2*x(i,j-1) + b1*y1(i,j-1) + b2*y1(i,j-2) ;
    }
  }

  for(i=rows-1;i>=0;--i){
    j = cols-1 ;
    y2(i,j) = 0 ;
    r(i,j) = c1*(y1(i,j)+y2(i,j));
    j = cols-2 ;
    y2(i,j) = a3*x(i,j+1) + b1*y2(i,j+1) ;
    r(i,j) = c1*(y1(i,j)+y2(i,j));
    for(j=cols-3;j>=0;--j){
      y2(i,j) = a3*x(i,j+1) + a4*x(i,j+2) + b1*y2(i,j+1) + b2*y2(i,j+2) ;
      r(i,j) = c1*(y1(i,j)+y2(i,j));
    }
  }  

  for(j=0;j<cols-1;++j){
    y1(0,j) = a5*r(0,j) ;
    y1(1,j) = a5*r(1,j) + a6*r(0,j) + b1*y1(0,j);
    for(i=2;i<rows-1;++i){
      y1(i,j) = a5*r(i,j) + a6*r(i-1,j) + b1*y1(i-1,j) + b2*y1(i-2,j) ;
    }
  }

  for(j=cols-1;j>=0;--j){
    i = rows-1 ;
    y2(i,j) = 0 ;
    y(i,j) = c2*(y1(i,j)+y2(i,j));
    i = rows-2 ;
    y2(i,j) = a7*r(i+1,j) + b1*y2(i+1,j) ;
    y(i,j) = c2*(y1(i,j)+y2(i,j));    
    for(i=rows-3;i>=0;--i){
      y2(i,j) = a7*r(i+1,j) + a8*r(i+2,j) + b1*y2(i+1,j) + b2*y2(i+2,j) ;
      y(i,j) = c2*(y1(i,j)+y2(i,j));
    }
  }

}

void smooth2ndOrder(Params& params, double alpha){
  const double expa = exp(-alpha) ;
  const double expa2 = exp(-2*alpha) ;
  params.k = (1-expa)*(1-expa)/(1+2*alpha*expa-expa2) ;
  params.b1 = 2*expa ;
  params.b2 = -expa2 ;
  params.a1 = params.a5 = params.k ;
  params.a2 = params.a6 = params.k*expa*(alpha-1) ;
  params.a3 = params.a7 = params.k*expa*(alpha+1) ;
  params.a4 = params.a8 = -params.k*expa2;
  params.c1 = params.c2 = 1 ;
}

void xderiv2ndOrderSmooth(Params& params, double alpha){
  const double expa = exp(-alpha) ;
  const double expa2 = exp(-2*alpha) ;
  params.k = (1-expa)*(1-expa)/(1+2*alpha*expa-expa2) ;
  params.b1 = 2*expa ;
  params.b2 = -expa2 ;
  params.a1 = 0 ;
  params.a2 = 1 ;
  params.a3 = -1 ;
  params.a4 = 0 ;
  params.a5 = params.k ;
  params.a6 = params.k*expa*(alpha-1) ;
  params.a7 = params.k*expa*(alpha+1) ;
  params.a8 = -params.k*expa2 ;
  params.c1 = -1*(1-expa)*(1-expa);
  params.c2 = 1 ;
}

void yderiv2ndOrderSmooth(Params& params, double alpha){
  const double expa = exp(-alpha) ;
  const double expa2 = exp(-2*alpha) ;
  params.k = (1-expa)*(1-expa)/(1+2*alpha*expa-expa2) ;
  params.b1 = 2*expa ;
  params.b2 = -expa2 ;
  params.a1 = params.k ;
  params.a2 = params.k*expa*(alpha-1) ;
  params.a3 = params.k*expa*(alpha+1) ;
  params.a4 = -params.k*expa2 ;
  params.a5 = 0 ;
  params.a6 = 1 ;
  params.a7 = -1 ;
  params.a8 = 0 ;
  params.c1 = 1 ;
  params.c2 = -1*(1-expa)*(1-expa);
}

void xderiv2ndOrder(Params& params, double alpha){
  const double expa = exp(-alpha) ;
  const double expa2 = exp(-2*alpha) ;
  params.k = (1-expa)*(1-expa)/(expa) ;
  params.b1 = 2*expa ;
  params.b2 = -2*expa2 ;
  params.a1 = 0 ;
  params.a2 = 1 ;
  params.a3 = -1 ;
  params.a4 = 0 ;
  params.c1 = params.k*expa ;
  params.c2 = params.k*expa ;
}

void yderiv2ndOrder(Params& params, double alpha){
  const double expa = exp(-alpha) ;
  const double expa2 = exp(-2*alpha) ;
  params.k = (1-expa)*(1-expa)/(expa) ;
  params.b1 = 2*expa ;
  params.b2 = -2*expa2 ;
  params.a5 = 0 ;
  params.a6 = 1 ;
  params.a7 = -1 ;
  params.a8 = 0 ;
  params.c1 = params.k*expa ;
  params.c2 = params.k*expa ;
}

void smooth1stOrder(Params& params, double alpha, double k0){
  const double expa = exp(-alpha) ;
  params.k = (1-expa)/(1+expa) ; //(1-exp(-2*alpha))/(2*alpha*expa) ; 
  params.b1 = expa ;
  params.b2 = 0;
  params.a1 = 1 ;
  params.a2 = 0 ;
  params.a3 = expa ;
  params.a4 = 0 ;
  params.a5 = 1 ;
  params.a6 = 0 ;
  params.a7 = expa ;
  params.a8 = 0 ;
  params.c1 = params.k ;
  params.c2 = params.k ;
}

void LL1stOrder(Params& params, double alpha){
  const double expa = exp(-alpha) ;
  const double expa2 = exp(-2*alpha);
  params.k = (1-expa2)/(2*alpha*expa) ; //(1-expa)/(1+expa) ;
  params.b1 = 2*expa ;
  params.b2 = -expa2;
  params.a1 = 0 ;
  params.a2 = 1 ;
  params.a3 = 1 ;
  params.a4 = 0 ;
  params.a5 = 0 ;
  params.a6 = 1 ;
  params.a7 = 1 ;
  params.a8 = 0 ;
  params.c1 = params.c2 = (1-expa2)/2 ;
}

void ubyteToDouble(const Basic2DArray<unsigned char>& img, Basic2DArray<double>& mat){
  mat.resize(img.rows(),img.cols());
  for(int i=img.rows()-1;i>=0;--i)
    for(int j=img.cols()-1;j>=0;--j)
      mat(i,j) = img(i,j) ;
}

void doubleToUbyte(const Basic2DArray<double>& mat, Basic2DArray<unsigned char>& img){
  img.resize(mat.rows(),mat.cols());
  for(int i=img.rows()-1;i>=0;--i)
    for(int j=img.cols()-1;j>=0;--j)
      img(i,j) = (unsigned char)mat(i,j) ;
}

double quadInterp(double x, double x0, double f0, double x1, double f1, double x2, double f2){
  const double f01 = firstDivDiff(x0,f0,x1,f1) ;
  const double f012 = secondDivDiff(x0,f0,x1,f1,x2,f2);
  double fx = f0 + (x-x0)*f01 + (x-x0)*(x-x1)*f012 ;
  return fx;
}

int findEdge(const Basic2DArray<double>& dx, const Basic2DArray<double>& dy, Basic2DArray<double> &edge, Basic2DArray<double>& gradN, double thresh){
  if(dx.rows() != dy.rows() || dx.cols() != dy.cols())
    return 0;
  edge.resize(dx) ;
  gradN.resize(dx);
  //Basic2DArray<double> gradN(dx.rows(),dx.cols());
  int i,j ;

  for(i=0;i<dx.rows();++i)
    for(j=0;j<dx.cols();++j){
      gradN(i,j) = sqrt(dx(i,j)*dx(i,j) + dy(i,j)*dy(i,j)) ;
    }
 

  for(i=1;i<dx.rows()-1;++i)
    for(j=1;j<dx.cols()-1;++j){
      if(absolute(dx(i,j)) > absolute(dy(i,j))){
	double d = 1/dx(i,j) ;
	double y = dy(i,j)/dx(i,j) ;
	double b = quadInterp(i+y,i-1,gradN(i-1,j+1),i,gradN(i,j+1),i+1,gradN(i+1,j+1));
	double c = quadInterp(i-y,i-1,gradN(i-1,j-1),i,gradN(i,j-1),i+1,gradN(i+1,j-1));
	if(gradN(i,j) >= b && gradN(i,j) >= c && gradN(i,j) > thresh)
	  edge(i,j) = 200;
	else
	  edge(i,j) = 0 ; 

      }
      else{
	double d = 1/dy(i,j) ;
	double x = dx(i,j)/dy(i,j) ;
	double b = quadInterp(j-x,j-1,gradN(i-1,j-1),j,gradN(i-1,j),j+1,gradN(i-1,j+1));
	double c = quadInterp(j+x,j-1,gradN(i+1,j-1),j,gradN(i+1,j),j+1,gradN(i+1,j+1));
	if(gradN(i,j) >= b && gradN(i,j) >= c && gradN(i,j) > thresh)
	  edge(i,j) = 200;
	else
	  edge(i,j) = 0 ; 
      }
    }
  return 1;
}

int findSubEdge(const Basic2DArray<double>& dx, const Basic2DArray<double>& dy, Basic2DArray<double> &edge, double thresh, const Basic2DArray<double>& in){
  if(dx.rows() != dy.rows() || dx.cols() != dy.cols())
    return 0;
  const int zoom = 7 ;
  const int zoom2 = 4 ;
  edge.resize(dx.rows()*zoom,dx.cols()*zoom) ;
  Basic2DArray<double> gradN(dx.rows(),dx.cols());
  int i,j ;

  for(i=0;i<dx.rows();++i)
    for(j=0;j<dx.cols();++j){
      gradN(i,j) = sqrt(dx(i,j)*dx(i,j) + dy(i,j)*dy(i,j)) ;
    }
 

  for(i=1;i<dx.rows()-1;++i)
    for(j=1;j<dx.cols()-1;++j)
      for(int k=0;k<zoom;++k)
	for(int l=0;l<zoom;l++)
	  edge(i*zoom+k-zoom2,j*zoom+l-zoom2) =  (in(i,j)>250) ? 250 : in(i,j);


  for(i=1;i<dx.rows()-1;++i)
    for(j=1;j<dx.cols()-1;++j){
      if(absolute(dx(i,j)) > absolute(dy(i,j))){
	double d = 1/dx(i,j) ;
	double y = dy(i,j)/dx(i,j) ;
	double c = quadInterp(i+y,i-1,gradN(i-1,j+1),i,gradN(i,j+1),i+1,gradN(i+1,j+1));
	double a = quadInterp(i-y,i-1,gradN(i-1,j-1),i,gradN(i,j-1),i+1,gradN(i+1,j-1));
	if(gradN(i,j) >= a && gradN(i,j) >= c && gradN(i,j) > thresh){
	  double m = (a-c)/( 2*(a-2*gradN(i,j)+c)) ;
	  double di, dj ;
	  if(absolute(m)>0.5) cerr << " m = " << m << " " << a << " " << gradN(i,j) << " " << c << " at " << i << j << endl ; 
	  dj = zoom*m*absolute(dx(i,j))/gradN(i,j) ;
	  di = zoom*m*absolute(dy(i,j))/gradN(i,j) ;
	  if(m<0 && dx(i,j)*dy(i,j)<0)
	    di *= -1 ;
	  if(m>0 && dx(i,j)*dy(i,j)<0)
	    di *= -1 ;
	  for(int k=0;k<zoom;++k)
	    for(int l=0;l<zoom;l++)
	      edge(i*zoom+k-zoom2,j*zoom+l-zoom2) = 255;
	  edge(i*zoom+di,j*zoom+dj) = 254 ; 
	}
      }
      else{
	double d = 1/dy(i,j) ;
	double x = dx(i,j)/dy(i,j) ;
	double a = quadInterp(j-x,j-1,gradN(i-1,j-1),j,gradN(i-1,j),j+1,gradN(i-1,j+1));
	double c = quadInterp(j+x,j-1,gradN(i+1,j-1),j,gradN(i+1,j),j+1,gradN(i+1,j+1));
	if(gradN(i,j) >= a && gradN(i,j) >= c && gradN(i,j) > thresh){
	  double m = (a-c)/( 2*(a-2*gradN(i,j)+c)) ;
	  double di, dj ;
	  if(absolute(m)>0.5) cerr << " m = " << m << " " << a << " " << gradN(i,j) << " " << c << " at " << i << j << endl ; 
	  dj = zoom*m*absolute(dx(i,j))/gradN(i,j);
	  if(m<0 && dx(i,j)*dy(i,j)<0)
	    dj *= -1 ; 
	  if(m>0 && dx(i,j)*dy(i,j)<0)
	    dj *= -1 ; 
	  di = zoom*m*absolute(dy(i,j))/gradN(i,j) ; 
	  for(int k=0;k<zoom;++k)
	    for(int l=0;l<zoom;l++)
	      edge(i*zoom+k-zoom2,j*zoom+l-zoom2) = 255 ;
	  edge(i*zoom+di,j*zoom+dj) = 254 ; 
	}
      }
    }
  return 1;
}


/*
int main(){
  IM_Image img;
  if(!img.read("test.png")){
    cerr << "Can't open file test.png\n" ;
  }
  Matrix<double> in,dx,dy,e,e2,g ;
  ubyteToDouble(img,in);
  Params params ;

  const double alpha = 1.0 ;

  xderiv2ndOrder(params,alpha);
  generalRFx(params,in,dx);

  yderiv2ndOrder(params,alpha);
  generalRFy(params,in,dy);
  cout << "Testing the quad interpolation\n" ;
  
  cout << quadInterp(2.5,0,10,5,5,10,0) << endl ; 
  cout << quadInterp(-0.5,-1,4,0,5,1,1) << endl ; 
  cout << quadInterp(-0.4,-1,4,0,5,1,1) << endl ; 
  cout << quadInterp(-0.3,-1,4,0,5,1,1) << endl ; 
  cout << quadInterp(-0.2,-1,4,0,5,1,1) << endl ; 
  cout << quadInterp(9.2,8.0,2.0794,9.0,2.1972,9.5,2.2513) << endl ; 

  findEdge(dx,dy,e,g,20);
  findSubEdge(dx,dy,e2,20,in) ;

  dx += 128 ;
  doubleToUbyte(dx,img);
  img.write("outputA.png");
  
  dy += 128 ;
  doubleToUbyte(dy,img);
  img.write("outputB.png");

  doubleToUbyte(e,img);
  img.write("edge.png");

  doubleToUbyte(e2,img);
  img.write("edge2.png");

  doubleToUbyte(g,img);
  img.write("grad.png");

  

}
*/

 template <class T> 
 RecursiveFilter<T>::RecursiveFilter(const Basic2DArray<T>& in, Basic2DArray<T>& out): input(in), output(out) {
   input_ = new Basic2DArray<double>(in.rows(),in.cols()) ;
   output_ = new Basic2DArray<double>(out.rows(),out.cols()) ;
   output.resize(out.rows(),out.cols());
   toDouble(input,*input_);
 }
 
 template <>
   RecursiveFilter<double>::RecursiveFilter(const Basic2DArray<double>& in, Basic2DArray<double>& out): input(in), output(out) {
   input_ = const_cast<Basic2DArray<double>*>(&in) ;
   //input_ = &in ;
   output_ = &out ;
   output.resize(out.rows(),out.cols());
 }
 
 template <class T>
   void RecursiveFilter<T>::compute_xderiv2ndOrderSmooth(double alpha){
   xderiv2ndOrderSmooth(params,alpha) ;
   generalRFx(params,*input_,*output_);
   cerr << "size = " << output_->rows() << "," << output_->cols() << endl ; 
   fromDouble(*output_,output) ;
 }
 
 template <class T>
   void RecursiveFilter<T>::compute_yderiv2ndOrderSmooth(double alpha){
   yderiv2ndOrderSmooth(params,alpha);
   generalRFy(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 template <class T>
   void RecursiveFilter<T>::compute_xderiv2ndOrder(double alpha){
   xderiv2ndOrder(params,alpha) ;
   generalRFx(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }				    
 
 template <class T>
   void RecursiveFilter<T>::compute_yderiv2ndOrder(double alpha){
   yderiv2ndOrder(params,alpha);
   generalRFy(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 template <class T>
   void RecursiveFilter<T>::compute_smooth1stOrder(double alpha, double k0){
   smooth1stOrder(params,alpha,k0) ;
   generalRF(params,*input_,*output_);
   fromDouble(*output_,output);
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_smooth1stOrder_x(double alpha, double k0){
   smooth1stOrder(params,alpha,k0) ;
   generalRFx(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_smooth1stOrder_y(double alpha, double k0){
   smooth1stOrder(params,alpha,k0) ;
   generalRFy(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_smooth2ndOrder(double alpha){
   smooth2ndOrder(params,alpha) ;
   generalRF(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_smooth2ndOrder_x(double alpha){
   smooth2ndOrder(params,alpha) ;
   generalRFx(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_smooth2ndOrder_y(double alpha){
   smooth2ndOrder(params,alpha) ;
   generalRFy(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_LL1stOrder(double alpha){
   LL1stOrder(params,alpha);
   generalRF(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_LL1stOrder_x(double alpha){
   LL1stOrder(params,alpha);
   generalRFx(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
 
 template <class T>
   void RecursiveFilter<T>::compute_LL1stOrder_y(double alpha){
   LL1stOrder(params,alpha);
   generalRFy(params,*input_,*output_);
   fromDouble(*output_,output) ;
 }
 
}



#endif
