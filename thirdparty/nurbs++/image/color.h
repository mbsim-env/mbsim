/*============================================================================
        File: color.h
     Purpose: 
    Revision: $Id: color.h,v 1.2 2002/05/13 21:07:45 philosophil Exp $
  Created by: Philippe Lavoie          (26 January 1999)
 Modified by: Martin Schuerch

 Copyright notice:
          Copyright (C) 1996-1999 Philippe Lavoie
 
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
#ifndef _Matrix_color_h_
#define _Matrix_color_h_

#include "matrix_global.h"
#include "matrixTool.h"

/*!
 */
namespace PLib {

  /*!
    \brief A class for a RGB color 

    Defines a class containing the RGB component of a color pixel.
    Each component is an unsigned char valid for the range 
    [0..255].

    \author Philippe Lavoie 
    \date 4 October 1996
  */
  class Color {
  public:
    unsigned char r,g,b ;
    Color(const unsigned char R=0, const unsigned char G=0, const unsigned char B=0) : r(R),g(G),b(B) {} 
    Color& operator+=(const Color& a) { r+=a.r ; g+=a.g ; b+=a.b; return *this ;} // the += operator
    Color& operator-=(const Color& a) { r+=a.r ; g+=a.g ; b+=a.b; return *this;} // the -= operator
    Color& operator*=(double a) { r = (unsigned char)(a*double(r)) ; g = (unsigned char)(a*double(g)) ; b = (unsigned char)(a*double(b)) ; return *this ; }
    Color& operator/=(double a) { r = (unsigned char)(double(r)/a) ; g = (unsigned char)(double(g)/a) ; b = (unsigned char)(double(b)/a) ; return *this ; }
    Color& operator=(const Color& a) { r=a.r ; g=a.g; b=a.b ; return *this ; } // the assignment operator

    void fromXYZ(double x, double y, double z) ;
    void toXYZ(double& x, double& y, double& z) ;

    void fromYIQ(double q, double i, double y);
    void toYIQ(double& q, double& i, double& y);

    void fromHSV(double h, double s, double v);
    void toHSV(double& h, double& s, double& v);


    friend Color operator*(const double d, const Color& a) ; // multiplies a color by a double 
    friend Color operator*(const Color& a, const Color& b) ; // multiplies a color by another color
    friend Color operator+(const Color& a, const Color& b) ; // Addition of two colors
    
    
    friend ostream& operator<<(ostream& os, const Color& point);
    friend istream& operator>>(istream& os, Color& point);
  };
  
  
  inline int operator==(const Color& a, const Color& b) { return (b.r==a.r)&&(b.g==a.g)&&(b.b==a.b) ; } // the equality operator
  inline int operator!=(const Color& a, const Color& b) { return !((b.r==a.r)||(b.g==a.g)||(b.b==a.b));} // the inequality operator
  
  
  inline int operator<(const Color& a, const Color& b){ 
    return (a.r<b.r && a.g<b.g && a.b<b.b) ; } // the smaller than operator
  inline int operator>(const Color& a, const Color& b){ 
    return (a.r>b.r && a.g>b.g && a.b>b.b) ; } // the greater than operator
  inline int operator<=(const Color& a, const Color& b){ 
    return (a.r<=b.r && a.g<=b.g && a.b<=b.b) ; } // the smaller or equal operator
  inline int operator>=(const Color& a, const Color& b){ 
    return (a.r>=b.r && a.g>=b.g && a.b>=b.b) ; } // the greater or equal operator
  
  const Color whiteColor(255,255,255);
  const Color redColor(255,0,0) ;
  const Color blueColor(0,0,255) ;
  const Color greenColor(0,255,0) ;
  const Color yellowColor(255,255,0) ;
  const Color cyanColor(0,255,255) ;
  const Color magentaColor(255,0,255);
  const Color gray80Color(204,204,204) ;
  const Color gray50Color(127,127,127) ;
  const Color blackColor(0,0,0) ;
  /*
  extern Color whiteColor ;
  extern Color redColor ;
  extern Color blueColor ;
  extern Color greenColor ;
  extern Color yellowColor ;
  extern Color cyanColor ;
  extern Color magentaColor ;
  extern Color gray80Color ;
  extern Color gray50Color ;
  extern Color blackColor ;
  */

  inline Color operator*(const double d, const Color& a) {
    Color result ;
    result.r = (unsigned char) (d*a.r) ;
    result.g = (unsigned char) (d*a.g) ;
    result.b = (unsigned char) (d*a.b) ;
    return result ;
  }
  
  inline Color operator*(const Color& a, const Color& b) {
    Color result;
    result.r = a.r*b.r ;
    result.g = a.g*b.g ;
    result.b = a.b*b.b ;
    return result ;
  }
  
  inline Color operator+(const Color& a, const Color& b) {
    Color result(a);
    result.r += b.r ;
    result.g += b.g ;
    result.b += b.b ;
    return result ;
  }
  
  /*!
    \brief A class for a RGB color defined with float variables

    Defines a class containing the RGB component of a color pixel. 
    Each component is defined with a float valid in the range 
    $[0..1]$.
    
    \author Philippe Lavoie 
    \date 4 October 1996
  */
  class ColorF {
  public:
    float r,g,b ;
    
    ColorF(float R=0.0, float G=0.0, float B=0.0) : r(R),g(G),b(B) {} 
    
    ColorF& operator=(const ColorF& a) { r=a.r ; g=a.g; b=a.b ; return *this ; }
  };
  




  /*!
     \brief The output operator of a color to an ostream

     \param os  the ostream
     \param c the color to output
     \return the ostream with the color $c$.

     \author Philippe Lavoie 
     \date 24 January 1997
  */
  inline ostream& operator<<(ostream& os,const Color& c)
  {
    os << (int)c.r << " " << (int)c.g << " " << (int)c.b << " " ;
    return os;	
  }

  /*!
     \brief the input operator of a color from an istream

     Initialize a color from an istream

     \param os  the input stream
     \param  c  the color to initialize
     \return the istream without the color
     \author Philippe Lavoie 
     \date 24 January 1997
  */
  inline istream& operator>>(istream& os, Color& c){
    os >> c.r >> c.g >> c.b ;
    return os ;
  }

  /*!
    \brief transforms from the XYZ color space to the RGB colorspace

    \author Philippe Lavoie
    \date 17 March 1999
  */
  inline void Color::fromXYZ(double x, double y, double z){
    r = (unsigned char)(255.0*(3.240479*x-1.537510*y-0.498535*z));
    g = (unsigned char)(255.0*(-0.969256*x+1.875992*y+0.041556*z));
    b = (unsigned char)(255.0*(0.055648*x-0.204043*y+1.057311*z));
  }

  inline void Color::toXYZ(double &x, double& y, double& z){
    
  }

  /*!
    \brief transforms from the YIQ color space to the RGB colorspace

    This is the same color space used by NTSC

    \param y the luminicance
    \param i the chromacity
    \param q the chromacity

    \author Philippe Lavoie
    \date 14 May 1999
  */
  inline void Color::fromYIQ(double y, double i, double q){
    r = (unsigned char)(255.0*(1.0030893*y+0.954849*i+0.6178597*q));
    g = (unsigned char)(255.0*(0.9967760*y-0.27070623*i-0.64478833*q));
    b = (unsigned char)(255.0*(1.0084978*y-1.11048518*i+1.69956753125));
  }

  /*!
    \brief transforms to the YIQ color space to the RGB colorspace

    This is the same color space used by NTSC

    \param y the luminicance
    \param i the chromacity
    \param q the chromacity

    \author Philippe Lavoie
    \date 14 May 1999
  */
  inline void Color::toYIQ(double &y, double &i, double &q){
    double R= double(R)/255.0 ;
    double G= double(R)/255.0 ;
    double B= double(R)/255.0 ;
    y = 0.299*R + 0.587*G + 0.114*B ;
    i = 0.596*R - 0.275*G - 0.321*B ;
    q = 0.212*R - 0.528*G + 0.311*B ;
  }

  /*!
    \brief from the HSV color space
    
    \param h hue valid inside [0,360]
    \param s saturation valid inside [0,1]
    \param v value valid inside [0,1]

    \author Philippe Lavoie
    \date 14 May 1999
   */
  inline void Color::fromHSV(double h, double s, double v){
    if(s==0.0){
      r=g=b=0;
      return;
    }
    if(h>=360)
      h = 0.0 ;
    if(h<=0.0)
      h = 0.0 ;
    h /= 60.0 ;
    int i = int(floor(h)) ;
    double f = h-double(i);
    double p = v*(1.0-s);
    double q = v*(1-(s*f));
    double t = v*(1-(s*(1-f)));
    switch(i){
    case 0: 
      r = (unsigned char)(255.0*v) ; 
      g = (unsigned char)(255.0*t) ; 
      b = (unsigned char)(255.0*p) ; break ; 
    case 1: 
      r = (unsigned char)(255.0*q) ; 
      g = (unsigned char)(255.0*v) ; 
      b = (unsigned char)(255.0*p) ; break ; 
    case 2: 
      r = (unsigned char)(255.0*p) ; 
      g = (unsigned char)(255.0*v) ; 
      b = (unsigned char)(255.0*t) ; break ; 
    case 3: 
      r = (unsigned char)(255.0*p) ; 
      g = (unsigned char)(255.0*q) ; 
      b = (unsigned char)(255.0*v) ; break ; 
    case 4: 
      r = (unsigned char)(255.0*t) ; 
      g = (unsigned char)(255.0*p) ; 
      b = (unsigned char)(255.0*v) ; break ; 
    case 5: 
    default:
      r = (unsigned char)(255.0*v) ; 
      g = (unsigned char)(255.0*p) ; 
      b = (unsigned char)(255.0*q) ; break ; 
    }
  }

  /*!
    \brief to the HSV color space

    \param h hue valid inside [0,360]
    \param s saturation valid inside [0,1]
    \param v value valid inside [0,1]

    \author Philippe Lavoie
    \date 14 May 1999
   */
  inline void Color::toHSV(double &h, double &s, double &v){
    double R = double(r)/255.0;
    double G = double(g)/255.0;
    double B = double(b)/255.0;
    
    double max = maximum(R,maximum(G,B));
    double min = minimum(R,minimum(G,B));
    
    int maxI = maximum(r,maximum(g,b));

    v = max ; 
    s = (max>0) ? (max-min)/max : 0 ; 
    h = 0 ;
    if(s>0){
      double delta = max-min ;
      if(r==maxI){
	h = (G-B)/delta ;
      }
      else{
	if(g==maxI){
	  h = 2.0 + (B-R)/delta ;
	}
	else{
	  h = 4.0 + (R-G)/delta ;
	}
      }
      h *= 60.0 ;
      if(h<0)
	h += 360.0 ; 
    }
  }


} // end namespace


#endif
