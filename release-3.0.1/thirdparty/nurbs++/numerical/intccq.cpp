/*=============================================================================
        File: intccq.cpp
     Purpose:       
    Revision: $Id: intccq.cpp,v 1.2 2002/05/13 21:07:45 philosophil Exp $
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

#ifndef intccqq_SOURCES
#define intccqq_SOURCES


/*
Clenshaw-Curtis-Quadrature
Numerical Automatic Integrator
    method    : Chebyshev Series Expansion
    dimension : one
    table     : use
function
    intcc  : integrator of f(x) over [a,b].
necessary package
    fft2f.c  : FFT package
*/

/*
intcc
    [description]
        I = integral of f(x) over [a,b]
    [declaration]
        void intccini(int lenw, double *w);
        void intcc(double (*f)(double), double a, double b, double eps, 
            int lenw, double *w, double *i, double *err);
    [usage]
        intccini(lenw, w);  // initialization of w
        ...
        intcc(f, a, b, eps, lenw, w, &i, &err);
    [parameters]
        lenw      : (length of w[]) - 1 (int)
        w         : work area and weights of the quadrature 
                    formula, w[0...lenw] (double *)
        f         : integrand f(x) (double (*f)(double))
        a         : lower limit of integration (double)
        b         : upper limit of integration (double)
        eps       : relative error requested (double)
        i         : approximation to the integral (double *)
        err       : estimate of the absolute error (double *)
    [remarks]
        initial parameters
            lenw >= 14 and 
            lenw > (maximum number of f(x) evaluations) * 3 / 2
            example :
                lenc = 3200;
        function
            f(x) needs to be analytic over [a,b].
        relative error
            eps is relative error requested excluding 
            cancellation of significant digits.
            i.e. eps means : (absolute error) / 
                             (integral_a^b |f(x)| dx).
            eps does not mean : (absolute error) / I.
        error message
            err >= 0 : normal termination.
            err < 0  : abnormal termination (n > nmax).
                       i.e. convergent error is detected :
                           1. f(x) or (d/dx)^n f(x) has 
                              discontinuous points or sharp 
                              peaks over [a,b].
                              you must use other routine.
                           2. relative error of f(x) is 
                              greater than eps.
                           3. f(x) has oscillatory factor 
                              and frequency of the oscillation 
                              is very high.
*/


#include "integrate.h"
#include <math.h>

/*!
 */
namespace PLib {

/*!
  \brief initialize the Chebychev Vector
  
  \param w  the vector to initialize

  \warning w must already be set to the proper size.

  \author Philippe Lavoie 
  \date 24 January 1997
*/
template <class T>
void intccini(BasicArray<T> &w)
{
  int lenw = w.n() - 1 ; 
  int j, k, l, m;
  T cos2, sin1, sin2, hl;
    
  cos2 = 0;
  sin1 = 1;
  sin2 = 1;
  hl = T(0.5);
  k = lenw;
  l = 2;
  while (l < k - l - 1) {
    w[0] = hl * T(0.5);
    for (j = 1; j <= l; j++) {
      w[j] = hl / (1 - 4 * j * j);
    }
    w[l] *= T(0.5);
    dfct(l, T(0.5) * cos2, sin1, w);
    cos2 = sqrt(2 + cos2);
    sin1 /= cos2;
    sin2 /= 2 + cos2;
    w[k] = sin2;
    w[k - 1] = w[0];
    w[k - 2] = w[l];
    k -= 3;
    m = l;
    while (m > 1) {
      m >>= 1;
      for (j = m; j <= l - m; j += (m << 1)) {
	w[k] = w[j];
	k--;
      }
    }
    hl *= T(0.5);
    l *= 2;
  }
}


/*!
  \brief integral of \a f(x) over \a [a,b]

  Computes the integral of \a f(x) over \a [a,b] using
  Chebyshev Series Expansion. This technique is
  called Clenshaw-Curtis-Quadrature. 

  \latexonly
  The code was originally writen by Takuya OOURA 
  (email: \verb|ooura@mmm.t.u-tokyo.ac.jp|)
  \endlatexonly
  \htmlonly
  The code was originally writen by 
  <A HREF="mailto:ooura@mmm.t.u-tokyo.ac.jp">Takuya OOURA</A>.
  \endhtmlonly

  \param   f  the function to integrate
  \param a  the lower range
  \param b  the uper range of the integration
  \param eps  relative error requested
  \param err  estimate of the absolute error
\latexonly
	            if  $>= 0$ : normal termination

		    if  $< 0$ : abnormal termination (n > nmax). 
		      i.e. convergent error is detected:
		      \begin{enumerate}
		      \item f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      \item relative error of f(x) is greater than eps.
		      \item f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      \end{enumerate}
\endlatexonly
\htmlonly
	            if  err &gt;= 0 : normal termination

		    if  err &lt;< 0 : abnormal termination (n &gt; nmax). 
		      i.e. convergent error is detected:
		      <UL>
		      <LI> f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      <LI> relative error of f(x) is greater than eps.
		      <LI> f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      </UL>
\endhtmlonly
  \return the approximation of the integral
  \warning
  \author Takuya OOURA (1996)
  \author Philippe Lavoie 
  \date 22 September 1998
*/
template <class T, class OPPtr>
T intcc(OPPtr f, T a, T b, T eps, BasicArray<T> &w, T& err)
{
  T i ; 
  int j, k, l;
  T esf, eref, erefh, hh, ir, iback, irback, ba, ss, x, y, fx, 
    errir;
  int lenw = w.n() - 1 ;
  esf = 10;
  ba = T(0.5) * (b - a);
  ss = 2 * w[lenw];
  x = ba * w[lenw];
  w[0] = T(0.5) * (*f)(a);
  w[3] = T(0.5) * (*f)(b);
  w[2] = (*f)(a + x);
  w[4] = (*f)(b - x);
  w[1] = (*f)(a + ba);
  eref = T(0.5) * (fabs(w[0]) + fabs(w[1]) + fabs(w[2]) + fabs(w[3]) + 
		   fabs(w[4]));
  w[0] += w[3];
  w[2] += w[4];
  ir = w[0] + w[1] + w[2];
  i = w[0] * w[lenw - 1] + w[1] * w[lenw - 2] + w[2] * w[lenw - 3];
  erefh = eref * sqrt(eps);
  eref *= eps;
  hh = T(0.25);
  l = 2;
  k = lenw - 5;
  do {
    iback = i;
    irback = ir;
    x = ba * w[k + 1];
    y = 0;
    i = w[0] * w[k];
    for (j = 1; j <= l; j++) {
      x += y;
      y += ss * (ba - x);
      fx = (*f)(a + x) + (*f)(b - x);
      ir += fx;
      i += w[j] * w[k - j] + fx * w[k - j - l];
      w[j + l] = fx;
    }
    ss = 2 * w[k + 1];
    err = esf * l * fabs(i - iback);
    hh *= T(0.25);
    errir = hh * fabs(ir - 2 * irback);
    l *= 2;
    k -= l + 2;
  } while ((err > erefh || errir > eref) && k > 4 * l);
  i *= b - a;
  if (err > erefh || errir > eref) {
    err *= -fabs(b - a);
  } else {
    err = eref * fabs(b - a);
  }
  return i ;
}

/*!
  \brief integral of \a f(x) over \a [a,b]

  Computes the integral of \a f(x) over \a [a,b] using
  Chebyshev Series Expansion. This technique is
  called Clenshaw-Curtis-Quadrature. 

  \latexonly
  The code was originally writen by Takuya OOURA 
  (email: \verb|ooura@mmm.t.u-tokyo.ac.jp|)
  \endlatexonly
  \htmlonly
  The code was originally writen by 
  <A HREF="mailto:ooura@mmm.t.u-tokyo.ac.jp">Takuya OOURA</A>.
  \endhtmlonly

  \param   f  the function to integrate
  \param a  the lower range
  \param b  the uper range of the integration
  \param eps  relative error requested
  \param err  estimate of the absolute error
\latexonly
	            if  $>= 0$ : normal termination

		    if  $< 0$ : abnormal termination (n > nmax). 
		      i.e. convergent error is detected:
		      \begin{enumerate}
		      \item f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      \item relative error of f(x) is greater than eps.
		      \item f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      \end{enumerate}
\endlatexonly
\htmlonly
	            if  err &gt;= 0 : normal termination

		    if  err &lt;< 0 : abnormal termination (n &gt; nmax). 
		      i.e. convergent error is detected:
		      <UL>
		      <LI> f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      <LI> relative error of f(x) is greater than eps.
		      <LI> f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      </UL>
\endhtmlonly
  \return the approximation of the integral
  \warning
  \author Takuya OOURA (1996)
  \author Philippe Lavoie 
  \date 22 September 1998
*/
template <class T, class OPPtr>
T intcc2(OPPtr f, T a, T b, T eps, BasicArray<T> w, T& err)
{
  T i ; 
  int j, k, l;
  T esf, eref, erefh, hh, ir, iback, irback, ba, ss, x, y, fx, 
    errir;
  int lenw = w.n() - 1 ;
  esf = 10;
  ba = T(0.5) * (b - a);
  ss = 2 * w[lenw];
  x = ba * w[lenw];
  w[0] = T(0.5) * (*f)(a);
  w[3] = T(0.5) * (*f)(b);
  w[2] = (*f)(a + x);
  w[4] = (*f)(b - x);
  w[1] = (*f)(a + ba);
  eref = T(0.5) * (fabs(w[0]) + fabs(w[1]) + fabs(w[2]) + fabs(w[3]) + 
		   fabs(w[4]));
  w[0] += w[3];
  w[2] += w[4];
  ir = w[0] + w[1] + w[2];
  i = w[0] * w[lenw - 1] + w[1] * w[lenw - 2] + w[2] * w[lenw - 3];
  erefh = eref * sqrt(eps);
  eref *= eps;
  hh = T(0.25);
  l = 2;
  k = lenw - 5;
  do {
    iback = i;
    irback = ir;
    x = ba * w[k + 1];
    y = 0;
    i = w[0] * w[k];
    for (j = 1; j <= l; j++) {
      x += y;
      y += ss * (ba - x);
      fx = (*f)(a + x) + (*f)(b - x);
      ir += fx;
      i += w[j] * w[k - j] + fx * w[k - j - l];
      w[j + l] = fx;
    }
    ss = 2 * w[k + 1];
    err = esf * l * fabs(i - iback);
    hh *= T(0.25);
    errir = hh * fabs(ir - 2 * irback);
    l *= 2;
    k -= l + 2;
  } while ((err > erefh || errir > eref) && k > 4 * l);
  i *= b - a;
  if (err > erefh || errir > eref) {
    err *= -fabs(b - a);
  } else {
    err = eref * fabs(b - a);
  }
  return i ;
}

template <class T, class OPPtr> 
T integrate(OPPtr f, T a, T b, T eps, int n, T &err) {
  BasicArray<T> w(n) ; 
  intccini(w) ; 
  return intcc(f,a,b,eps,w,err) ; 
}

template <class T, class OPPtr> 
T integrate2(OPPtr f, T a, T b, T eps, int n, T &err) {
  BasicArray<T> w(n) ; 
  intccini(w) ; 
  return intcc2(f,a,b,eps,w,err) ; 
}

/*!
  \brief integral of \a f(x) over \a [a,b]

  Computes the integral of \a f(x) over \a [a,b] using
  Chebyshev Series Expansion. This technique is
  called Clenshaw-Curtis-Quadrature. 

  \latexonly
  The code was originally writen by Takuya OOURA 
  (email: \verb|ooura@mmm.t.u-tokyo.ac.jp|)
  \endlatexonly
  \htmlonly
  The code was originally writen by 
  <A HREF="mailto:ooura@mmm.t.u-tokyo.ac.jp">Takuya OOURA</A>.
  \endhtmlonly

  \param   f  the function to integrate
  \param a  the lower range
  \param b  the uper range of the integration
  \param eps  relative error requested
  \param err  estimate of the absolute error
\latexonly
	            if  $>= 0$ : normal termination

		    if  $< 0$ : abnormal termination (n > nmax). 
		      i.e. convergent error is detected:
		      \begin{enumerate}
		      \item f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      \item relative error of f(x) is greater than eps.
		      \item f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      \end{enumerate}
\endlatexonly
\htmlonly
	            if  err &gt;= 0 : normal termination

		    if  err &lt;< 0 : abnormal termination (n &gt; nmax). 
		      i.e. convergent error is detected:
		      <UL>
		      <LI> f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      <LI> relative error of f(x) is greater than eps.
		      <LI> f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      </UL>
\endhtmlonly
  \return the approximation of the integral
  \warning
  \author Takuya OOURA (1996)
  \author Philippe Lavoie 
  \date 22 September 1998
*/
template <class T, class OPvPtr>
T intcc(OPvPtr f, void* userData, T a, T b, T eps, BasicArray<T> &w, T& err)
{
  T i ; 
  int j, k, l;
  T esf, eref, erefh, hh, ir, iback, irback, ba, ss, x, y, fx, 
    errir;
  int lenw = w.n() - 1 ;
  esf = 10;
  ba = T(0.5) * (b - a);
  ss = 2 * w[lenw];
  x = ba * w[lenw];
  w[0] = T(0.5) * (*f)(a,userData);
  w[3] = T(0.5) * (*f)(b,userData);
  w[2] = (*f)(a + x,userData);
  w[4] = (*f)(b - x,userData);
  w[1] = (*f)(a + ba,userData);
  eref = T(0.5) * (fabs(w[0]) + fabs(w[1]) + fabs(w[2]) + fabs(w[3]) + 
		   fabs(w[4]));
  w[0] += w[3];
  w[2] += w[4];
  ir = w[0] + w[1] + w[2];
  i = w[0] * w[lenw - 1] + w[1] * w[lenw - 2] + w[2] * w[lenw - 3];
  erefh = eref * sqrt(eps);
  eref *= eps;
  hh = T(0.25);
  l = 2;
  k = lenw - 5;
  do {
    iback = i;
    irback = ir;
    x = ba * w[k + 1];
    y = 0;
    i = w[0] * w[k];
    for (j = 1; j <= l; j++) {
      x += y;
      y += ss * (ba - x);
      fx = (*f)(a + x,userData) + (*f)(b - x,userData);
      ir += fx;
      i += w[j] * w[k - j] + fx * w[k - j - l];
      w[j + l] = fx;
    }
    ss = 2 * w[k + 1];
    err = esf * l * fabs(i - iback);
    hh *= T(0.25);
    errir = hh * fabs(ir - 2 * irback);
    l *= 2;
    k -= l + 2;
  } while ((err > erefh || errir > eref) && k > 4 * l);
  i *= b - a;
  if (err > erefh || errir > eref) {
    err *= -fabs(b - a);
  } else {
    err = eref * fabs(b - a);
  }
  return i ;
}

/*!
  \brief integral of \a f(x) over \a [a,b]

  Computes the integral of \a f(x) over \a [a,b] using
  Chebyshev Series Expansion. This technique is
  called Clenshaw-Curtis-Quadrature. 

  \latexonly
  The code was originally writen by Takuya OOURA 
  (email: \verb|ooura@mmm.t.u-tokyo.ac.jp|)
  \endlatexonly
  \htmlonly
  The code was originally writen by 
  <A HREF="mailto:ooura@mmm.t.u-tokyo.ac.jp">Takuya OOURA</A>.
  \endhtmlonly

  \param   f  the function to integrate
  \param a  the lower range
  \param b  the uper range of the integration
  \param eps  relative error requested
  \param err  estimate of the absolute error
\latexonly
	            if  $>= 0$ : normal termination

		    if  $< 0$ : abnormal termination (n > nmax). 
		      i.e. convergent error is detected:
		      \begin{enumerate}
		      \item f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      \item relative error of f(x) is greater than eps.
		      \item f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      \end{enumerate}
\endlatexonly
\htmlonly
	            if  err &gt;= 0 : normal termination

		    if  err &lt;< 0 : abnormal termination (n &gt; nmax). 
		      i.e. convergent error is detected:
		      <UL>
		      <LI> f(x) or (d/dx)^n f(x) has discontinuous points 
		            or sharp peaks over [a,b]. You must use another 
			    routine.
		      <LI> relative error of f(x) is greater than eps.
		      <LI> f(x) has oscillatory factor  and frequency of 
		            the oscillation is very high.
		      </UL>
\endhtmlonly
  \return the approximation of the integral
  \warning
  \author Takuya OOURA (1996)
  \author Philippe Lavoie 
  \date 22 September 1998
*/
template <class T, class OPvPtr>
T intcc2(OPvPtr f, void* userData, T a, T b, T eps, BasicArray<T> w, T& err)
{
  T i ; 
  int j, k, l;
  T esf, eref, erefh, hh, ir, iback, irback, ba, ss, x, y, fx, 
    errir;
  int lenw = w.n() - 1 ;
  esf = 10;
  ba = T(0.5) * (b - a);
  ss = 2 * w[lenw];
  x = ba * w[lenw];
  w[0] = T(0.5) * (*f)(a,userData);
  w[3] = T(0.5) * (*f)(b,userData);
  w[2] = (*f)(a + x,userData);
  w[4] = (*f)(b - x,userData);
  w[1] = (*f)(a + ba,userData);
  eref = T(0.5) * (fabs(w[0]) + fabs(w[1]) + fabs(w[2]) + fabs(w[3]) + 
		   fabs(w[4]));
  w[0] += w[3];
  w[2] += w[4];
  ir = w[0] + w[1] + w[2];
  i = w[0] * w[lenw - 1] + w[1] * w[lenw - 2] + w[2] * w[lenw - 3];
  erefh = eref * sqrt(eps);
  eref *= eps;
  hh = T(0.25);
  l = 2;
  k = lenw - 5;
  do {
    iback = i;
    irback = ir;
    x = ba * w[k + 1];
    y = 0;
    i = w[0] * w[k];
    for (j = 1; j <= l; j++) {
      x += y;
      y += ss * (ba - x);
      fx = (*f)(a + x,userData) + (*f)(b - x,userData);
      ir += fx;
      i += w[j] * w[k - j] + fx * w[k - j - l];
      w[j + l] = fx;
    }
    ss = 2 * w[k + 1];
    err = esf * l * fabs(i - iback);
    hh *= T(0.25);
    errir = hh * fabs(ir - 2 * irback);
    l *= 2;
    k -= l + 2;
  } while ((err > erefh || errir > eref) && k > 4 * l);
  i *= b - a;
  if (err > erefh || errir > eref) {
    err *= -fabs(b - a);
  } else {
    err = eref * fabs(b - a);
  }
  return i ;
}

template <class T, class OPvPtr> 
T integrate(OPvPtr f, void* userData, T a, T b, T eps, int n, T &err) {
  BasicArray<T> w(n) ; 
  intccini(w) ; 
  return intcc(f,userData,a,b,eps,w,err) ; 
}

template <class T, class OPvPtr> 
T integrate2(OPvPtr f, void* userData, T a, T b, T eps, int n, T &err) {
  BasicArray<T> w(n) ; 
  intccini(w) ; 
  return intcc2(f,userData,a,b,eps,w,err) ; 
}



}


#endif
