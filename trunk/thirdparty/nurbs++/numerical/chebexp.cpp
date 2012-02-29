/*
Chebyshev Series Expansion and Interpolation routine
functions
    chebexp  : function f(x) -> Chebyshev series
    chebeval : Chebyshev series -> evaluation of the f(x)
necessary package
    fft2f.c  : FFT package
*/

/*
chebexp, chebeval
    [description]
        evaluation of f(x) from Chebyshev series of f(x)
    [declaration]
        void chebexp(double (*f)(double), double a, double b, 
            double eps, int lenc, double *c, double *err);
        double chebeval(double x, double *c);
    [usage]
        chebexp(f, a, b, eps, lenc, c, &err);  // f(x) -> c[]
        ...
        y = chebeval(x, c);  // evaluation of the f(x) from c[]
    [parameters]
        f         : function f(x) (double (*f)(double))
        a         : lower limit of interpolation (double)
        b         : upper limit of interpolation (double)
        eps       : relative error of interpolation (double)
        lenc      : (length of c[]) - 1 (int)
        c         : data of Chebyshev expansion, 
                    c[0...lenc] (double *)
        err       : estimate of the maximum absolute error 
                    of the interpolation over [a,b] (double *)
    [remarks]
        initial parameters
            lenc >= 12
            example :
                lenc = 1024 + 4;
        function
            f(x) needs to be analytic over [a,b].
        relative error
            eps is relative error requested excluding 
            cancellation of significant digits.
            i.e. eps means : (maximum absolute error) / 
                             (integral_a^b |f(x)| dx).
            eps does not mean : (maximum absolute error) / I.
        error message
            err >= 0 : normal termination.
            err < 0  : abnormal termination (n > lenc-4).
                       i.e. convergent error is detected :
                           1. f(x) or (d/dx)^n f(x) has 
                              discontinuous points or sharp 
                              peaks over [a,b].
                           2. relative error of f(x) is 
                              greater than eps.
                           3. f(x) has oscillatory factor 
                              and frequency of the oscillation 
                              is very high.
        array of c[]
            lenc           : (int) c[0]
            n              : (int) c[1]
            (b+a)/2        : c[2]
            2/(b-a)        : c[3]
            f(c[2]-t/c[3]) : c[lenc]*T_0(t) + c[lenc-1]*T_1(t) 
                             + ... + c[lenc-n]*T_n(t)
*/

#ifndef chebexp_SOURCES
#define chebexp_SOURCES

#include "integrate.h"
#include <math.h>

/*!
 */
namespace PLib {

template <class T>
void chebexp(double (*f)(T), T a, T b, T eps, 
    BasicArray<T> &c, T &err)
{
    int lenc = c.n() ;
    int j, k, l, n;
    T esf, ba, cos2, sin2, wi, ss, x, y, t, h, eref, erefh, errh;

    

    esf = 10;
    ba = T(0.5) * (b - a);
    c[0] = T(0.5) * (*f)(a);
    c[2] = T(0.5) * (*f)(b);
    c[1] = (*f)(a + ba);
    c[lenc - 1] = c[0] - c[2];
    c[lenc] = c[0] + c[2] + c[1];
    c[lenc - 2] = c[0] + c[2] - c[1];
    cos2 = 0;
    sin2 = 1;
    wi = -1;
    h = 1;
    l = 1;
    n = 2;
    eref = erefh = T() ; 
    do {
        ss = 2 * sin2;
        cos2 = sqrt(2 + cos2);
        sin2 /= 2 + cos2;
        x = ba * sin2;
        y = 0;
        for (j = 0; j <= l - 1; j++) {
            x += y;
            y += ss * (ba - x);
            c[j] = (*f)(a + x);
            c[n - 1 - j] = (*f)(b - x);
        }
        wi /= cos2;
        ddct(n, T(0.5) * cos2, wi, c);
        l = n;
        n *= 2;
        for (j = l - 1; j >= 0; j--) {
            k = lenc - j;
            t = c[k] - c[j];
            c[k] += c[j];
            c[lenc - n + j] = t;
        }
        if (n == 4) {
            eref = T(0.25) * (fabs(c[lenc]) + fabs(c[lenc - 1]) + 
                fabs(c[lenc - 2]) + fabs(c[lenc - 3]) + fabs(c[lenc - 4]));
            erefh = eref * sqrt(eps);
            eref *= eps;
            err = erefh;
        }
        h *= T(0.5);
        errh = esf * err;
        err = h * (T(0.5) * fabs(c[lenc - n]) + fabs(c[lenc - n + 1]));
    } while ((err > eref || errh > erefh) && 2 * n + 4 <= lenc);
    c[lenc - n] *= T(0.5);
    c[lenc] *= T(0.5);
    for (j = lenc - n; j <= lenc; j++) {
        c[j] *= h;
    }
    if (err > eref || errh > erefh) {
        err = -(err);
    } else {
        do {
            n -= 2;
            err += fabs(c[lenc - n]) + fabs(c[lenc - n + 1]);
        } while (err < eref && n > 2);
        n += 2;
        err = eref;
    }
    if (ba != 0) {
        c[3] = 1 / ba;
    } else {
        c[3] = 0;
    }
    c[2] = T(0.5) * (b + a);
    c[1] = n + T(0.5);
    c[0] = lenc + T(0.5);
}


template <class T>
T chebeval(T x, const BasicArray<T> &c)
{
    int lenc, n, j;
    T t, t2, v0, v1;

    lenc = (int) c[0];
    n = (int) c[1];
    t = (c[2] - x) * c[3];
    t2 = 2 * t;
    v0 = c[lenc - n];
    v1 = c[lenc - n + 1] + t2 * v0;
    for (j = lenc - n + 2; j <= lenc - 2; j += 2) {
        v0 = c[j] + t2 * v1 - v0;
        v1 = c[j + 1] + t2 * v0 - v1;
    }
    return c[lenc] + t * v1 - v0;
}

template <class T>
void chebexp(double (*f)(T,void*), void* userData, T a, T b, T eps, 
    BasicArray<T> &c, T &err)
{
    int lenc = c.n() ;
    int j, k, l, n;
    T esf, ba, cos2, sin2, wi, ss, x, y, t, h, eref, erefh, errh;

    

    esf = 10;
    ba = T(0.5) * (b - a);
    c[0] = T(0.5) * (*f)(a,userData);
    c[2] = T(0.5) * (*f)(b,userData);
    c[1] = (*f)(a + ba,userData);
    c[lenc - 1] = c[0] - c[2];
    c[lenc] = c[0] + c[2] + c[1];
    c[lenc - 2] = c[0] + c[2] - c[1];
    cos2 = 0;
    sin2 = 1;
    wi = -1;
    h = 1;
    l = 1;
    n = 2;
    eref = erefh = T() ; 
    do {
        ss = 2 * sin2;
        cos2 = sqrt(2 + cos2);
        sin2 /= 2 + cos2;
        x = ba * sin2;
        y = 0;
        for (j = 0; j <= l - 1; j++) {
            x += y;
            y += ss * (ba - x);
            c[j] = (*f)(a + x,userData);
            c[n - 1 - j] = (*f)(b - x,userData);
        }
        wi /= cos2;
        ddct(n, T(0.5) * cos2, wi, c);
        l = n;
        n *= 2;
        for (j = l - 1; j >= 0; j--) {
            k = lenc - j;
            t = c[k] - c[j];
            c[k] += c[j];
            c[lenc - n + j] = t;
        }
        if (n == 4) {
            eref = T(0.25) * (fabs(c[lenc]) + fabs(c[lenc - 1]) + 
                fabs(c[lenc - 2]) + fabs(c[lenc - 3]) + fabs(c[lenc - 4]));
            erefh = eref * sqrt(eps);
            eref *= eps;
            err = erefh;
        }
        h *= T(0.5);
        errh = esf * err;
        err = h * (T(0.5) * fabs(c[lenc - n]) + fabs(c[lenc - n + 1]));
    } while ((err > eref || errh > erefh) && 2 * n + 4 <= lenc);
    c[lenc - n] *= T(0.5);
    c[lenc] *= T(0.5);
    for (j = lenc - n; j <= lenc; j++) {
        c[j] *= h;
    }
    if (err > eref || errh > erefh) {
        err = -(err);
    } else {
        do {
            n -= 2;
            err += fabs(c[lenc - n]) + fabs(c[lenc - n + 1]);
        } while (err < eref && n > 2);
        n += 2;
        err = eref;
    }
    if (ba != 0) {
        c[3] = 1 / ba;
    } else {
        c[3] = 0;
    }
    c[2] = T(0.5) * (b + a);
    c[1] = n + T(0.5);
    c[0] = lenc + T(0.5);
}


}

#endif
