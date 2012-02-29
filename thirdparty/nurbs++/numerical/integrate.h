#ifndef MATRIX_INTEGRATE_H
#define MATRIX_INTEGRATE_H

#include "barray.h"

/*!
 */
namespace PLib {

  template <class T> void cdft(int n, T wr, T wi, BasicArray<T> &a) ;
  template <class T> void rdft(int n, T wr, T wi, BasicArray<T> &a) ;
  template <class T> void ddct(int n, T wr, T wi, BasicArray<T> &a) ;
  template <class T> void ddst(int n, T wr, T wi, BasicArray<T> &a) ;
  template <class T> void dfct(int n, T wr, T wi, BasicArray<T> &a) ;
  template <class T> void dfst(int n, T wr, T wi, BasicArray<T> &a) ;


  template <class T> void chebexp(double (*f)(T), T a, T b, T eps, 
				  BasicArray<T> &c, T &err) ;
  template <class T> void chebexp(double (*f)(T,void*), void*, T a, T b, 
				  T eps, BasicArray<T> &c, T &err) ;
  template <class T> T chebeval(T x, const BasicArray<T> &c) ;


  template <class T> void intccini(BasicArray<T> &w) ;


  template <class T>
    struct ClassPO {
      virtual T operator()(T a) =0;
    };

  template <class T>
    struct ClassPOvoid {
      virtual T operator()(T a, void*) =0;
    };

  // POPtr is a pointer to a class that defines the operator()(T)
  // Using ClassPO as a base class is a good idea as they are instantiated
  // in the source file


  template <class T, class POPtr> T integrate(POPtr f, T a, T b, T eps, int n, T &err) ;
  template <class T, class POPtr> T intcc(POPtr f, T a, T b, T eps, BasicArray<T> &w, T &err);
  template <class T, class POPtr> T integrate2(POPtr f, T a, T b, T eps, int n, T &err) ;
  template <class T, class POPtr> T intcc2(POPtr f, T a, T b, T eps, BasicArray<T> w, T &err);


  // POvPtr is a pointer to a class that defines the operator()(T,void*)
  // Using ClassPOvoid as a base class is a good idea as they are instantiated
  // in the source file

  template <class T, class POvPtr> T integrate(POvPtr f,void*, T a, T b, T eps, int n, T &err) ;
  template <class T, class POvPtr> T intcc(POvPtr,void*, T a, T b, T eps, BasicArray<T> &w, T &err) ;
  template <class T, class POvPtr> T integrate2(POvPtr f,void*, T a, T b, T eps, int n, T &err) ;
  template <class T, class POvPtr> T intcc2(POvPtr,void*, T a, T b, T eps, BasicArray<T> w, T &err) ;

}

#ifdef INCLUDE_TEMPLATE_SOURCE
#include "fft.cpp"
#include "chebexp.cpp"
#include "intccq.cpp"
#endif

#endif
