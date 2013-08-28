#ifndef NURBS_DEFS_H_
#define NURBS_DEFS_H_

#include <fmatvec/fmatvec.h>

namespace fmatvec {
  /*!
   * \brief wrapper class for nurbs type Point
   */
  template <int N>
  class Point : public Vector<Fixed<N>, double> {
    public:
      Point<N>();

      Point<N>(Vector<Fixed<N>, double> pnt);

      virtual ~Point<N>();

    protected:
  };

  template <int N>
  Point<N>::Point() :
      Vector<Fixed<N>, double>() {

  }

  template <int N>
  Point<N>::Point(Vector<Fixed<N>, double> pnt) :
      Vector<Fixed<N>, double>(pnt) {
  }


  template <int N>
  Point<N>::~Point() {
  }

  /*!
   * \brief wrapper class for nurbs type HPoint
   */
  template <int N>
  class HPoint : public Vector<Fixed<N + 1>, double> {
    public:
      HPoint<N>();

      HPoint<N>(Vector<Fixed<N + 1>, double> pnt);

      virtual ~HPoint<N>();

      Point<N> projectW();

      double w();

      double w() const;

    protected:
  };

  template <int N>
  HPoint<N>::HPoint() :
      Vector<Fixed<N + 1>, double>() {
  }

  template <int N>
  HPoint<N>::HPoint(Vector<Fixed<N + 1>, double> pnt) :
      Vector<Fixed<N + 1>, double>(pnt) {
  }

  template <int N>
  HPoint<N>::~HPoint() {
  }

  template <int N>
  Point<N> HPoint<N>::projectW() {
    Point<N> pnt;
    for (int i = 0; i < N; i++)
      pnt(i) = (*this)(i);
    return pnt;
  }

  template <int N>
  double HPoint<N>::w() {
    return (*this)(N);
  }

  template <int N>
  double HPoint<N>::w() const {
    return (*this)(N);
  }

  /*mathematical operations*/
  template <int N>
  Point<N> project(const HPoint<N> & hPnt) {
    Point<N> pnt;
    for (int i = 0; i < N; i++)
      pnt(i) = hPnt(i) / hPnt.w();
    return pnt;
  }

}

#endif
