#ifndef NURBS_DEFS_H_
#define NURBS_DEFS_H_

#include <fmatvec/fmatvec.h>
#include <mbsim/mbsim_event.h>
#include <iostream>

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
  Point<N>::~Point() = default;

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
  HPoint<N>::~HPoint() = default;

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
//  /*!
//   * \brief wrapper class for nurbs surface interpolating data point
//   */

  template <typename T>
  class GeneralMatrix {
    public:
      GeneralMatrix();
      GeneralMatrix(int nrows_, int ncols_);
      GeneralMatrix(const GeneralMatrix<T>& m);
      ~GeneralMatrix();

      GeneralMatrix<T>& operator=(const GeneralMatrix<T>& m);

      T& operator()(int row, int col);
      T const& operator()(int row, int col) const;
      int rows() const;
      int cols() const;
      void resize(int nrows_, int ncols_);
      
    private:
      int nrows{0}, ncols{0};
      T* data;
  };

  template <typename T>
  inline GeneralMatrix<T>::GeneralMatrix() :
       data(nullptr) {
  }

  template <typename T>
  inline GeneralMatrix<T>::GeneralMatrix(int nrows_, int ncols_) :
      nrows(nrows_), ncols(ncols_) {
    if (nrows_ == 0 || ncols_ == 0)
      throw MBSim::MBSimError("(GeneralMatrix:: The number of rows or columns of the matrix is zero!)");
    data = new T[nrows_ * ncols_];
  }
  template <typename T>
  inline GeneralMatrix<T>::GeneralMatrix(const GeneralMatrix<T>& m) :
      nrows(m.nrows), ncols(m.ncols) {
    data = new T[nrows * ncols];
    std::copy(m.data, m.data + nrows * ncols, data);
  }

  template <typename T>
  inline GeneralMatrix<T>::~GeneralMatrix() {
    delete[] data;
  }

  template <typename T>
  inline GeneralMatrix<T>& GeneralMatrix<T>::operator=(const GeneralMatrix<T>& m) {
    nrows = m.nrows;
    ncols = m.ncols;
    data = m.data;
    return *this;
  }

  template <typename T>
  inline T& GeneralMatrix<T>::operator()(int row, int col) {
    if (row >= nrows || col >= ncols)
      throw MBSim::MBSimError("(GeneralMatrix::operator(): trying to access data out of range of the GeneralMatrix!)");
    return data[row * ncols + col];
  }

  template <typename T>
  inline T const& GeneralMatrix<T>::operator()(int row, int col) const {
    if (row >= nrows || col >= ncols)
      throw MBSim::MBSimError("(GeneralMatrix::operator(): trying to access data out of range of the GeneralMatrix!)");
    return data[row * ncols + col];
  }

  template <typename T>
  inline int GeneralMatrix<T>::rows() const {
    return nrows;
  }

  template <typename T>
  inline int GeneralMatrix<T>::cols() const {
    return ncols;
  }

  template <typename T>
  void GeneralMatrix<T>::resize(int nrows_, int ncols_) {

    if (nrows_ <= 0 || ncols_ <= 0) {
      throw MBSim::MBSimError("(GeneralMatrix::resize(): non-positive resize number of rows or columns!)");
      return;
    }

    if (nrows_ == nrows && ncols_ == ncols)
      return;

    auto* newData = new T[nrows_ * ncols_];

    delete[] data;
    data = newData;
    nrows = nrows_;
    ncols = ncols_;
  }

  template <typename T>
  std::ostream& operator<<(std::ostream &os, const GeneralMatrix<T> &A) {
    os << A.rows() << " x " << A.cols() << std::endl;
    os << " = " << std::endl;
    os << "[ ";
    for (int i = 0; i < A.rows(); ++i) {
      for (int j = 0; j < A.cols(); ++j) {
        os << trans(A.data[i * A.ncols + j]);
      }
      if (i != A.rows() - 1)
        os << std::endl << "  ";
    }
    os << " ];";
    return os;
  }

}

#endif
