#include <config.h>

#include "nurbs_curve.h"
#include <vector>
#include <mbsim/mbsim_event.h>

#include <iostream>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  HPoint<3> NurbsCurve::operator()(double u) const {
    return hpointAt(u, findSpan(u));
  }

  HPoint<3> NurbsCurve::hpointAt(double u, int span) const {
    Vec Nb(deg + 1, NONINIT);

    basisFuns(u, span, deg, U, Nb);

    RowVec4 p;
    for (int i = deg; i >= 0; --i) {
      p += Nb(i) * P.row(span - deg + i);
    }
    return trans(p);
  }

  Point<3> NurbsCurve::pointAt(double u) const {
    return project(hpointAt(u));
  }

  void NurbsCurve::deriveAtH(double u, int d, MatVx4& ders) const {
    int du = min(d, deg);
    int span;
    Mat derF(du + 1, deg + 1);
    ders.resize(d + 1);

    span = findSpan(u);
    dersBasisFuns(du, u, span, deg, U, derF);
    //TODO: for one point not the complete loop is needed?!
    for (int k = du; k >= 0; --k) {
      ders.set(k, RowVec4());
      for (int j = deg; j >= 0; --j) {
        ders.add(k, derF(k, j) * P.row(span - deg + j));
      }
    }
  }

  HPoint<3> NurbsCurve::derive(double u, int d) const {
    MatVx4 ders;
    deriveAtH(u, d, ders);
    return trans(ders.row(d));
  }

  HPoint<3> NurbsCurve::firstD(double u) const {
    return firstD(u, findSpan(u));
  }

  HPoint<3> NurbsCurve::firstD(double u, int span) const {
    int i;

    Vec N(deg, NONINIT);

    basisFuns(u, span, deg - 1, U, N);

    RowVec4 Cd;

    for (i = deg - 1; i >= 0; --i) {
      int j = span - deg + i;
      RowVec4 Qi = P.row(j + 1) - P.row(j);
      Qi *= double(deg) / (U(j + deg + 1) - U(j + 1));
      Cd += N(i) * Qi;
    }

    return trans(Cd);
  }

  Point<3> NurbsCurve::firstDn(double u) const {
    return firstDn(u, findSpan(u));
  }

  Point<3> NurbsCurve::firstDn(double u, int span) const {
    Point < 3 > Cp;
    HPoint<3> Cd;
    Cd = firstD(u, span);

    Point < 3 > pd(Cd.projectW());
    double w = Cd.w();
    Cd = hpointAt(u, span);
    pd -= w * project(Cd);
    pd /= Cd.w();

    return pd;
  }

  Point<3> NurbsCurve::normal(double u, const fmatvec::Point<3> & v) const {
    return crossProduct(firstDn(u), v);
  }

  // use the user offered knot point vector uk  to calculate the knotvector U by funtion knotAveraging
  //  knotAveraging is the same as updateUVecs if offering uniform u vector.
  void NurbsCurve::globalInterp(const std::vector<fmatvec::Point<3> >& Q, const std::vector<double>& uk, int d, bool updateLater) {
    MatVx3 Qnew(Q.size(), NONINIT);

    // Transformation from the user given standard vector form to fmetvec::MatVx3 form
    for (int i = 0; i < Qnew.rows(); i++)
      for (int j = 0; j < 3; j++)
        Qnew(i, j) = Q.at(i)(j);

    if (d <= 0) {
      throw runtime_error("Degree is too small!");
    }
    if (d >= Qnew.rows()) {
      throw runtime_error("The degree specified is greater then Q.rows()+1\n");
    }

    // resize control point matrix and knotVector matrix
    deg = d;
    P.resize(Qnew.rows());
    U.resize(Qnew.rows() + d + 1);

    SqrMat A(Qnew.rows(), INIT, 0.);

    knotAveraging(uk, deg); // as deg=d, in resize();

    // Initialize the basis matrix A
    Vec N(deg + 1, NONINIT);

    for (int i = 1; i < Qnew.rows() - 1; i++) {
      int span = findSpan(uk.at(i));
      basisFuns(uk.at(i), span, deg, U, N);
      for (int j = 0; j <= deg; j++) {
        A(i, span - deg + j) = N(j);
      }
    }
    A(0, 0) = 1.0;
    A(Qnew.rows() - 1, Qnew.rows() - 1) = 1.0;

    // Set weights to 1
    for (int i = 0; i < P.rows(); i++) {
      P(i, 3) = 1.0;
    }

    if (updateLater) {
      inverse.resize() = inv(A);
      update(Qnew);
    }
    else {
      P.set(RangeV(0, P.rows() - 1), RangeV(0, 2), slvLU(A, Mat(Qnew)));
    }
  }

  void NurbsCurve::globalInterp(const std::vector<fmatvec::Point<3> >& Q, double uMin, double uMax, int d, bool updateLater) {
    MatVx3 Qnew(Q.size(), NONINIT);

    // Transformation from the user given standard vector form to fmetvec::MatVx3 form
    for (int i = 0; i < Qnew.rows(); i++)
      for (int j = 0; j < 3; j++)
        Qnew(i, j) = Q.at(i)(j);

    globalInterp(Qnew, uMin, uMax, d, updateLater);
  }

  void NurbsCurve::globalInterp(const MatVx3& Q, double uMin, double uMax, int d, bool updateLater) {
    int i, j;

    if (d <= 0) {
      throw runtime_error("Degree is too small!");
    }
    if (d >= Q.rows()) {
      throw runtime_error("The degree specified should be smaller or equal than Q.rows()-1\n");
    }

    resize(Q.rows(), d);
    SqrMat A(Q.rows(), INIT, 0.);

    updateUVecs(uMin, uMax);

//    double chordLength = chordLengthParam(Q,u);
//    msg(Debug) << "chrodLength" << chordLength << endl;
//    knotAveraging(u, deg);

    // Initialize the basis matrix A
    Vec N(deg + 1, NONINIT);

    for (i = 1; i < Q.rows() - 1; i++) {
      int span = findSpan(u(i));
//      msg(Debug) << span << endl;
      basisFuns(u(i), span, deg, U, N);
      for (j = 0; j <= deg; j++) {
        A(i, span - deg + j) = N(j);
//        msg(Debug) << N(j) << endl;
      }
    }
    A(0, 0) = 1.0;
    A(Q.rows() - 1, Q.rows() - 1) = 1.0;

    // Set weights to 1
    for (i = 0; i < P.rows(); i++) {
      P(i, 3) = 1.0;
    }

    if (updateLater) {
      inverse.resize() = inv(A);
      update(Q);
    }
    else {
      P.set(RangeV(0, P.rows() - 1), RangeV(0, 2), slvLU(A, Mat(Q)));
    }
  }

  void NurbsCurve::globalInterpClosed(const fmatvec::MatVx3 & Q, double uMin, double uMax, int d, bool updateLater) {
    int i, j;

    int iN = Q.rows();
    resize(iN + d, d);

    SqrMat A(iN, INIT, 0.);

    updateUVecsClosed(uMin, uMax);

// Initialize the basis matrix A
    Vec N(d + 1);

    for (i = 0; i < iN; i++) {
      int span = findSpan(u(i));
      basisFuns(u(i), span, d, U, N);
      for (j = span - d; j <= span; j++)
        A(i, j % (iN)) = (double) N(j - span + d);
    }

    // Set weights to 1
    for (i = 0; i < P.rows(); i++) {
      P(i, 3) = 1.0;
    }

    if (updateLater) {
      inverse.resize() = inv(A);
      update(Q);
    }
    else {
      P.set(RangeV(0, P.rows() - d - 1), RangeV(0, 2), slvLU(A, Mat(Q)));
      // Wrap around of control points
      //Possible: wrapping around is just a reference ?
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j < 3; j++)
          P(P.rows() - deg + i, j) = P(i, j);
      }
    }

  }

  void NurbsCurve::globalInterpH(const MatVx4& Qw, int d, Method method) {
    int i,j;

    resize(Qw.rows(), d);

    if(method == chordLength) {
      chordLengthParamH(Qw,u) ;

      // Setup the Knot Vector for the curve
      for(i=0; i<=deg; i++)
        U(i) = 0 ;
      for(i=P.rows(); i<U.rows(); i++)
        U(i) = 1.0 ;
      for(j=1; j<Qw.rows()-deg; j++){
        double t=0 ;
        for(i=j; i< j+deg; i++)
          t += u(i) ;
        U(j+deg) = t/(double)deg ;
      }
    }
    else if(method == equallySpaced)
      updateUVecs(0, 1);
    else
      throw runtime_error("(NurbsCurve::globalInterpH: method unknown)");

    globalInterpH(Qw,u,U,d);
  }

  void NurbsCurve::globalInterpH(const MatVx4& Qw, const Vec& ub, const Vec& Uc, int d, bool updateLater) {
    int i, j;

    SqrMat A(Qw.rows(), INIT, 0.);

    if (Uc.rows() != U.rows())
      resize(Qw.rows(), d);

    U = Uc;

    // Init matrix for LSE
//    Matrix<T> qq(Q.rows(),D+1) ;
//    Matrix<T> xx(Q.rows(),D+1) ;

//    for(i=0;i<Q.rows();i++)
//      for(j=0; j<4;j++)
//        qq(i,j) = (double)Q[i].data[j] ;

//    if (Inverse_setted != 1) //changed
//      computeInverse(ub, Uc, d);
//
//    xx = Inverse * qq;
//
//    // Store the data
//    for(i=0;i<xx.rows();i++){
//      for(j=0;j<D+1;j++)
//        P[i].data[j] = (T)xx(i,j) ;
//    }

    Vec N(deg + 1, NONINIT);

    for (i = 1; i < Qw.rows() - 1; i++) {
      int span = findSpan(ub(i));
//      msg(Debug) << span << endl;
      basisFuns(ub(i), span, deg, U, N);
      for (j = 0; j <= deg; j++) {
        A(i, span - deg + j) = N(j);
//        msg(Debug) << N(j) << endl;
      }
    }
    A(0, 0) = 1.0;
    A(Qw.rows() - 1, Qw.rows() - 1) = 1.0;
//    msg(Debug) << "A = " << A << endl << endl;
    if (updateLater) {
      inverse.resize() = inv(A);
      update(Qw);
    }
    else {
      P = slvLU(A, Mat(Qw));
    }

  }
  /*!
   \brief global curve interpolation with homogenous points

   Global curve interpolation with 4D points, a knot vector
   defined and the parametric value vector defined.The curve will have C(d-1)
   continuity at the point u=0 and u=1.

   \param Qw  the 3D points to interpolate (wrapped around)
   \param ub  the parametric values vector
   \param Uc  the knot vector computed using knotAveragingClosed
   \param d   the degree of the closed curve

   \warning The number of points to interpolate must be greater than
   the degree specified for the curve. Uc must be compatible with
   the values given for Q.n(), ub.n().
   \author  Alejandro Frangi
   \date 13 July, 1998
   */
  void NurbsCurve::globalInterpClosedH(const MatVx4& Qw, const Vec& ub, const Vec& Uc, int d, bool updateLater) {
    int i, j;

//  int iN = Qw.rows() - d - 1;
//  resize(Qw.rows(),d) ;
    int iN = Qw.rows();
    resize(iN + d, d);

    SqrMat A(iN, INIT, 0.);
    if (Uc.rows() != U.rows())
      throw runtime_error("(NurbsCurve::globalInterpClosedH: The length of knot vectors are not equal !)");

    U = Uc;
    // Initialize the basis matrix A
    Vec N(d + 1);

    for (i = 0; i < iN; i++) {
      int span = findSpan(ub(i));
      basisFuns(ub(i), span, d, U, N);
      for (j = span - d; j <= span; j++)
        A(i, j % (iN)) = (double) N(j - span + d);
    }

//    msg(Debug) << "A = "  << A  << endl << endl;

    if (updateLater) {
      inverse.resize() = inv(A);
      update(Qw);
    }
    else {
      P = slvLU(A, Mat(Qw));
      // Wrap around of control points
      //Possible: wrapping around is just a reference ?
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j < 3; j++)
          P(P.rows() - deg + i, j) = P(i, j);
      }
    }

  }
  void NurbsCurve::update(const MatVx3 & Q) {

    if (P.rows() > Q.rows()) { //closed interpolation

      P.set(RangeV(0, P.rows() - deg - 1), RangeV(0, 2), inverse * Q);
      // Wrap around of control points
      //Possible: wrapping around is just a reference ?
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j < 3; j++)
          P(P.rows() - deg + i, j) = P(i, j);
      }
    }
    else {
      P.set(RangeV(0, P.rows() - 1), RangeV(0, 2), inverse * Q);
    }
  }

  void NurbsCurve::update(const MatVx4 & Qw) {

    if (P.rows() > Qw.rows()) { //closed interpolation

      P.set(RangeV(0, P.rows() - deg - 1), RangeV(0, 3), inverse * Qw);
      // Wrap around of control points
      //Possible: wrapping around is just a reference ?
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j < 4; j++)
          P(P.rows() - deg + i, j) = P(i, j);
      }
    }
    else {
      P = inverse * Qw;
    }
//    msg(Debug) << "fmatvec_surface: Qw =" << Qw << endl;
//    msg(Debug) << "fmatvec_surface: P =" << P << endl;

  }
  void NurbsCurve::resize(int n, int Deg) {
    deg = Deg;
    P.resize(n);
    u.resize(n);
    U.resize(n + Deg + 1);
  }

  void NurbsCurve::knotAveraging(const vector<double>& uk, int deg) {
    int j;
    for (j = 1; j < int(uk.size() - deg); ++j) {
      U(j + deg) = 0.0;
      for (int i = j; i < j + deg; ++i)
        U(j + deg) += uk[i];
      U(j + deg) /= (double) deg;
    }
    for (j = 0; j <= deg; ++j)
      U(j) = 0.0;
    for (j = U.size() - deg - 1; j < U.size(); ++j)
      U(j) = 1.0;
  }

  double NurbsCurve::chordLengthParam(const MatVx3& Q, Vec& ub) {
    int i;
    double d = 0;

    ub.resize(Q.rows());
    ub(0) = 0;
    for (i = 1; i < ub.rows(); i++) {
      d += nrm2(Q.row(i) - Q.row(i - 1));
    }
    if (d > 0) {
      for (i = 1; i < ub.rows() - 1; ++i) {
        ub(i) = ub(i - 1) + nrm2(Q.row(i) - Q.row(i - 1)) / d;
      }
      ub(ub.rows() - 1) = 1.0; // In case there is some addition round-off
    }
    else {
      for (i = 1; i < ub.rows() - 1; ++i)
        ub(i) = double(i) / double(ub.rows() - 1);
      ub(ub.rows() - 1) = 1.0;
    }
    return d;
  }

  double NurbsCurve::chordLengthParamH(const MatVx4& Qw, Vec& ub) {
    int i;
    double d = 0.0;

    ub.resize(Qw.rows());
    ub(0) = 0;
    for(i=1; i<ub.rows(); i++) {
      d += nrm2(Qw.row(i)-Qw.row(i-1));
    }
    for(i=1; i<ub.rows()-1; i++) {
      ub(i) = ub(i-1) + nrm2(Qw.row(i)-Qw.row(i-1))/d;
    }
    ub(ub.rows()-1) = 1.0; // In case there is some addition round-off
    return d;
  }

  void NurbsCurve::updateUVecs(double uMin, double uMax) {
    const double stepU = (uMax - uMin) / (u.rows() - 1);
    u(0) = uMin;
    for (int i = 1; i < u.rows(); i++) {
      u(i) = u(i - 1) + stepU;
    }

    int j;
    for (j = 0; j < deg + 1; j++) { // the first and last (degV+1)-Values have to be equal
      U(j) = uMin;
      U(U.rows() - j - 1) = uMax;
    }

    for (j = 1; j < u.rows() - deg; ++j) {
      U(j + deg) = 0.0;
      for (int i = j; i < j + deg; ++i)
        U(j + deg) += u(i);
      U(j + deg) /= deg;
    }
  }

  void NurbsCurve::knotAveragingClosed(const vector<double>& uk, int deg) {

    U.resize(uk.size() + deg + 1);

    int i, j;
    int index;
    int iN = uk.size() - deg - 1;
    int n = uk.size() - 1;
    int m = U.rows() - 1;

    // Build temporary average sequence
    // Data stored in range U[deg+1 .. n]
    for (j = 0; j <= iN; j++) {
      index = j + deg + 1;
      U(index) = 0.0;
      for (i = j; i <= j + deg - 1; i++)
        U(index) += uk[i];
      U(index) /= deg;
    }

    // Now make the left and right periodic extensions
    // Left
    for (j = 0; j < deg; j++)
      U(j) = U(j + iN + 1) - 1;
    // Right
    for (j = n + 1; j <= m; j++)
      U(j) = 1 + U(j - (iN + 1));

  }

  void NurbsCurve::updateUVecsClosed(double uMin, double uMax) {
    const double stepU = (uMax - uMin) / (u.rows() - deg);

    u(0) = 0.;
    for (int i = 1; i < u.rows(); i++) {
      u(i) = u(i - 1) + stepU;
    }

    U(0) = -deg * stepU;
    for (int i = 1; i < U.rows(); i++) {
      U(i) = U(i - 1) + stepU;
    }

  }

  int NurbsCurve::findSpan(double u) const {
    if (u >= U(P.rows()))
      return P.rows() - 1;
    if (u <= U(deg))
      return deg;

    int low = 0;
    int high = P.rows() + 1;
    int mid = (low + high) / 2;

    while (u < U(mid) || u >= U(mid + 1)) {
      if (u < U(mid))
        high = mid;
      else
        low = mid;
      mid = (low + high) / 2;
    }
    return mid;

  }

  /*!
   \brief Generates a knot vector using the averaging technique
   \relates NurbsCurve

   The technique is as follows:

   - \f$ u_0 = \cdots = u_{deg} = 0 \f$
   - \f$ u_{m-deg} = \cdots = u_{m-1} = 1 \f$
   - \f$ u_{j+deg} = \frac{1}{deg}\sum_{i=j}^{j+deg+1}\bar{u}_i\hspace{0.5in} j= 1,\ldots,n-deg-1 \f$

   where \f$n\f$ is the size of the \f$\bar{u}\f$ knot coefficient vector,
   \f$m=n+deg+1\f$ is the size of the knot vector and $deg$ is the
   degree of the curve.

   \param uk  the knot coefficients
   \param deg  the degree of the curve associated with the knot vector
   \param U  an average knot vector

   \author Philippe Lavoie
   \date 24 January, 1997
   */
  void knotAveraging(const Vec& uk, int deg, Vec& U) {
    //    U.resize(uk.n()+deg+1) ;
    int j;
    for (j = 1; j < uk.rows() - deg; ++j) {
      U(j + deg) = 0.0;
      for (int i = j; i < j + deg; ++i)
        U(j + deg) += uk(i);
      U(j + deg) /= (double) deg;
    }
    for (j = 0; j <= deg; ++j)
      U(j) = 0.0;
    for (j = U.rows() - deg - 1; j < U.rows(); ++j)
      U(j) = 1.0;
  }

  /*!
   \brief generates a knot vector using the averaging technique for interpolation with closed curve.

   Generates a knot vector using the averaging technique for interpolation with closed curve. See eq 9.9 in the NURBS Book

   \param uk  the knot coefficients
   \param deg  the degree of the curve associated with the knot vector
   \param U  an average knot vector

   \author Alejandro Frangi
   \date 13 July, 1998
   */
  void knotAveragingClosed(const Vec& uk, int deg, Vec& U) {
//    U.resize(uk.n()+deg+1) ;
    int i, j;
    int index;
    int iN = uk.rows() - deg - 1;
    int n = uk.rows() - 1;
    int m = U.rows() - 1;

    // Build temporary average sequence
    // Data stored in range U(deg+1 .. n)
    for (j = 0; j <= iN; j++) {
      index = j + deg + 1;
      U(index) = 0.0;
      for (i = j; i <= j + deg - 1; i++)
        U(index) += uk(i);
      U(index) /= (double) deg;
    }

    // Now make the left and right periodic extensions
    // Left
    for (j = 0; j < deg; j++)
      U(j) = U(j + iN + 1) - 1;
    // Right
    for (j = n + 1; j <= m; j++)
      U(j) = 1 + U(j - (iN + 1));

  }

  void basisFuns(double u, int span, int deg, const Vec & U, Vec& funs) {
    auto* left = (double*) alloca(2 * (deg + 1) * sizeof(double));
    double* right = &left[deg + 1];

    double temp, saved;

    funs.resize(deg + 1);

    funs(0) = 1.0;
    for (int j = 1; j <= deg; j++) {
      left[j] = u - U(span + 1 - j);
      right[j] = U(span + j) - u;
      saved = 0.0;
      for (int r = 0; r < j; r++) {
        temp = funs(r) / (right[r + 1] + left[j - r]);
        funs(r) = saved + right[r + 1] * temp;
        saved = left[j - r] * temp;
      }
      funs(j) = saved;
    }
  }

  void dersBasisFuns(int n, double u, int span, int deg, const Vec & U, Mat & ders) {
    auto* left = (double*) alloca(2 * (deg + 1) * sizeof(double));
    double* right = &left[deg + 1];

    SqrMat ndu(deg + 1);
    double saved, temp;
    int j, r;

    ders.resize(n + 1, deg + 1);

    ndu(0, 0) = 1.0;
    for (j = 1; j <= deg; j++) {
      left[j] = u - U(span + 1 - j);
      right[j] = U(span + j) - u;
      saved = 0.0;

      for (r = 0; r < j; r++) {
        // Lower triangle
        ndu(j, r) = right[r + 1] + left[j - r];
        temp = ndu(r, j - 1) / ndu(j, r);
        // Upper triangle
        ndu(r, j) = saved + right[r + 1] * temp;
        saved = left[j - r] * temp;
      }

      ndu(j, j) = saved;
    }

    for (j = deg; j >= 0; --j)
      ders(0, j) = ndu(j, deg);

    // Compute the derivatives
    SqrMat a(deg + 1);
    for (r = 0; r <= deg; r++) {
      int s1, s2;
      s1 = 0;
      s2 = 1; // alternate rows in array a
      a(0, 0) = 1.0;
      // Compute the kth derivative
      for (int k = 1; k <= n; k++) {
        double d;
        int rk, pk, j1, j2;
        d = 0.0;
        rk = r - k;
        pk = deg - k;

        if (r >= k) {
          a(s2, 0) = a(s1, 0) / ndu(pk + 1, rk);
          d = a(s2, 0) * ndu(rk, pk);
        }

        if (rk >= -1) {
          j1 = 1;
        }
        else {
          j1 = -rk;
        }

        if (r - 1 <= pk) {
          j2 = k - 1;
        }
        else {
          j2 = deg - r;
        }

        for (j = j1; j <= j2; j++) {
          a(s2, j) = (a(s1, j) - a(s1, j - 1)) / ndu(pk + 1, rk + j);
          d += a(s2, j) * ndu(rk + j, pk);
        }

        if (r <= pk) {
          a(s2, k) = -a(s1, k - 1) / ndu(pk + 1, r);
          d += a(s2, k) * ndu(r, pk);
        }
        ders(k, r) = d;
        j = s1;
        s1 = s2;
        s2 = j; // Switch rows
      }
    }

    // Multiply through by the correct factors
    r = deg;
    for (int k = 1; k <= n; k++) {
      for (j = deg; j >= 0; --j)
        ders(k, j) *= r;
      r *= deg - k;
    }

  }

  // Setup the binomial coefficients into th matrix Bin
  // Bin(i,j) = (i  j)
  // The binomical coefficients are defined as follow
  //   (n)         n!
  //   (k)  =    k!(n-k)!       0<=k<=n
  // and the following relationship applies
  // (n+1)     (n)   ( n )
  // ( k ) =   (k) + (k-1)
  /*!
   \brief Setup a matrix containing binomial coefficients

   Setup the binomial coefficients into th matrix Bin

   \[ Bin(i,j) = \left( \begin{array}{c}i \\ j\end{array} \right)\]
   The binomical coefficients are defined as follow
   \[ \left(\begin{array}{c}   n \\ k \end{array} \right)= \frac{ n!}{k!(n-k)!} \mbox{for $0\leq k \leq n$} \]
   and the following relationship applies
   \[ \left(\begin{array}{c} n+1 \\ k \end{array} \right) =
   \left(\begin{array}{c} n \\ k \end{array} \right) +
   \left(\begin{array}{c} n \\ k-1 \end{array} \right) \]

   \param Bin  the binomial matrix
   \author Philippe Lavoie
   \date 24 January, 1997
   */
  void binomialCoef(Mat& Bin) {
    int n, k;
    // Setup the first line
    Bin(0, 0) = 1.0;
    for (k = Bin.cols() - 1; k > 0; --k)
      Bin(0, k) = 0.0;
    // Setup the other lines
    for (n = 0; n < Bin.rows() - 1; n++) {
      Bin(n + 1, 0) = 1.0;
      for (k = 1; k < Bin.cols(); k++) {
        if (n + 1 < k)
          Bin(n, k) = 0.0;
        else
          Bin(n + 1, k) = Bin(n, k) + Bin(n, k - 1);
      }
    }
  }

}
