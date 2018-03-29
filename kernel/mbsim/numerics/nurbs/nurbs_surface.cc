#include <config.h>
//#include "plib.h"
#include <cstring>
//#include <matrixRT.h>
#include <cmath>
#include "nurbs_surface.h"

//#include "integrate.h"
//#include <malloc.h>

#include <mbsim/mbsim_event.h>
#include <iostream>

using namespace std;
using namespace fmatvec;

namespace MBSim {


  /*!
   \brief NurbsSurface assignment
   \param nS the NURBS surface to copy

   \author Philippe Lavoie
   \date 24 January, 1997
   */
  NurbsSurface& NurbsSurface::operator=(const NurbsSurface& nS) {
    P = nS.P;
    U = nS.U;
    V = nS.V;
    degU = nS.degU;
    degV = nS.degV;
    return *this;
  }

  /*!
   \brief Resize the surface

   Resize the surface. Proper values must be assigned once this
   function has been called since the resize operator is
   destructive.

   \param  Pu  the number of control points in the U direction
   \param  Pv  the number of control points in the V direction
   \param  DegU  the degree of the surface in the U direction
   \param  DegV  the degree of the surface in the V direction

   \author Philippe Lavoie
   \date 24 January, 1997
   */

  void NurbsSurface::resize(int Pu, int Pv, int DegU, int DegV) {
    P.resize(Pu, Pv);
    degU = DegU;
    degV = DegV;
    U.resize(Pu + DegU + 1);
    V.resize(Pv + DegV + 1);
  }

  /*!
   \brief Generates a surface using global interpolation

   \param Q   a matrix of 3D points
   \param DegU  the degree of interpolation in the U direction
   \param DegV  the degree of interpolation in the V direction

   \author Philippe Lavoie
   \date 24 January, 1997
   */

  void NurbsSurface::globalInterp(const GeneralMatrix<fmatvec::Vec3 >& Q, int DegU, int DegV) {

    VecV vk, uk;
    resize(Q.rows(), Q.cols(), DegU, DegV);

    MatVx4 Pts(Q.rows(), NONINIT);
    NurbsCurve R;

    for (int v = 0; v < Pts.rows(); v++)
      Pts(v, 3) = 1.0;   // set the weight to be 1 for every point

    int i, j;
    for (j = 0; j < Q.cols(); j++) {
      for (i = 0; i < Q.rows(); i++){
        Pts.set(RangeV(i, i), RangeV(0, 2), trans(Q(i, j)));
      }

      if (j == 0){
        surfMeshParams(Q, uk, vk); // map the Q point into u, v parameter space  // TODO: use the undeformed state Q to gennerate the uk and U;
        knotAveraging(uk, DegU, U);  // generate knot vector
        R.globalInterpH(Pts, uk, U, DegU, true);
      }else
        R.update(Pts);

      for (i = 0; i < Q.rows(); i++){
        P(i, j) = R.ctrlPnts(i);
      }
    }

    Pts.resize(Q.cols());

    for (i = 0; i < Q.rows(); i++) {
      for (j = 0; j < Q.cols(); j++){
        Pts.set(RangeV(j, j), RangeV(0, 3), trans(P(i, j)));
      }

      if (i == 0){
        knotAveraging(vk, DegV, V);
        R.globalInterpH(Pts, vk, V, DegV, true);  // TODO: update the V for control point in each step.
      }else
        R.update(Pts);

      for (j = 0; j < Q.cols(); j++)
        P(i, j) = R.ctrlPnts(j);
    }
  }
  
  /*!
   \brief Generates a surface using global interpolation

   \param Q   a matrix of 3D points
   \param uk  the vector of lagrange parameters of the interpolated point in U direction
   \param vk  the vector of lagrange parameters of the interpolated point in V direction
   \param DegU  the degree of interpolation in the U direction
   \param DegV  the degree of interpolation in the V direction

   */

  void NurbsSurface::globalInterp(const GeneralMatrix<fmatvec::Vec3 >& Q, const VecV& uk, const VecV& vk, int DegU, int DegV) {
    if (uk.size() != Q.rows())
      throw runtime_error("(NurbsCurve::globalInterpH: In the U direction, the length of lagrange parameters vectors is not equal to the number of the given interpolated points!)");
    if (vk.size() != Q.cols())
      throw runtime_error("(NurbsCurve::globalInterpH: In the V direction, the length of lagrange parameters vectors is not equal to the number of the given interpolated points!)");

    resize(Q.rows(), Q.cols(), DegU, DegV);

    MatVx4 Pts(Q.rows(), NONINIT);
    NurbsCurve R;

    for (int v = 0; v < Pts.rows(); v++)
      Pts(v, 3) = 1.0;   // set the weight to be 1 for every point

    int i, j;
    for (j = 0; j < Q.cols(); j++) {
      for (i = 0; i < Q.rows(); i++){
        Pts.set(RangeV(i, i), RangeV(0, 2), trans(Q(i, j)));
      }

      if (j == 0){
//        surfMeshParams(Q, uk, vk); // map the Q point into u, v parameter space  // TODO: use the undeformed state Q to gennerate the uk and U;
        knotAveraging(uk, DegU, U);  // generate knot vector
        R.globalInterpH(Pts, uk, U, DegU, true);
      }else
        R.update(Pts);

      for (i = 0; i < Q.rows(); i++){
        P(i, j) = R.ctrlPnts(i);
      }
    }

    Pts.resize(Q.cols());

    for (i = 0; i < Q.rows(); i++) {
      for (j = 0; j < Q.cols(); j++){
        Pts.set(RangeV(j, j), RangeV(0, 3), trans(P(i, j)));
      }

      if (i == 0){
        knotAveraging(vk, DegV, V);
        R.globalInterpH(Pts, vk, V, DegV, true);  // TODO: update the V for control point in each step.
      }else
        R.update(Pts);

      for (j = 0; j < Q.cols(); j++)
        P(i, j) = R.ctrlPnts(j);
    }
  }

  /*!
   \brief Generates a closed surface using global interpolation.

   Generates a NURBS surface using global interpolation. In the u
   direction the curve will be closed and with C(pU-1)
   continuity. Each column in Q indicates the points
   for a closed curve in the u direction. First and
   last point have to be equal.

   \param  Q  a matrix of 3D points (without repeated points)
   \param pU  the degree of interpolation in the U direction
   \param pV  the degree of interpolation in the V direction

   \author Alejandro Frangi
   \date 30 June, 1998
   */

  void NurbsSurface::globalInterpClosedU(const GeneralMatrix<fmatvec::Vec3 >& Q, int DegU, int DegV) {
    VecV vk, uk;
    resize(Q.rows() + DegU, Q.cols(), DegU, DegV); // the U direction is closed interpolation, thus need to plus additional DegU size.

    MatVx4 Pts(Q.cols(), NONINIT);
    NurbsCurve R;

    for (int v = 0; v < Pts.rows(); v++)
      Pts(v, 3) = 1.0;   // set the weight to be 1 for every point

    int i, j;
    for (i = 0; i < Q.rows(); i++) {  // Q does not contains the repeated points in U direction
      for (j = 0; j < Q.cols(); j++)
        Pts.set(RangeV(j, j), RangeV(0, 2), trans(Q(i, j)));

      if (i == 0){
        surfMeshParamsClosedU(Q, uk, vk, DegU);  // numU = unrepeated points in U direction.  Q: numU * numV;   uk: (numU + degU) * 1; vk = numv * 1
        knotAveraging(vk, DegV, V);
        R.globalInterpH(Pts, vk, V, DegV, true);
      }else
        R.update(Pts);

      for (j = 0; j < Q.cols(); j++)
        P(i, j) = R.ctrlPnts(j);
    }

//    cout << "fmatvec_suface: control points after interpolating in V direction " << P;
    Pts.resize(Q.rows());

    for (j = 0; j < Q.cols(); j++) {
      for (i = 0; i < Q.rows(); i++)
        Pts.set(RangeV(i, i), RangeV(0, 3), trans(P(i, j)));

//      cout << "fmatvec_suface: control points for interpolating in U direction " << Pts << endl;
      if (j == 0){
        knotAveragingClosed(uk, DegU, U);  // uk = (numU + degU)*1; U = (numU + degU + degU +1)*1
        R.globalInterpClosedH(Pts, uk, U, DegU, true); // Pts does not contains the repeated points, however, uk and U contains the knot information for the repeated nodes.
      }else
        R.update(Pts);

      for (i = 0; i < Q.rows() + degU; i++) // + degU is to get the repeated control points, P has to contains those repeated points for the operator()
        P(i, j) = R.ctrlPnts(i);
    }
//   cout << "fmatvec_suface: final control points for U and V direction" << P << endl;
  }

  /*!
   \brief Generates a closed surface using global interpolation.

   Generates a NURBS surface using global interpolation. In the u
   direction the curve will be closed and with C(pU-1)
   continuity. Each column in Q indicates the points
   for a closed curve in the u direction. First and
   last point have to be equal.

   \param  Q  a matrix of 3D points (without repeated points)
   \param uk  the vector of lagrange parameters of the interpolated point in U direction
   \param vk  the vector of lagrange parameters of the interpolated point in V direction
   \param pU  the degree of interpolation in the U direction
   \param pV  the degree of interpolation in the V direction

   */

  void NurbsSurface::globalInterpClosedU(const GeneralMatrix<fmatvec::Vec3 >& Q, const VecV& uk, const VecV& vk, int DegU, int DegV) {
    if (uk.size() != Q.rows() + DegU)
      throw runtime_error("(NurbsCurve::globalInterpH: In the U direction, the length of lagrange parameters vectors is not equal to the number of the given interpolated points plus degU!)");
    if (vk.size() != Q.cols())
      throw runtime_error("(NurbsCurve::globalInterpH: In the V direction, the length of lagrange parameters vectors is not equal to the number of the given interpolated points!)");

    resize(Q.rows() + DegU, Q.cols(), DegU, DegV); // the U direction is closed interpolation, thus need to plus additional DegU size.

    MatVx4 Pts(Q.cols(), NONINIT);
    NurbsCurve R;

    for (int v = 0; v < Pts.rows(); v++)
      Pts(v, 3) = 1.0;   // set the weight to be 1 for every point

    int i, j;
    for (i = 0; i < Q.rows(); i++) {  // Q does not contains the repeated points in U direction
      for (j = 0; j < Q.cols(); j++)
        Pts.set(RangeV(j, j), RangeV(0, 2), trans(Q(i, j)));

      if (i == 0){
//        surfMeshParamsClosedU(Q, uk, vk, DegU);  // numU = unrepeated points in U direction.  Q: numU * numV;   uk: (numU + degU) * 1; vk = numv * 1
        knotAveraging(vk, DegV, V);
        R.globalInterpH(Pts, vk, V, DegV, true);
      }else
        R.update(Pts);

      for (j = 0; j < Q.cols(); j++)
        P(i, j) = R.ctrlPnts(j);
    }

//    cout << "fmatvec_suface: control points after interpolating in V direction " << P;
    Pts.resize(Q.rows());

    for (j = 0; j < Q.cols(); j++) {
      for (i = 0; i < Q.rows(); i++)
        Pts.set(RangeV(i, i), RangeV(0, 3), trans(P(i, j)));

//      cout << "fmatvec_suface: control points for interpolating in U direction " << Pts << endl;
      if (j == 0){
        knotAveragingClosed(uk, DegU, U);  // uk = (numU + degU)*1; U = (numU + degU + degU +1)*1
        R.globalInterpClosedH(Pts, uk, U, DegU, true); // Pts does not contains the repeated points, however, uk and U contains the knot information for the repeated nodes.
      }else
        R.update(Pts);

      for (i = 0; i < Q.rows() + degU; i++) // + degU is to get the repeated control points, P has to contains those repeated points for the operator()
        P(i, j) = R.ctrlPnts(i);
    }
//   cout << "fmatvec_suface: final control points for U and V direction" << P << endl;
  }
  
  /*!
   \brief Finds the multiplicity of a knot in the U knot

   \param r the knot to observe

   \return the multiplicity of the knot
   \warning \a r must be a valid U knot index

   \author  Philippe Lavoie
   \date 24 January, 1997
   */

//int NurbsSurface::findMultU(int r) const {
//  int s=1 ;
//  for(int i=r;i>degU+1;i--)
//    if(U[i]<=U[i-1])
//      s++ ;
//    else
//      return s ;
//  return s ;
//}
  /*!
   \brief finds the multiplicity of a knot in the V knot

   \param r  the knot to observe

   \return the multiplicity of the V knot

   \warning \a r must be a valid knot index

   \author  Philippe Lavoie
   \date 24 January, 1997
   */
//
//int NurbsSurface::findMultV(int r) const {
//  int s=1 ;
//  for(int i=r;i>degV+1;i--)
//    if(V[i]<=V[i-1])
//      s++ ;
//    else
//      return s ;
//  return s ;
//}
//
  /*!
   \brief finds the span in the U and V direction

   Finds the span in the U and V direction. The spanU is the index
   \a k for which the parameter \a u is valid in the \a [u_k,u_{k+1}]
   range. The spanV is the index \a k for which the parameter \a v is
   valid in the \a [v_k,v_{k+1}] range.

   \param u  find the U span for this parametric value
   \param v  find the V span for this parametric value
   \param spanU  the U span
   \param spanV  the V span

   \author Philippe Lavoie
   \date 24 January, 1997
   */

  void NurbsSurface::findSpan(double u, double v, int spanU, int spanV) const {
    //spanU = findSpanU(u);
    //spanV = findSpanV(v);
  }

  /*!
   \brief finds the span in the U direction

   Finds the span in the U direction. The span is the index
   \a k for which the parameter \a u is valid in the \a [u_k,u_{k+1}]
   range.

   \param u --> find the span for this parametric value

   \return the span for \a u

   \author    Philippe Lavoie
   \date 24 January, 1997

   \modified 20 January, 1999 (Alejandro Frangi)
   */

  int NurbsSurface::findSpanU(double u) const {
    if (u >= U(P.rows()))
      return P.rows() - 1;
    if (u <= U(degU))
      return degU;

    //AF
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
   \brief finds the span in the V direction

   Finds the span in the V direction. The span is the index
   \a k for which the parameter \a v is valid in the \a [v_k,v_{k+1}]
   range.

   \param v  find the span for this parametric value

   \return the span for \a v

   \author Philippe Lavoie
   \date 24 January, 1997
   \modified 20 January, 1999 (Alejandro Frangi)

   */

  int NurbsSurface::findSpanV(double v) const {
    if (v >= V(P.cols()))
      return P.cols() - 1;
    if (v <= V(degV))
      return degV;

    //AF
    int low = 0;
    int high = P.cols() + 1;
    int mid = (low + high) / 2;

    while (v < V(mid) || v >= V(mid + 1)) {
      if (v < V(mid))
        high = mid;
      else
        low = mid;
      mid = (low + high) / 2;
    }
    return mid;
  }

  /*!
   \brief Find the non-zero basis functions in the U and V direction

   \param   u  the u parametric value
   \param   v  the v parametric value
   \param spanU  the span of u
   \param spanV  the span of v
   \param    Nu  the vector containing the U non-zero basis functions
   \param    Nv  the vector containing the V non-zero basis functions

   \author Philippe Lavoie
   \date 24 January, 1997
   */

//void NurbsSurface::basisFuns(T u, T v, int spanU, int spanV, Vector<T>& Nu, Vector<T> &Nv) const{
//  basisFuns(u,spanU,Nu) ;
//  basisFuns(v,spanV,Nv) ;
//}
///*!
//  \brief Finds the non-zero basis function in the U direction
//
//  \param   u  the u parametric value
//  \param span  the span of u
//  \param    N  the vector containing the basis functions
//
//  \author    Philippe Lavoie
//  \date 24 January, 1997
//*/
//template <class T, int nD>
//void NurbsSurface<T,nD>::basisFunsU(T u, int span, Vector<T>& N) const {
//  //Vector<T> left(degU+1), right(degU+1) ;
//  T* left = (T*) alloca((degU+1)*sizeof(T)) ;
//  T* right = (T*) alloca((degU+1)*sizeof(T)) ;
//  T temp,saved ;
//
//
//  N.resize(degU+1) ;
//
//  N(0] = 1.0 ;
//  for(int j=1; j<= degU ; j++){
//    left[j] = u-U[span+1-j] ;
//    right[j] = U[span+j]-u ;
//    saved = 0.0 ;
//    for(int r=0 ; r<j; r++){
//      temp = N[r]/(right[r+1]+left[j-r]) ;
//      N[r] = saved+right[r+1]*temp ;
//      saved = left[j-r]*temp ;
//    }
//    N[j] = saved ;
//  }
//
//}
//
///*!
//  \brief Finds the non-zero basis function in the V direction
//
//  \param    v  the v parametric value
//  \param span  the span of v
//  \param    N  the vector containing the basis functions
//
//  \author Philippe Lavoie
//  \date 24 January, 1997
//*/
//template <class T, int nD>
//void NurbsSurface<T,nD>::basisFunsV(T v, int span, Vector<T>& N) const {
//  //Vector<T> left(degV+1), right(degV+1) ;
//  T* left = (T*) alloca((degV+1)*sizeof(T)) ;
//  T* right = (T*) alloca((degV+1)*sizeof(T)) ;
//  T temp,saved ;
//
//
//  N.resize(degV+1) ;
//
//  N[0] = 1.0 ;
//  for(int j=1; j<= degV ; j++){
//    left[j] = v-V[span+1-j] ;
//    right[j] = V[span+j]-v ;
//    saved = 0.0 ;
//    for(int r=0 ; r<j; r++){
//      temp = N[r]/(right[r+1]+left[j-r]) ;
//      N[r] = saved+right[r+1]*temp ;
//      saved = left[j-r]*temp ;
//    }
//    N[j] = saved ;
//  }
//
//}
  /*!
   \brief Computes the point and the derivatives of degree
   \a d and below at \a (u,v)

   Computes the matrix of derivatives at \a u,v .
   The value of skl(k,l) represents the
   derivative of the surface \a S(u,v) with respect to
   \a u, \a k times and to \a v, \a l times.

   \param   u  the u parametric value
   \param   v  the v parametric value
   \param   d  the derivative is computed up to and including to  this value
   \param skl  the matrix containing the derivatives

   \author    Philippe Lavoie
   \date 24 January, 1997
   */

  void NurbsSurface::deriveAt(double u, double v, int d, GeneralMatrix<Vec3> &skl) const {
    int k, l;
    GeneralMatrix<Vec4> ders;
    Vec3 pv, pv2;

    skl.resize(d + 1, d + 1);

    deriveAtH(u, v, d, ders);

    MatV Bin(d + 1, d + 1, NONINIT);  // TODO check whether need init.
    binomialCoef(Bin);
    int i, j;

    for (k = 0; k <= d; ++k) {
      for (l = 0; l <= d - k; ++l) {
        pv(0) = ders(k, l)(0);
        pv(1) = ders(k, l)(1);
        pv(2) = ders(k, l)(2);
        for (j = 1; j <= l; j++)
          pv = pv - Bin(l, j) * ders(0, j)(3) * skl(k, l - j);
        for (i = 1; i <= k; i++) {
          pv = pv - Bin(k, i) * ders(i, 0)(3) * skl(k - i, l);
          pv2 = Vec3("[0; 0; 0]");
          for (j = 1; j <= l; j++)
            pv2 = pv2 + Bin(l, j) * ders(i, j)(3) * skl(k - i, l - j);
          pv = pv - Bin(k, i) * pv2;
        }
        skl(k, l) = pv / ders(0, 0)(3);
      }
    }
  }

  /*!
   \brief computes the point and the derivatives of degree
   \a d and below at \a (u,v)

   Computes the matrix of derivatives at \a u,v .
   The value of skl(k,l) represents the
   derivative of the surface \a S(u,v) with respect to
   \a u \a k times and to $v$ $l$ times.

   \param  u  the u parametric value
   \param  v  the v parametric value
   \param  d  the derivative is computed up to and including to this value
   \param skl  the matrix containing the derivatives

   \author    Philippe Lavoie
   \date 24 January, 1997
   */
  void NurbsSurface::deriveAtH(double u, double v, int d, GeneralMatrix<Vec4> &skl) const {
    int k, l, du, dv;
    skl.resize(d + 1, d + 1);

    du = (d <= degU) ? d : degU;
    for (k = degU + 1; k <= d; ++k)
      for (l = 0; l <= d - k; ++l)
        skl(k, l) = Vec4("[0.0; 0.0; 0.0; 0.0]");
    dv = (d <= degV) ? d : degV;
    for (l = degV + 1; l <= d; ++l)
      for (k = 0; k <= d - l; ++k)
        skl(k, l) = Vec4("[0.0; 0.0; 0.0; 0.0]");
    int uspan = findSpanU(u);
    int vspan = findSpanV(v);
//  Matrix<T> Nu,Nv ;
    Mat Nu, Nv;
    dersBasisFuns(du, u, uspan, degU, U, Nu);
    dersBasisFuns(dv, v, vspan, degV, V, Nv);

//  Vector< HPoint_nD<T,nD> > temp(degV+1) ;
    Mat temp(4, degV + 1, NONINIT);
    int dd, r, s;
    for (k = 0; k <= du; ++k) {
      for (s = 0; s <= degV; ++s) {
        temp.col(s) = Vec4("[0.0; 0.0; 0.0; 0.0]");
        for (r = 0; r <= degU; ++r)
          temp.col(s) = temp.col(s) + Nu(k, r) * P(uspan - degU + r, vspan - degV + s);
      }
      dd = ((d - k) <= dv) ? (d - k) : dv;
      for (l = 0; l <= dd; ++l) {
        skl(k, l) = Vec4("[0.0; 0.0; 0.0; 0.0]");
        for (s = 0; s <= degV; ++s)
          skl(k, l) = skl(k, l) + Nv(l, s) * temp.col(s);
      }
    }
  }

  /*!
   \brief Computes the normal of the surface at \a (u,v)

   \param  u  the u parametric value
   \param  v  the v parametric value

   \return the normal at \a (u,v) .

   \author    Philippe Lavoie
   \date 24 January, 1997
   */

  Vec3 NurbsSurface::normal(double u, double v) const {
    GeneralMatrix<Vec3> ders;

    deriveAt(u, v, 1, ders);

    return crossProduct(ders(1, 0), ders(0, 1));
  }

  /*!
   \brief Returns the point on the surface at \a u,v

   Returns the point on the surface at \a u,v

   \param u  the u parametric value
   \param v  the v parametric value

   \return The homogenous point at \a u,v

   \author    Philippe Lavoie
   \date 24 January, 1997
   */

  fmatvec::HPoint<3> NurbsSurface::operator()(double u, double v) const {
    int uspan = findSpanU(u);
    int vspan = findSpanV(v);
    VecV Nu, Nv;
//  Vector< HPoint_nD > temp(degV+1)  ;
    Mat temp(4, degV + 1, NONINIT);

    Nu.resize(degU + 1);
    Nv.resize(degV + 1);
//  basisFuns(u,v,uspan,vspan,Nu,Nv) ;
    basisFuns(u, uspan, degU, U, Nu);
    basisFuns(v, vspan, degV, V, Nv);

    int l;
    for (l = 0; l <= degV; l++) {
      temp.col(l) = Vec4("[0.0; 0.0; 0.0; 0.0]");
      for (int k = 0; k <= degU; k++) {
        temp.col(l) = temp.col(l) + Nu(k) * P(uspan - degU + k, vspan - degV + l);
      }
    }

//  HPoint_nD sp(0,0,0,0) ;
    Vec4 sp("[0.0; 0.0; 0.0; 0.0]");

    for (l = 0; l <= degV; l++) {
      sp = sp + Nv(l) * temp.col(l);
    }
    return sp;
  }

  //! Projects the point in the normal space
  fmatvec::Point<3> NurbsSurface::pointAt(double u, double v) const
    { return project(operator()(u,v)) ; }

//inline
//int max3(int a,int b, int c){
//  int m = a ;
//  if(m <b)
//    m = b ;
//  if(m <c)
//    m = c ;
//  return m ;
//}

  /*!
   \relates NurbsSurface
   \brief Computes the parameters for global surface interpolation

   Computes the parameters for global surface interpolation.
   For more information, see A9.3 on p377 on the NURBS book.

   \param Q   the matrix of 3D points
   \param uk  the knot coefficients in the U direction
   \param vl  the knot coefficients in the V direction

   \return 0 if an error occurs, 1 otherwise

   \warning

   \author Philippe Lavoie
   \date 24 January, 1997
   */

  int surfMeshParams(const GeneralMatrix<fmatvec::Vec3 >& Q, VecV& uk, VecV& vl) {
    int n, m, k, l, num;
    double d, total;
    //Vector<T> cds(Q.rows()) ;
    auto* cds = new double[ (Q.rows() >= Q.cols()) ? Q.rows():Q.cols()]; // alloca might not have enough space

    n = Q.rows();
    m = Q.cols();
    uk.resize(n);
    vl.resize(m);
    num = m;

    // Compute the uk
//  uk.reset(0) ; // done in the resize

    for (l = 0; l < m; l++) {
      total = 0.0;
      for (k = 1; k < n; k++) {
        cds[k] = nrm2(Q(k, l) - Q(k - 1, l));
        total += cds[k];
      }
      if (total <= 0.0)
        num--;
      else {
        d = 0.0;
        for (k = 1; k < n; k++) {
          d += cds[k];
          uk(k) += d / total;
        }
      }
    }

    if (num == 0) {
      delete[] cds;
      return 0;
    }
    for (k = 1; k < n - 1; k++)
      uk(k) /= num;
    uk(n - 1) = 1.0;

    // Compute the vl
//  vl.reset(0) ;

    //cds.resize(m) ; // this line removed since the maximum is allocated at the beginning

    num = n;

    for (k = 0; k < n; k++) {
      total = 0.0;
      for (l = 1; l < m; l++) {
        cds[l] = nrm2(Q(k, l) - Q(k, l - 1));
        total += cds[l];
      }
      if (total <= 0.0)
        num--;
      else {
        d = 0.0;
        for (l = 1; l < m; l++) {
          d += cds[l];
          vl(l) += d / total;
        }
      }
    }

    delete[] cds;

    if (num == 0)
      return 0;
    for (l = 1; l < m - 1; l++)
      vl(l) /= num;
    vl(m - 1) = 1.0;

    return 1;
  }

  int surfMeshParamsClosedU(const GeneralMatrix<fmatvec::Vec3 >& Q, VecV& uk, VecV& vl, int degU) {

    int n, m, k, l, num;
    double d, total;
    vector<double> cds(Q.rows() + degU);

//    n = Q.rows();
    n = Q.rows() + degU; // in plb Q: (number of unrepeated points in U direction + 3) * number of point in V direction
                         // but in fmatvec: Q :  number of unrepeated points in U direction * number of point in V direction
    m = Q.cols();
    uk.resize(n);
    vl.resize(m);
    num = m;

    // Compute the uk
//  uk.reset(0) ;

    for (l = 0; l < m; l++) {
      total = 0.0;
      for (k = 1; k < n - degU; k++) {
        cds.at(k) = nrm2(Q(k, l) - Q(k - 1, l));
        total += cds.at(k);
      }
      // the last element in the closed direction
      cds.at(k) = nrm2(Q(0, l) - Q(k - 1, l));
      total += cds.at(k);

      //TODO: in uk there is two times the one...
      for (k = n - degU + 1; k < n; k++)
        cds.at(k) = nrm2(Q(k - Q.rows(), l) - Q(k -Q.rows() - 1, l));

      if (total <= 0.0)
        num--;
      else {
        d = 0.0;
        for (k = 1; k < n; k++) {
          d += cds.at(k);
          uk(k) += d / total;
        }
      }
    }
    if (num == 0)
      return 0;
    for (k = 1; k < n; k++)
      uk(k) /= num;

    // Compute the vl
//  vl.reset(0) ;
    cds.resize(m);

    num = n - degU;

    for (k = 0; k < n -degU; k++) {
      total = 0.0;
      for (l = 1; l < m; l++) {
        cds.at(l) = nrm2(Q(k, l) - Q(k, l - 1));
        total += cds.at(l);
      }
      if (total <= 0.0)
        num--;
      else {
        d = 0.0;
        for (l = 1; l < m; l++) {
          d += cds.at(l);
          vl(l) += d / total;
        }
      }
    }
    if (num == 0)
      return 0;
    for (l = 1; l < m - 1; l++)
      vl(l) /= num;
    vl(m - 1) = 1.0;

    return 1;
  }

/*! 
 \brief Write a surface to a file

 \param filename  the filename to write to

 \return 1 on success, 0 on failure

 \author Philippe Lavoie
 \date 24 January, 1997
 */
//
//int NurbsSurface::write(const char* filename) const {
//  ofstream fout(filename) ;
//  if(!fout)
//    return 0 ;
//  return write(fout);
//}
///*!
//  \brief Writes a post-script file representing the curve
//
//  \param filename  the file to write the postscript file to
//  \param nu  the number of lines in the U direction
//  \param nv  the number of lines in the V direction
//  \param camera  where the camera is
//  \param lookAt  where the camera is looking at
//  \param plane  where is the projection plane from the camera
//  \param cp  a flag indicating if the control points should be
//       drawn, 0 = no and 1 = yes
//  \param magFact  a magnification factor, the 2D point of the control
//                  points will be magnified by this value. The size is
//      measured in postscript points. If the magFact is
//      set to a value smaller or equal to 0, than the
//      program will try to guess a magnification factor
//      such that the curve is large enough to fill the
//      page.
//  \param dash  the size of the dash in postscript points . A size
//  smaller or equal to 0 indicates that
//  the line joining the control points is plain.
//
//  \return 0 if an error occurs, 1 otherwise
//
//  \warning If the weights of the curve are not all at 1, the result might
//               not be representative of the true NURBS curve.
//
//  \author Philippe Lavoie
//  \date 7 October 1998
//*/
//
//int NurbsSurface::writePS(const char* filename, int nu, int nv, const Point_nD& camera, const Point_nD& lookAt, int cp,T magFact,T dash) const {
//  NurbsCurveArray Ca ;
//  if(nu<=0 || nv<=0)
//    return 0 ;
//
//
//  // We need to find the rotation matrix to have z axis along nv
//  Point_nD np  = lookAt-camera ;
//  np = np.unitLength() ;
//  np *= -1 ;
//
//  T rx = atan2(np.z(),np.y()) ;
//  T ry = atan2(np.z(),np.x()) ;
//
//  MatrixRT<T> Tx(rx,ry,0,camera.x(),camera.y(),camera.z()) ;
//  //MatrixRT Sc ; Sc.scale(1,1,T(norm(lookAt-camera))/plane) ;
//  //MatrixRT Tg(Sc*Tx) ;
//
//  Ca.resize(nu+nv+2) ;
//  int i ;
//  for(i=0;i<=nu;++i){
//    T u = U[0]+T(i)*(U[U.n()-1]-U[0])/T(nu) ;
//    isoCurveU(u , Ca[i]) ;
//    Ca[i].transform(Tx) ;
//  }
//  for(;i<=nv+nu+1;++i){
//    T v = V[0]+T(i-nu-1)*(V[V.n()-1]-V[0])/T(nv) ;
//    isoCurveV(v , Ca[i]) ;
//    Ca[i].transform(Tx) ;
//  }
//
//
//  return Ca.writePS(filename,cp,magFact,dash) ;
//}
//
///*!
//  \brief writes a post-script file representing the curve
//
//  Writes the curve in the postscript format to a file, it also
//  draws the points defined in \a points with their associated
//  vectors if \a vector is used.
//
//  \param filename  the file to write the postscript file to
//  \param nu  the number of lines in the U direction
//  \param nv  the number of lines in the V direction
//  \param camera  where the camera is
//  \param lookAt  where the camera is looking at
//  \param plane  where is the projection plane from the camera
//  \param points  draws these additional points as empty circles
//  \param vectors  specify a vector associated with the points
//       (this can be an empty vector)
//  \param cp  a flag indicating if the control points should be
//             drawn, 0 = no and 1 = yes
//  \param magFact  a magnification factor, the 2D point of the control
//      points will be magnified by this value. The size
//      is measured in postscript points. If the magFact
//      is set to a value smaller or equal to 0, than the
//      program will try to guess a magnification factor
//      such that the curve is large enough to fill the
//      page.
//  \param dash  the size of the dash in postscript points . A size
//  smaller or equal to 0 indicates that
//  the line joining the control points is plain.
//
//  \return 0 if an error occurs, 1 otherwise
//
//  \warning If the weights of the curve are not all at 1, the result might
//               not be representative of the true NURBS curve. If vectors is
//         used, then it must be of the same size as points. If a vector
//         element is (0,0,0) it will not be drawn.
//
//  \author Philippe Lavoie
//  \date 7 October 1998
//*/
//
//int NurbsSurface::writePSp(const char*, int nu, int nv, const Point_nD& camera, const Point_nD& lookAt, const Vector< Point_nD >&,const Vector< Point_nD >&, int cp,T magFact,T dash) const {
//  cerr << "Not implemented. Not sure what is different between this and writePS\n";
//  return 0;
//}
//
///*!
//  \brief Sends the NURBS Surface to ostream for display
//
//  \return the ostream
//
//  \author Philippe Lavoie
//  \date 9 November 1998
//*/
//
//ostream& NurbsSurface::print(ostream& o) const {
//  o << "Degree: " << degU << ' ' << degV << endl;
//  o << "U : " << U << endl;
//  o << "V: " << V << endl ;
//  o << "matrix size: " << P.rows() << ' ' << P.cols() << endl ;
//  o << P << endl;
//  return o;
//}
}// end namespace

