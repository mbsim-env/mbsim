/* Copyright (C) 2004-2014 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef NURBS_CURVE_FMATVEC_H_
#define NURBS_CURVE_FMATVEC_H_

#include <mbsim/numerics/nurbs/nurbs_defs.h>

namespace fmatvec {
  typedef Matrix<General, Fixed<4>, Var, double> Mat4xV;

  typedef Matrix<General, Var, Fixed<4>, double> MatVx4;
}

namespace MBSim {

  /*!
   * \brief class that copies the nurbs++-library using the fmatvec as a basis-math-library
   *
   * \todo: could/should be derived by function<Vec3(double)>
   */
  class NurbsCurve {
    public:
      enum Method {
        equallySpaced = 0,
        chordLength,
      };

      /*!
       * \brief standard constructor
       */
      NurbsCurve() = default;
//        NurbsCurve(Matrix<T> inverse) {
//          Inverse = inverse;
//          Inverse_setted = 1;
//        } //changed
//        NurbsCurve(const NurbsCurve<T, N>& nurb);
//        NurbsCurve(const Vector<fmatvec::HPoint<3>  >& P1, const std::vector<double> &U1, int deg = 3);
//        NurbsCurve(const Vector<fmatvec::Point<3>  >& P1, const std::vector<double> &W, const std::vector<double> &U1, int deg = 3);
      virtual ~NurbsCurve() = default;

      // Reference to internal data
      //! a reference to the degree of the curve
      int degree() const { return deg; }
      //! a reference to the vector of control points
      const fmatvec::MatVx4 & ctrlPnts() const { return P; }
      //! a reference to one of the control points
      const fmatvec::Vec4 ctrlPnts(int i) const { return trans(P.row(i)); }
      //! a reference to the vector of knots
      const fmatvec::VecV& knot() const { return U; }
      //! the i-th knot
      double knot(int i) const { return U(i); }
      //! a reference to the parametric points
      const fmatvec::VecV getuVec() const { return u; }

      void setKnot(const fmatvec::VecV& U_) { U = U_; }
      void setCtrlPnts(const fmatvec::MatVx4& P_) { P = P_; }
      void setDegree(int deg_) { deg = deg_; }
      void setu(const fmatvec::VecV& u_) { u = u_; }

      void resize(int n, int Deg);

      // basic functions

//        virtual void reset(const Vector<fmatvec::HPoint<3>  >& P1, const std::vector<double> &U1, int deg);
//        virtual NurbsCurve& operator=(const NurbsCurve<T, N>&);
//
//        // Evaluattion functions
      virtual fmatvec::HPoint<3> operator()(double u) const;
      //! calls operator()
      fmatvec::HPoint<3> hpointAt(double u) const
      {
        return operator()(u);
      }
      fmatvec::HPoint<3> hpointAt(double u, int span) const;
      fmatvec::Point<3> pointAt(double u) const;
//        //! a function interface to operator()
//        friend fmatvec::HPoint<3>  C(double u, const NurbsCurve<T, N>& nurb) {
//          return nurb(u);
//        }
//        //! returns the curvePoint in 3D
//        friend fmatvec::Point<3>  Cp(double u, const NurbsCurve<T, N>& nurb) {
//          return project(nurb(u));
//        }

      // derivative functions
      void deriveAtH(double u, int d, fmatvec::MatVx4 & ders) const;
//        void deriveAt(double u, int, Vector<fmatvec::Point<3>  >&) const;
//        void deriveAtH(double u, int, int, Vector<fmatvec::HPoint<3>  >&) const;
//        void deriveAt(double u, int, int, Vector<fmatvec::Point<3>  >&) const;
      fmatvec::Point<3> derive3D(double u, int d) const;
      fmatvec::HPoint<3> derive(double u, int d) const;
      fmatvec::Point<3> normal(double u, const fmatvec::Point<3> & v) const;

      fmatvec::HPoint<3> firstD(double u) const;
      fmatvec::HPoint<3> firstD(double u, int span) const;
      fmatvec::Point<3> firstDn(double u) const;
      fmatvec::Point<3> firstDn(double u, int span) const;

//        // Basis functions
//        T basisFun(T u, int i, int p = -1) const;

//        void dersBasisFuns(int n, T u, int span, Matrix<T>& M) const;
//
//        // Knot functions
//        T minKnot() const //! the minimal value for the knot vector
//        {
//          return U[0];
//        }
//        //! the maximal value for the knot vector
//        T maxKnot() const
//        {
//          return U[U.n() - 1];
//        }
//        void findMultSpan(T u, int& r, int& s) const;
//        int findMult(int r) const;
//        int findKnot(T u) const;
//        T getRemovalBnd(int r, int s) const;
//
//        void removeKnot(int r, int s, int num);
//        void removeKnotsBound(const std::vector<double>& ub, std::vector<double>& ek, T E);
//        int knotInsertion(T u, int r, NurbsCurve<T, N>& nc);
//        void refineKnotVector(const std::vector<double>& X);
//        void refineKnotVectorClosed(const std::vector<double>& X);
//        void mergeKnotVector(const std::vector<double> &Um);
//
//        void clamp();
//        void unclamp();
//
//        // Curve fitting functions
//        int leastSquares(const Vector<fmatvec::Point<3>  >& Q, int degC, int n);
//        int leastSquares(const Vector<fmatvec::Point<3>  >& Q, int degC, int n, const std::vector<double>& ub);
//        int leastSquaresH(const Vector<fmatvec::HPoint<3>  >& Q, int degC, int n, const std::vector<double>& ub);
//        int leastSquares(const Vector<fmatvec::Point<3>  >& Q, int degC, int n, const std::vector<double>& ub, const std::vector<double>& knot);
//        int leastSquaresH(const Vector<fmatvec::HPoint<3>  >& Q, int degC, int n, const std::vector<double>& ub, const std::vector<double>& knot);
//
//        int leastSquaresClosed(const Vector<fmatvec::Point<3>  >& Q, int degC, int n);
//        int leastSquaresClosed(const Vector<fmatvec::Point<3>  >& Q, int degC, int n, const std::vector<double>& ub);
//        int leastSquaresClosedH(const Vector<fmatvec::HPoint<3>  >& Q, int degC, int n, const std::vector<double>& ub);
//        int leastSquaresClosed(const Vector<fmatvec::Point<3>  >& Q, int degC, int n, const std::vector<double>& ub, const std::vector<double>& knot);
//        int leastSquaresClosedH(const Vector<fmatvec::HPoint<3>  >& Q, int degC, int n, const std::vector<double>& ub, const std::vector<double>& knot);
//
//        void globalApproxErrBnd(Vector<fmatvec::Point<3>  >& Q, int deg, T E);
//        void globalApproxErrBnd(Vector<fmatvec::Point<3>  >& Q, std::vector<double>& ub, int deg, T E);
//        void globalApproxErrBnd2(Vector<fmatvec::Point<3>  >& Q, int degC, T E);
//        void globalApproxErrBnd3(Vector<fmatvec::Point<3>  >& Q, int degC, T E);
//        void globalApproxErrBnd3(Vector<fmatvec::Point<3>  >& Q, const std::vector<double> &ub, int degC, T E);
//
//        void globalInterp(const Vector<fmatvec::Point<3>  >& Q, int d);
      /*!
       * \brief do global interpolation for given interpolation-points list and knots with the given degree
       */
      void globalInterp(const std::vector<fmatvec::Point<3> >& Q, const std::vector<double>& uk, int d, bool updateLater = false);
      void globalInterp(const std::vector<fmatvec::Point<3> >& Q, double uMin, double uMax, int d, bool updateLater = false);
      void globalInterp(const fmatvec::MatVx3& Q, double uMin, double uMax, int d, bool updateLater = false);

      void globalInterpH(const fmatvec::MatVx4& Qw, int d, Method method=chordLength);
      void globalInterpH(const fmatvec::MatVx4& Qw, const fmatvec::VecV& ub, const fmatvec::VecV& Uc, int d, bool updateLater = false);
      /*!
       * \brief closed interpolation of the given (not yet wrapped) points at the given knot vector "ub" in a degree of "d"
       */
      void globalInterpClosed(const fmatvec::MatVx3 & Q, double uMin, double uMax, int d, bool updateLater = false);

      /*!
       * \brief update the control points with the same matrix as before
       */
      void update(const fmatvec::MatVx3& Q);
      void update(const fmatvec::MatVx4& Qw);

      void globalInterpClosedH(const fmatvec::MatVx4& Qw, int d, Method method=chordLength);
      void globalInterpClosedH(const fmatvec::MatVx4& Qw, const fmatvec::VecV& ub, const fmatvec::VecV& Uc, int d, bool updateLater = false);
//
//        void globalInterpD(const Vector<fmatvec::Point<3>  >& Q, const Vector<fmatvec::Point<3>  >& D, int d, int unitD, T a = 1.0);
//
//        Matrix<T> computeInverse(const std::vector<double> &v, const std::vector<double> &V, const int p);
//        Matrix<T> computeInverseClosed(const std::vector<double> &v, const std::vector<double> &V, const int p);
//
//        void projectTo(const fmatvec::Point<3> & p, T guess, T& u, fmatvec::Point<3> & r, T e1 = 0.001, T e2 = 0.001, int maxTry = 100) const;
//
//        T length(T eps = 0.001, int n = 100) const;
//        T lengthIn(T us, T ue, T eps = 0.001, int n = 100) const;
//        T lengthF(T) const;
//        T lengthF(T, int) const;
//
//        // Generate type of curve
//        void makeCircle(const fmatvec::Point<3> & O, const fmatvec::Point<3> & X, const fmatvec::Point<3> & Y, T r, double as, double ae);
//        void makeCircle(const fmatvec::Point<3> & O, T r, double as, double ae);
//        void makeCircle(const fmatvec::Point<3> & O, T r);
//        void makeLine(const fmatvec::Point<3> & P0, const fmatvec::Point<3> & P1, int d);
//        virtual void degreeElevate(int t);
//
//  #ifndef HAVE_ISO_FRIEND_DECL
//        friend void generateCompatibleCurves(NurbsCurveArray<T, N> &ca);
//  #else
//        friend void generateCompatibleCurves <>(NurbsCurveArray<T,N> &ca);
//  #endif
//
//        void decompose(NurbsCurveArray<T, N>& c) const;
//        void decomposeClosed(NurbsCurveArray<T, N>& c) const;
//
//        int splitAt(T u, NurbsCurve<T, N>& cl, NurbsCurve<T, N>& cu) const;
//        int mergeOf(const NurbsCurve<T, N>& cl, const NurbsCurve<T, N> &cu);
//
//        // Modifies the NURBS curve
//        void transform(const MatrixRT<T>& A);
      void modCP(int i, const fmatvec::HPoint<3> & a) {
        P.set(i, a.T());
      } // To manipulate the value of the control point $P[i]$
//        void modCPby(int i, const fmatvec::HPoint<3> & a) {
//          P[i] += a;
//        } // To manipulate the value of the control point $P[i]$
//        virtual void modKnot(const std::vector<double>& knotU) {
//          if (knotU.n() - deg_ - 1 == P.n())
//            U = knotU;
//        }  // to change the values of the knot vector only if the size is compatible with P.n

//        int movePoint(T u, const fmatvec::Point<3> & delta);
//        int movePoint(T u, const BasicArray<fmatvec::Point<3>  >& delta);
//        int movePoint(const BasicArray<T>& ur, const BasicArray<fmatvec::Point<3>  >& D);
//        int movePoint(const BasicArray<T>& ur, const BasicArray<fmatvec::Point<3>  >& D, const BasicArray_INT& Dr, const BasicArray_INT& Dk);
//        int movePoint(const BasicArray<T>& ur, const BasicArray<fmatvec::Point<3>  >& D, const BasicArray_INT& Dr, const BasicArray_INT& Dk, const BasicArray_INT& fixCP);
//
//        void setTangent(T u, const fmatvec::Point<3> & T0);
//        void setTangentAtEnd(const fmatvec::Point<3> & T0, const fmatvec::Point<3> & T1);
//
//        // I/O functions
//        int read(const char*);
//        int write(const char*) const;
//        virtual int read(ifstream &fin);
//        int write(ofstream &fout) const;
//        int writePS(const char*, int cp = 0, T magFact = T(-1), T dash = T(5), bool bOpen = true) const;
//        int writePSp(const char*, const Vector<fmatvec::Point<3>  >&, const Vector<fmatvec::Point<3>  >&, int cp = 0, T magFact = 0.0, T dash = 5.0, bool bOpen = true) const;
//
//        int writeVRML(ostream &fout, T radius, int K, const Color& color, int Nu, int Nv, T u_s, T u_e) const;
//        int writeVRML(const char* filename, T radius, int K, const Color& color, int Nu, int Nv, T u_s, T u_e) const;
//        int writeVRML(const char* filename, T radius = 1, int K = 5, const Color& color = whiteColor, int Nu = 20, int Nv = 20) const {
//          return writeVRML(filename, radius, K, color, Nu, Nv, U[0], U[U.n() - 1]);
//        } // writes the curve to a VRML file
//        int writeVRML(ostream& fout, T radius = 1, int K = 5, const Color& color = whiteColor, int Nu = 20, int Nv = 20) const {
//          return writeVRML(fout, radius, K, color, Nu, Nv, U[0], U[U.n() - 1]);
//        } // writes the curve to a VRML file
//
//        int writeVRML97(const char* filename, T radius, int K, const Color& color, int Nu, int Nv, T u_s, T u_e) const;
//        int writeVRML97(ostream &fout, T radius, int K, const Color& color, int Nu, int Nv, T u_s, T u_e) const;
//        int writeVRML97(const char* filename, T radius = 1, int K = 5, const Color& color = whiteColor, int Nu = 20, int Nv = 20) const {
//          return writeVRML97(filename, radius, K, color, Nu, Nv, U[0], U[U.n() - 1]);
//        } // writes the curve to a VRML file
//        int writeVRML97(ostream& fout, T radius = 1, int K = 5, const Color& color = whiteColor, int Nu = 20, int Nv = 20) const {
//          return writeVRML97(fout, radius, K, color, Nu, Nv, U[0], U[U.n() - 1]);
//        } // writes the curve to a VRML file
//
//        int writeDisplayLINE(const char* filename, int iNu, const Color& color = blueColor, T fA = 1) const;
//        int writeDisplayLINE(const char* filename, const Color& color, int iNu, T u_s, T u_e) const;
//        void drawImg(Image_UBYTE& Img, unsigned char color = 255, T step = 0.01);
//        void drawImg(Image_Color& Img, const Color& color, T step = 0.01);
//        void drawAaImg(Image_Color& Img, const Color& color, int precision = 3, int alpha = 1);
//        void drawAaImg(Image_Color& Img, const Color& color, const NurbsCurve<T, 3>& profile, int precision = 3, int alpha = 1);
//        NurbsSurface<T, 3> drawAaImg(Image_Color& Img, const Color& color, const NurbsCurve<T, 3>& profile, const NurbsCurve<T, 3> &scaling, int precision = 3, int alpha = 1);
//
//        BasicList<fmatvec::Point<3>  > tesselate(T tolerance, BasicList<T> *uk) const;

      int findSpan(double u) const;

    protected:
      fmatvec::MatVx4 P; // the vector of control points
      fmatvec::SqrMat inverse; //Inverse of Ansatz-functions in case of only update later (different points, same knot-Vecs and same degree)
      fmatvec::VecV u;  // the parameteric points
      fmatvec::VecV U;  // the knot vector
      int deg{0};  // the degree of the NURBS curve

      void knotAveraging(const std::vector<double>& uk, int deg);
      double chordLengthParam(const fmatvec::MatVx3& Q, fmatvec::VecV& ub);
      double chordLengthParamH(const fmatvec::MatVx4& Q, fmatvec::VecV& ub);
      double chordLengthParamClosedH(const fmatvec::MatVx4& Q, fmatvec::VecV& ub, int d);
      void updateUVecs(double uMin, double uMax);

      void knotAveragingClosed(const std::vector<double>& uk, int deg);
      void updateUVecsClosed(double uMin, double uMax);

//      Matrix<T> Inverse; //changed
//      int Inverse_setted; //changed
  };

  //TODO: put those functions into mother nurbs class (maybe) to make them "func(...) const"
  void knotAveraging(const fmatvec::VecV& uk, int deg, fmatvec::VecV& U);
  void knotAveragingClosed(const fmatvec::VecV& uk, int deg, fmatvec::VecV& U);
  void basisFuns(double u, int span, int deg, const fmatvec::VecV & U, fmatvec::VecV& funs);
  void dersBasisFuns(int n, double u, int span, int deg, const fmatvec::VecV & U, fmatvec::Mat& ders);
  void binomialCoef(fmatvec::MatV& Bin);

}
#endif
