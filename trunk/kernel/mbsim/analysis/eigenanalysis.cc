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

#include <config.h>
#include "eigenanalysis.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/eps.h"
#include "fmatvec/linear_algebra_complex.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include <iostream>

using namespace std;
using namespace fmatvec;

// TODO: remove the following two functions and provide an uniform concept to
// cast between complex and double
Vector<Ref, complex<double> > toComplex(const Vector<Ref, double> &x) {
  Vector<Ref, complex<double> > y(x.size(),NONINIT);
  for(int i=0; i<x.size(); i++)
    y(i) = complex<double>(x(i),0);
  return y;
}
const Vector<Ref, double> fromComplex(const Vector<Ref, complex<double> > &x) {
  Vector<Ref, double> y(x.size(),NONINIT);
  for(int i=0; i<x.size(); i++)
    y(i) = x(i).real();
  return y;
}

namespace MBSim {

  DynamicSystemSolver * Eigenanalysis::system = 0;

  Eigenanalysis::Residuum::Residuum(DynamicSystemSolver *sys_, double t_) : sys(sys_), t(t_) {}

  Vec Eigenanalysis::Residuum::operator()(const Vec &z) {
      Vec res;
      res = sys->zdot(z,t);
      return res;
    } 

  void Eigenanalysis::analyse(DynamicSystemSolver& system_) {
    system = &system_;

    double t=tStart;
    int zSize=system->getzSize();
    Vec z(zSize);
    if(z0.size())
      z = z0;
    else
      system->initz(z);          

    Residuum f(system,t);
    MultiDimNewtonMethod newton(&f);
    newton.setLinearAlgebra(1);
    Vec zEq = newton.solve(z);
    if(newton.getInfo() != 0)
      throw MBSimError("ERROR in Eigenanalysis: computation of equilibrium position failed!");

    double delta = epsroot();
    SqrMat A(zEq.size());
    Vec zd, zdOld;
    zdOld = system->zdot(zEq,t);
    for (int i = 0; i < z.size(); i++) {
      double ztmp = zEq(i);
      zEq(i) += delta;
      zd = system->zdot(zEq,t);
      A.col(i) = (zd - zdOld) / delta;
      zEq(i) = ztmp;
    }
    SquareMatrix<Ref, complex<double> > V;
    Vector<Ref, complex<double> > w;
    eigvec(A,V,w);
    ofstream os("Eigenanalysis.mat");
    os << "# name: lambda" << endl;
    os << "# type: complex matrix" << endl;
    os << "# rows: " << w.size() << endl;
    os << "# columns: " << 1 << endl;
    for (int i=0; i < w.size(); ++i)
      os << setw(26) << w.e(i) << endl;
    os << endl;
    os << "# name: V" << endl;
    os << "# type: complex matrix" << endl;
    os << "# rows: " << V.rows() << endl;
    os << "# columns: " << V.cols() << endl;
    for (int i=0; i < V.rows(); ++i) {
      for (int j=0; j < V.cols(); ++j) 
        os << setw(26) << V.e(i,j);
      os << endl;
    }
    os.close();

    if(deltaz0.size()==0)
      deltaz0.resize(zEq.size());
    Vector<Ref, complex<double> > c = slvLU(V,toComplex(deltaz0));
    Vector<Ref, complex<double> > deltaz(zEq.size(),NONINIT);

    for(double t=tStart; t<tEnd+dtPlot; t+=dtPlot) {
      deltaz.init(0);
      for(int i=0; i<w.size(); i++)
        deltaz += c(i)*V.col(i)*exp(w(i)*t); 
      z = zEq + fromComplex(deltaz);
      system->plot(z,t);
    }
  }

}

