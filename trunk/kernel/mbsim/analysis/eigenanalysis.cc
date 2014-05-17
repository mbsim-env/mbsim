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
#include <iostream>

using namespace std;
using namespace fmatvec;

namespace MBSim {

  DynamicSystemSolver * Eigenanalysis::system = 0;

  void Eigenanalysis::analyse(DynamicSystemSolver& system_) {
    system = &system_;

    double t=0;
    int zSize=system->getzSize();
    Vec z(zSize);
    if(z0.size())
      z = z0;
    else
      system->initz(z);          
    Vec zOld = z.copy();
    double delta = epsroot();
    SqrMat A(z.size());
    Vec zd, zdOld;
    zdOld = system->zdot(z,t);
    for (int i = 0; i < z.size(); i++) {
      double ztmp = z(i);
      z(i) += delta;
      zd = system->zdot(z,t);
      A.col(i) = (zd - zdOld) / delta;
      z(i) = ztmp;
    }
    SqrMat B;
    Vector<Ref, complex<double> > w;
    eigvec(A,B,w);
    ofstream os("Eigenanalysis.mat");
    os << "# name: V" << endl;
    os << "# type: matrix" << endl;
    os << "# rows: " << B.rows() << endl;
    os << "# columns: " << B.cols() << endl;
    for (int i=0; i < B.rows(); ++i) {
      for (int j=0; j < B.cols(); ++j) 
        os << setw(14) << B.e(i,j);

      os << endl;
    }
    os << endl;
    os << "# name: lambda" << endl;
    os << "# type: complex matrix" << endl;
    os << "# rows: " << w.size() << endl;
    os << "# columns: " << 1 << endl;
    for (int i=0; i < w.size(); ++i)
      os << setw(14) << w.e(i) << endl;
    os.close();
  }

}

