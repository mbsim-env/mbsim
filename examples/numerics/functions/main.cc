#include <mbsim/utils/stopwatch.h>
#include <mbsim/utils/eps.h>
#include <mbsim/functions/two_dimensional_tabular_function.h>
#include <mbsim/functions/multivariate_linear_function.h>
#include <mbsim/functions/multivariate_quadratic_function.h>

#include <fstream>
#include <type_traits>

using namespace fmatvec;
using namespace std;
using namespace MBSim;

#define CHECK(expr1, expr2) ret+=check(1e-12, expr1, expr2, #expr1, #expr2, __FILE__, __LINE__)
#define CHECKEPS(expr1, expr2, eps) ret+=check(eps, expr1, expr2, #expr1, #expr2, __FILE__, __LINE__)

template<class T>
int check(double eps, const T& expr1, const T& expr2, const string &expr1Str, const string &expr2Str, const string &file, int line) {
  bool error = false;
  if constexpr (is_same_v<T, double>) // scalar double -> check by eps
    error = abs(expr1-expr2)>eps;
  else if constexpr (is_same_v<typename T::value_type, double>) // vector double -> check by eps
    error = nrm2(expr1-expr2)>eps;
  else // rest -> check equal
    error = expr1!=expr2;
  if(!error)
    return 0;
  cout<<file<<":"<<line<<": error: "<<expr1Str<<" != "<<expr2Str<<" ["<<expr1<<" != "<<expr2<<"]"<<endl;
  return 1;
}

/*!
 * \brief reads the values from a file with a predefined structure
 *
 * The first line are the x-values of the x-vector (dim n) in typical matlab-format: [x1;x2;x3;...;xn]
 * The second line are the y-values of the y-vector (dim m) in typical matlab-format: [y1;y2;y3;...;ym]
 * The third line are the z-values of the xy-values (dim nxm) in typical matlab-format: [z11, z12, z13, ... z1n; z21, z22, z23, ..., z2n; ...; zm1, zm2, zm3, ... zmn]
 */
TwoDimensionalTabularFunction<double(double,double)> read2DTabFun(string filename) {
  TwoDimensionalTabularFunction<double(double,double)> func;

  ifstream ifs(filename.c_str());
  string line;

  getline(ifs, line);
  Vec x(line.c_str());
  func.setx(x);
  getline(ifs, line);
  Vec y(line.c_str());
  func.sety(y);
  getline(ifs, line);
  Mat Z(line.c_str());
  func.setz(Z);

  return func;
}

int main(int argc, char* argv[]) {
  int ret = 0;

  /*************************************************************************************************/

//  StopWatch sw;

  TwoDimensionalTabularFunction<double(double,double)> func = read2DTabFun("func.dat");

  double xi, yj;
  int dimx = 21, dimy = 101;
  Mat xyNew(dimx, dimy);
  Vec x(dimx);
  Vec y(dimy);

  for (int i = 0; i < dimx; i++) {
    xi = func.getxMin() + i * (func.getxMax() - func.getxMin()) / (dimx - 1);
    x(i) = xi;
    for (int j = 0; j < dimy; j++) {
      yj = func.getyMin() + j * (func.getyMax() - func.getyMin()) / (dimy - 1);
      y(j) = yj;
      xyNew(i, j) = func(xi, yj);
    }
  }

  cout << x << endl;
  cout << y << endl;
  cout << xyNew << endl;

  /*************************************************************************************************/

  {
    MultivariateLinearFunction<double(VecV)> func;
    func.seta0(4.3);
    VecV a1({2.4,5.3,3.7});
    func.seta1(a1);
    VecV x({1.5, 8.3, 3.6});
    CHECK(func(x), 65.21);
    double d=1e-8;
    VecV d0({d,0,0});
    VecV d1({0,d,0});
    VecV d2({0,0,d});
    CHECKEPS(func.parDer(x), RowVecV({(func(x+d0)-func(x))/d,(func(x+d1)-func(x))/d,(func(x+d2)-func(x))/d}), 1e-5);
    CHECK(func.parDerDirDer(VecV({2,0,0}), x), RowVecV({0,0,0}));
    CHECK(func.parDerDirDer(VecV({0,2,0}), x), RowVecV({0,0,0}));
    CHECK(func.parDerDirDer(VecV({0,0,2}), x), RowVecV({0,0,0}));
  }

  /*************************************************************************************************/

  {
    MultivariateQuadraticFunction<double(VecV)> func;
    func.seta0(4.3);
    VecV a1({2.4,5.3,3.7});
    func.seta1(a1);
    SqrMatV a2({{2.4,5.3,3.7},{4.2,7.2,1.1},{8.2,7.4,5.5}});
    func.seta2(a2);
    VecV x({1.5, 8.3, 3.6});
    CHECK(func(x), 1074.413);
    double d=1e-8;
    VecV d0({d,0,0});
    VecV d1({0,d,0});
    VecV d2({0,0,d});
    CHECKEPS(func.parDer(x), RowVecV({(func(x+d0)-func(x))/d,(func(x+d1)-func(x))/d,(func(x+d2)-func(x))/d}), 1e-4);
    CHECKEPS(func.parDerDirDer(VecV({2,0,0}), x), (func.parDer(x+d0)-func.parDer(x))/d *2+
                                                  (func.parDer(x+d1)-func.parDer(x))/d *0+
                                                  (func.parDer(x+d2)-func.parDer(x))/d *0 , 1e-4);
    CHECKEPS(func.parDerDirDer(VecV({0,2,0}), x), (func.parDer(x+d0)-func.parDer(x))/d *0+
                                                  (func.parDer(x+d1)-func.parDer(x))/d *2+
                                                  (func.parDer(x+d2)-func.parDer(x))/d *0 , 1e-4);
    CHECKEPS(func.parDerDirDer(VecV({0,0,2}), x), (func.parDer(x+d0)-func.parDer(x))/d *0+
                                                  (func.parDer(x+d1)-func.parDer(x))/d *0+
                                                  (func.parDer(x+d2)-func.parDer(x))/d *2 , 1e-4);
  }

  /*************************************************************************************************/

  return ret;
}
