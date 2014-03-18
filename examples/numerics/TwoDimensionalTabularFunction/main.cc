#include <mbsim/utils/stopwatch.h>
#include <mbsim/utils/eps.h>
#include <mbsim/functions/tabular_functions.h>

#include <fstream>

using namespace fmatvec;
using namespace std;
using namespace MBSim;

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
  func.setXValues(x);
  getline(ifs, line);
  Vec y(line.c_str());
  func.setYValues(y);
  getline(ifs, line);
  Mat Z(line.c_str());
  func.setXYMat(Z);

  return func;
}

int main(int argc, char* argv[]) {

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

  return 0;

}
