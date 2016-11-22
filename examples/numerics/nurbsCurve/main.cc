#include <mbsim/utils/stopwatch.h>
#include <mbsim/utils/eps.h>
#include <mbsim/numerics/nurbs/nurbs_curve.h>

#include <nurbs++/nurbs.h>
using namespace PLib;

#include <iostream>

using namespace fmatvec;
using namespace std;
using namespace MBSim;

int main(int argc, char* argv[]) {




  cout << setprecision(3) << fixed << scientific;

  StopWatch sw;

  int degU = 3;
  int Elements = 10;
  int uPoints = 100;

  double radius = 1;

  MatVx3 points(Elements, INIT, 0.);
  MatVx3 pointsClosed(Elements, INIT, 0.);
  std::vector<double> uvec;
  std::vector<double> uvecClosed;

  GeneralMatrix<Vec3> GM(3,4);
  GM(0,0) = Vec3("[1; 2; 3]");
  GM(2,2) = Vec3("[3; 3; 3]");
  cout << GM(2,2);

  for (int i = 0; i < Elements; i++) {
    double phi = i * M_PI / Elements;
    points(i, 0) = radius * cos(phi);
    points(i, 1) = radius * sin(phi);

    uvec.push_back(((double) i) / (Elements - 1));

    double phiClosed = i * 2 * M_PI / Elements;
    pointsClosed(i, 0) = radius * cos(phiClosed);
    pointsClosed(i, 1) = radius * sin(phiClosed);

    uvecClosed.push_back(((double) i) / (Elements - 1));
  }

  if(1) {
    cout << "Original Points: " << endl;
    ostringstream xStream;
    xStream << " x = [";
    ostringstream yStream;
    yStream << " y = [";
    for (int i = 0; i < Elements; i++) {
      xStream << pointsClosed(i, 0) << ",";
      yStream << pointsClosed(i, 1) << ",";
    }
    xStream << "];";
    yStream << "];";

    cout << xStream.str() << endl << yStream.str() << endl;
  }

  /*NURBS++-Library*/

  PlNurbsCurved plcurve;
  PLib::Vector<double> pluVec(Elements);
  for(int i = 0; i < Elements; i++)
  pluVec[i] = uvec[i];

  PLib::Vector<Point3Dd> Nodelist(Elements);
  for (int i = 0; i < Elements; i++) {

    Nodelist[i] = Point3Dd(points(i,0), points(i,1), points(i,2));
  }

  sw.start();
  plcurve.globalInterp(Nodelist, pluVec, degU);
  cout << "PLIB: Open Interpolation time = " << sw.stop() << endl;

  PlNurbsCurved plcurveClosed;
  PLib::Vector<double> pluVecClosed(Elements);
  pluVecClosed[0] = 0.;
  for(int i = 1; i < Elements; i++)
  pluVecClosed[i] = pluVecClosed[i-1]+ 1. / (Elements - 1);

  PLib::Vector<double> plUVecClosed(Elements + degU + degU + 1);
  plUVecClosed[0] = - double(degU) / (Elements - 1);
  for(int i = 1; i < Elements + +degU + degU + 1; i++)
  plUVecClosed[i] = plUVecClosed[i-1] + 1. / (Elements - 1);

  PLib::Vector<HPoint3Dd> NodelistClosed(Elements+degU);
  for (int i = 0; i < Elements; i++) {
    NodelistClosed[i] = HPoint3Dd(pointsClosed(i,0), pointsClosed(i,1), pointsClosed(i,2), 1.);
  }
  for (int i = 0; i < degU; i++) {
    NodelistClosed[Elements+i] = NodelistClosed[i];
  }

  sw.start();
  plcurveClosed.globalInterpClosedH(NodelistClosed, pluVecClosed, plUVecClosed, degU);
  cout << "PLIB: Closed Interpolation time = " << sw.stop() << endl;

  /*MBSim-Library*/
  MBSim::NurbsCurve mbsimcurve;

  sw.start();
  mbsimcurve.globalInterp(points, uvec[0], uvec[uvec.size()-1], degU);
  cout << "MBSim: Open Interpolation time (once with LU-decomposition) = " << sw.stop() << endl;

  sw.start();
  mbsimcurve.globalInterp(pointsClosed, uvec[0], uvec[uvec.size()-1], degU, true);
  cout << "MBSim: Open Interpolation time (once with inverse-decomposition) = " << sw.stop() << endl;

  sw.start();
  mbsimcurve.update(points);
  cout << "MBSim: Open Interpolation time (using inverse) = " << sw.stop() << endl;

  MBSim::NurbsCurve mbsimcurveClosed;
  sw.start();
  mbsimcurveClosed.globalInterpClosed(pointsClosed, uvecClosed[0], uvecClosed[uvecClosed.size()-1], degU);
  cout << "MBSim: Closed Interpolation time (once with LU-decomposition) = " << sw.stop() << endl;

  sw.start();
  mbsimcurveClosed.globalInterpClosed(points, uvecClosed[0], uvecClosed[uvecClosed.size()-1], degU, true);
  cout << "MBSim: Closed Interpolation time (once with inverse-decomposition) = " << sw.stop() << endl;

  sw.start();
  mbsimcurveClosed.update(pointsClosed);
  cout << "MBSim: Closed Interpolation time (using inverse) = " << sw.stop() << endl;

  if(1) {
    cout << "Original Closed Points: " << endl;
    stringstream x;
    x << "xorig = [";
    stringstream y;
    y << "yorig = [";
    for (int i = 0; i < uPoints; i++) {
      double phiClosed = i * 2 * M_PI / uPoints;
      x << radius * cos(phiClosed) << ", ";
      y << radius * sin(phiClosed) << ", ";
    }
    x << "];" << endl;
    y << "];" << endl;
    cout << x.str() << y.str();
  }

  if(1) {
    cout << "PLIB-Points Closed Points: " << endl;
    stringstream x;
    x << "xpl = [";
    stringstream y;
    y << "ypl = [";
    for (int i = 0; i < uPoints; i++) {
      double u = double(i) * (uvecClosed[uvecClosed.size() - 1] - uvecClosed[0]) / uPoints;
      x << plcurveClosed.pointAt(u).x()<< ", ";
      y << plcurveClosed.pointAt(u).y() << ", ";
    }
    x << "];" << endl;
    y << "];" << endl;
    cout << x.str() << y.str();
  }

  if(1) {
    cout << "MBSim-Points Closed Points: " << endl;
    stringstream x;
    x << "xmb = [";
    stringstream y;
    y << "ymb = [";
    for (double u = uvecClosed[0]; u <= uvecClosed[uvecClosed.size() - 1]; u += 1e-2) {
      x << mbsimcurveClosed.pointAt(u)(0) << ", ";
      y << mbsimcurveClosed.pointAt(u)(1) << ", ";
    }
    x << "];" << endl;
    y << "];" << endl;
    cout << x.str() << y.str();
  }

  if(1) {
    cout << "Control Points Open:" << endl;
    cout << "PLIB " << plcurve.ctrlPnts() << endl;
    cout << "MBSIM " << mbsimcurve.ctrlPnts() << endl;
  }

  if(1) {
    cout << "Control Points Closed:" << endl;
    cout << "PLIB " << plcurveClosed.ctrlPnts() << endl;
    cout << "MBSIM " << mbsimcurveClosed.ctrlPnts() << endl;
  }


  if(1) {
    cout << "Error in Control-Points Open" << endl;
    for(int i = 0; i < Elements; i++) {
      for(int j = 0; j < 4; j++) {
        cout << fabs(plcurve.ctrlPnts(i).data[j] - mbsimcurve.ctrlPnts()(i,j)) << ", ";
      }
      cout << endl;
    }
  }

  if(1) {
    cout << "Error in Control-Points Closed" << endl;
    for(int i = 0; i < Elements; i++) {
      for(int j = 0; j < 4; j++) {
        cout << fabs(plcurveClosed.ctrlPnts(i).data[j] - mbsimcurveClosed.ctrlPnts()(i,j)) << ", ";
      }
      cout << endl;
    }
  }

  if(1) {
    for(int degDer = 0; degDer < 3; degDer++) {
      cout << "Comparing derivatives Open with degree = " << degDer << endl;
      for (double u = uvec[0]; u < uvec[uvec.size() - 1]; u += 1e-2) {
        cout << fabs(mbsimcurve.derive(u, degDer)(0) - plcurve.derive(u, degDer).x()) << " "
        << fabs(mbsimcurve.derive(u, degDer)(1) - plcurve.derive(u, degDer).y()) << " "
        << fabs(mbsimcurve.derive(u, degDer)(2) - plcurve.derive(u, degDer).z()) << endl;
      }
      cout << endl;
    }
  }

  if(1) {
  cout << "Normals-Error: " << endl;

  for (double u = uvec[0]; u < uvec[uvec.size() - 1]; u += 1e-2) {
    cout << fabs(mbsimcurve.normal(u, Vec3("[0;0;1]"))(0) - plcurve.normal(u,Point3Dd(0,0,1)).x()) << " "
    << fabs(mbsimcurve.normal(u, Vec3("[0;0;1]"))(1) - plcurve.normal(u,Point3Dd(0,0,1)).y()) << " "
    << fabs(mbsimcurve.normal(u, Vec3("[0;0;1]"))(2) - plcurve.normal(u,Point3Dd(0,0,1)).z()) << endl;
  }
  }

  return 0;

}
