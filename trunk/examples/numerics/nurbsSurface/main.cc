#include <nurbs++/nurbsS.h>
#include <nurbs++/nurbsSub.h>
#include <math.h>
#include <mbsim/numerics/nurbs/nurbs_surface.h>

using namespace fmatvec;
using namespace std;
using namespace PLib ;

int main(){
  /******   plib ********/
  Matrix_Point3Df Pts(4,5) ;
  int i,j ;
  for(i=0;i<Pts.rows();++i){
    for(j=0;j<Pts.cols();++j){
      Pts(i,j) = Point3Df(i,j,cos(j));
      cerr << Pts(i,j) << "";
    }
    cerr << endl;
  }
  cerr << Pts << endl;

  /******   fmatvec ********/
  GeneralMatrix<fmatvec::Vec3 > Q(4,5);
  for(i=0;i<Q.rows();++i){
    for(j=0;j<Q.cols();++j){
      Vec3 temp; temp(0) = i; temp(1) = j; temp(2) = j;
      Q(i,j) = temp;
    }
  }
 cerr  << "fmatvec: Q :"<< Q  << endl;  // TODO:: debug this <<

  PlNurbsSurfacef surf;
  surf.globalInterp(Pts,3,3);
  double u = 0.2;
  double v = 0.2;
  cerr << "/********PLB ***********/" << endl;
  cerr << "u, v = " << u << " " << v << endl;
  cerr << "Point at (u,v) with weight= " << surf(u,v) << endl ;
  cerr << "\t Point at (u,v) ==  " << surf.pointAt(u,v) << endl ;
  cerr << "\t normal = " << surf.normal(u,v) << endl ;
  Matrix_Point3Df pskl;
  surf.deriveAt(u, v, 1, pskl);
  cerr << "\t derive = " << pskl << endl ;
  surf.writePS("tnurbsS1.ps",5,5,Point3Df(-5,-5,5),Point3Df(0,0,1),1) ;


  MBSim::NurbsSurface fsurf;
  fsurf.globalInterp(Q, 3, 3);
  cerr << "/******** fmatvec ***********/" << endl;
  cerr << "Point at (0.5,0.5) = " << fsurf(0.5,0.5) << endl ;
  cerr << "\t=  " << fsurf.pointAt(0.5,0.5) << endl ;
  cerr << "\t normal = " << fsurf.normal(0.5,0.5) << endl ;
  fmatvec::GeneralMatrix<Vec3> fskl;
  fsurf.deriveAt(0.5, 0.5, 1, fskl);
  for (int i = 0; i < fskl.rows(); i++)
    for (int j = 0; j < fskl.cols(); j++)
      cerr << "\t derive: i=" << i << ", j= " << j  << fskl(i,j) << endl ;


/***********************************************************************************************************************/
  /******   plib ********/
  for(i=0;i<Pts.rows();++i){
    for(j=0;j<Pts.cols();++j){
      Pts(i,j) = Point3Df(i*20,j*20,(1+cos(i+j))*20) ;
      cerr << Pts(i,j) << "";
    }
    cerr << endl;
  }

  /******   fmatvec ********/
  for(i=0;i<Q.rows();++i){
    for(j=0;j<Q.cols();++j){
      Vec3 temp; temp(0) = i * 20; temp(1) = j * 20; temp(2) = (1+cos(i+j))*20;
      Q(i,j) = temp;
//      cerr << "i = " << i << "j= " << j <<trans(Q(i,j)) << endl;
    }
  }

  surf.globalInterp(Pts,3,3) ;
  fsurf.globalInterp(Q, 3, 3);
  cerr << "/********PLB ***********/" << endl;
  cerr << "Point at (0.5,0.5) = " << surf(0.5,0.5) << endl ;
  cerr << "\t=  " << surf.pointAt(0.5,0.5) << endl ;
  cerr << "\t normal = " << surf.normal(0.5,0.5) << endl ;

  cerr << "/******** fmatvec ***********/" << endl;
  cerr << "Point at (0.5,0.5) = " << fsurf(0.5,0.5) << endl ;
  cerr << "\t=  " << fsurf.pointAt(0.5,0.5) << endl ;
  cerr << "\t normal = " << fsurf.normal(0.5,0.5) << endl ;


  surf.write("tnurbsS.ns") ;
  surf.read("tnurbsS.ns");
  surf.writeVRML("tnurbsS.wrl") ;
  surf.writePS("tnurbsS.ps",5,5,Point3Df(10,10,10),Point3Df(0,0,0),1) ;

  // for surface closed interpolatin
  surf.makeTorus(Point3Df(0,0,0),4,1) ;
  surf.writeVRML("torus.wrl");
  surf.writePS("tnurbs4.ps",5,5,Point3Df(10,10,10),Point3Df(0,0,0),1) ;

  /******   fmatvec ********/
  int eleU = 10;
  int eleV = 5;
  double radius = 5;
  int degU = 3;
  int degV = 3;
 GeneralMatrix<fmatvec::Vec3 > Q2(eleU, eleV);
  for(i=0;i<Q2.rows();++i){
    double phi = i * 2 * M_PI / eleU;
    Vec3 temp;
    temp(0) = radius * cos(phi);
    temp(1) = radius * sin(phi);
    for(j=0;j<Q2.cols();++j){
      temp(2) = j;
      Q2(i,j) = temp;
//      cerr << "phi" << phi << endl;
//      cerr << "i = " << i << "j= " << j <<Q2(i,j) << endl;
    }
  }
 cerr  << "fmatvec: Q2 :"<< Q2  << endl;

  Matrix_Point3Df Pts2(eleU+degU,eleV);
  for (i=0; i<Pts2.rows()-degU;++i){
    for (j=0; j<Pts2.cols();++j){
      Pts2(i,j) = Point3Df(Q2(i,j)(0),Q2(i,j)(1),Q2(i,j)(2));
//      cerr << Pts2(i,j) << "";
    }
//    cerr << endl;
  }
  for (i = 0; i < degU; i++)
    for (j = 0; j < eleV; j++)
    Pts2(eleU+i, j) = Pts2(i, j);

  cerr << Pts2 << endl;

  surf.globalInterpClosedU(Pts2, degU, degV);
  fsurf.globalInterpClosedU(Q2, degU, degV);

  // output
  double k, l;

  k = 1.2; l = 0.8;
  cerr << "PLB: k = " <<  k << " l = " << l << " "<< surf.pointAt(k,l) << endl;
  cerr << "fmatvec      :" << trans(fsurf.pointAt(k,l)) << endl << endl;

  for (k = 0; k <= 1; k+=0.1){
    for (l = 0; l <= 1; l+=0.1){
      cerr << "PLB: k = " <<  k << " l = " << l << " "<< surf.pointAt(k,l) << endl;
      cerr << "fmatvec      :" << trans(fsurf.pointAt(k,l)) << endl << endl;
      cerr << "PLB_normal: k = " <<  k << " l = " << l << " "<< surf.normal(k,l) << endl;
      cerr << "fmatvec_normal      :" << trans(fsurf.normal(k,l)) << endl << endl;
      surf.deriveAt(k,l,1,pskl);
      fsurf.deriveAt(k,l,1,fskl);
      cerr << "PLB_derive: k = " <<  k << " l = " << l << " "<< pskl << endl;
      cerr << "fmatvec_derive      :" << fskl << endl << endl;

    }
  }

  surf.writePS("tnurbs_closed.ps",5,5,Point3Df(10,10,10),Point3Df(0,0,0),1) ;

  return 0 ;
}
