#include <nurbs.h>

int main(){
  using namespace PLib ; 
#ifdef WITH_IMAGE_MAGICK

  // read input data
  IM_Image img ; 
  
  img.read("tnApprox.pnm");

  Vector_Point3Df data(img.rows());
  
  int i,j,k ;
  k=0  ;
  for(i=0;i<img.rows();++i){
    for(j=0;j<img.cols();++j){
      if(img(i,j)==0){
	data[k++] = Point3Df(j,i,0) ;
      }
    }
  }

  data.resize(k) ;

  PlNurbsCurvef curveA,curveB,curveC ;

  cerr << "The input curve is shown in tnApprox.pnm\n\n" ;

  cerr << "Approximating the curve with an error of 0.5\n" ;
  cerr << "The original number of points is " << k << " and \n" ;
  curveA.globalApproxErrBnd(data,3,0.5) ;
  cerr << "It resulted in " << curveA.ctrlPnts().n() << " control points (algo 1)." ;
  curveB.globalApproxErrBnd2(data,3,0.5) ;
  cerr << "\nIt resulted in " << curveB.ctrlPnts().n() << " control points (algo 2)." ;
  curveC.globalApproxErrBnd3(data,3,0.5) ;
  cerr << "\nIt resulted in " << curveC.ctrlPnts().n() << " control points (algo 3)." ;
  cerr << "\nThe result can be seen in tnApproxA.pnm\n" ;


  cerr << "\nApproximating the curve with an error of 1.5\n" ;
  cerr << "The original number of points is " << k << " and \n" ;
  curveA.globalApproxErrBnd(data,3,1.5) ;
  cerr << "It resulted in " << curveA.ctrlPnts().n() << " control points (algo 1)." ;
  curveB.globalApproxErrBnd2(data,3,1.5) ;
  cerr << "\nIt resulted in " << curveB.ctrlPnts().n() << " control points (algo 2)." ;
  curveC.globalApproxErrBnd3(data,3,1.5) ;
  cerr << "\nIt resulted in " << curveC.ctrlPnts().n() << " control points (algo 3)." ;
  cerr << "\n\nThe 1.5 approximation can be seen in tnApproxA.pnm (algo 1)" ;
  cerr << "\nThe 1.5 approximation can be seen in tnApproxB.pnm (algo 2)" ;
  cerr << "\nThe 1.5 approximation can be seen in tnApproxC.pnm (algo 3)\n\n" ;

  img.reset(255) ;
  curveA.drawImg(img,0,0.001) ;
  img.write("tnApproxA.pnm");
  img.reset(255) ;
  curveB.drawImg(img,0,0.001) ;
  img.write("tnApproxB.pnm");
  img.reset(255) ;
  curveC.drawImg(img,0,0.001) ;
  img.write("tnApproxC.pnm");


  cerr << "\nApproximating the curve with an error of 2.5\n" ;
  cerr << "The original number of points is " << k << " and \n" ;
  curveA.globalApproxErrBnd(data,3,2.5) ;
  cerr << "It resulted in " << curveA.ctrlPnts().n() << " control points (algo 1)." ;
  curveB.globalApproxErrBnd2(data,3,2.5) ;
  cerr << "\nIt resulted in " << curveB.ctrlPnts().n() << " control points (algo 2)." ;
  curveC.globalApproxErrBnd3(data,3,2.5) ;
  cerr << "\nIt resulted in " << curveC.ctrlPnts().n() << " control points (algo 3)." ;
  cerr << "\n\nThe 2.5 approximation can be seen in tnApproxAb.pnm (algo 1)" ;
  cerr << "\nThe 2.5 approximation can be seen in tnApproxBb.pnm (algo 2)" ;
  cerr << "\nThe 2.5 approximation can be seen in tnApproxCb.pnm (algo 3)\n\n" ;

  img.reset(255) ;
  curveA.drawImg(img,0,0.001) ;
  img.write("tnApproxAb.pnm");
  img.reset(255) ;
  curveB.drawImg(img,0,0.001) ;
  img.write("tnApproxBb.pnm");
  img.reset(255) ;
  curveC.drawImg(img,0,0.001) ;
  img.write("tnApproxCb.pnm");
 
  cerr << "To compare the results, the same curve with interpolation and"
    " least square fitting (with 80% of the points) can be viewed"
    " in tnApproxI.pnm and tnApproxLS.pnm\n" ; 

  curveA.globalInterp(data,3) ;
  img.reset(255);
  curveA.drawImg(img,0,0.0001) ;
  img.write("tnApproxI.pnm") ; 

  curveA.leastSquares(data,3,double(data.n())*0.8) ;
  img.reset(255);
  curveA.drawImg(img,0,0.0001) ;
  img.write("tnApproxLS.pnm") ; 

#else
  
  cerr << "The approximation test requires Image Magick support\n" ;

#endif

  return 0 ;
}
