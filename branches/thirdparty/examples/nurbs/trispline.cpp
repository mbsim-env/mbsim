#include <tri_spline.h>

int main(){
  using namespace PLib ;
  TriangularBSpline<float,3> ts(4) ;

  ts.b(0,0,4) = Point_nD<float,3>(0,0,0) ; 
  ts.b(1,0,3) = Point_nD<float,3>(1,0,0) ; 
  ts.b(2,0,2) = Point_nD<float,3>(2,0,1) ; 
  ts.b(3,0,1) = Point_nD<float,3>(3,0,2) ; 
  ts.b(4,0,0) = Point_nD<float,3>(4,0,0) ; 

  ts.b(0,1,3) = Point_nD<float,3>(0.5,1,1) ;
  ts.b(1,1,2) = Point_nD<float,3>(1.5,1,1) ;
  ts.b(2,1,1) = Point_nD<float,3>(2.5,1,0) ;
  ts.b(3,1,0) = Point_nD<float,3>(3.5,1,1) ;

  ts.b(0,2,2) = Point_nD<float,3>(1,2,0) ;
  ts.b(1,2,1) = Point_nD<float,3>(2,2,2) ;
  ts.b(2,2,0) = Point_nD<float,3>(3,2,-1) ;

  ts.b(0,3,1) = Point_nD<float,3>(1.5,3,1) ;
  ts.b(1,3,0) = Point_nD<float,3>(2.5,3,3) ;

  ts.b(0,4,0) = Point_nD<float,3>(2,4,0) ;   


  cout << "Point at (0.5,0.5) = " << ts(0.5,0.5) << endl ;
  cout << "Point at (0.333,0.333) = " << ts(0.333,0.333) << endl ;

  RTriangularBSpline<float,3> rts(4) ;

  rts.b(0,0,4) = HPoint_nD<float,3>(0,0,0,1) ;   
  rts.b(1,0,3) = HPoint_nD<float,3>(1,0,0,1) ;   
  rts.b(2,0,2) = HPoint_nD<float,3>(2,0,1,1) ;   
  rts.b(3,0,1) = HPoint_nD<float,3>(3,0,2,1) ;   
  rts.b(4,0,0) = HPoint_nD<float,3>(4,0,0,1) ;   
		                               
  rts.b(0,1,3) = HPoint_nD<float,3>(0.5,1,1,1) ; 
  rts.b(1,1,2) = HPoint_nD<float,3>(1.5,1,1,1) ; 
  rts.b(2,1,1) = HPoint_nD<float,3>(2.5,1,0,1) ; 
  rts.b(3,1,0) = HPoint_nD<float,3>(3.5,1,1,1) ; 
		                               
  rts.b(0,2,2) = HPoint_nD<float,3>(1,2,0,1) ;   
  rts.b(1,2,1) = HPoint_nD<float,3>(2,2,2,1) ;   
  rts.b(2,2,0) = HPoint_nD<float,3>(3,2,-1,1) ;  
		                               
  rts.b(0,3,1) = HPoint_nD<float,3>(1.5,3,1,1) ; 
  rts.b(1,3,0) = HPoint_nD<float,3>(2.5,3,3,1) ; 
		                               
  rts.b(0,4,0) = HPoint_nD<float,3>(2,4,0,1) ;   

  cout << "HPoint at (0.5,0.5) = " << rts(0.5,0.5) << endl ;
  cout << "HPoint at (0.333,0.333) = " << rts(0.333,0.333) << endl ;


  cout << " You can view the triangular B-Spline in trispline.wrl\n" ;
  cout << std::flush ;

  rts.writeVRML("trispline.wrl");

  cout << " Now testing the conversion between NURBS surfaces and triangular B-Splines\n" ;

  NurbsSurface<float,3> surf ;
  surf.makeSphere(Point_nD<float,3>(0,0,0),2.0) ;

  NurbsSurfaceArray<float,3> sa ;

  surf.decompose(sa) ;

  ofstream fout ;

  fout.open("triconvert.wrl");

  for(int i=0;i<sa.n();++i){
    RTriangularBSpline<float,3> t1(4), t2(4) ;

    convert(sa[i],t1,t2) ;

    t1.writeVRML(fout,Color(255,0,0)) ;
    t2.writeVRML(fout,Color(0,0,255)) ;
  }
  
  fout.close();

  cout << "You can view a sphere converted into multiple triangular B-splines in triconvert.wrl\n" ;
  
}
