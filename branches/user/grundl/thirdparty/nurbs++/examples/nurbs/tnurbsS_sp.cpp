#include <nurbsS_sp.h>

int main(){
  using namespace PLib ; 

  cerr << "Testing the surface point interface for a NURBS surface.\n\n" ;

  PlNurbsSurfaceSPf s ;

  s.makeSphere(Point3Df(0,0,0),1) ;

  s.print(cout);

  s.updateMaxUV() ;
  
  cout << "This shows that there is a point on the surface associated" << endl 
       << "with a control point. We can manipulate the point on the surface" << endl 
       << "directly instead of manipulating the control point." << endl 
       << "This might provide a more intuitive method to manipulate a NURBS surface." << endl 
       << endl << endl ; 

  cout << "The point associated with the control point 3 is\n" ;
  cout << "\tSurfP(3,3) = " << s.surfP(3,3) << 
    "\n\tctrlPnts(3,3) = " << s.ctrlPnts()(3,3) << endl << endl; ;

  cout << "By offsetting this point by (1,2,3,0), it is now at\n" ;
  s.modSurfCPby(3,3,HPoint3Df(1,2,3,0)) ;
  cout << "\tSurfP(3,3) = " <<  s.surfP(3,3) << 
    "\n\tctrlPnts()(3,3) = " << s.ctrlPnts()(3,3) << endl << endl; 

  
  cout << "We can also set it to a certain value. (5,6,7,1) for example.\n" ;
  s.modSurfCP(3,3,HPoint3Df(5,6,7,1)) ;  
  cout << "\tSurfP(3,3) = " << s.surfP(3,3) << 
    "\n\tctrlPnts(3,3) = " << s.ctrlPnts()(3,3) << endl << endl; ;


  cout << "We want to generate a parallel surface to this one.\n" ;

  PlNurbsSurfaceSPf p = s.generateParallel(1.0) ;

  cout << "Checking if the surface are indeed 1.0 apart: " ;
  Point3Df d = p.pointAt(0.5,0.5) - s.pointAt(0.5,0.5) ;
  cout << norm(d) << endl ;


  cout << "The result can be seen in tnurbsS_spA.ns and tnurbsS_spB.ns.\n" ;

  s.write("tnurbsS_spA.ns") ;
  p.write("tnurbsS_spB.ns") ;
  
  return 0 ;
}
