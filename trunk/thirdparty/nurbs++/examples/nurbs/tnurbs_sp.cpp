#include <nurbs_sp.h>

int main(){
  using namespace PLib ; 

  cerr << "Testing the surface point interface for a NURBS curve.\n\n" ;

  PlNurbsCurveSPf c ;

  c.makeCircle(Point3Df(100,100,0),30,0,2.0*M_PI) ;
  c.updateMaxU() ; 
  
  cout << "This shows that there is a point on the curve associated" << endl 
       << "with a control point. We can manipulate the point on the surface" << endl 
       << "directly instead of manipulating the control point." << endl 
       << "This might provide a more intuitive method to manipulate a NURBS curve." << endl 
       << endl << endl ; 

  cout << "The point associated with the control point 3 is\n" ;
  cout << "\tSurfP[3] = " << c.surfP(3) << ", ctrlPnts()[3] = " << c.ctrlPnts()[3] << endl ;

  cout << "By offsetting this point by (10,10,0,0), it is now at\n" ;
  c.modSurfCPby(3,HPoint3Df(10,10,0,0)) ;
  cout << "\tSurfP[3] = " <<  c.surfP(3) << ", ctrlPnts()[3] = " << c.ctrlPnts()[3] << endl; 

  cout << "We can also set it to a certain value. (100,150,0,1) for example.\n" ;
  c.modSurfCP(3,HPoint3Df(100,150,0,1)) ;  
  cout << "\tSurfP[3] = " << c.surfP(3) << ", ctrlPnts()[3] = " << c.ctrlPnts()[3] << endl ;


  cout << "\n\nNow testing to see if the maximal point of influence is computed correctly.\n" ;
  cout << "Look in the NURBS book for the figure2.10. \nThe values should correspond to the ones seen in that graphic.\n" ;

  
  Vector_HPoint3Df P(10) ;
  Vector_FLOAT U(14) ;

  U[0] = U[1] = U[2] = U[3] = 0.0 ;
  U[4] = 2.0 ;
  U[5] = U[6] = 4.0 ;
  U[7] = U[8] = U[9] = 6.0 ; 
  U[10] = U[11] = U[12] = U[13] = 8.0 ;
  
  c.reset(P,U,3) ;

  for(int i=0;i<c.ctrlPnts().n();++i)
    cout << i << " max at " << c.maxAt(i) << endl ;

  return 0 ;
}
