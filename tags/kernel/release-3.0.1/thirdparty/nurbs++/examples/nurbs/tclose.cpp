#include <nurbs.h>

using namespace PLib ;  


int main(){
  using namespace PLib ;  
  int deg = 3 ;

  //using namespace PLib ; 

  Vector_HPoint3Df P(10) ;

  P[0] = HPoint3Df(70,50,0,1) ;
  P[1] = HPoint3Df(100,60,0,1) ;
  P[2] = HPoint3Df(110,120,0,1) ;
  P[3] = HPoint3Df(100,150,0,1) ;
  P[4] = HPoint3Df(180,150,0,1) ;
  P[5] = HPoint3Df(200,200,0,1) ;
  P[6] = HPoint3Df(80,190,0,1) ;
  P[7] = HPoint3Df(60,100,0,1) ;
  P[8] = HPoint3Df(50,70,0,1) ;
  P[9] = P[0] ;
  
  PlNurbsCurvef curve1 ;
  PlNurbsCurvef curve2 ;

  curve1.globalInterpH(P,deg) ;

  cout << "U1= " << curve1.knot() << endl ; 
  cout << "P1 = " << curve1.ctrlPnts() << endl ; 
  cout << "D1 = " << curve1.degree() << endl ;

  P.resize(9); // the closed loop routine doesn't need P[0] = P[last]

  Vector_HPoint3Df Pw ;
  wrapPointVectorH(P,deg,Pw);
  curve2.globalInterpClosedH(Pw,deg) ;


  cout << "U2 = " << curve2.knot() << endl ; 
  cout << "P2 = " << curve2.ctrlPnts() << endl ; 
  cout << "D2 = " << curve2.degree() << endl ;

#ifdef WITH_IMAGE_MAGICK
  IM_ColorImage image ;
  image.resize(256,256) ;

  curve1.drawImg(image,Color(0,0,255),0.01) ;
  curve2.drawImg(image,Color(0,255,0),0.01) ;
 
  image.write("closed.png");
#endif

  cout << "\nPrinting the result to tnClose.ps\n" ; 
  curve2.writePS("tnClosedA.ps",1,2.0,5,false) ; 

  cout << "Clamping the curve in c2.\n" ; 

  curve1 = curve2 ; 
  curve1.clamp();

  cout << "Testing if it worked: c1(0) = " << curve2(0) << endl ; 
  cout << "                      c2(0) = " << curve1(0) << endl ; 
  cout << "                      c1(0.01) = " << curve2(0.01) << endl ; 
  cout << "                      c2(0.01) = " << curve1(0.01) << endl ; 
  cout << "                      c1(1) = " << curve2(1) << endl ; 
  cout << "                      c2(1) = " << curve1(1) << endl ; 

  cout << "The clamped curve is printed inside tnClosedB.ps\n" ; 
  curve1.writePS("tnClosedB.ps",1,2.0,5,true) ; 


  cout << "\nThe c2 curve is now unclamped.\n" ; 
  curve1.unclamp() ;
  cout << "Testing if it worked: c2(0) = " << curve1(0) << endl ; 
  cout << "                      c2(0.01) = " << curve1(0.01) << endl ; 
  cout << "                      c2(1) = " << curve1(1) << endl ; 
  cout << "The unclamped curve is printed inside tnClosedC.ps\n" ; 
  curve1.writePS("tnClosedC.ps",1,2.0,5,false) ; 

  cout << "If you see a difference between the Postscript files,";
  cout << "\nsomething went wrong.\n\n" ;
  cout << "Done testing the closed NURBS curve functions.\n" ; 


  return 0;

}
