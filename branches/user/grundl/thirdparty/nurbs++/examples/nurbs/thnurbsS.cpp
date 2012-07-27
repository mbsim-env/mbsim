#include <hnurbsS.h>

int main(){
  using namespace PLib ; 

  Matrix_Point3Df pts(5,6) ;
  int i,j ;
  
  for(i=0;i<pts.rows();++i)
    for(j=0;j<pts.cols();++j){
      pts(i,j).x() = float(j) ;
      pts(i,j).y() = float(i) ;
      pts(i,j).z() = 0 ;
    }

  PlNurbsSurfacef s ;
  s.globalInterp(pts,3,3) ;

  cout << "knotU for S is " << s.knotU() << endl; 
  cout << "KnotV for S is " << s.knotV() << endl; 


  PlMatrix_Point3Df ders ;
  s.deriveAt(0,0,1,ders) ;
  cout << "Deriving the base surface at (0,0) << \n" << ders ; 


  cout << "Cross product = " << crossProduct(ders(0,1),ders(1,0))  << endl ; 

  PlHNurbsSurfacef H(s) ;

  cout << "H(0.0,0.0) = " << H(0.0,0.0) << endl ;
  cout << "H(0.01,0.01) = " << H(0.01,0.01) << endl ;
  cout << "H(0.0,0.01) = " << H(0.0,0.01) << endl ;
  cout << "H(0.01,0.0) = " << H(0.01,0.0) << endl ;

  PlHNurbsSurfacef *p,*p2,*p3 ;
  p = p2 = 0 ;

  p = H.addLevel(4) ;

  if(p){
    p->offset(0,0) = HPoint3Df(0,0,1,0) ;
  }

  p3 = 0 ;
  p3 = p->addLevel(2) ;

  if(p3){
    p3->offset(2,2) = HPoint3Df(0,0,0.5,0) ;
    p3->offset(0,0) = HPoint3Df(0,0,-0.2,0) ;
  }

  H.updateLevels() ;

  cout << "Checking if the control points were updated properly...\n" ;
  cout << "At level " << H.level() << " : " << H.ctrlPnts()(0,0) << endl ;
  cout << "At level " << p->level() << " : " << p->ctrlPnts()(0,0) << endl ;
  cout << "At level " << p3->level() << " : " << p3->ctrlPnts()(0,0) << endl ;
  

  cout << "After adding some modifications to the surface.\n" ;
  cout << "H(0.0,0.0) = " << H(0.0,0.0) << endl ;
  cout << "H(0.01,0.01) = " << H(0.01,0.01) << endl ;
  cout << "H(0.0,0.01) = " << H(0.0,0.01) << endl ;
  cout << "H(0.01,0.0) = " << H(0.01,0.0) << endl ;

  H.updateLevels() ;

  cout << "Checking if the control points were updated properly...\n" ;
  cout << "At level " << H.level() << " : " << H.ctrlPnts()(0,0) << endl ;
  cout << "At level " << p->level() << " : " << p->ctrlPnts()(0,0) << endl ;
  cout << "At level " << p3->level() << " : " << p3->ctrlPnts()(0,0) << endl ;
  
  cout << "After adding some modifications to the surface.\n" ;
  cout << "H(0.0,0.0) = " << H(0.0,0.0) << endl ;
  cout << "H(0.01,0.01) = " << H(0.01,0.01) << endl ;
  cout << "H(0.0,0.01) = " << H(0.0,0.01) << endl ;
  cout << "H(0.01,0.0) = " << H(0.01,0.0) << endl ;


  H.writeVRML("H1.wrl") ;

  //H.modCp(3,3,H.ctrlPnts()(3,3)+HPoint3Df(0,0,2,0)) ;

  H.writeVRML("H2.wrl") ;


  H.write("thnurbsS.nhs") ;

  PlHNurbsSurfacef H2 ;
  

  cerr << "Reading the saved file..." ;
  if(!H2.read("thnurbsS.nhs") ){
    cerr << "EROR READING THE FILE\n" ;
  }
  else
    cerr << "done\n" ;

  H2.updateLevels() ;
  
  H2.writeVRML("H3.wrl") ;

  cout << "H2(0.0,0.0) = " << H2(0.0,0.0) << endl ;
  cout << "H2(0.01,0.01) = " << H2(0.01,0.01) << endl ;
  cout << "H2(0.0,0.01) = " << H2(0.0,0.01) << endl ;
  cout << "H2(0.01,0.0) = " << H2(0.01,0.0) << endl ;

  cout << "At level " << H2.level() << " : " << H2.ctrlPnts()(0,0) << endl ;
  cout << "Max level = " << H2.maxLevel() << endl ;

  if(H2.firstLevel()){
    cout << "H2 patch is set to\n" ;
    cout << H2.firstLevel()->ctrlPnts() << endl ;
  }
  else{
    cout << "Bummer, the reading process didn't work properly.\n" ;
  }

  cout << "Done.\n" ;

  return 0 ;
}
