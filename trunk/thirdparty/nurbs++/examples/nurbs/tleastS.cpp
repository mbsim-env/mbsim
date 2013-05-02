#include <nurbsS.h>

int main(){
  using namespace PLib ; 
#ifdef WITH_IMAGE_MAGICK
  IM_Image img ;

  if(!img.read("tleastS.png")){
    cout << "Problem reading the tleastS.png image.\n" ;
    return 1 ;
  }

  Matrix_Point3Df Pts(img.rows(),img.cols()) ;
  for(int i=0;i<Pts.rows();++i)
    for(int j=0;j<Pts.cols();++j){
      Pts(i,j).x() = i ;
      Pts(i,j).y() = j ;
      Pts(i,j).z() = img(i,j) ;
    }

  PlNurbsSurfacef S ;

  S.leastSquares(Pts,3,3,Pts.rows()/3,Pts.cols()/3) ;

  S.writeVRML("tleastS.wrl",Color(255,100,255),50,80) ;

  #else
  cout << "The least squares surface approximation test requires Image Magick\n" ;
  #endif
  return 0 ;
}
