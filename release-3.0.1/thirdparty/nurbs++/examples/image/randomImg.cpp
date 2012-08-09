#include <image.h>
#include <stdlib.h>
#include <cmath>

const double MaxRandom = 32767 ;

double randUnity(){
  return double(rand())/MaxRandom ;
}

int randColor(){
  return int(randUnity()*255.0);
}

int main(){

#ifdef WITH_IMAGE_MAGICK
  IM_Image A ;
  IM_ColorImage B ;

  srand(356) ;

  A.resize(100,100) ;
  B.resize(A.rows(),A.cols()) ;


  double sinF = M_PI/double(A.cols())*0.5 ;
  for(int i=0;i<A.rows();++i)
    for(int j=0;j<A.cols();++j){
      //A(i,j) = randColor() ;
      //B(i,j) = Color(randColor(),randColor(),randColor());
      A(i,j) = (int)(sin(double(j)*sinF)*127.0) + 128 ;
      //B(i,j).r = (int)(sin(double(j)*sinF)*127.0) + 128 ;
      //B(i,j).g = (int)(sin(double(j)*sinF+M_PI/2.0)*127.0) + 128 ;
      //B(i,j).b = (int)(sin(double(j)*sinF+M_PI/4.0)*127.0) + 128 ;
      B(i,j).fromHSV(double(j)/double(A.cols()-1)*360.0,1,1) ;
    }

  A.write("randomG.png");
  B.write("randomC.png");

  return 0 ;
#else
  cerr << "This test program requires Image Magick support.\n" ; 
  return 1; 
#endif
}
