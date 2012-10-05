#include "vector.h"

int main(){
  PLib::Vector<float> V(100) ;

  int i= 0 ;
#ifdef USE_EXCEPTION
  try{
    for(;;++i){
      V[i] = 0 ; 
    }
  }
  catch(PLib::MatrixErr&){
    cerr << "An error occured\n" ; 
  }
#else
  for(;;++i){
    V[i] = 0 ; 
  }
#endif  

  return 0 ; 
}

