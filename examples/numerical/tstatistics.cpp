#include "statistic.h"
#include <iostream>

int main(){
  using namespace PLib ; 
  

  cout << "Testing the statistic package.\n" ;

  initStatistic();

  int i ; 
  for(i=0;i<10;++i){
    cout << "!" << i << " = " << factorial<float>(i) << " and ln !" << i << " = " << lnOfFactorial<float>(i) << endl ; 
  }

  cout << "The error function(0) = " << errorFcn<float>(0) << endl ; 
  cout << "The error function(0.2) = " << errorFcn<float>(0.2) << endl ; 
  cout << "The error function(0.5) = " << errorFcn<float>(0.5) << endl ; 
  cout << "The error function(1.0) = " << errorFcn<float>(1.0) << endl ; 
  cout << "The error function(2.0) = " << errorFcn<float>(2.0) << endl ; 

}
