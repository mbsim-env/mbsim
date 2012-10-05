#include "fft.cpp"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

template void bitrv2(int , BasicArray<float> &) ;
template void bitrv(int , BasicArray<float> &) ;
template void cdft(int , float , float , BasicArray<float> &) ;
template void rdft(int , float , float , BasicArray<float> &) ;
template void ddct(int , float , float , BasicArray<float> &) ;
template void ddst(int , float , float , BasicArray<float> &) ;
template void dfct(int , float , float , BasicArray<float> &) ;
template void dfst(int , float , float , BasicArray<float> &) ;


template void bitrv2(int , BasicArray<double> &) ;
template void bitrv(int , BasicArray<double> &) ;
template void cdft(int , double , double , BasicArray<double> &) ;
template void rdft(int , double , double , BasicArray<double> &) ;
template void ddct(int , double , double , BasicArray<double> &) ;
template void ddst(int , double , double , BasicArray<double> &) ;
template void dfct(int , double , double , BasicArray<double> &) ;
template void dfst(int , double , double , BasicArray<double> &) ;

 
#endif

}
