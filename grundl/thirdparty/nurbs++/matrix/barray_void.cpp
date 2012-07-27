#include "barray.cpp"

namespace PLib {

#ifdef NO_IMPLICIT_TEMPLATES

  template class BasicArray<void*> ;
  template void resizeBasicArray<void*>(BasicArray<void*>&,int) ;
  template int operator!=(const BasicArray<void*>&,const BasicArray<void*>&); 
  template int operator==(const BasicArray<void*>&,const BasicArray<void*>&); 
  //template istream& operator>>(istream& is, BasicArray<void*>& ary);
  //template ostream& operator<<(ostream& os, const BasicArray<void*>& ary);

#endif

}
