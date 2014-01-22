/*! File allowing override of memory function of MBSim by environment call
 *  replace new, new[] and delete, delete[]
 */
//#include <new>
//#include "model_instance.h"
//
//void* operator new(std::size_t n) throw(std::bad_alloc)
//{
//  printf("coucou\n");
//  return fmi::ModelInstance::functions.allocateMemory(1,n);
//}
//void operator delete(void * p) throw()
//{
//  fmi::ModelInstance::functions.freeMemory(p);
//}
//
//void* operator new[](std::size_t s) throw(std::bad_alloc)
//{
//  return ::operator new(s);
//}
//void operator delete[](void *p) throw()
//{
//  ::operator delete(p);
//}
