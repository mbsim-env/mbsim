// check target language
#ifndef SWIGPYTHON
  #error "Only Python as target language is supported."
#endif

// add code to the generated code
%{

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <typeinfo>
#include <boost/version.hpp>
#if BOOST_VERSION >= 105600
  #include <boost/core/demangle.hpp>
#else
  #include <cxxabi.h>
  #ifndef BOOST_CORE_DEMANGLE_REPLACEMENT
  #define BOOST_CORE_DEMANGLE_REPLACEMENT
  namespace boost {
    namespace core {
      inline std::string demangle(const std::string &name) {
        int status;
        char* retc=abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
        if(status!=0) throw std::runtime_error("Cannot demangle c++ symbol.");
        std::string ret(retc);
        free(retc);
        return ret;
      }
    }
  }
  #endif
#endif
#include <fmatvec/fmatvec.h>

template<typename AT> void _checkNumPyType(int type);
template<> void _checkNumPyType<int>(int type) {
  if(type!=NPY_SHORT    && type!=NPY_USHORT    &&
     type!=NPY_INT      && type!=NPY_UINT      &&
     type!=NPY_LONG     && type!=NPY_ULONG     &&
     type!=NPY_LONGLONG && type!=NPY_ULONGLONG)
    throw std::runtime_error("Value is not of type integer.");
}
template<> void _checkNumPyType<double>(int type) {
  if(type!=NPY_SHORT    && type!=NPY_USHORT    &&
     type!=NPY_INT      && type!=NPY_UINT      &&
     type!=NPY_LONG     && type!=NPY_ULONG     &&
     type!=NPY_LONGLONG && type!=NPY_ULONGLONG &&
     type!=NPY_FLOAT    && type!=NPY_DOUBLE    && type!=NPY_LONGDOUBLE)
    throw std::runtime_error("Value is not of type floating point.");
}

template<typename AT> AT _arrayGet(PyArrayObject *a, int type, int r, int c=-1);
template<> int _arrayGet<int>(PyArrayObject *a, int type, int r, int c) {
  switch(type) {
    case NPY_SHORT:      return *static_cast<npy_short*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_USHORT:     return *static_cast<npy_ushort*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_INT:        return *static_cast<npy_int*>       (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_UINT:       return *static_cast<npy_uint*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONG:       return *static_cast<npy_long*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONG:      return *static_cast<npy_ulong*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGLONG:   return *static_cast<npy_longlong*>  (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONGLONG:  return *static_cast<npy_ulonglong*> (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
  }
  throw std::runtime_error("Value is not of type floating point (wrong element type).");
}
template<> double _arrayGet<double>(PyArrayObject *a, int type, int r, int c) {
  switch(type) {
    case NPY_SHORT:      return *static_cast<npy_short*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_USHORT:     return *static_cast<npy_ushort*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_INT:        return *static_cast<npy_int*>       (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_UINT:       return *static_cast<npy_uint*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONG:       return *static_cast<npy_long*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONG:      return *static_cast<npy_ulong*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGLONG:   return *static_cast<npy_longlong*>  (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONGLONG:  return *static_cast<npy_ulonglong*> (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_FLOAT:      return *static_cast<npy_float*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_DOUBLE:     return *static_cast<npy_double*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGDOUBLE: return *static_cast<npy_longdouble*>(c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
  }
  throw std::runtime_error("Value is not of type floating point (wrong element type).");
}

template<class T> inline T& _derefIfPointer(T & t) { return  t; }
template<class T> inline T& _derefIfPointer(T*& t) { return *t; }

template<class T> inline void _assignIfPointer(T & d, T& s) {}
template<class T> inline void _assignIfPointer(T*& d, T& s) { d=&s; }

template<class T> inline void _assignOrCopy(T & d, T* s) { d=*s; }
template<class T> inline void _assignOrCopy(T*& d, T* s) { d= s; }

template<typename AT> constexpr int _numPyType();
template<> constexpr int _numPyType<int>() { return NPY_LONG; }
template<> constexpr int _numPyType<double>() { return NPY_DOUBLE; }



template<typename Vec>
void _typemapOutVec_P_R(Vec *_1, PyObject *&_result) {
  npy_intp dims[1];
  dims[0]=_1->size();
  _result=PyArray_SimpleNewFromData(1, dims, _numPyType<typename Vec::AtomicType>(), &(*_1)(0));
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
}

template<typename Vec>
void _typemapOutVec_V_CR(Vec &_1, PyObject *&_result) {
  typedef typename std::remove_pointer<Vec>::type VecBT;
  npy_intp dims[1];
  dims[0]=_derefIfPointer(_1).size();
  _result=PyArray_SimpleNew(1, dims, _numPyType<typename VecBT::AtomicType>());
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
  std::copy(&_derefIfPointer(_1)(0), &_derefIfPointer(_1)(0)+_derefIfPointer(_1).size(),
            static_cast<typename VecBT::AtomicType*>(PyArray_GETPTR1(reinterpret_cast<PyArrayObject*>(_result), 0)));
}

template<typename Vec>
void _typemapInVec_P_R_V(Vec &_1, PyObject *_input, typename std::remove_pointer<Vec>::type &localVar, swig_type_info *_1_descriptor) {
  typedef typename std::remove_pointer<Vec>::type VecBT;
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    _assignOrCopy(_1, reinterpret_cast<VecBT*>(inputp));
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    if(PyArray_NDIM(input)!=1)
      throw std::runtime_error("Must have 1 dimension.");
    int type=PyArray_TYPE(input);
    _checkNumPyType<typename VecBT::AtomicType>(type);
    _assignIfPointer(_1, localVar);
    npy_intp *dims=PyArray_SHAPE(input);
    _derefIfPointer(_1).resize(dims[0]);
    for(int i=0; i<dims[0]; ++i)
      _derefIfPointer(_1)(i)=_arrayGet<typename VecBT::AtomicType>(input, type, i);
  }
  else
    throw std::runtime_error("Wrong type.");
}

template<typename Vec>
void _typemapArgoutVec_R(PyObject *_input, typename std::remove_reference<Vec>::type &localVar) {
  typedef typename std::remove_reference<Vec>::type VecBT;
  PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
  int type=PyArray_TYPE(input);
  if(type!=_numPyType<typename VecBT::AtomicType>())
    throw std::runtime_error(std::string("Must have atomic type ")+boost::core::demangle(typeid(typename VecBT::AtomicType).name()));
  std::copy(&localVar(0), &localVar(0)+localVar.size(),
            static_cast<typename VecBT::AtomicType*>(PyArray_GETPTR1(input, 0)));
}

template<typename Mat>
void _typemapOutMat_P_R(Mat *_1, PyObject *&_result) {
  npy_intp dims[2];
  dims[0]=_1->rows();
  dims[1]=_1->cols();
  _result=PyArray_SimpleNewFromData(2, dims, _numPyType<typename Mat::AtomicType>(), &(*_1)(0, 0));
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
  npy_intp *strides=PyArray_STRIDES(reinterpret_cast<PyArrayObject*>(_result));
  strides[_1->transposed() ? 1 : 0]=sizeof(typename Mat::AtomicType)*1;
  strides[_1->transposed() ? 0 : 1]=sizeof(typename Mat::AtomicType)*_1->ldim();
}

template<typename Mat>
void _typemapOutMat_V_CR(Mat &_1, PyObject *&_result) {
  typedef typename std::remove_pointer<Mat>::type MatBT;
  npy_intp dims[2];
  dims[0]=_derefIfPointer(_1).rows();
  dims[1]=_derefIfPointer(_1).cols();
  _result=PyArray_SimpleNew(2, dims, _numPyType<typename MatBT::AtomicType>());
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
  PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_result);
  for(int r=0; r<dims[0]; r++)
    for(int c=0; c<dims[1]; c++)
      *static_cast<typename MatBT::AtomicType*>(PyArray_GETPTR2(input, r, c))=
        _derefIfPointer(_1)(r, c);
}

template<typename Mat>
void _typemapInMat_P_R_V(Mat &_1, PyObject *_input, typename std::remove_pointer<Mat>::type &localVar, swig_type_info *_1_descriptor) {
  typedef typename std::remove_pointer<Mat>::type MatBT;
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    _assignOrCopy(_1, reinterpret_cast<MatBT*>(inputp));
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    if(PyArray_NDIM(input)!=2)
      throw std::runtime_error("Must have 2 dimension.");
    int type=PyArray_TYPE(input);
    _checkNumPyType<typename MatBT::AtomicType>(type);
    _assignIfPointer(_1, localVar);
    npy_intp *dims=PyArray_SHAPE(input);
    _derefIfPointer(_1).resize(dims[0], dims[1]);
    for(int r=0; r<dims[0]; ++r)
      for(int c=0; c<dims[1]; ++c)
        _derefIfPointer(_1)(r, c)=_arrayGet<typename MatBT::AtomicType>(input, type, r, c);
  }
  else
    throw std::runtime_error("Wrong type.");
}

template<typename Mat>
void _typemapArgoutMat_R(PyObject *_input, typename std::remove_reference<Mat>::type &localVar) {
  typedef typename std::remove_reference<Mat>::type MatBT;
  PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
  int type=PyArray_TYPE(input);
  if(type!=_numPyType<typename MatBT::AtomicType>())
    throw std::runtime_error(std::string("Must have atomic type ")+boost::core::demangle(typeid(typename MatBT::AtomicType).name()));
  for(int r=0; r<localVar.rows(); ++r)
    for(int c=0; c<localVar.cols(); ++c)
      *static_cast<typename MatBT::AtomicType*>(PyArray_GETPTR2(input, r, c))=
        localVar(r, c);
}

%}

// init numpy
%init %{
  _import_array();
%}

// use SWIG_exception to throw a target language exception
%include exception.i
