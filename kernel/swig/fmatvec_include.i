// check target language
#ifndef SWIGPYTHON
  #error "Only Python as target language is supported."
#endif

// add code to the generated code
%{

#include <cfenv>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <typeinfo>
#include <boost/core/demangle.hpp>
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

template<class T> inline const T& _deref(T* const& t) { return *t; }
template<class T> inline const T& _deref(const SwigValueWrapper<T>& t) { return static_cast<const T&>(t); }

template<typename AT> constexpr int _numPyType();
template<> constexpr int _numPyType<int>() { return NPY_LONG; }
template<> constexpr int _numPyType<double>() { return NPY_DOUBLE; }



// VECTOR

template<typename Vec, typename Arg1>
void _typemapOutVecOwnMemory(const Arg1 &_1, PyObject *&_result) {
  npy_intp dims[1];
  dims[0]=_deref(_1).size();
  _result=PyArray_SimpleNew(1, dims, _numPyType<typename Vec::value_type>());
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
  std::copy(&_deref(_1)(0), &_deref(_1)(0)+_deref(_1).size(),
            static_cast<typename Vec::value_type*>(PyArray_GETPTR1(reinterpret_cast<PyArrayObject*>(_result), 0)));
}

template<typename Vec>
void _typemapOutVecShareMemory(Vec *_1, PyObject *&_result) {
  npy_intp dims[1];
  dims[0]=_1->size();
  _result=PyArray_SimpleNewFromData(1, dims, _numPyType<typename Vec::value_type>(), &(*_1)(0));
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
}

template<typename Vec>
void _typemapInVecValue(SwigValueWrapper<Vec> &_1, PyObject *_input, swig_type_info *_1_descriptor) {
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    _1=*reinterpret_cast<Vec*>(inputp);
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    if(PyArray_NDIM(input)!=1)
      throw std::runtime_error("Must have 1 dimension.");
    int type=PyArray_TYPE(input);
    _checkNumPyType<typename Vec::value_type>(type);
    npy_intp *dims=PyArray_SHAPE(input);
    _1=Vec();
    #if __cplusplus >= 201103L && SWIG_VERSION >= 0x040100
      static_cast<Vec&&>(_1).resize(dims[0]);
    #else
      static_cast<Vec&>(_1).resize(dims[0]);
    #endif
    for(int i=0; i<dims[0]; ++i)
      #if __cplusplus >= 201103L && SWIG_VERSION >= 0x040100
        static_cast<Vec&&>(_1)(i)=_arrayGet<typename Vec::value_type>(input, type, i);
      #else
        static_cast<Vec&>(_1)(i)=_arrayGet<typename Vec::value_type>(input, type, i);
      #endif
  }
  else
    throw std::runtime_error("Wrong type.");
}

template<typename Vec>
void _typemapInVecPtr(Vec *&_1, PyObject *_input, swig_type_info *_1_descriptor, Vec &localVar) {
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    _1=reinterpret_cast<Vec*>(inputp);
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    if(PyArray_NDIM(input)!=1)
      throw std::runtime_error("Must have 1 dimension.");
    int type=PyArray_TYPE(input);
    _checkNumPyType<typename Vec::value_type>(type);
    npy_intp *dims=PyArray_SHAPE(input);
    localVar.resize(dims[0]);
    for(int i=0; i<dims[0]; ++i)
      localVar(i)=_arrayGet<typename Vec::value_type>(input, type, i);
    _1=&localVar;
  }
  else
    throw std::runtime_error("Wrong type.");
}

template<typename Vec>
void _typemapArgoutVec(const Vec *_1, PyObject *_input, swig_type_info *_1_descriptor, Vec &localVar) {
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    *reinterpret_cast<Vec*>(inputp)=*_1;
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    int type=PyArray_TYPE(input);
    if(_numPyType<typename Vec::value_type>()!=type)
      throw std::runtime_error("Wrong type of numpy array.");
    npy_intp *dims=PyArray_SHAPE(input);
    if(dims[0]!=localVar.size())
      throw std::runtime_error("Dimension has changed");
    std::copy(&localVar(0), &localVar(0)+localVar.size(),
              static_cast<typename Vec::value_type*>(PyArray_GETPTR1(input, 0)));
  }
  else
    throw std::runtime_error("Wrong type.");
}



// MATRIX

template<typename Mat, typename Arg1>
void _typemapOutMatOwnMemory(const Arg1 &_1, PyObject *&_result) {
  npy_intp dims[2];
  dims[0]=_deref(_1).rows();
  dims[1]=_deref(_1).cols();
  _result=PyArray_SimpleNew(2, dims, _numPyType<typename Mat::value_type>());
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
  PyArrayObject *_resultArr=reinterpret_cast<PyArrayObject*>(_result);
  for(int r=0; r<dims[0]; r++)
    for(int c=0; c<dims[1]; c++)
      *static_cast<typename Mat::value_type*>(PyArray_GETPTR2(_resultArr, r, c))=_deref(_1).operator()(r, c);
}

template<typename Mat>
void _typemapOutMatShareMemory(Mat *_1, PyObject *&_result) {
  npy_intp dims[2];
  dims[0]=_1->rows();
  dims[1]=_1->cols();
  _result=PyArray_SimpleNewFromData(2, dims, _numPyType<typename Mat::value_type>(), &(*_1)(0, 0));
  if(!_result)
    throw std::runtime_error("Cannot create ndarray");
  npy_intp *strides=PyArray_STRIDES(reinterpret_cast<PyArrayObject*>(_result));
  strides[(_1->blasOrder()==101) ? 1 : 0]=sizeof(typename Mat::value_type)*1;
  strides[(_1->blasOrder()==101) ? 0 : 1]=sizeof(typename Mat::value_type)*_1->ldim();
}

template<typename Mat>
void _typemapInMatValue(SwigValueWrapper<Mat> &_1, PyObject *_input, swig_type_info *_1_descriptor) {
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    _1=*reinterpret_cast<Mat*>(inputp);
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    if(PyArray_NDIM(input)!=2)
      throw std::runtime_error("Must have 2 dimension.");
    int type=PyArray_TYPE(input);
    _checkNumPyType<typename Mat::value_type>(type);
    npy_intp *dims=PyArray_SHAPE(input);
    _1=Mat();
    #if __cplusplus >= 201103L && SWIG_VERSION >= 0x040100
      static_cast<Mat&&>(_1).resize(dims[0], dims[1]);
    #else
      static_cast<Mat&>(_1).resize(dims[0], dims[1]);
    #endif
    for(int r=0; r<dims[0]; ++r)
      for(int c=0; c<dims[1]; ++c)
        #if __cplusplus >=201103L && SWIG_VERSION >= 0x040100
          static_cast<Mat&&>(_1)(r, c)=_arrayGet<typename Mat::value_type>(input, type, r, c);
        #else
          static_cast<Mat&>(_1)(r, c)=_arrayGet<typename Mat::value_type>(input, type, r, c);
        #endif
  }
  else
    throw std::runtime_error("Wrong type.");
}

template<typename Mat>
void _typemapInMatPtr(Mat *&_1, PyObject *_input, swig_type_info *_1_descriptor, Mat &localVar) {
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    _1=reinterpret_cast<Mat*>(inputp);
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    if(PyArray_NDIM(input)!=2)
      throw std::runtime_error("Must have 2 dimension.");
    int type=PyArray_TYPE(input);
    _checkNumPyType<typename Mat::value_type>(type);
    npy_intp *dims=PyArray_SHAPE(input);
    localVar.resize(dims[0], dims[1]);
    for(int r=0; r<dims[0]; ++r)
      for(int c=0; c<dims[1]; ++c)
        localVar(r, c)=_arrayGet<typename Mat::value_type>(input, type, r, c);
    _1=&localVar;
  }
  else
    throw std::runtime_error("Wrong type.");
}

template<typename Mat>
void _typemapArgoutMat(const Mat *_1, PyObject *_input, swig_type_info *_1_descriptor, Mat &localVar) {
  void *inputp;
  int res=SWIG_ConvertPtr(_input, &inputp, _1_descriptor, 0);
  if(SWIG_IsOK(res))
    *reinterpret_cast<Mat*>(inputp)=*_1;
  else if(PyArray_Check(_input)) {
    PyArrayObject *input=reinterpret_cast<PyArrayObject*>(_input);
    int type=PyArray_TYPE(input);
    if(_numPyType<typename Mat::value_type>()!=type)
      throw std::runtime_error("Wrong type of numpy array.");
    npy_intp *dims=PyArray_SHAPE(input);
    if(dims[0]!=localVar.rows())
      throw std::runtime_error("Number of rows has changed");
    if(dims[1]!=localVar.cols())
      throw std::runtime_error("Number of cols has changed");
    for(int r=0; r<dims[0]; ++r)
      for(int c=0; c<dims[1]; ++c)
        *static_cast<typename Mat::value_type*>(PyArray_GETPTR2(input, r, c))=_deref(_1)(r, c);
  }
  else
    throw std::runtime_error("Wrong type.");
}

%}

// init numpy
%init %{
  // numpy generates a overflow during initialization -> dislabe this FPE exception (save it first to restore it later)
  PythonCpp::DisableFPE disableFPE;
  import_array();
%}

// use SWIG_exception to throw a target language exception
%include exception.i
