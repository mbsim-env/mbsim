// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") mbsim


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%template() std::pair<int,int>;

// includes needed in the generated swig c++ code
%{
#include "mbsim/functions/contact/funcpair_planarcontour_point.h"
#include "mbsim/numerics/functions/newton_method_jacobian_functions.h"
#include "mbsim/numerics/functions/damping_functions.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
%}

// wrap some std::vector<...> types used by the above wrapped classes
%template(VectorElement) std::vector<MBSim::Element*>;
%template(VectorString)  std::vector<std::string>;
//MISSING%template(VectorMat)     std::vector<fmatvec::Mat>;
//MISSING%template(VectorVec)     std::vector<fmatvec::Vec>;
%template(VectorFrame)   std::vector<MBSim::Frame*>;



// wrap the following classes
%include "mbsim/utils/plotfeatureenum.h"
%include "mbsim/element.h"



// Wrap MBSim::Function<Sig>
// SWIG cannot handle template partial specializations of the form MBSim::Function<double(int)>.
// Hence, we need to wrap it specially:

// wrap function.h -> this only wraps MBSim::FunctionBase (all others are templates)
%include "mbsim/functions/function.h"

// Define a SWIG macro for MBSim::Function<Ret(Arg)> with also renames it to allowed SWIG name.
// This definition must be keept in sync with the definition in mbsim/functions/function.h
%define FUNCTION1(Ret, Arg, namePostfix, DRetDArg, DRetDDir, DDRetDDArg)
%rename(Function_##namePostfix) MBSim::Function<Ret(Arg)>;
class MBSim::Function<Ret(Arg)> : public MBSim::FunctionBase, virtual public fmatvec::Atom {
  public:
    Function();
    void initializeUsingXML(xercesc::DOMElement *element);
    enum { retSize1 = StaticSize<Ret>::size1, retSize2 = StaticSize<Ret>::size2 };
    static constexpr int argSize = StaticSize<Arg>::size1;
    virtual std::pair<int, int> getRetSize() const;
    virtual int getArgSize() const;
    virtual Ret operator()(const Arg &arg)=0;
    virtual DRetDArg parDer(const Arg &arg);
    virtual DRetDDir dirDer(const Arg &argDir, const Arg &arg);
    virtual DDRetDDArg parDerParDer(const Arg &arg);
    virtual DRetDArg parDerDirDer(const Arg &argDir, const Arg &arg);
    virtual DRetDDir dirDerDirDer(const Arg &argDir_1, const Arg &argDir_2, const Arg &arg);
    virtual bool constParDer() const;
};
%enddef

// instantiate MBSim::Function<Ret(Arg)> for different types
//        Ret             , Arg          , namePostfix , DRetDArg          , DRetDDir        , DDRetDDArg
FUNCTION1(double          , double       , d_d         , double            , double          , double            )
FUNCTION1(double          , int          , d_int       , fmatvec::ErrorType, double          , fmatvec::ErrorType) // DRetDDir is wrong but defined like this in fmatvec
FUNCTION1(fmatvec::MatV   , fmatvec::VecV, MatV_VecV   , fmatvec::ErrorType, fmatvec::MatV   , fmatvec::ErrorType)
FUNCTION1(fmatvec::RotMat3, double       , RotMat3_d   , fmatvec::Vec3     , fmatvec::Vec3   , fmatvec::Vec3     )
FUNCTION1(fmatvec::RotMat3, fmatvec::VecV, RotMat3_VecV, fmatvec::Mat3xV   , fmatvec::Vec3   , fmatvec::ErrorType)
FUNCTION1(fmatvec::SqrMat , fmatvec::Vec , SqrMat_Vec  , fmatvec::ErrorType, fmatvec::SqrMat , fmatvec::ErrorType)
FUNCTION1(fmatvec::SqrMatV, double       , SqrMatV_d   , fmatvec::SqrMatV  , fmatvec::SqrMatV, fmatvec::SqrMatV  )
FUNCTION1(fmatvec::SymMatV, double       , SymMatV_d   , fmatvec::SymMatV  , fmatvec::SymMatV, fmatvec::SymMatV  )
FUNCTION1(fmatvec::Vec1   , double       , Vec1_d      , fmatvec::Vec1     , fmatvec::Vec1   , fmatvec::Vec1     )
FUNCTION1(fmatvec::Vec2   , double       , Vec2_d      , fmatvec::Vec2     , fmatvec::Vec2   , fmatvec::Vec2     )
FUNCTION1(fmatvec::Vec3   , double       , Vec3_d      , fmatvec::Vec3     , fmatvec::Vec3   , fmatvec::Vec3     )
FUNCTION1(fmatvec::Vec3   , fmatvec::Vec2, Vec3_Vec2   , fmatvec::Mat3x2   , fmatvec::Vec3   , fmatvec::ErrorType)
FUNCTION1(fmatvec::Vec3   , fmatvec::VecV, Vec3_VecV   , fmatvec::Mat3xV   , fmatvec::Vec3   , fmatvec::ErrorType)
FUNCTION1(fmatvec::Vec    , fmatvec::Vec , Vec_Vec     , fmatvec::Mat      , fmatvec::Vec    , fmatvec::ErrorType)
FUNCTION1(fmatvec::VecV   , double       , VecV_d      , fmatvec::VecV     , fmatvec::VecV   , fmatvec::VecV     )
FUNCTION1(fmatvec::VecV   , fmatvec::VecV, VecV_VecV   , fmatvec::MatV     , fmatvec::VecV   , fmatvec::ErrorType)
FUNCTION1(int             , fmatvec::Vec , int_Vec     , fmatvec::ErrorType, int             , fmatvec::ErrorType) // DRetDDir is wrong but defined like this in fmatvec
FUNCTION1(fmatvec::Vec    , double       , Vec_d       , fmatvec::Vec      , fmatvec::Vec    , fmatvec::Vec      )

// Define a SWIG macro for MBSim::Function<Ret(Arg1,Arg2)> with also renames it to allowed SWIG name.
// This definition must be keept in sync with the definition in mbsim/functions/function.h
%define FUNCTION2(Ret, Arg1, Arg2, namePostfix, DRetDArg1, DRetDArg2, DDRetDDArg1, DDRetDDArg2, DDRetDArg1DArg2, DRetDDir1, DRetDDir2)
%rename(Function_##namePostfix) MBSim::Function<Ret(Arg1,Arg2)>;
class MBSim::Function<Ret(Arg1, Arg2)> : public MBSim::FunctionBase, virtual public fmatvec::Atom {
  public:
    Function();
    void initializeUsingXML(xercesc::DOMElement *element);
    enum { retSize1 = StaticSize<Ret>::size1, retSize2 = StaticSize<Ret>::size2 };
    static constexpr int arg1Size = StaticSize<Arg1>::size1;
    static constexpr int arg2Size = StaticSize<Arg2>::size1;
    virtual std::pair<int, int> getRetSize() const;
    virtual int getArg1Size() const;
    virtual int getArg2Size() const;
    virtual Ret operator()(const Arg1 &arg1, const Arg2 &arg2)=0;
    virtual DRetDArg1 parDer1(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDDir1 dirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg2 parDer2(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDDir2 dirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DDRetDDArg1 parDer1ParDer1(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg1 parDer1DirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDDir1 dirDer1DirDer1(const Arg1 &arg1Dir_1, const Arg1 &arg1Dir_2, const Arg1 &arg1, const Arg2 &arg2);
    virtual DDRetDDArg2 parDer2ParDer2(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg2 parDer2DirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDDir2 dirDer2DirDer2(const Arg2 &arg2Dir_1, const Arg2 &arg2Dir_2, const Arg1 &arg1, const Arg2 &arg2);
    virtual DDRetDArg1DArg2 parDer1ParDer2(const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg1 parDer1DirDer2(const Arg2 &arg2Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDDir2 dirDer2DirDer1(const Arg2 &arg2Dir, const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual DRetDArg2 parDer2DirDer1(const Arg1 &arg1Dir, const Arg1 &arg1, const Arg2 &arg2);
    virtual bool constParDer1() const;
    virtual bool constParDer2() const;
};
%enddef

// instantiate MBSim::Function<Ret(Arg1,Arg2)> for different types
//        Ret             , Arg1         , Arg2         , namePostfix   , DRetDArg1      , DRetDArg2      , DDRetDDArg1       , DDRetDDArg2       , DDRetDArg1DArg2   , DRetDDir1    , DRetDDir2
FUNCTION2(double          , double       , double       , d_d_d         , double         , double         , double            , double            , double            , double       , double       )
FUNCTION2(double          , fmatvec::Vec , fmatvec::Vec , d_Vec_Vec     , fmatvec::RowVec, fmatvec::RowVec, fmatvec::ErrorType, fmatvec::ErrorType, fmatvec::ErrorType, double       , double       )
FUNCTION2(fmatvec::RotMat3, fmatvec::VecV, double       , RotMat3_VecV_d, fmatvec::Mat3xV, fmatvec::Vec3  , fmatvec::ErrorType, fmatvec::Vec3     , fmatvec::Mat3xV   , fmatvec::Vec3, fmatvec::Vec3)
FUNCTION2(fmatvec::Vec3   , fmatvec::VecV, double       , Vec3_VecV_d   , fmatvec::Mat3xV, fmatvec::Vec3  , fmatvec::ErrorType, fmatvec::Vec3     , fmatvec::Mat3xV   , fmatvec::Vec3, fmatvec::Vec3)
FUNCTION2(fmatvec::Vec    , fmatvec::Vec , double       , Vec_Vec_d     , fmatvec::Mat   , fmatvec::Vec   , fmatvec::ErrorType, fmatvec::Vec      , fmatvec::Mat      , fmatvec::Vec , fmatvec::Vec )
FUNCTION2(fmatvec::VecV   , fmatvec::VecV, double       , VecV_VecV_d   , fmatvec::MatV  , fmatvec::VecV  , fmatvec::ErrorType, fmatvec::VecV     , fmatvec::MatV     , fmatvec::VecV, fmatvec::VecV)
FUNCTION2(fmatvec::VecV   , fmatvec::VecV, fmatvec::VecV, VecV_VecV_VecV, fmatvec::MatV  , fmatvec::MatV  , fmatvec::ErrorType, fmatvec::ErrorType, fmatvec::ErrorType, fmatvec::VecV, fmatvec::VecV)

// wrap the following classes
%include "mbsim/numerics/functions/criteria_functions.h"
%include "mbsim/numerics/functions/newton_method_jacobian_functions.h"
%include "mbsim/numerics/functions/damping_functions.h"
%include "mbsim/functions/contact/funcpair_planarcontour_point.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// The following code is special to MBSim kernel this is not required by any .i file for a MBSim module //
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// includes needed in the generated swig c++ code
%{
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>
#include <xercesc/dom/DOMProcessingInstruction.hpp>
%}



// wrap part of the MBSim object factory
%include "mbsim/objectfactory_part.h"
%template(registerEnum_internal_PlotFeatureEnum) MBSim::registerEnum_internal<MBSim::PlotFeatureEnum>;



// dummy wrap MBXMLUtils::FQN (just to create proper memory management; for object creation _FQN is used)
namespace MBXMLUtils { class FQN {}; }

%inline %{

// internal helper function create a MBXMLUtils::FQN (MBXMLUtils is not wrapped at all)
MBXMLUtils::FQN _FQN(const std::string &ns, const std::string &name) {
  return MBXMLUtils::FQN(ns, name);
}

// internal helper function to cast a MBSim::AllocateBase to the Python director object
// or None if this MBSim::AllocateBase does not represent a director
PyObject* _dynamic_cast_Director(const MBSim::AllocateBase *x) {
  const Swig::Director *d=dynamic_cast<const Swig::Director*>(x);
  PyObject* ret=d ? d->swig_get_self() : Py_None;
  Py_INCREF(ret);
  return ret;
}

// Internal helper function to get the processing instruction of a pyScript element.
// This is required since xml.etree.cElementTree skip all comments and processing instruction when parsing!
std::string _getPyScriptProcessingInstruction(xercesc::DOMElement *e, const std::string &ns) {
  xercesc::DOMElement *pys=MBXMLUtils::E(e)->getFirstElementChildNamed(MBXMLUtils::FQN(ns, "pyScript"));
  if(!pys)
    return "";
  xercesc::DOMProcessingInstruction *pi=MBXMLUtils::E(pys)->getFirstProcessingInstructionChildNamed("ScriptParameter");
  if(!pi)
    return "";
  return MBXMLUtils::X()%pi->getData();
}

%}



%pythoncode %{

# initialization
from _mbsim_part050 import *
from _mbsim_part100 import *
from _mbsim_part125 import *
from _mbsim_part150 import *
from _mbsim_part200 import *
from _mbsim_part250 import *
from _mbsim_part300 import *
from _mbsim_part350 import *
from _mbsim_part400 import *

# internal helper class to register a director class in the MBSim::ObjectFactory
class _AllocatePython(AllocateBase):
  def __init__(self, className):
    super(_AllocatePython, self).__init__() 
    self.className=className
  def __call__(self):
    return self.className().__disown__()
  def isEqual(self, other):
    otherDirector=_dynamic_cast_Director(other)
    if otherDirector==None:
      return False
    if type(otherDirector)==_AllocatePython and otherDirector.className==self.className:
      return True
    return False;

# internal helper class to register a director class in the MBSim::ObjectFactory
class _DeallocatePython(DeallocateBase):
  def __init__(self):
    super(_DeallocatePython, self).__init__() 
  def __call__(self, e):
    del e

# fix module name: convert _<name>_part[0-9] to <name> (this enables splitting of large modules)
def _fixModuleName(moduleName):
  import re
  m=re.compile(r'^_([a-zA-Z0-9_]+)_part[0-9]+$').search(moduleName)
  if m!=None:
    return m.group(1)
  return moduleName

# get namespace of moudle moudleName (sourounded with { and } for the python moudle moduleName
def _getNSOf(moduleName):
  import sys
  ns=sys.modules[_fixModuleName(moduleName)].NS
  return ns[1:-1]

_moduleData={}

# fix local xml name (remove template)
def _fixXMLLocalName(name):
  c=name.find("_")
  if c>0:
    return name[0:c]
  return name

# internal helper function to extend a Python class
def _extendClass(className):
  import xml.etree.cElementTree as ET
  # create XML schema element and store it in _moduleData
  # store also all required modules in _moduleData
  allBaseClassName=className.__bases__
  if len(allBaseClassName)!=1:
    raise RuntimeError('Can only handle classed with one base class.')
  baseClassName=allBaseClassName[0]
  if not hasattr(className, 'getSchema'):
    # no getSchema method -> empty XML
    xsdpart=None
  else:
    # getSchema method defined -> use the returned Schema part
    xsdpart=className.getSchema()
  xsd1=ET.Element(XS+"element", {"name": ET.QName(_getNSOf(className.__module__), className.__name__),
                                 "substitutionGroup": ET.QName(_getNSOf(baseClassName.__module__),
                                                               _fixXMLLocalName(baseClassName.__name__)),
                                 "type": ET.QName(_getNSOf(className.__module__), className.__name__+"Type")})
  xsd2=ET.Element(XS+"complexType", {"name": ET.QName(_getNSOf(className.__module__), className.__name__+"Type")})
  cc=ET.Element(XS+"complexContent")
  xsd2.append(cc)
  ext=ET.Element(XS+"extension", {'base': ET.QName(_getNSOf(baseClassName.__module__),
                                                   _fixXMLLocalName(baseClassName.__name__)+'Type')})
  cc.append(ext)
  if xsdpart!=None:
    ext.append(xsdpart)
  xsd=[xsd1, xsd2]
  global _moduleData
  moduleName=_fixModuleName(className.__module__)
  if not moduleName in _moduleData:
    _moduleData[moduleName]={'xsdElements': [], 'requiredModules': set()}
  _moduleData[moduleName]['xsdElements'].extend(xsd)
  _moduleData[moduleName]['requiredModules'].add(_fixModuleName(baseClassName.__module__))






# register class in the MBSim::ObjectFactory.
# The class name of the class is used as the XML local name (excluding any part after _)
# This also extents className with some special members.
def registerClass(className):
  _extendClass(className)
  registerClass_internal(_FQN(_getNSOf(className.__module__), _fixXMLLocalName(className.__name__)),
    _AllocatePython(className).__disown__(), _DeallocatePython().__disown__())

# create the xsd for this module.
def generateXMLSchemaFile(moduleName):
  import os.path
  import xml.etree.cElementTree as ET
  global _moduleData

  # register namespace/prefix mappings
  ET.register_namespace("pv", PV[1:-1])
  ET.register_namespace("xs", XS[1:-1])
  ET.register_namespace("", _getNSOf(moduleName))
  for module in _moduleData[moduleName]['requiredModules']:
    ET.register_namespace(module, _getNSOf(module))
  # create xsd file
  xsd=ET.Element(XS+"schema", {"targetNamespace": _getNSOf(moduleName),
                               "elementFormDefault": "qualified",
                               "attributeFormDefault": "unqualified"})
  xsd.append(ET.Element(XS+"import", {"namespace": PV[1:-1]}))
  for module in _moduleData[moduleName]['requiredModules']:
    xsd.append(ET.Element(XS+"import", {'namespace': _getNSOf(module)}))
  xsd.extend(_moduleData[moduleName]['xsdElements'])
  return xsd

# create XML schema for a pyScript element
def pyScriptSchema():
  import xml.etree.cElementTree as ET
  xsdpart=ET.Element(XS+"sequence")
  xsdpart.append(ET.Element(XS+"element", {'name': 'pyScript', 'minOccurs': '0',
                                           'type': ET.QName(PV+'script')}))
  return xsdpart

# initializeUsingXML for a pyScript element
def pyScriptInitializeUsingXML(self, e, className):
  import xml.etree.cElementTree as ET
  import re
  import numpy
  super(className, self).initializeUsingXML(e)
  pys=e.find("{"+_getNSOf(className.__module__)+"}"+"pyScript")
  code=pys.text
  # fix python indentation
  lines=code.split("\n") # split to a vector of lines
  indent=-1
  firstNoneSpaceRE=re.compile(r'[^ ]')
  for lineNr, l in enumerate(lines):
    m=firstNoneSpaceRE.search(l) # get first none space character
    if m==None: continue # not found -> pure empty line -> do not modify
    pos=m.start()
    if l[pos]=='#': continue # found and first char is '#' -> pure comment line -> do not modify
    # now we have a line with a python statement
    if indent==-1: indent=pos # at the first python statement line use the current indent as indent for all others
    if l[0:indent]!=' '*indent: # check if line starts with at least indent spaces ...
      # ... if not its an indentation error
      raise RuntimeError("Unexpected indentation at line "+str(lineNr)+": "+code);
    lines[lineNr]=l[indent:] # remove the first indent spaces from the line
  code="\n".join(lines) # join the lines to a single string
  # read the processing instruction <?scriptParameter ...?> with the parameters
  globals={}
  # we cannot use python here, see _getPyScriptProcessingInstruction
  parstr=_getPyScriptProcessingInstruction(e, _getNSOf(className.__module__))
  scalarRE=re.compile(r"^scalar:([_a-zA-Z0-9]+)=(.*)$")
  vectorRE=re.compile(r"^vector:([_a-zA-Z0-9]+)=\[(.*)\]$")
  matrixRE=re.compile(r"^matrix:([_a-zA-Z0-9]+)=\[(.*)\]$")
  stringRE=re.compile(r"^string:([_a-zA-Z0-9]+)='(.*)'$")
  for line in parstr.split("\n"): # loop over all varibles (one is listed per line)
    line=line.strip() # remove leading/trailing spaces
    if line=="": continue # skip empty lines
    # handles scalars -> return as float
    if line.startswith("scalar:"):
      m=scalarRE.search(line)
      value=float(m.group(2))
    # handles vector -> return as numpy.array 1D
    elif line.startswith("vector:"):
      m=vectorRE.search(line)
      value=numpy.array(m.group(2).split(";")).astype(numpy.double)
    # handles matrix -> return as numpy.array 2D
    elif line.startswith("matrix:"):
      m=matrixRE.search(line)
      first=True
      for r in m.group(2).split(";"):
        rn=numpy.array([r.split(",")]).astype(numpy.double)
        if first:
          first=False
          value=rn
        else:
          value=numpy.concatenate((value, rn))
    # handles sting -> return as str
    elif line.startswith("string:"):
      m=stringRE.search(line)
      value=m.group(2)
    # add parameter to dict globas
    globals[m.group(1)]=value
  # execute the python script using all variables of the preprocessor
  globals[pys.attrib['objName']]=self # add self to list of variables (under the name ob the objName attribute)
  exec(code, globals)



# register plot feature enum in MBSim::EnumFactory<PlotFeatureEnum>.
# The name of the variables enumVar is used as the XML enum name.
def registerEnum_PlotFeatureEnum(ns, enumVar, enumName):
  registerEnum_internal_PlotFeatureEnum(_FQN(ns[1:-1], enumName), enumVar)



# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://www.mbsim-env.de/MBSim}"

XS="{http://www.w3.org/2001/XMLSchema}"
PV="{http://www.mbsim-env.de/MBXMLUtils}"

%}
