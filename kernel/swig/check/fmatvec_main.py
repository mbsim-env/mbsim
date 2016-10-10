import mymod
import numpy
import sys

exList=[]

for className in ['FooVecV', 'FooRowVec3', 'FooVec']:
  print('START--------------'+className)

  print('1--------------')
  try:
    f=mymod.__dict__[className](2)
    ret=f.v
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 1 1")
    if ret[0]!=2: raise RuntimeError("At "+className+" 1 2")
    ret[0]=-1
    if f.v[0]!=-1: raise RuntimeError("At "+className+" 1 3")
    f.v[0]=-2;
    if ret[0]!=-2: raise RuntimeError("At "+className+" 1 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('2--------------')
  try:
    f=mymod.__dict__[className](2)
    ret=f.v[0]
    if type(ret).__name__!="float64": raise RuntimeError("At "+className+" 2 1")
    if ret!=2: raise RuntimeError("At "+className+" 2 2")
    ret=-1
    if f.v[0]==-1: raise RuntimeError("At "+className+" 2 3")
    f.v[0]=-2
    if ret==-2: raise RuntimeError("At "+className+" 2 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('3--------------')
  try:
    f=mymod.__dict__[className](2)
    a=numpy.array([3.0, 3.0, 3.0])
    f.v=a # label a
    if f.v[0]!=3: raise RuntimeError("At "+className+" 3 1")
    a[0]=-1
    if f.v[0]==-1: raise RuntimeError("At "+className+" 3 2") # label a is a deep copy
    f.v[0]=-2
    if a[0]==-2: raise RuntimeError("At "+className+" 3 3") # label a is a deep copy
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('4--------------')
  try:
    f=mymod.__dict__[className](2)
    a=4.0
    f.v[0]=a
    if f.v[0]!=4: raise RuntimeError("At "+className+" 4 1")
    a=-1
    if f.v[0]==-1: raise RuntimeError("At "+className+" 4 2")
    f.v[0]=-2
    if a==-2: raise RuntimeError("At "+className+" 4 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('5--------------')
  try:
    f=mymod.__dict__[className](2)
    ret=f.funcret_v()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 5 1")
    if ret[0]!=2: raise RuntimeError("At "+className+" 5 2")
    f.v[0]=-1
    if ret[0]==-1: raise RuntimeError("At "+className+" 5 3")
    ret[0]=-2
    if f.v[0]==-2: raise RuntimeError("At "+className+" 5 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('6--------------')
  try:
    f=mymod.__dict__[className](2)
    ret=f.funcret_vc()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 6 1")
    if ret[0]!=2: raise RuntimeError("At "+className+" 6 2")
    f.v[0]=-1
    if ret[0]==-1: raise RuntimeError("At "+className+" 6 3")
    ret[0]=-2
    if f.v[0]==-2: raise RuntimeError("At "+className+" 6 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('7--------------')
  try:
    f=mymod.__dict__[className](2)
    ret=f.funcret_r()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 7 1")
    if ret[0]!=2: raise RuntimeError("At "+className+" 7 2")
    ret[0]=-1
    if f.v[0]!=-1: raise RuntimeError("At "+className+" 7 3")
    f.v[0]=-1
    if ret[0]!=-1: raise RuntimeError("At "+className+" 7 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('8--------------')
  try:
    f=mymod.__dict__[className](2)
    ret=f.funcret_rc()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 8 1")
    if ret[0]!=2: raise RuntimeError("At "+className+" 8 2")
    f.v[0]=-1
    if ret[0]==-1: raise RuntimeError("At "+className+" 8 3")
    ret[0]=-2
    if f.v[0]==-2: raise RuntimeError("At "+className+" 8 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('9--------------')
  try:
    for val in [3, 3.0]:
      f=mymod.__dict__[className](2)
      arg=numpy.array([val, val, val])
      f.func_v(arg)
      if f.v[0]!=3: raise RuntimeError("At "+className+" 9 1")
      f.v[0]=-1
      if arg[0]==-1: raise RuntimeError("At "+className+" 9 2")
      arg[0]=-2
      if f.v[0]==-2: raise RuntimeError("At "+className+" 9 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('10--------------')
  try:
    for val in [3, 3.0]:
      f=mymod.__dict__[className](2)
      arg=numpy.array([val, val, val])
      f.func_vc(arg)
      if f.v[0]!=3: raise RuntimeError("At "+className+" 10 1")
      f.v[0]=-1
      if arg[0]==-1: raise RuntimeError("At "+className+" 10 2")
      arg[0]=-2
      if f.v[0]==-2: raise RuntimeError("At "+className+" 10 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('11--------------')
  try:
    f=mymod.__dict__[className](2)
    arg=numpy.array([5, 5, 5])
    try: f.func_r(arg)
    except: pass
    else: raise RuntimeError("At "+className+" 11 1")
  
    f=mymod.__dict__[className](2)
    arg=numpy.array([5.0, 5.0, 5.0])
    f.func_r(arg)
    if f.v[0]!=5: raise RuntimeError("At "+className+" 11 2")
    if arg[0]!=10: raise RuntimeError("At "+className+" 11 3")
    f.v[0]=-1
    if arg[0]==-1: raise RuntimeError("At "+className+" 11 4")
    arg[0]=-2
    if f.v[0]==-2: raise RuntimeError("At "+className+" 11 5")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('12--------------')
  try:
    for val in [3, 3.0]:
      f=mymod.__dict__[className](2)
      arg=numpy.array([val, val, val])
      f.func_rc(arg)
      if f.v[0]!=3: raise RuntimeError("At "+className+" 12 1")
      f.v[0]=-1
      if arg[0]==-1: raise RuntimeError("At "+className+" 12 2")
      arg[0]=-2
      if f.v[0]==-2: raise RuntimeError("At "+className+" 12 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")

  print('DONE--------------')



def addNumberIf(l, add):
  if add:
    l.append(-l[0])
    return l
  else:
    return l

for className in ['FooMat', 'FooMatVV', 'FooMat32', 'FooMatV2', 'FooMat3V', 'FooRotMat3',
                  'FooSymMat', 'FooSymMatV', 'FooSymMat33', 'FooSqrMat', 'FooSqrMatV', 'FooSqrMat33']:
  print('START--------------'+className)
  if className=="FooRotMat3" or className=="FooSymMat" or className=="FooSymMatV" or className=="FooSymMat33" or \
     className=="FooSqrMat" or className=="FooSqrMatV" or className=="FooSqrMat33":
    dim3=True
  else:
    dim3=False

  print('1--------------')
  try:
    f=mymod.__dict__[className]()
    ret=f.m
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 1 1")
    if ret[2,1]!=19: raise RuntimeError("At "+className+" 1 2")
    ret[2,1]=-1
    if f.m[2,1]!=-1: raise RuntimeError("At "+className+" 1 3")
    f.m[2,1]=-2;
    if ret[2,1]!=-2: raise RuntimeError("At "+className+" 1 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('2--------------')
  try:
    f=mymod.__dict__[className]()
    ret=f.m[2,1]
    if type(ret).__name__!="float64": raise RuntimeError("At "+className+" 2 1")
    if ret!=19: raise RuntimeError("At "+className+" 2 2")
    ret=-1
    if f.m[2,1]==-1: raise RuntimeError("At "+className+" 2 3")
    f.m[2,1]=-2
    if ret==-2: raise RuntimeError("At "+className+" 2 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('3--------------')
  try:
    f=mymod.__dict__[className]()
    a=numpy.array([addNumberIf([11.0, 12.0], dim3), addNumberIf([13.0, 14.0], dim3), addNumberIf([15.0, 16.0], dim3)])
    f.m=a # label b
    if f.m[2,1]!=16: raise RuntimeError("At "+className+" 3 1")
    a[2,1]=-1
    if f.m[2,1]==-1: raise RuntimeError("At "+className+" 3 2") # label b is a deep copy
    f.m[2,1]=-2
    if a[2,1]==-2: raise RuntimeError("At "+className+" 3 3") # label b is a deep copy
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('4--------------')
  try:
    f=mymod.__dict__[className]()
    a=4.0
    f.m[2,1]=a
    if f.m[2,1]!=4: raise RuntimeError("At "+className+" 4 1")
    a=-1
    if f.m[2,1]==-1: raise RuntimeError("At "+className+" 4 2")
    f.m[2,1]=-2
    if a==-2: raise RuntimeError("At "+className+" 4 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('5--------------')
  try:
    f=mymod.__dict__[className]()
    ret=f.funcret_v()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 5 1")
    if ret[2,1]!=19: raise RuntimeError("At "+className+" 5 2")
    f.m[2,1]=-1
    if ret[2,1]==-1: raise RuntimeError("At "+className+" 5 3")
    ret[2,1]=-2
    if f.m[2,1]==-2: raise RuntimeError("At "+className+" 5 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('6--------------')
  try:
    f=mymod.__dict__[className]()
    ret=f.funcret_vc()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 6 1")
    if ret[2,1]!=19: raise RuntimeError("At "+className+" 6 2")
    f.m[2,1]=-1
    if ret[2,1]==-1: raise RuntimeError("At "+className+" 6 3")
    ret[2,1]=-2
    if f.m[2,1]==-2: raise RuntimeError("At "+className+" 6 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('7--------------')
  try:
    f=mymod.__dict__[className]()
    ret=f.funcret_r()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 7 1")
    if ret[2,1]!=19: raise RuntimeError("At "+className+" 7 2")
    ret[2,1]=-1
    if f.m[2,1]!=-1: raise RuntimeError("At "+className+" 7 3")
    f.m[2,1]=-1
    if ret[2,1]!=-1: raise RuntimeError("At "+className+" 7 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('8--------------')
  try:
    f=mymod.__dict__[className]()
    ret=f.funcret_rc()
    if type(ret).__name__!="ndarray": raise RuntimeError("At "+className+" 8 1")
    if ret[2,1]!=19: raise RuntimeError("At "+className+" 8 2")
    f.m[2,1]=-1
    if ret[2,1]==-1: raise RuntimeError("At "+className+" 8 3")
    ret[2,1]=-2
    if f.m[2,1]==-2: raise RuntimeError("At "+className+" 8 4")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('9--------------')
  try:
    for val in [3, 3.0]:
      f=mymod.__dict__[className]()
      arg=numpy.array([addNumberIf([val*2, val*2], dim3), addNumberIf([val*2, val*2], dim3), addNumberIf([val*2, val], dim3)])
      f.func_v(arg)
      if f.m[2,1]!=3: raise RuntimeError("At "+className+" 9 1")
      f.m[2,1]=-1
      if arg[2,1]==-1: raise RuntimeError("At "+className+" 9 2")
      arg[2,1]=-2
      if f.m[2,1]==-2: raise RuntimeError("At "+className+" 9 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('10--------------')
  try:
    for val in [3, 3.0]:
      f=mymod.__dict__[className]()
      arg=numpy.array([addNumberIf([val*2, val*2], dim3), addNumberIf([val*2, val*2], dim3), addNumberIf([val*2, val], dim3)])
      f.func_vc(arg)
      if f.m[2,1]!=3: raise RuntimeError("At "+className+" 10 1")
      f.m[2,1]=-1
      if arg[2,1]==-1: raise RuntimeError("At "+className+" 10 2")
      arg[2,1]=-2
      if f.m[2,1]==-2: raise RuntimeError("At "+className+" 10 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('11--------------')
  try:
    f=mymod.__dict__[className]()
    arg=numpy.array([addNumberIf([5*2, 5*2], dim3), addNumberIf([5*2, 5*2], dim3), addNumberIf([5*2, 5], dim3)])
    try: f.func_r(arg)
    except: pass
    else: raise RuntimeError("At "+className+" 11 1")
  
    f=mymod.__dict__[className]()
    arg=numpy.array([addNumberIf([5.0*2, 5.0*2], dim3), addNumberIf([5.0*2, 5.0*2], dim3), addNumberIf([5.0*2, 5.0], dim3)])
    f.func_r(arg)
    if f.m[2,1]!=5: raise RuntimeError("At "+className+" 11 2")
    if arg[2,1]!=10: raise RuntimeError("At "+className+" 11 3")
    f.m[2,1]=-1
    if arg[2,1]==-1: raise RuntimeError("At "+className+" 11 4")
    arg[2,1]=-2
    if f.m[2,1]==-2: raise RuntimeError("At "+className+" 11 5")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")
  
  print('12--------------')
  try:
    for val in [3, 3.0]:
      f=mymod.__dict__[className]()
      arg=numpy.array([addNumberIf([val*2, val*2], dim3), addNumberIf([val*2, val*2], dim3), addNumberIf([val*2, val], dim3)])
      f.func_rc(arg)
      if f.m[2,1]!=3: raise RuntimeError("At "+className+" 12 1")
      f.m[2,1]=-1
      if arg[2,1]==-1: raise RuntimeError("At "+className+" 12 2")
      arg[2,1]=-2
      if f.m[2,1]==-2: raise RuntimeError("At "+className+" 12 3")
  except BaseException as ex:
    exList.append(ex)
    print("ERROR")



print('ALL DONE--------------')

if len(exList)>0:
  print("THE FOLLOWING EXCEPTIONS OCCURED:")
  for ex in exList:
    print(str(ex))
  sys.exit(1)
