"""Helper function for matplotlib based plotting in MBSimGUI"""

import PySide2.QtWidgets
import mbxmlutils
import mbxmlutils.Qt
import sys
import ctypes
import os.path

if sys.platform.startswith('linux'):
  _libmbsimgui=ctypes.cdll.LoadLibrary("libmbsimgui.so")
else:
  _libmbsimgui=ctypes.cdll.LoadLibrary("libmbsimgui-0")
_libmbsimgui.mbsimgui_MainWindow_prepareForPropertyDialogOpen.restype=None
_libmbsimgui.mbsimgui_MainWindow_prepareForPropertyDialogClose.restype=None
_libmbsimgui.mbsimgui_Element_setParameterValue.restype=None
_libmbsimgui.mbsimgui_Element_setParameterValue.argtypes=[ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p]
_libmbsimgui.mbsimgui_Parameter_setValue.restype=None
_libmbsimgui.mbsimgui_Parameter_setValue.argtypes=[ctypes.c_void_p, ctypes.c_char_p]
_libmbsimgui.mbsimgui_MainWindow_refresh.restype=None
_libmbsimgui.mbsimgui_MainWindow_refresh.argtypes=[]

class BasicPropertyDialog(PySide2.QtWidgets.QDialog):
  """reimplementation of the C++ class MBSimGUI::BasicPropertyDialog -> keep it simple and in sync"""
  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    self.setModal(False)
  def showEvent(self, event):
    _libmbsimgui.mbsimgui_MainWindow_prepareForPropertyDialogOpen()
    super().showEvent(event)
  def hideEvent(self, event):
    _libmbsimgui.mbsimgui_MainWindow_prepareForPropertyDialogClose()
    super().hideEvent(event)

class MatplotlibDialog(BasicPropertyDialog, mbxmlutils.Qt.MatplotlibDialog):
  """Same as mbxmlutils.Qt.MatplotlibDialog but extended for usage in MBSimGUI"""
  def __init__(self):
    super().__init__()

class StdMatplotlibDialog(BasicPropertyDialog, mbxmlutils.Qt.StdMatplotlibDialog):
  """Same as mbxmlutils.Qt.StdMatplotlibDialog but extended for usage in MBSimGUI"""
  def __init__(self, **fig_kwargs):
    super().__init__(**fig_kwargs)

class _MainWindow(object):
  def waitForPropertyDialogs(self, *args):
    import warnings
    warnings.warn("mw.waitForPropertyDialogs is deprecated, please use mbxmlutils.Qt.blockUntilDialogsAreClosed(*args)")
    mbxmlutils.Qt.blockUntilDialogsAreClosed(*args)
  def refresh(self):
    _libmbsimgui.mbsimgui_MainWindow_refresh()

mw=_MainWindow()

class _Element(object):
  def __init__(self, nativePtr):
    self.nativePtr=nativePtr
  def setParameterCode(self, parName, code):
    import warnings
    warnings.warn("mbsimgui_element.setParameterCode is deprecated, please use mbsimgui_element.setParameterValue")
    self.setParameterValue(parName, code)
  def setParameterValue(self, parName, code):
    _libmbsimgui.mbsimgui_Element_setParameterValue(self.nativePtr, parName.encode("utf8"), code.encode("utf8"))

class _Parameter(object):
  def __init__(self, nativePtr):
    self.nativePtr=nativePtr
  def setValue(self, code):
    _libmbsimgui.mbsimgui_Parameter_setValue(self.nativePtr, code.encode("utf8"))

def setParameterUsingFileOrDirDialog(parOrEle, caption, filter, fileOrDir="", relativeTo=os.path.dirname(mbxmlutils.getOriginalFilename()), skipRefresh=False):
  """Set the parameter defined by parOrEle to the file/dir selected by the user using a file/dir dialog box.

  'parOrEle' is the parameter which should be set, either of
  - mbsimgui_parameter
  - (mbsimgui_element, "name of par")
  'caption' is the title of the dialog box
  'filter' is 'DIR' to select a directory or a Qt filter (e.g. 'Text files (*.txt);;All (*.*)') to select a file
  'fileOrDir', if not empty, is the initial path of the dialog box
  'relativeTo', if not None, set the parameter with a relative path relative to 'relativeTo'
  'skipRefresh' defines whether to all mbsimgui.mw.refresh() or not"""
  import PySide2.QtWidgets
  import collections.abc
  import mbsimgui
  if not os.path.isabs(fileOrDir):
    fileOrDir = os.path.normpath(os.path.join(relativeTo, fileOrDir))
  if filter=="DIR":
    fn = PySide2.QtWidgets.QFileDialog.getExistingDirectory(None, caption, fileOrDir)
  else:
    fn = PySide2.QtWidgets.QFileDialog.getOpenFileName(None, caption, fileOrDir, filter)[0]
  if len(fn)==0:
    return
  if relativeTo is not None:
    fn = os.path.relpath(fn, relativeTo)
  if isinstance(parOrEle, collections.abc.Iterable):
    parOrEle[0].setParameterValue(parOrEle[1], f'"{fn}"')
  else:
    parOrEle.setValue(f'"{fn}"')
  if not skipRefresh:
    mw.refresh()
