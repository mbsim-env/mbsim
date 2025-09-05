"""Helper function for matplotlib based plotting in MBSimGUI"""

import PySide2.QtWidgets
import mbxmlutils.Qt
import sys
import ctypes

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
    warnings.warn("mw.waitForPropertyDialogs is deprecated, please use mbxmlutils.Qt.blockUntilDialoagsAreClosed(*args)")
    mbxmlutils.Qt.blockUntilDialoagsAreClosed(*args)
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
