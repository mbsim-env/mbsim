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
_libmbsimgui.mbsimgui_Element_setParameterCode.restype=None
_libmbsimgui.mbsimgui_Element_setParameterCode.argtypes=[ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p]
_libmbsimgui.mbsimgui_MainWindow_refresh.restype=None
_libmbsimgui.mbsimgui_MainWindow_refresh.argtypes=[]

# reimplementation of the C++ class MBSimGUI::BasicPropertyDialog -> keep it simple and in sync
class BasicPropertyDialog(PySide2.QtWidgets.QDialog):
  def __init__(self):
    super().__init__()
    self.setModal(False)
  def showEvent(self, event):
    _libmbsimgui.mbsimgui_MainWindow_prepareForPropertyDialogOpen()
    super().showEvent(event)
  def hideEvent(self, event):
    _libmbsimgui.mbsimgui_MainWindow_prepareForPropertyDialogClose()
    super().hideEvent(event)

class MatplotlibDialog(BasicPropertyDialog, mbxmlutils.Qt.MatplotlibDialog):
  pass

class StdMatplotlibDialog(BasicPropertyDialog, mbxmlutils.Qt.StdMatplotlibDialog):
  pass

class _MainWindow(object):
  def waitForPropertyDialogs(self, *args):
    import PySide2.QtCore
    openDialogs=len(args)
    el=PySide2.QtCore.QEventLoop()
    def finished():
      nonlocal openDialogs
      openDialogs-=1
      if openDialogs==0:
        el.exit(0)
    for d in args:
      d.finished.connect(finished)
    el.exec_()
  def refresh(self):
    _libmbsimgui.mbsimgui_MainWindow_refresh()

mw=_MainWindow()

class _Element(object):
  def __init__(self, nativePtr):
    self.nativePtr=nativePtr
  def setParameterCode(self, parName, code):
    _libmbsimgui.mbsimgui_Element_setParameterCode(self.nativePtr, parName.encode("utf8"), code.encode("utf8"))
