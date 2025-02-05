import PySide2.QtWidgets
import mbxmlutils.Qt

# reimplementation of the C++ class MBSimGUI::BasicPropertyDialog -> keep it simple and in sync
class BasicPropertyDialog(PySide2.QtWidgets.QDialog):
  def __init__(self):
    import ctypes
    import sys
    super().__init__()
    self.setModal(False)
    if sys.platform.startswith('linux'):
      self.dll=ctypes.cdll.LoadLibrary("libmbsimgui.so")
    else:
      self.dll=ctypes.cdll.LoadLibrary("libmbsimgui")
    self.dll.mbsimgui_MainWindow_prepareForPropertyDialogOpen.restype=None
    self.dll.mbsimgui_MainWindow_prepareForPropertyDialogClose.restype=None
  def showEvent(self, event):
    self.dll.mbsimgui_MainWindow_prepareForPropertyDialogOpen()
    super().showEvent(event)
  def hideEvent(self, event):
    self.dll.mbsimgui_MainWindow_prepareForPropertyDialogClose()
    super().hideEvent(event)

class MatplotlibDialog(BasicPropertyDialog, mbxmlutils.Qt.MatplotlibDialog):
  pass

class StdMatplotlibDialog(BasicPropertyDialog, mbxmlutils.Qt.StdMatplotlibDialog):
  pass

def waitForDialogs(*args):
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
