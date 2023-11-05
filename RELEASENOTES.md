Release NEXT
============ 

All
---
- Various bug fixes.

MBSimGUI
--------
- Show disabled elements greyed out in model tree
- Define a mimimum size for text editor in expression widget
- Enable highlighting in xml editor
- Show error in xml code while debugging the model
- Use monospace font for octave/python/XML widgets (was not working on Windows)
- Enable general embeddings for openMBVObject within MBSimEnvironment

OpenMBV
-------
- Implemented 3D stereo rendering using full screen side-by-side (SBS) 3D TV format
- Replaced the mouse cursor in the 3D scene by a moving frame in world direction
- Implemented new mouse 3D scene rotations (e.g. to ensure that the world z-axis is always vertical)
- Reworked the settings dialog and make the mouse and tap interaction fully configurable

H5Plotserie
-----------
- Only show new plot windows maximized if the last plot window was maximized (h5plotserie).
- Enable context menu for plots
