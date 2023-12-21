Release NEXT
============

MBSimGUI
--------
- Enable comments for elements, parameters and all top level widgets.

Release 10.1
============ 

BACKWARD INCOMPATIBLITY
-----------------------
- The Embed counterName attribute (named Array/Pattern counterName in MBSimGUI) is no longer
  evaluated, its now a plain string. Models which use e.g. "index{n}" as counterName will no longer work!
  However, its very unusual that this feature is used in existing any models.
  The new behaviour is aligned with parameter names which are also not evaluated.

All
---
- Various bug fixes.

MBSim
-----
- Use shape of cross section contour and contour parameters to define tyre contour instead of rim radius.
  For backward compatibiltiy rim radius is still supported by MBSimXML and MBSimGUI but it should no longer
  be used.
- Enable feature "tyre side" and "contact point transformation" in implementation of Magic Formula 6.2.
- Enable tyre cross section contours with flat, circular and elliptical shape.
- Enable contact of tyres with elliptical shape to arbitray spatial contours.

MBSimGUI
--------
- Show disabled elements greyed out in model tree.
- Define a mimimum size for text editor in expression widget.
- Enable highlighting in xml editor.
- Show error in xml code while debugging the model.
- Use monospace font for octave/python/XML widgets (was not working on Windows).
- Enable general embeddings for openMBVObject within MBSimEnvironment.
- Show all possible item names in the model and parameter tree. There can be more than one name
  if the item name depends on a Array/Pattern counter variable.
- Show default values only when line edit is cleared.
- Enable feature "default evaluator" within settings dialog.

MBSimXML
--------
- The Embed counterName (named Array/Pattern countername in MBSimGUI) is no longer
  evaluated but used as a plain string. This is none backward compatible change see above.

OpenMBV
-------
- Implemented 3D stereo rendering using full screen side-by-side (SBS) 3D TV format.
- Replaced the mouse cursor in the 3D scene by a moving frame in world direction.
- Implemented new mouse 3D scene rotations (e.g. to ensure that the world z-axis is always vertical).
- Reworked the settings dialog and make the mouse and tap interaction fully configurable.

H5Plotserie
-----------
- Only show new plot windows maximized if the last plot window was maximized.
- Enable context menu for plots.
