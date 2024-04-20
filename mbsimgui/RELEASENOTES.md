Release 10.2
============

BACKWARD INCOMPATIBLITY
-----------------------
- In all XML code text (and CDATA) nodes which are separated by processing instruction or comment nodes
  are no longer merged to a single text node at the position of the first text node. Instead now always
  the first none empty text node is interpreted as the content text node. All other text nodes must be
  empty (formatting only) text nodes.
  Its very unusual that this feature is used in existing any models. MBSimGUI has even never created such models.

All
---
- Various bug fixes.

MBSim
-----
- Use default impact law if it is not defined by the user.
- Allow multiplexing of input signals in signal operation
- Added new visualization for SignalObserver (e.g. use the new IvScreenAnnotation from OpenMBV)
- Improved preprocessing speed: read and parse XML files only ones even if needed multiple times using array/pattern.
- Allow array/pattern (Embed) for all container elements (even for Function's).
- Added a angleMode to Joint to support also large angles without the need to integrate the velocities, but only
  for one dimensional rotations (proper warning messages are printed and can be disable now).
- Use maximumNumberOfIterations from DynamicSystemSolver in all local none-linear solvers like in JointConstraint.
- Added localSolverTolerance to DynamicSystemSolver and use the tolerance in all local none-linear solvers.

MBSimGUI
--------
- Enable comments for elements, parameters and all top level widgets.
- Added a new File menu entry to open the 3D view (OpenMBV) settings dialog.
- Enable filtering in element browser.
- Added a new context menu to expand and collapse tree items to various depths

MBSimXML
--------
- Top level parameters can now be overwritten on command line in mbsimxml

OpenMBV
--------
- Allow to remove nodes from IvBody (wrl) by type and name.
- Improved 3D view regarding movement and clipping planes.
- Enabled the OpenMBV object factory to copy construct OpenMBV objects.
- Keep the cameras relative position and orientation when "Move camera with body".
- Overwide the VRML Background node to include the Coin3D https://github.com/coin3d/coin/pull/517.
- Add new settings option to define the Coin3D transparency type.
- Allow to pass the content of a IV file in IvBody as a string in XML instead of specifying
  the filename to read.
- Reworked the screen foreground elements in the 3D view and allow user defined foreground elements.
- Enable direct rendering on some Linux system on which the binary distribution used software rendering for now.

HDF5Serie
---------
- Set on Windows the hidden flag for all '.*.lock' files to hide these files also on Windows.
- Show legend in plot window.
- Added a new context menu to expand and collapse tree items to various depths.
- Added a new context menu in order to remove curves from a plot window.
- Enable shortcuts.
- Plot window name can be changed by double clicking on a tab.

fmatvec
-------
- Allow the native fmatvec Function objects inside of symbolic expressions. If the Function provides
  the derivatives then also parder(...) can be called in a symbolic expression (only two times, since
  Function provides at most second derivatives).
- Added a generic fast LRUCache which can e.g. be used in objects derived from Function to cache already
  calculated results.
- Fixed wrong symbolic expression evaluation due to double -> int convertion with overflows.
- Spezial handling of some operators regarding symbolic differentiantion when one argument is constant
  to simplify the result (shorter/faster expressions for differentations are generated)





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
