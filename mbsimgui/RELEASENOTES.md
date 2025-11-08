Release NEXT
============

MBSim
-----
- Feature "contact point transformation" is removed from magic formula 62 as it leads to incorrect tyre moments.

MBSimXML
--------
- Added a new class MBSimProject and SWIG Python directors in order to have access to mbsim xml models from Python.
- MBSim can now handle boolean expressions in Python symbolic functions using sympy.Piecewise. This allows e.g. to use
  sympy.functions.special.bsplines.bspline_basis_set as symbolic functions.
- Added the class MassValues to the mbxmlutils Python preprocessor to enable easy calculations with mass values (mass,
  center of mass and inertia tensor).
- Improved the stability of the RADAU5 integreator if system shift's are executed.
- Added a multivariate linear function and multivariate quadratic function element.
- Added the possibility to set a scalar tolerance for position,velocity,1st-order and force status separately to
  improve integrator performance, specially for DAE integrators.
- Add extern signal source and sink to mbsim to SWIG.
- Added more user interruption points to kill a simulation without corrupting the output file, even during preprocessing.
- Added buildInfo.json content to hdf5 output file. This way one can see the details software version which was used to create the file.
- Added a "storage" attribute to plot attributes which allows to save plotAttributes as datasets in instead of attributes.
  This allows e.g. to use compression for plotAttributes (HDF5 uses compression only on datasets but not on attributes)

MBSimFMI
--------
- Enable module path in mbsimCreateFMU to allow user defined modules in FMUs.

MBSimGUI
--------
- Improved the initialization of the python interpreter to allow more interaction with python in mbsimgui
- Added more helper function to the python evaluator to improve model specific plots in mbsimgui.
- Improved the responsibility of mbsimgui by using the new write mode in H5::File and moving more of the mbsimxml 
  initilaization code to a separate process during "update simulation". Moreover, this process is kept open to avoid relaunching
  it including to avoid reinitializing it (e.g. reinit python and its modules each time).
- Mark the 3D scene as "Outdated" if it is not yet updated with the newest model changes.
- Added plotting: model defined element context menu actions can now be added even to parameters.
  When clicked the deposited evaluator code is executed. The output is displayed in the Echo Area. The evaluator code can even
  create dialogs e.g. using PySide2 and create plots using e.g. matplotlib. Setting parameters from the evaluator code is also possible.
- Added a option to show all parameters not only the parameters of the current element. This is especially useful for users
  which should only parametrize models but not doing modelling in mbsimgui.
- Added a button to skip the mbsim initial projection when just the 3D scene is updated. This may improve the 3D scene view update
  speed a lot when the initial projection is expensive (but you will see the none projection model in the 3D scene).
- Added a button to disable the auto update of the 3D scene. This can be used if you do not need the visual feedback temporarily.
- Add a label to import parameters (similar to the name of "normal" parameters) just to give a import parameter a name=label
- The hidden flag for parameters can now use evaluator code which results in true or false. This allows to hide parameters
  dependent on the value of other parameters.
- The parameter view now shows the parameters inside a "Parameters" group. This allows all parameters of a element to be collapsed.
- Feature "contact point transformation" is removed from magic formula 62 as it leads to incorrect tyre moments.

H5Plotserie
-----------
- A mouse alt-left-click now replaces the x-axis of all curves of the current windows with the selection one.

hdf5serie
---------
- Added a new write mode to H5::File which does not lock the file during none SWMR mode. This shortens the time the file
  is locked in none SWMR mode which improved the interactivity when a file is used in a GUI. This is done by first writing,
  in none SWMR mode, to another file then rename the file to the original filename and open it in SWMR mode.
- Allow datasets of type fixed length string. This enables compression on strings.

OpenMBV
-------
- Allow to embed the ombvx file in the ombvh5 file. This way OpenMBV only need one input file and the ombvx file is even compressed.


Release 10.4
============

All
---
- Several release notes in 10.3 were missed. These are added now with * instead of - as enumerator sign, see Release 10.3.
- Complete new build on Windows: Instead of cross-compiling for Windows on Linux the Windows build is now done using MSYS2.

MBSim
-----
- A linear tyre model is now available.
- Output of tyre models is now unified
- Allow the first frame of Aerodynamics to be arbitrarily rotated to enable models were the global y-axis is not in gravity direction.
- The links "generalized initial position" and "generalized initial velocity" are now available. These new links allow to define the generalized initial position and velocity of an object by constraints.
- Removed/Deprecate useConstraintSolverForSmoothMotion in DSS. false is the only valid default now and true is an error.
- Fixed Jacobian submatrices for none-linear direct constraint solver (smoothSolver=directNonlinear)
- If the model cannot be made consistent during the initial velocity projection an error is thrown now
- Catch Windows WM_CLOSE signals and exit the simulation propably (WM_CLOSE is the Windows variant of SIG_INT/SIG_TERM)
- Added a new "action" attribute for <import> in the preprocessor if the Python evaluator is used.
- A revolution is now available in order to define a rotationally symmetric contour.
- An extrusion is now available in order to define a contour by extruding a cross section along a spine.
- Remove deprecated features in contour Tyre.
- Revised the interface for piecewise polynom function when breaks and coefficients are used. The old interface is still available for backward compatibility.

MBSimControl
------------
- A motion observer is now available in order to visualize a body that moves according to a given position and orientation signal.
- A stop link is now available that stops the simulation when its input signal reaches a defined threshold.

MBSimGUI
--------
- Frame chaser observer is now available.
- Motion observer is available (see MBSimControl).
- Use the Integrator startTime in the 3D-view of mbsimgui. t=0 was used for now to draw the 3D-view.
- Update element names if a parameter of a element changes to display the correct new element name.
- Added plotting: model defined element context menu actions can now be added. When clicked the deposited evaluator code is executed.
  The output is displayed in the Echo Area. The evaluator code can even create dialogs e.g. using PySide2 and create
  plots using e.g. matplotlib. Setting parameters from the evaluator code is also possible.

OpenMBV
-------
- Improved/reworked video export to allow fast regeneration of the video with different bitrate (quality/filesize)
- Enabled video export on Linux: note that IndirectGLX must be enabled in the X11 server flags (see error message),
  or the native resolution/background must be used to enable direct GLX export.


Release 10.3
============

All
---
- Various bug fixes.

MBSimGUI
--------
- The property dialog of a anyParameter has now also a "Eval" button which shows the evaluated parameter value.
  The format of this evaluated output is evaluator dependent. For Python the pretty printer "pprint" is used.
- The Embed = Array/Pattern feature of MBSimXML has more features then MBSimGUI can handle. Unhandled features are
  now handled in MBSimGUI as "Unknown XML Elements". Hence, MBSimGUI can now handle every feature but with reduced
  user experience.
- Enable project templates.
- The base index for plot is removed.

MBSimXML
--------
- The stdout/stderr output of evaluations are now redirected to the Info/Warn streams of MBSim.
- More XML elements (with maxOccurs>1) allow now to use the Embed = Array/Pattern feature.
- The base index for plot is removed.
* The preprocessor allow now doubles to be inf and nan. Note that calculating with these will still throw FPEs.
* The python-preprocessor now caches the python byte-code for each evaluation to improve performance.
* The mbxmlutils namespace of the octave and python evaluator contains now a namedColor function which converts a ansi color name
  or #rrggbb string to HSV.
* The preprocessor supports now Embed for local-elements. This is a major but fully backward compatible change.
  This allows now e.g. to use Embed for any local-element with maxOccurs>1 to add multiple elements using Embed.

MBSim
-----
- States in element "state machine" are now internal states of the dynamic system. This enables a restart of the simulation with the correct initial states when using state machines.
- The base index for plot is removed.
* Added dynamicSystemSolverTolerance to specify a different tolerance for the global DSS-level solver and local element-level solvers.
* Allow root-finding (=stop vectors) also for single values links.
* Added SWIG Python directors for mbsimControl
* Add different extrapolation types to PiecewisePolynomFunction: error, continue and linear
* Added a new solver type directNonlinear. This allows for models with nonlinear lambda with bilateral constraints (unilateral
  constraints are not supported). Elements must implement updater and updateJrla to support this or must still be linear in lambda.


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
