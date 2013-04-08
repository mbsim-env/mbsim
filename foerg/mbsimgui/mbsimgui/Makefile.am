include $(top_srcdir)/qt.mk

bin_PROGRAMS = mbsimgui

mbsimguidir = $(includedir)/mbsimgui

INCLUDES = -DTIXML_USE_STL $(QT_CFLAGS) $(MKOCTFILE_CFLAGS) $(OPENMBV_CFLAGS)

LDADD = $(QT_LIBS) $(MKOCTFILE_LIBS) $(OPENMBV_LIBS) 

mbsimgui_CPPFLAGS = $(MBXMLUTILS_CFLAGS)

mbsimgui_LDFLAGS = $(HDF5SERIE_LIBS) $(MBXMLUTILS_LIBS) -Wl,-rpath -Wl,\$$ORIGIN/../lib

mbsimgui_SOURCES = element.cc \
		   object.cc \
		   body.cc \
		   rigidbody.cc \
		   group.cc \
		   objectfactory.cc \
		   solver.cc \
		   frame.cc \
		   contour.cc \
		   integrator.cc \
		   parameter.cc \
		   octaveutils.cc \
		   treeitem.cc \
		   treemodel.cc \
		   delegate.cc \
		   basic_properties.cc \
		   string_properties.cc \
		   kinematics_properties.cc \
		   kinetics_properties.cc \
		   function_properties.cc \
		   ombv_properties.cc \
		   extended_properties.cc \
		   basic_widgets.cc \
		   string_widgets.cc \
		   function_widgets.cc \
		   kinematics_widgets.cc \
		   kinetics_widgets.cc \
		   ombv_widgets.cc \
		   extended_widgets.cc \
		   property_widget.cc\
		   dialogs.cc  \
		   utils.cc \
		   mainwindow.cc \
		   main.cc
		   

nodist_mbsimgui_SOURCES=$(QT_BUILT_SOURCES)

# C++ source file to build automatically (e.g. abc.moc.cc by abc.h)
QT_BUILT_SOURCES = basic_widgets.moc.cc \
		   string_widgets.moc.cc \
		   function_widgets.moc.cc \
		   kinematics_widgets.moc.cc \
		   kinetics_widgets.moc.cc \
		   ombv_widgets.moc.cc \
		   extended_widgets.moc.cc \
		   frame.moc.cc \
		   contour.moc.cc \
		   property_widget.moc.cc  \
		   dialogs.moc.cc  \
		   integrator.moc.cc \
		   delegate.moc.cc \
		   parameter.moc.cc \
		   mainwindow.moc.cc

mbsimgui_HEADERS = 
		   
		   
		   
mbsimguisharedir = @datadir@/mbsimgui
dist_mbsimguishare_DATA = inlineopenmbv.ombv.wst \
			  empty.ombv.xml \
			  empty.ombv.h5

mbsimguiicondir = @datadir@/mbsimgui/icons
dist_mbsimguiicon_DATA = icons/exit.svg \
			 icons/simulate.svg \
			 icons/preview.svg \
			 icons/openmbv.svg \
			 icons/h5plotserie.svg \
			 icons/resize.svg \
			 icons/workdir.svg

