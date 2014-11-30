all: .mbsimcontrol.done

xml/index.xml: $(top_srcdir)/schema/mbsimcontrol.xsd $(top_srcdir)/mbsimControl/*.h
	doxygen Doxyfile

.mbsimcontrol.done: xml/index.xml $(top_srcdir)/schema/mbsimcontrol.xsd $(top_srcdir)/mbsimControl/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimControl/mbsimcontrol.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimControl MBSim::


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimcontrol.done
