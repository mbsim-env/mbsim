# Makefile for Qt
#
SUFFIXES=.moc.cc .moc.cpp .moc.cxx .moc.C\
	 .h .hh \
         .ui .ui.h .ui.hh \
         .qrc .qrc.cc .qrc.cpp .qrc.cxx .qrc.C \
         .rc .o

# Moc rules

.hh.moc.cc:
	$(MOC) $(QT_CFLAGS) $< -o $@
.h.moc.cc:
	$(MOC) $(QT_CFLAGS) $< -o $@

.hh.moc.cpp:
	$(MOC) $(QT_CFLAGS) $< -o $@
.h.moc.cpp:
	$(MOC) $(QT_CFLAGS) $< -o $@

.hh.moc.cxx:
	$(MOC) $(QT_CFLAGS) $< -o $@
.h.moc.cxx:
	$(MOC) $(QT_CFLAGS) $< -o $@

.hh.moc.C:
	$(MOC) $(QT_CFLAGS) $< -o $@
.h.moc.C:
	$(MOC) $(QT_CFLAGS) $< -o $@

# Uic rules

.ui.ui.hh:
	$(UIC) $< -o $@

.ui.ui.h:
	$(UIC) $< -o $@

# Qrc rules

.qrc.qrc.cc:
	$(RCC) -name $$(echo "$<" | sed 's/\.qrc$$//') $< -o $@

.qrc.qrc.cpp:
	$(RCC) -name $$(echo "$<" | sed 's/\.qrc$$//') $< -o $@

.qrc.qrc.cxx:
	$(RCC) -name $$(echo "$<" | sed 's/\.qrc$$//') $< -o $@

.qrc.qrc.C:
	$(RCC) -name $$(echo "$<" | sed 's/\.qrc$$//') $< -o $@

# windows rc rules
.rc.o:
	windres $^ -o $@

CLEANFILES = $(QT_BUILT_SOURCES)
