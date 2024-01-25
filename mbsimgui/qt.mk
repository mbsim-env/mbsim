# Makefile for Qt
#
SUFFIXES=.moc.cc .moc.cpp .moc.cxx .moc.C\
	 .h .hh \
         .ui .ui.h .ui.hh \
         .o

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

CLEANFILES = $(QT_BUILT_SOURCES)
