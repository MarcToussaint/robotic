BASE = rai

target: build bin

DEPEND = Core Algo Geo Plot Kin Gui Optim KOMO ry

build: $(DEPEND:%=inPath_makeLib/%)

bin: force
	+make -C rai bin

installUbuntuAll: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntuAll: $(DEPEND:%=inPath_printUbuntu/%) printUbuntu

clean: $(DEPEND:%=inPath_clean/%)

dependAll: cleanLocks cleanDepends $(DEPEND:%=inPath_depend/%)

include $(BASE)/build/generic.mk

.NOTPARALLEL:
