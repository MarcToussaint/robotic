BASE = rai

target: build bin

DEPEND = Core Algo Geo Plot Kin Gui Optim KOMO LGP RosCom Operate Perception ry

build: $(DEPEND:%=inPath_makeLib/%)

bin:
	+make -C rai bin

installUbuntuAll: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntuAll: $(DEPEND:%=inPath_printUbuntu/%) printUbuntu

clean: $(DEPEND:%=inPath_clean/%)

dependAll: cleanLocks cleanDepends $(DEPEND:%=inPath_depend/%)

include $(BASE)/build/generic.mk

.NOTPARALLEL:
