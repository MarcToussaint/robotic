BASE = rai
BASE2 = src

default: build bin

DEPEND = Core Algo Geo Plot Kin Gui KOMO LGP ry

test_paths = $(shell find . -maxdepth 1 -name '??-*' -printf "%f ")

build: $(DEPEND:%=inPath_makeLib/%)

bin:
	+make -C rai bin

initUbuntuPackages: $(DEPEND:%=inPath_installUbuntu/%)

printUbuntu: $(DEPEND:%=inPath_printUbuntuPackages/%) printUbuntuPackages

clean: $(DEPEND:%=inPath_clean/%)

depend: $(DEPEND:%=inPath_depend/%) $(test_paths:%=inPath_depend/%)

runTests:
	+make -C rai tests
	make -C rai runTests

include $(BASE)/build/generic.mk

.NOTPARALLEL:
