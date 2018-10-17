BASE   = ../../rai
NAME   = $(shell basename `pwd`)
OUTPUT = lib$(NAME).so

DEPEND = Core Kin KOMO LGP

DEPEND_UBUNTU += python3 python3-numpy python3-pip

PYBIND = 1

SRCS = $(shell find . -maxdepth 1 -name '*.cpp' )
OBJS = $(SRCS:%.cpp=%.o)

include $(BASE)/build/generic.mk
