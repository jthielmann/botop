INC = -I/../include

DEPEND = Core Algo Gui Geo Kin Franka Control BotOp
OPENCV = 0

OBJS = ../include/jt_base.o main.o
BASE = ../../botop/rai
BASE2 = ../../botop/src

include $(BASE)/build/generic.mk

jt_base.o:
	cd ../include;g++ $(CXXFLAGS) $(INC) -c jt_base.cpp -o jt_base.o;