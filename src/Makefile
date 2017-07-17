##################################################
# PROJECT: P1MC Makefile
# AUTHOR : Howard.Dong
##################################################

#---------------------------------------------------------------------
# Makefile template for projects using DXL SDK
#
# Please make sure to follow these instructions when setting up your
# own copy of this file:
#
#   1- Enter the name of the target (the TARGET variable)
#   2- Add additional source files to the SOURCES variable
#   3- Add additional static library objects to the OBJECTS variable
#      if necessary
#   4- Ensure that compiler flags, INCLUDES, and LIBRARIES are
#      appropriate to your needs
#
#
# This makefile will link against several libraries, not all of which
# are necessarily needed for your project.  Please feel free to
# remove libaries you do not need.
#---------------------------------------------------------------------

# *** ENTER THE TARGET NAME HERE ***
TARGET      = P1MC

# important directories used by assorted rules and other variables
DIR_DXL    = ..
DIR_OBJS   = objects

# compiler options
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -w -DLINUX -std=c++11 -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
CXFLAGS     = -O2 -O3 -w -DLINUX -std=c++11 -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
LNKCC       = $(CX)
LNKFLAGS    = $(CXFLAGS) #-Wl,-rpath,$(DIR_THOR)/lib
FORMAT      = -m64

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES   += -I$(DIR_DXL)/include
#LIBRARIES  += -ldxl_x64_cpp
LIBRARIES  += -lrt -larmadillo -lboost_system -lboost_filesystem -lboost_iostreams 
EXTRA_LIBS += -lpthread
#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES =    P1MC.cpp \
          ./controller/pid.cpp \
          ./port/port.cpp \
          ./port/dxl_packet.cpp \
          ./actuator/dxl_actuator \
          ./sensor/ati.cpp \
          ./sensor/mti.cpp \
          ./motion_ctrl/motion_ctrl.cpp\
          ./motion_ctrl/zmp_ctrl.cpp\
          ./motion_ctrl/kalman.cpp
    # *** OTHER SOURCES GO HERE ***

OBJECTS  = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))
#OBJETCS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***


#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(EXTRA_LIBS) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

mount:
	sudo mount -t nfs 192.168.1.133:/home/p1/Work /home/robot/nfs/

umount:
	sudo umount /home/robot/nfs

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/%.o: ./%.c
	$(CC) $(CCFLAGS) -c $? -o $@     #$? means $^,$@ ——目标文件的名称；$^ ——所有的依赖文件，以空格分开，不包含重复的依赖文件；$< ——第一个依赖文件的名称

$(DIR_OBJS)/%.o: ./%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ./controller/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ./port/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ./actuator/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ./sensor/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: ./motion_ctrl/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@
#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------
