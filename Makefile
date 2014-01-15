
####### Output directories

ODIR = obj
EDIR = bin
SDIR = src

####### Define ACADO related variables

LOCAL_PATH_PREFIX = /home/rkg/ros/acado
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk

####### Compiler, tools and options

CC       = gcc
CXX      = g++
CFLAGS   = -pipe -Wall -W -O2 -fPIC 
CXXFLAGS =  ${CPP_GLOBAL_FLAGS} ${NO_PARENTHESES_WARNING}  #use the ACADO flags
AR       = ar cqs 
DEL_FILE = rm -f

####### Include directories for external library headers

INCPATH  = ${HEADER_PATHS} #add ACADO headers

####### External libraries

LIBS = ${TOOLKIT_LIBS} #add ACADO libs

####### Files for compiling the code

DS_HEADERS = 	

DS_SOURCES = 	./src/nlsq.cpp \
		./src/fwL2.cpp \
		./src/nlsq_full.cpp \
		./src/ninf_full.cpp \
		./src/nlsq_full_bounds.cpp 

DS_OBJECTS =	./obj/nlsq.o \
		./obj/fwL2.o \
		./obj/nlsq_full.o \
		./obj/ninf_full.o \
		./obj/nlsq_full_bounds.o

DS_EXECS   =    ./bin/nlsq \
		./bin/fwL2 \
		./bin/nlsq_full \
		./bin/inf_full \
		./bin/nlsq_full_bounds

####### Implicit rules

.SUFFIXES: .c .o .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<

####### Build rules

all: nlsq fwL2 nlsq_full ninf_full nlsq_full_bounds

nlsq: ./obj/nlsq.o
	$(CXX)  $< $(INCPATH) $(LIBS) -o $(EDIR)/nlsq 

nlsq_full: ./obj/nlsq_full.o
	$(CXX)  $< $(INCPATH) $(LIBS) -o $(EDIR)/nlsq_full 

nlsq_full_bounds: ./obj/nlsq_full_bounds.o
	$(CXX)  $< $(INCPATH) $(LIBS) -o $(EDIR)/nlsq_full_bounds

ninf_full: ./obj/ninf_full.o
	$(CXX)  $< $(INCPATH) $(LIBS) -o $(EDIR)/ninf_full 

fwL2: ./obj/fwL2.o
	$(CXX)  $< $(INCPATH) $(LIBS) -o $(EDIR)/fwL2

####### Clean up	

clean:
	-$(DEL_FILE) $(DS_OBJECTS) $(DS_EXECS) 
	-$(DEL_FILE) *~ core *.core

####### Compile the source code

$(ODIR)/nlsq.o: $(SDIR)/nlsq.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/nlsq.o $(SDIR)/nlsq.cpp

$(ODIR)/nlsq_full.o: $(SDIR)/nlsq_full.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/nlsq_full.o $(SDIR)/nlsq_full.cpp

$(ODIR)/nlsq_full_bounds.o: $(SDIR)/nlsq_full_bounds.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/nlsq_full_bounds.o $(SDIR)/nlsq_full_bounds.cpp

$(ODIR)/fwL2.o: $(SDIR)/fwL2.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/fwL2.o $(SDIR)/fwL2.cpp

$(ODIR)/ninf_full.o: $(SDIR)/ninf_full.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/ninf_full.o $(SDIR)/ninf_full.cpp
















