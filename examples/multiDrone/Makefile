#
# compiler
#
CC        = g++
#CC       	  = clang++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3 -DNDEBUG -fopenmp
CXXDEBUG		= -g -Wall -Wextra -std=c++11 

#	
# scots 
#
SCOTSROOT		= ../..
SCOTSINC		= -I$(SCOTSROOT)/src
DYNINC		= -I$(SCOTSROOT)/dynamics

.PHONY: MultiDrone simulation 

TARGET = MultiDrone simulation 

all: $(TARGET)

MultiDrone: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(DYNINC)  multiDrone.cpp -o multiDrone 
simulation:
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(DYNINC)  simulation.cpp -o simulate
clean:
	rm multiDrone
