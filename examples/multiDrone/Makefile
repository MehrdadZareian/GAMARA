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

.PHONY: MultiDrone simulation 

TARGET = MultiDrone simulation 

all: $(TARGET)

MultiDrone: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) multiDrone.cpp -o abs_syn 
simulation:
	$(CC) $(CXXFLAGS) $(SCOTSINC) simulation.cpp -o simulate
clean:
	rm multiDrone
