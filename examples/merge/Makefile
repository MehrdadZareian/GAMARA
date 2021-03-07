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


.PHONY: Merge simulation 

TARGET = Merge simulation 

all: $(TARGET)

Merge: 
	$(CC) $(CXXFLAGS) $(SCOTSINC)  merge.cpp -o merge 
simulation:
	$(CC) $(CXXFLAGS) $(SCOTSINC)  simulation.cpp -o simulate
clean:
	rm merge
