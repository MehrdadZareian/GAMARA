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

.PHONY: Formation simulation 

TARGET = Formation simulation 

all: $(TARGET)

Formation: 
	$(CC) $(CXXFLAGS) $(SCOTSINC)  formation.cpp -o formation 
simulation:
	$(CC) $(CXXFLAGS) $(SCOTSINC)  simulation.cpp -o simulate
clean:
	rm formation
