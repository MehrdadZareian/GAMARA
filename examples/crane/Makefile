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

.PHONY: Crane simulation 

TARGET = Crane simulation 

all: $(TARGET)

Crane: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) crane.cpp -o abs_syn 
simulation:
	$(CC) $(CXXFLAGS) $(SCOTSINC) simulation.cpp -o simulate
clean:
	rm abs_syn simulate
