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

.PHONY: Cartpole simulation 

TARGET = Cartpole simulation 

all: $(TARGET)

Cartpole: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(DYNINC)  cartpole.cpp -o cartpole 
simulation:
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(DYNINC)  simulation.cpp -o simulate
clean:
	rm formation
