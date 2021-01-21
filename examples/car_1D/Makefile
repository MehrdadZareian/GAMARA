#
# compiler
#
CC        = g++
#CC       	  = clang++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3 -DNDEBUG -fopenmp
CXXDEBUG		= -g -Wall -Wextra -std=c++11 
PyFlags			= -fPIC `/usr/bin/python3-config --cflags` `/usr/bin/python3-config --ldflags`
#	
# scots 
#
SCOTSROOT		= ../..
SCOTSINC		= -I$(SCOTSROOT)/src
DYNINC		= -I$(SCOTSROOT)/dynamics

.PHONY: car 

TARGET = Car1D 

all: $(TARGET)

Car1D: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(DYNINC)  car1D.cpp -o car1D 

clean:
	rm car1D
