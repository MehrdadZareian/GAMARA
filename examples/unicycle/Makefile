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

.PHONY: Unicycle 

TARGET = Unicycle 

all: $(TARGET)

Unicycle: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(DYNINC)  unicycle.cpp -o unicycle 

clean:
	rm unicycle
