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

.PHONY: CartPole 

TARGET = CartPole 

CartPole: $(TARGET)

CartPole: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(DYNINC)  cart_pole.cpp -o cart_pole 

clean:
	rm cart_pole
