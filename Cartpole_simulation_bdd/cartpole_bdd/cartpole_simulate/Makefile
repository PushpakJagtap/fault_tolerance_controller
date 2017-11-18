#
# compiler
#
#CC        = g++
CC       	  = clang++
CXXFLAGS 		= -g -Wall -Wextra -std=c++11 -DSCOTS_BDD
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3 -DNDEBUG -DSCOTS_BDD
#
# scots 
#
SCOTSROOT		= ../..
SCOTSINC		= -I$(SCOTSROOT)/src
#
# cudd 
#
CUDD		    =  -I/opt/local/include -L/opt/local/lib -lcudd


.PHONY: simulate

TARGET = simulate

all: $(TARGET)

dcdc: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(CUDD) cartpole.cc -o cartpole

simulate: 
	$(CC) $(CXXFLAGS) $(SCOTSINC) $(CUDD) simulate.cc -o simulate

clean:
	rm simulate
