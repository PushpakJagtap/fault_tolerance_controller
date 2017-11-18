#
# compiler
#
CC        		= g++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -g -O0


TARGET = cartpole_simulate

all: $(TARGET)

%.o:%.cc
	$(CC) -c $(CXXFLAGS) $< -o $@

$(TARGET): $(TARGET).o
	$(CC) $(CXXFLAGS) -o $(TARGET) $(TARGET).o 

clean:
	rm  ./$(TARGET)  ./$(TARGET).o
