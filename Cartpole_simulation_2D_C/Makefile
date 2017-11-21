#
# compiler
#
CC        		= gcc
CXXFLAGS 		= 


TARGET = cartpole_simulate

all: $(TARGET)

%.o:%.c
	$(CC) -c $(CXXFLAGS) $< -o $@

$(TARGET): $(TARGET).o
	$(CC) $(CXXFLAGS) -o $(TARGET) $(TARGET).o 

clean:
	rm  ./$(TARGET)  ./$(TARGET).o
