# the compiler: gcc for C program, define as g++ for C++
CC = clang
CXX = clang++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -g -Wall -lstdc++

# the build target executable:
TARGET = silhouette
CCSRC = $(wildcard src/lib/*.cc) \
		$(wildcard src/*.cc)
OBJ = $(CCSRC:.cc=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) -o $@ $^ $(CFLAGS)
	$(RM) $(OBJ)

.PHONY: clean
clean:
	$(RM) $(TARGET) $(OBJ)
