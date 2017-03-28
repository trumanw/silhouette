# the compiler: gcc for C program, define as g++ for C++
CC = gcc
CXX = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CFLAGS  = -g -Wall
TARGET = silhouette
TARGET_DIR = build/

# C source code
CSRC = $(wildcard src/c/lib/*.c) \
	$(wildcard src/c/*.c)

# C++ source code
CCSRC = $(wildcard src/cpp/lib/*.cpp) \
	$(wildcard src/cpp/*.cpp)

SRC = $(wildcard src/*.cpp)

COBJ = $(CSRC:.c=.o)
CCOBJ = $(CCSRC:.cpp=.o)
OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(COBJ) $(CCOBJ) $(OBJ)
	mkdir -p $(TARGET_DIR)
	$(CXX) -o $(TARGET_DIR)$@ $^ $(CFLAGS)
	# $(CC) -o $(TARGET_DIR)$@ $^ $(CFLAGS)

	$(RM) $(CCOBJ) $(COBJ) $(OBJ)

anim: $(CCTARGET)
	python plot.py

.PHONY: clean all
clean:
	$(RM) -rf $(CCOBJ) $(COBJ) $(OBJ) $(TARGET_DIR)
