# Compiler
CXX := g++

# Compiler flags
CXXFLAGS := -Wall -Wextra -std=c++17

# Target executable
TARGET := main

# Source files
SRCS := main.cpp

# Build directory
BUILD_DIR := build

# Object files
OBJS := $(SRCS:%.cpp=$(BUILD_DIR)/%.o)

# Default target
all: $(BUILD_DIR) $(BUILD_DIR)/$(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build target
$(BUILD_DIR)/$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files to object files
$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean