# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20

# Output directory and target executable
OUT_DIR = .dist
TARGET = $(OUT_DIR)/yatch_simulator

# Source files
SRCS = src/main.cpp

# Default target
all: $(TARGET)

# Rule to create the output directory if it doesn't exist
$(OUT_DIR):
	mkdir -p $(OUT_DIR)

# Rule for building the target
$(TARGET): $(OUT_DIR) $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

# Run the compiled executable with the given argument
run: $(TARGET)
	$(TARGET) src/io

# Clean up build files
clean:
	rm -rf $(OUT_DIR)

# PHONY targets to avoid file name conflicts
.PHONY: all clean run
