# Charvak Physics Engine Makefile

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g
LDFLAGS = -lm

# Directories
SRC_DIR = src
INCLUDE_DIR = include
EXAMPLES_DIR = examples
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.c)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Library name
LIB_NAME = libcharvak_physics
STATIC_LIB = $(BUILD_DIR)/$(LIB_NAME).a
SHARED_LIB = $(BUILD_DIR)/$(LIB_NAME).so

# Example programs
DEMO = $(BUILD_DIR)/demo

# Default target
all: $(STATIC_LIB) $(SHARED_LIB) $(DEMO)

# Create build directories
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(OBJ_DIR): | $(BUILD_DIR)
	mkdir -p $(OBJ_DIR)

# Compile source files to object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -fPIC -c $< -o $@

# Create static library
$(STATIC_LIB): $(OBJECTS) | $(BUILD_DIR)
	ar rcs $@ $(OBJECTS)
	@echo "Static library created: $@"

# Create shared library
$(SHARED_LIB): $(OBJECTS) | $(BUILD_DIR)
	$(CC) -shared -o $@ $(OBJECTS) $(LDFLAGS)
	@echo "Shared library created: $@"

# Build demo program
$(DEMO): $(EXAMPLES_DIR)/demo.c $(STATIC_LIB) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -o $@ $< $(STATIC_LIB) $(LDFLAGS)
	@echo "Demo program created: $@"

# Install headers and libraries (optional)
install: $(STATIC_LIB) $(SHARED_LIB)
	@echo "Installing physics engine..."
	sudo mkdir -p /usr/local/include/charvak_physics
	sudo cp $(INCLUDE_DIR)/*.h /usr/local/include/charvak_physics/
	sudo cp $(STATIC_LIB) /usr/local/lib/
	sudo cp $(SHARED_LIB) /usr/local/lib/
	sudo ldconfig
	@echo "Installation complete!"

# Uninstall
uninstall:
	@echo "Uninstalling physics engine..."
	sudo rm -rf /usr/local/include/charvak_physics
	sudo rm -f /usr/local/lib/$(LIB_NAME).a
	sudo rm -f /usr/local/lib/$(LIB_NAME).so
	sudo ldconfig
	@echo "Uninstall complete!"

# Run demo
run-demo: $(DEMO)
	./$(DEMO)

# Run with valgrind for memory checking
valgrind: $(DEMO)
	valgrind --leak-check=full --show-leak-kinds=all ./$(DEMO)

# Clean build files
clean:
	rm -rf $(BUILD_DIR)
	@echo "Clean complete!"

# Create documentation with doxygen (if available)
docs:
	@if command -v doxygen >/dev/null 2>&1; then \
		doxygen Doxyfile 2>/dev/null || echo "Create Doxyfile first: doxygen -g"; \
	else \
		echo "Doxygen not found. Install doxygen to generate documentation."; \
	fi

# Show help
help:
	@echo "Charvak Physics Engine Build System"
	@echo "===================================="
	@echo ""
	@echo "Available targets:"
	@echo "  all         - Build static library, shared library, and demo"
	@echo "  static      - Build static library only"
	@echo "  shared      - Build shared library only"
	@echo "  demo        - Build demo program only"
	@echo "  run-demo    - Build and run demo program"
	@echo "  install     - Install libraries and headers to system"
	@echo "  uninstall   - Remove installed files from system"
	@echo "  valgrind    - Run demo with valgrind memory checking"
	@echo "  clean       - Remove all build files"
	@echo "  docs        - Generate documentation (requires doxygen)"
	@echo "  help        - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make              # Build everything"
	@echo "  make run-demo     # Build and run demo"
	@echo "  make clean all    # Clean and rebuild"

# Individual targets for convenience
static: $(STATIC_LIB)
shared: $(SHARED_LIB)
demo: $(DEMO)

# Debug build with extra flags
debug: CFLAGS += -DDEBUG -O0 -g3
debug: clean all

# Release build with optimizations
release: CFLAGS += -DNDEBUG -O3 -march=native
release: clean all

# Check code formatting (requires clang-format)
format:
	@if command -v clang-format >/dev/null 2>&1; then \
		find $(SRC_DIR) $(INCLUDE_DIR) $(EXAMPLES_DIR) -name "*.c" -o -name "*.h" | xargs clang-format -i; \
		echo "Code formatted successfully!"; \
	else \
		echo "clang-format not found. Install clang-format to format code."; \
	fi

# Static analysis (requires cppcheck)
analyze:
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --std=c99 --suppress=missingIncludeSystem $(SRC_DIR) $(INCLUDE_DIR); \
	else \
		echo "cppcheck not found. Install cppcheck for static analysis."; \
	fi

# Phony targets
.PHONY: all clean install uninstall run-demo valgrind docs help static shared demo debug release format analyze

# Dependency tracking
-include $(OBJECTS:.o=.d)

$(OBJ_DIR)/%.d: $(SRC_DIR)/%.c | $(OBJ_DIR)
	@$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -MM -MT $(@:.d=.o) $< > $@