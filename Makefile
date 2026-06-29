# Compiler
CXX = g++
CC = gcc

# Directories
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

SDL_DIR = vendor/SDL
GLAD_DIR = vendor/glad
IMGUI_DIR = vendor/imgui

# Executable
TARGET = $(BUILD_DIR)/imu_visualizer.exe

# Project sources
CPP_SRC := \
	$(wildcard $(SRC_DIR)/*.cpp) \
	$(wildcard $(SRC_DIR)/filter/*.cpp)

# ImGui sources
IMGUI_SRC := \
	$(IMGUI_DIR)/imgui.cpp \
	$(IMGUI_DIR)/imgui_draw.cpp \
	$(IMGUI_DIR)/imgui_tables.cpp \
	$(IMGUI_DIR)/imgui_widgets.cpp \
	$(IMGUI_DIR)/backends/imgui_impl_sdl3.cpp \
	$(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp

# C sources
C_SRC := $(GLAD_DIR)/src/gl.c

# Object files
CPP_OBJ := \
	$(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(wildcard $(SRC_DIR)/*.cpp)) \
	$(patsubst $(SRC_DIR)/filter/%.cpp,$(OBJ_DIR)/filter/%.o,$(wildcard $(SRC_DIR)/filter/*.cpp))

IMGUI_OBJ := \
	$(OBJ_DIR)/imgui.o \
	$(OBJ_DIR)/imgui_draw.o \
	$(OBJ_DIR)/imgui_tables.o \
	$(OBJ_DIR)/imgui_widgets.o \
	$(OBJ_DIR)/imgui_impl_sdl3.o \
	$(OBJ_DIR)/imgui_impl_opengl3.o

C_OBJ := $(OBJ_DIR)/gl.o

# Includes
INCLUDES = \
	-I$(INC_DIR) \
	-I$(INC_DIR)/filter \
	-I$(SDL_DIR)/include \
	-I$(GLAD_DIR)/include \
	-I$(IMGUI_DIR) \
	-I$(IMGUI_DIR)/backends

# Libraries
LIBS = \
	-L$(SDL_DIR)/build \
	-lSDL3 \
	-lopengl32

# Flags
CXXFLAGS = -std=c++20 -Wall -Wextra $(INCLUDES)
CFLAGS = -Wall -Wextra $(INCLUDES)

all: $(BUILD_DIR) $(OBJ_DIR) $(OBJ_DIR)/filter $(TARGET)

$(BUILD_DIR):
	if not exist build mkdir build

$(OBJ_DIR):
	if not exist build\obj mkdir build\obj

$(OBJ_DIR)/filter:
	if not exist build\obj\filter mkdir build\obj\filter

$(TARGET): $(CPP_OBJ) $(IMGUI_OBJ) $(C_OBJ)
	$(CXX) $^ -o $@ $(LIBS)
	xcopy assets build\assets /E /I /Y
	copy /Y vendor\SDL\build\SDL3.dll build\

# Project
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/filter/%.o: $(SRC_DIR)/filter/%.cpp | $(OBJ_DIR)/filter
	$(CXX) $(CXXFLAGS) -c $< -o $@

# GLAD
$(OBJ_DIR)/gl.o: $(GLAD_DIR)/src/gl.c
	$(CC) $(CFLAGS) -c $< -o $@

# ImGui
$(OBJ_DIR)/imgui.o:
	$(CXX) $(CXXFLAGS) -c $(IMGUI_DIR)/imgui.cpp -o $@

$(OBJ_DIR)/imgui_draw.o:
	$(CXX) $(CXXFLAGS) -c $(IMGUI_DIR)/imgui_draw.cpp -o $@

$(OBJ_DIR)/imgui_tables.o:
	$(CXX) $(CXXFLAGS) -c $(IMGUI_DIR)/imgui_tables.cpp -o $@

$(OBJ_DIR)/imgui_widgets.o:
	$(CXX) $(CXXFLAGS) -c $(IMGUI_DIR)/imgui_widgets.cpp -o $@

$(OBJ_DIR)/imgui_impl_sdl3.o:
	$(CXX) $(CXXFLAGS) -c $(IMGUI_DIR)/backends/imgui_impl_sdl3.cpp -o $@

$(OBJ_DIR)/imgui_impl_opengl3.o:
	$(CXX) $(CXXFLAGS) -c $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp -o $@

run: $(TARGET)
	$(TARGET)

clean:
	@if exist $(BUILD_DIR) rmdir /S /Q $(BUILD_DIR)

.PHONY: all run clean