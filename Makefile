CXX          = g++
CC           = gcc

# CC         = /opt/local/bin/g++
# AR         = /opt/local/bin/ar
# RANLIB     = /opt/local/bin/ranlib

FLAGS_COM  = -Isrc/ -fPIC -Wall -O3 -std=c++11 -Wunknown-pragmas
LIBS       = -lz

SRC        = src
SOURCES    = $(shell find $(SRC) -name '*.cpp')

OBJROOT    = build
OBJECTS    = $(patsubst $(SRC)/%.cpp,$(OBJROOT)/%.o,$(SOURCES))

SO_TARGET  = build/libdress.so
AR_TARGET  = build/libdress.a

PREFIX     = /usr/local/libdress

PLATFORM := $(shell uname)

ifeq "$(PLATFORM)" "Darwin"
CXX          = clang++
CC           = clang

CXXFLAGS     = $(FLAGS_COM) -D_ALL_LOGS

SO_TARGET    = build/libdress.dylib
else
CXXFLAGS     = $(FLAGS_COM)
endif

default: shared

$(OBJROOT)/%.o: $(SRC)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

shared: $(OBJECTS)
	$(CC) -shared -o $(SO_TARGET) $^ $(LDFLAGS) $(LIBS) -lstdc++

static: $(OBJECTS)
	$(AR) rcs $(AR_TARGET) $^

$(OBJECTS): | $(OBJROOT)

$(OBJROOT):
	mkdir -p $(OBJROOT)

install: default
	mkdir -p $(PREFIX)/include
	mkdir -p $(PREFIX)/lib
	cp src/libdress.h $(PREFIX)/include
	cp $(SO_TARGET) $(PREFIX)/lib

clean:
	rm build/*
