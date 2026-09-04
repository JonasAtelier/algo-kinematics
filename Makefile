CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB     = libdyn.a
OBJ     = src/dyn.o

all: $(LIB) gravity test_dyn

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/dyn.o: src/dyn.c include/dyn.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

gravity: examples/gravity.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_dyn: test/test_dyn.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_dyn cpp-smoke
	./test_dyn

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: gravity
	./gravity

clean:
	rm -f $(OBJ) $(LIB) gravity test_dyn test_cpp_linkage

.PHONY: all check cpp-smoke run clean
