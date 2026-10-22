CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB     = libmob.a
OBJ     = src/mob.o

all: $(LIB) rover test_mob

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/mob.o: src/mob.c include/mob.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

rover: examples/rover.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_mob: test/test_mob.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_mob cpp-smoke
	./test_mob

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: rover
	./rover

clean:
	rm -f $(OBJ) $(LIB) rover test_mob test_cpp_linkage

.PHONY: all check cpp-smoke run clean
