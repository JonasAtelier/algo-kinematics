CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB     = libkin.a
OBJ     = src/kin.o

all: $(LIB) arm6 test_kin

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/kin.o: src/kin.c include/kin.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

arm6: examples/arm6.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_kin: test/test_kin.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_kin cpp-smoke
	./test_kin

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: arm6
	./arm6

clean:
	rm -f $(OBJ) $(LIB) arm6 test_kin test_cpp_linkage

.PHONY: all check cpp-smoke run clean
