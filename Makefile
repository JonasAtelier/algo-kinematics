CC      ?= cc
AR      ?= ar
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS  = -lm

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

check: test_kin
	./test_kin

run: arm6
	./arm6

clean:
	rm -f $(OBJ) $(LIB) arm6 test_kin

.PHONY: all check run clean
